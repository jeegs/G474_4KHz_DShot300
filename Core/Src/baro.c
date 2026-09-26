/*
 *  baro.c
 *  BMP390L Driver implementation
 *  Re_created for BMP390L on: (Current Date)
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 *   - Bosch 보정 계수 스케일링 추가(정수 그대로 쓰던 치명적 오류 수정), double 연산
 *   - OSR/ODR 을 유효한 조합으로 수정 (압력 x8 / 온도 x1 / ODR 50Hz / IIR 계수 3)
 *   - 소프트웨어 필터 중첩 제거(하드웨어 IIR 하나만 사용) -> 위상 지연 감소
 *   - 변화율(brake_throttle)을 float(Pa/s)로 변경 (int16 양자화 제거)
 *   - CHIP ID 확인, DRDY 확인 후 읽기, 워밍업(baro.ready), 초기값 0 과도상태 제거
 */

#include "baro.h"
#include "pid.h"
#include "dshot.h"
#include <math.h>

BARO baro;
uint32_t baro_counter;
float brake_throttle;

#define baro_CS 1

#define BARO_POLL_LOOPS      20   // 4kHz/20 = 200Hz 로 DRDY 만 확인 (실제 읽기는 새 샘플이 있을 때만)
#define BARO_WARMUP_SAMPLES  10   // 0.2s 동안은 ready = false
#define BARO_RATE_WINDOW     10   // 변화율 계산 창 = 10샘플 = 0.2s

// --- BMP390L 레지스터 주소 ---
#define BMP3_CHIP_ID_ADDR       0x00
#define BMP3_STATUS_ADDR        0x03
#define BMP3_DATA_ADDR          0x04
#define BMP3_PWR_CTRL_ADDR      0x1B
#define BMP3_OSR_ADDR           0x1C
#define BMP3_ODR_ADDR           0x1D
#define BMP3_CONFIG_ADDR        0x1F
#define BMP3_CALIB_DATA_ADDR    0x31
#define BMP3_CMD_ADDR           0x7E

#define BMP3_CHIP_ID            0x60
#define BMP3_STATUS_DRDY_MASK   0x60 // bit5: drdy_press, bit6: drdy_temp

#define BMP3_PRESS_MIN_PA       30000.0
#define BMP3_PRESS_MAX_PA       125000.0

// --- BMP390L 양자화(스케일 적용된) 캘리브레이션 계수 ---
struct bmp3_calib_data {
	double par_t1, par_t2, par_t3;
	double par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
	double par_p7, par_p8, par_p9, par_p10, par_p11;
};
static struct bmp3_calib_data calib;
static bool baro_present = false;

// SPI LL 통신 함수 (MS5611과 동일)
void spi_ll_open() {
	if (baro_CS == 1)
		LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
}
void spi_ll_close() {
	if (baro_CS == 1)
		LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
}

uint8_t spi_ll_transfer(uint8_t data) {
	while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
	}
	LL_SPI_TransmitData8(SPI1, data);
	while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
	}
	return LL_SPI_ReceiveData8(SPI1);
}

void bmp3_write_reg(uint8_t reg, uint8_t data) {
	spi_ll_open();
	spi_ll_transfer(reg & 0x7F); // Write: MSB 0
	spi_ll_transfer(data);
	spi_ll_close();
}

uint8_t bmp3_read_reg(uint8_t reg) {
	spi_ll_open();
	spi_ll_transfer(reg | 0x80); // Read: MSB 1
	spi_ll_transfer(0x00);       // Dummy byte for SPI read delay in BMP390
	uint8_t data = spi_ll_transfer(0x00);
	spi_ll_close();
	return data;
}

void bmp3_read_regs(uint8_t reg, uint8_t *data, uint16_t len) {
	spi_ll_open();
	spi_ll_transfer(reg | 0x80);
	spi_ll_transfer(0x00); // Dummy byte
	for (uint16_t i = 0; i < len; i++) {
		data[i] = spi_ll_transfer(0x00);
	}
	spi_ll_close();
}

// 캘리브레이션 데이터 파싱 + Bosch 데이터시트(9.1) 스케일링
void bmp3_get_calib_data(void) {
	uint8_t d[21];
	bmp3_read_regs(BMP3_CALIB_DATA_ADDR, d, 21);

	uint16_t nvm_t1 = (uint16_t) (d[1] << 8 | d[0]);
	uint16_t nvm_t2 = (uint16_t) (d[3] << 8 | d[2]);
	int8_t nvm_t3 = (int8_t) d[4];
	int16_t nvm_p1 = (int16_t) (d[6] << 8 | d[5]);
	int16_t nvm_p2 = (int16_t) (d[8] << 8 | d[7]);
	int8_t nvm_p3 = (int8_t) d[9];
	int8_t nvm_p4 = (int8_t) d[10];
	uint16_t nvm_p5 = (uint16_t) (d[12] << 8 | d[11]);
	uint16_t nvm_p6 = (uint16_t) (d[14] << 8 | d[13]);
	int8_t nvm_p7 = (int8_t) d[15];
	int8_t nvm_p8 = (int8_t) d[16];
	int16_t nvm_p9 = (int16_t) (d[18] << 8 | d[17]);
	int8_t nvm_p10 = (int8_t) d[19];
	int8_t nvm_p11 = (int8_t) d[20];

	calib.par_t1 = (double) nvm_t1 * 256.0;                    // / 2^-8
	calib.par_t2 = (double) nvm_t2 / 1073741824.0;             // / 2^30
	calib.par_t3 = (double) nvm_t3 / 281474976710656.0;        // / 2^48
	calib.par_p1 = ((double) nvm_p1 - 16384.0) / 1048576.0;    // (x-2^14) / 2^20
	calib.par_p2 = ((double) nvm_p2 - 16384.0) / 536870912.0;  // (x-2^14) / 2^29
	calib.par_p3 = (double) nvm_p3 / 4294967296.0;             // / 2^32
	calib.par_p4 = (double) nvm_p4 / 137438953472.0;           // / 2^37
	calib.par_p5 = (double) nvm_p5 * 8.0;                      // / 2^-3
	calib.par_p6 = (double) nvm_p6 / 64.0;                     // / 2^6
	calib.par_p7 = (double) nvm_p7 / 256.0;                    // / 2^8
	calib.par_p8 = (double) nvm_p8 / 32768.0;                  // / 2^15
	calib.par_p9 = (double) nvm_p9 / 281474976710656.0;        // / 2^48
	calib.par_p10 = (double) nvm_p10 / 281474976710656.0;      // / 2^48
	calib.par_p11 = (double) nvm_p11 / 36893488147419103232.0; // / 2^65
}

void baro_init(Sensor type) {
	baro.ready = false;
	baro.sample_count = 0;
	baro_present = false;

	spi_ll_close(); // CS HIGH

	// 1. Soft Reset
	bmp3_write_reg(BMP3_CMD_ADDR, 0xB6);
	dshot_delay_ms(10);

	// 2. SPI 활성화를 위한 더미 리드
	bmp3_read_reg(BMP3_CHIP_ID_ADDR);
	dshot_delay_ms(10);

	// 3. CHIP ID 확인 (BMP390 = 0x60). 다르면 baro.ready 는 false 로 남는다.
	if (bmp3_read_reg(BMP3_CHIP_ID_ADDR) != BMP3_CHIP_ID) {
		return;
	}

	// 4. Calibration 팩터 획득
	bmp3_get_calib_data();

	// 5. 센서 설정
	// OSR: Pressure x8, Temperature x1  (osr_t[5:3]=000, osr_p[2:0]=011)
	//  - 변환시간 약 19ms < ODR 20ms. (x16/x2 는 약 37ms 라 50Hz 에서 무효)
	bmp3_write_reg(BMP3_OSR_ADDR, 0x03);

	// ODR: 50Hz (20ms)
	bmp3_write_reg(BMP3_ODR_ADDR, 0x02);

	// IIR Filter: 계수 3 (iir_filter[3:1]=010). 이것이 유일한 필터 (SW 필터 제거)
	bmp3_write_reg(BMP3_CONFIG_ADDR, 0x04);

	// Power Control: Pressure + Temp ON, Normal Mode
	bmp3_write_reg(BMP3_PWR_CTRL_ADDR, 0x33);

	dshot_delay_ms(30); // 첫 변환 대기

	baro_present = true;
}

// --- 보상 알고리즘 (Bosch 데이터시트, double 정밀도) ---
static double t_lin; // [degC]
static void bmp3_compensate_temp(uint32_t uncomp_t) {
	double partial_data1 = (double) uncomp_t - calib.par_t1;
	double partial_data2 = partial_data1 * calib.par_t2;
	t_lin = partial_data2 + (partial_data1 * partial_data1) * calib.par_t3;
}

static double bmp3_compensate_press(uint32_t uncomp_p) {
	double partial_data1, partial_data2, partial_data3, partial_data4;
	double partial_out1, partial_out2;
	double up = (double) uncomp_p;

	partial_data1 = calib.par_p6 * t_lin;
	partial_data2 = calib.par_p7 * (t_lin * t_lin);
	partial_data3 = calib.par_p8 * (t_lin * t_lin * t_lin);
	partial_out1 = calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = calib.par_p2 * t_lin;
	partial_data2 = calib.par_p3 * (t_lin * t_lin);
	partial_data3 = calib.par_p4 * (t_lin * t_lin * t_lin);
	partial_out2 = up
			* (calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = up * up;
	partial_data2 = calib.par_p9 + calib.par_p10 * t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + (up * up * up) * calib.par_p11;

	return partial_out1 + partial_out2 + partial_data4; // Pascal(Pa)
}

// 압력 변화율 [Pa/s]: 최근 BARO_RATE_WINDOW 샘플 창의 (현재 - 과거)/시간
// 정수 양자화 없이 float 로 계산. 워밍업 동안, 또는 고도 setpoint 변경 중에는 0.
static float brake_rate(double press_pa) {
	static float hist[BARO_RATE_WINDOW];
	static uint8_t idx = 0;
	static uint8_t filled = 0;

	float oldest = hist[idx];
	hist[idx] = (float) press_pa;
	idx++;
	if (idx >= BARO_RATE_WINDOW)
		idx = 0;

	if (filled < BARO_RATE_WINDOW) {
		filled++;
		return 0.0f;
	}
	if (alt.setpoint_changing)
		return 0.0f;

	return ((float) press_pa - oldest) / (BARO_RATE_WINDOW * BARO_DT);
}

float get_pressure(Sensor type) {

	// 메인루프(4kHz) 20회마다 DRDY 만 확인하고, 새 샘플이 있을 때만 읽는다 (센서 ODR 50Hz 에 동기)
	if (baro_present && (baro_counter++ % BARO_POLL_LOOPS) == 0) {

		if ((bmp3_read_reg(BMP3_STATUS_ADDR) & BMP3_STATUS_DRDY_MASK)
				== BMP3_STATUS_DRDY_MASK) {

			uint8_t raw_data[6];
			bmp3_read_regs(BMP3_DATA_ADDR, raw_data, 6);

			uint32_t uncomp_p = (uint32_t) raw_data[0]
					| ((uint32_t) raw_data[1] << 8)
					| ((uint32_t) raw_data[2] << 16);
			uint32_t uncomp_t = (uint32_t) raw_data[3]
					| ((uint32_t) raw_data[4] << 8)
					| ((uint32_t) raw_data[5] << 16);

			bmp3_compensate_temp(uncomp_t);
			double press_pa = bmp3_compensate_press(uncomp_p);

			// 범위 밖(리셋 직후 0x800000 등) 값은 버린다
			if (press_pa > BMP3_PRESS_MIN_PA && press_pa < BMP3_PRESS_MAX_PA) {
				float press_hpa = (float) (press_pa / 100.0);

				baro.short_pressure = press_hpa;
				baro.long_pressure = press_hpa;
				baro.press_compensated = press_hpa;
				brake_throttle = brake_rate(press_pa);

				baro.sample_count++;
				if (baro.sample_count >= BARO_WARMUP_SAMPLES) {
					baro.ready = true;
				}
			}
		}
	}
	return baro.press_compensated;
}
