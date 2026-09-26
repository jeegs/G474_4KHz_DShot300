/*
 * imu.c
 *
 *  Re_created on: Mar 08, 2024
 *      Author: GyooSuhn, JEE
 *
 *  Revised: 2026-0922, by JEE.
 *
 *  Revised: 2026-09-25 22:10 (Claude)
 *   - 요 보정의 sinf() 2회/루프 제거: 한 샘플의 회전각은 <0.13deg(0.0022rad)라
 *     sin(x)=x 로 오차 <2e-9. 4kHz 루프 시간 절약 + 값 한 번만 계산
 *   - GYRO_DT/FC_LOOP_TIME 을 main.h 의 FC_LOOP_DT 로 통일
 *   - CONFIG=0x00 주석 정정: DLPF 우회가 아니라 자이로 DLPF 250Hz / 8kHz ODR
 *
 *  Revised: 2026-09-26 (Claude)
 *   - 자이로 FIFO 읽기(IMU_USE_FIFO): 8kHz 샘플을 모두 받아 루프(4kHz)마다 평균 (보통 2개)
 *   - 자이로 범위 +-500 -> +-1000dps (GYRO_FS_SEL / DPS, imu.h)
 *   - 자이로 소프트웨어 LPF 100Hz -> 150Hz (지연 약 1.6ms -> 1.1ms). 하드웨어 DLPF 250Hz 는 유지
 *   - CONFIG 레지스터를 명시적으로 기록 (기존엔 리셋 기본값 0x00 에 의존)
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 *   - double 연산(pow/sqrt/asin/sin/dmul) 전부 float 로 교체 (Cortex-M4F 는 float 만 HW 지원)
 *   - 가속도 각도는 4루프에 1번만 계산(가속도 출력이 1kHz)
 *   - 자이로 LPF 를 fc≈13Hz -> 약 100Hz 로 수정 (0.98/0.02 -> α=0.145)
 *   - 초기화 지연을 dshot_delay_ms 로 교체 (ESC 에 0 프레임 유지)
 */

#include "imu.h"
#include "dshot.h"

//Local:

// float 상수 (double 연산 회피)
#define GYRO_DT          FC_LOOP_DT          // 4kHz (main.h)
#define DPS_F            ((float) DPS)       // 65.5 LSB/(deg/s)
#define RAD2DEG_F        57.2957795f
#define DEG2RAD_F        0.0174532925f
#define ACCEL_ANGLE_DIV  4                   // 가속도 각도 계산 주기 (4kHz/4 = 1kHz)

// 자이로 1차 LPF: α = 1 - exp(-2π·fc·dt) @4kHz
//   fc=100Hz -> 0.145 (지연 약 1.6ms),  fc=150Hz -> 0.21 (약 1.1ms),  fc=200Hz -> 0.27 (약 0.8ms)
// 2026-09-26: 100Hz -> 150Hz. 비행 후 모터가 뜨겁거나 떨림 소리가 커지면 0.145 로 되돌릴 것.
// (기존 0.02 는 fc≈12.9Hz 로 레이트 루프 지연이 과도했음. 2kHz 시절 0.1 = 33Hz)
#define GYRO_LPF_ALPHA   0.21f

IMU accel;
IMU gyro;
RPY gyroAngle;
RPY accelAngle;
RPY gyroAngular;
IMU_FIFO_DIAG imu_fifo;
static void imu_readback(void);
//Kalman_test:
KalmanFilter kf_roll;
KalmanFilter kf_pitch;

void spi2_open() {
	if (1) {
		LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin); //CS_open:
	}
}

void spi2_close() {
	if (1) {
		LL_GPIO_SetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin); //CS_close:
	}
}

uint8_t spi2_transfer(uint8_t data) {
	while (!LL_SPI_IsActiveFlag_TXE(SPI2)) {
	}
	LL_SPI_TransmitData8(SPI2, data); //When TXE
	while (!LL_SPI_IsActiveFlag_RXNE(SPI2)) {
	}
	return LL_SPI_ReceiveData8(SPI2); //When RXNE
}

void spi2_write(uint8_t address, uint8_t data) {
	spi2_open();
	spi2_transfer(address | 0x00); //127, | 0x00 -> & 0x7F???
	spi2_transfer(data);
	spi2_close();
}

uint8_t spi2_ll_read(uint8_t address) {
	uint8_t val;
	spi2_open();
	spi2_transfer(address | 0x80); //0x80 = 128(10) = 1000 0000(2)
	val = spi2_transfer(0x00);
	spi2_close();
	return val;
}

void spi2_ll_reads(uint8_t address, uint8_t *data, uint8_t length) {
	uint8_t i = 0;
	spi2_open();
	spi2_transfer(address | 0x80);
	while (i < length) {
		data[i++] = spi2_transfer(0x00);
	}
	spi2_close();
}

void imu_configure(void) {
	spi2_write(PWR_MGMT_1, 0x80);
	dshot_delay_ms(100);
	spi2_write(SIGNAL_PATH_RESET, 0x03); //ACCEL&TEMP_RST
	dshot_delay_ms(100);
	spi2_write(PWR_MGMT_1, 0x01); //PLL enabled
	dshot_delay_ms(10);
	spi2_write(PWR_MGMT_2, 0x00);
	dshot_delay_ms(10);
	// DLPF_CFG=0 (FCHOICE_B=0): 자이로 DLPF 대역 250Hz(지연 ~0.97ms), 자이로 ODR 8kHz.
	// (DLPF '우회'가 아님. 우회는 GYRO_CONFIG 의 FCHOICE_B 를 켜야 함: BW 3.6kHz, 지연 ~0.17ms)
	// FIFO 사용 시 bit6 FIFO_MODE=1: 가득 차면 더 쓰지 않음(패킷 경계가 깨지지 않게) -> 넘치면 리셋
	spi2_write(I2C_IF, 0x40); // I2C 인터페이스 끔 (SPI 전용)
	dshot_delay_ms(10);
	spi2_write(CONFIG, IMU_USE_FIFO ? 0x40 : 0x00);
	dshot_delay_ms(10);
	spi2_write(GYRO_CONFIG, GYRO_FS_SEL); // imu.h (현재 +-1000dps)
	dshot_delay_ms(10);
	spi2_write(ACCEL_CONFIG, 0x10); //±8g
	dshot_delay_ms(10);
	spi2_write(ACCEL_CONFIG2, 0x03);
	dshot_delay_ms(10);
	spi2_write(SMPLRT_DIV, 0x00);
#if IMU_USE_FIFO
	dshot_delay_ms(10);
	imu_fifo_reset();
#endif
	dshot_delay_ms(1);
	imu_readback();
}

#if IMU_USE_FIFO
// FIFO 비우고 다시 시작 (가속도 + 자이로, 한 패킷 14바이트)
// 기체 확인(2026-09-26): 리셋 직후 바로 FIFO_EN 을 쓰면 USER_CTRL 이 0x00 으로 남아 FIFO 가 꺼져 있었음.
// -> 리셋 후 충분히 기다리고, FIFO_EN 을 쓴 뒤 되읽어서 들어갈 때까지 최대 5회 다시 쓴다.
static void imu_short_delay(void) {
	for (volatile int d = 0; d < 5000; d++) { // 약 0.1~0.3ms (최적화 수준에 따라 다름)
	}
}

void imu_fifo_reset(void) {
	spi2_write(ICM_FIFO_EN, 0x00);
	spi2_write(USER_CTRL, 0x00);   // FIFO 끔
	imu_short_delay();
	spi2_write(USER_CTRL, 0x04);   // FIFO_RST (FIFO 끈 상태에서)
	imu_short_delay();
	spi2_write(ICM_FIFO_EN, 0x18); // GYRO + ACCEL
	imu_short_delay();
	for (int retry = 0; retry < 5; retry++) {
		spi2_write(USER_CTRL, 0x40); // FIFO_EN
		imu_short_delay();
		imu_fifo.rb_user_ctrl = spi2_ll_read(USER_CTRL);
		if (imu_fifo.rb_user_ctrl & 0x40)
			break;
	}
}
#endif

// 설정이 실제로 들어갔는지 되읽기 (imu_fifo.rb_*)
static void imu_readback(void) {
	imu_fifo.rb_config = spi2_ll_read(CONFIG);
	imu_fifo.rb_gyro_config = spi2_ll_read(GYRO_CONFIG);
	imu_fifo.rb_fifo_en = spi2_ll_read(ICM_FIFO_EN);
	imu_fifo.rb_user_ctrl = spi2_ll_read(USER_CTRL); // 0x40 이어야 FIFO 가 켜진 것
	imu_fifo.rb_pwr_mgmt_1 = spi2_ll_read(PWR_MGMT_1);
	imu_fifo.rb_i2c_if = spi2_ll_read(I2C_IF);
	imu_fifo.rb_whoami = spi2_ll_read(WHO_AM_I);
}

void gyro_offset_calculate(void) {
	gyro.x_offset = 0;
	gyro.y_offset = 0;
	gyro.z_offset = 0;
	gyro.offset_calculated = false;
	for (int i = 0; i < 2000; i++) {
		imu_read();
		if (i % 100 == 0)
			LL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
		gyro.x_offset += gyro.x;
		gyro.y_offset += gyro.y;
		gyro.z_offset += gyro.z;
		dshot_delay_ms(1);
	}
	gyro.x_offset /= 2000;
	gyro.y_offset /= 2000;
	gyro.z_offset /= 2000;
	gyro.offset_calculated = true;
}

void accel_offset_calculate(void) {
	accel.x_offset = 0;
	accel.y_offset = 0;
	accel.offset_calculated = false;
	for (int i = 0; i < 64; i++) {
		imu_read();
		if (i % 16 == 0)
			LL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
		accel.x_offset += accel.x;
		accel.y_offset += accel.y;
		dshot_delay_ms(1);
	}
	accel.x_offset /= 64;
	accel.y_offset /= 64;

	//printf("%ld %ld\n", accel.x_offset, accel.y_offset);
	// 수평면에서 미리 측정한 값을 의도적으로 사용 (위에서 계산한 평균은 덮어씀) - 의도된 동작
	//G474 predefined: 2024-04-2
	accel.x_offset = -80 - 60;//-110;
	accel.y_offset = -80 + 95;//95;

	accel.offset_calculated = true;
}

void imu_init(Sensor type) {
	imu_configure();
	gyro_offset_calculate();
	accel_offset_calculate();
	//printf("%d %d %d %d %d\n", accel.x, accel.y, gyro.x, gyro.y, gyro.z);
}

void imu_read(void) {

	uint8_t imuData[14];

#if IMU_USE_FIFO
	// 1) FIFO 에 쌓인 바이트 수
	static uint8_t empty_run = 0;
	uint8_t cnt[2];
	spi2_ll_reads(FIFO_COUNTH, cnt, 2);
	imu_fifo.count_h = cnt[0];
	imu_fifo.count_l = cnt[1];
	uint16_t count = ((uint16_t) (cnt[0] & 0x1F) << 8) | cnt[1]; // 상위 예약 비트 제거
	uint16_t n = count / IMU_FIFO_PACKET;

	if (count >= IMU_FIFO_SIZE || n > IMU_FIFO_MAX_READ) {
		// 넘침(또는 오래 못 읽음): 비우고 이번엔 직전 값을 그대로 쓴다
		imu_fifo_reset();
		imu_fifo.overflow_count++;
		return;
	}
	if (n == 0) {
		imu_fifo.empty_count++;
		if (++empty_run < IMU_FIFO_EMPTY_FALLBACK)
			return; // 잠깐 빈 것: 직전 값 유지
		// 계속 비어 있음 = FIFO 가 동작하지 않음 -> 자이로가 멈추지 않도록 레지스터를 직접 읽는다
		empty_run = IMU_FIFO_EMPTY_FALLBACK;
		imu_fifo.fallback_count++;
		imu_fifo.last_samples = 0;
		spi2_ll_reads(ACCEL_XOUT_H, imuData, 14);
		goto parse;
	}
	empty_run = 0;

	// 2) n 개 패킷을 한 번에 읽는다 (FIFO_R_W 는 주소가 증가하지 않고 계속 꺼내진다)
	uint8_t buf[IMU_FIFO_MAX_READ * IMU_FIFO_PACKET];
	spi2_ll_reads(FIFO_R_W, buf, (uint8_t) (n * IMU_FIFO_PACKET));
	imu_fifo.last_samples = (uint8_t) n;

	// 3) 가속도/온도는 가장 최근 패킷, 자이로는 n 개 평균 -> 레지스터 읽기와 같은 14바이트 형식으로 만든다
	uint8_t *last = &buf[(n - 1) * IMU_FIFO_PACKET];
	for (int i = 0; i < 8; i++)
		imuData[i] = last[i];
	for (int axis = 0; axis < 3; axis++) {
		int32_t sum = 0;
		for (uint16_t k = 0; k < n; k++) {
			uint8_t *p = &buf[k * IMU_FIFO_PACKET + 8 + axis * 2];
			sum += (int16_t) (((uint16_t) p[0] << 8) | p[1]);
		}
		int16_t avg = (int16_t) (sum / (int32_t) n);
		imuData[8 + axis * 2] = (uint8_t) ((uint16_t) avg >> 8);
		imuData[9 + axis * 2] = (uint8_t) avg;
	}
parse:
#else
	spi2_ll_reads(ACCEL_XOUT_H, imuData, 14);
#endif

	accel.y = ((int16_t) imuData[0] << 8) | imuData[1];
	accel.x = ((int16_t) imuData[2] << 8) | imuData[3];
	accel.z = ((int16_t) imuData[4] << 8) | imuData[5];

	if (accel.offset_calculated) {
		accel.x -= accel.x_offset;
		accel.y -= accel.y_offset;
	}

	gyro.x = ((int16_t) imuData[8] << 8) | imuData[9];
	gyro.y = ((int16_t) imuData[10] << 8) | imuData[11];
	gyro.z = ((int16_t) imuData[12] << 8) | imuData[13];
	gyro.y *= -1;
	gyro.z *= -1;

	if (gyro.offset_calculated) {
		gyro.x -= gyro.x_offset;
		gyro.y -= gyro.y_offset;
		gyro.z -= gyro.z_offset;
	}
}

//Kalman test:
void KalmanInit(KalmanFilter *kf, float q_angle, float q_bias, float r_measure) {
    kf->q_angle = q_angle;
    kf->q_bias = q_bias;
    kf->r_measure = r_measure;
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->rate = 0.0f;
    kf->p[0][0] = 0.0f;
    kf->p[0][1] = 0.0f;
    kf->p[1][0] = 0.0f;
    kf->p[1][1] = 0.0f;
}

void KalmanUpdate(KalmanFilter *kf, float newAngle, float newRate, float dt) {
    // 예측(Prediction step)
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->p[0][0] += dt * (dt * kf->p[1][1] - kf->p[0][1] - kf->p[1][0] + kf->q_angle);
    kf->p[0][1] -= dt * kf->p[1][1];
    kf->p[1][0] -= dt * kf->p[1][1];
    kf->p[1][1] += kf->q_bias * dt;

    // 측정 업데이트(Measurement update step)
    float y = newAngle - kf->angle;
    float s = kf->p[0][0] + kf->r_measure;
    float k0 = kf->p[0][0] / s;
    float k1 = kf->p[1][0] / s;

    kf->angle += k0 * y;
    kf->bias += k1 * y;

    float p00_temp = kf->p[0][0];
    float p01_temp = kf->p[0][1];

    kf->p[0][0] -= k0 * p00_temp;
    kf->p[0][1] -= k0 * p01_temp;
    kf->p[1][0] -= k1 * p00_temp;
    kf->p[1][1] -= k1 * p01_temp;
}

RPY get_angle(Sensor type) {

	static uint8_t accel_div = 0;

	imu_read();

	//accel angle ----------------------------------------------------------------------------> 1)
	// 가속도 출력은 1kHz(DLPF 44.8Hz)라서 4루프에 1번만 계산. 모두 float 연산.
	if (accel_div == 0) {
		float ax = (float) accel.x;
		float ay = (float) accel.y;
		float az = (float) accel.z;
		float vsum = sqrtf(ax * ax + ay * ay + az * az);
		accel.vector_sum = (int32_t) vsum;
		if (vsum > 0.0f) {
			if (fabsf(ax) < vsum) {
				accelAngle.x = asinf(ax / vsum) * RAD2DEG_F;
			}
			if (fabsf(ay) < vsum) {
				accelAngle.y = asinf(ay / vsum) * RAD2DEG_F;
			}
		}
	}
	if (++accel_div >= ACCEL_ANGLE_DIV) {
		accel_div = 0;
	}

	//gyro angle -----------------------------------------------------------------------------> 2)
	// 4kHz 적분 상수(dt) 0.00025 적용 (float 상수)
	gyroAngle.x += (float) gyro.x * (GYRO_DT / DPS_F);
	gyroAngle.y += (float) gyro.y * (GYRO_DT / DPS_F);
	float z_rotated = (float) gyro.z * (GYRO_DT / DPS_F);
	gyroAngle.z += z_rotated;

	//roll & pitch correction(called yaw_compensation):
	// sin(x) ~= x (|x| < 0.0022rad @ 500dps, 4kHz) -> sinf() 호출 제거
	float yaw_rad = z_rotated * DEG2RAD_F;
	gyroAngle.x += gyroAngle.y * yaw_rad;
	gyroAngle.y -= gyroAngle.x * yaw_rad;
	gyroAngle.z = yaw_angle_limit(gyroAngle.z);//0_to_360

	//Complimentary ------------------------------------------------> 1) + 2)
	if (1) {
		// 4kHz 샘플링 비율에 맞춰 필터 가중치 조정(0.9998/0.0002 -> 0.9999/0.0001), 시정수 약 2.5s
		gyroAngle.x = gyroAngle.x * 0.9999f + accelAngle.x * 0.0001f;
		gyroAngle.y = gyroAngle.y * 0.9999f + accelAngle.y * 0.0001f;
	}

	//Kalman_test --------------------------------------------------> 1) + 2)
	if (0) {
		// 4kHz 루프에 맞춰 dt를 0.00025f로 수정
		float dt = FC_LOOP_DT;
		KalmanUpdate(&kf_roll, accelAngle.x, ((float) gyro.x / DPS_F), dt);
		KalmanUpdate(&kf_pitch, accelAngle.y, ((float) gyro.y / DPS_F), dt);
		gyroAngle.x = kf_roll.angle;
		gyroAngle.y = kf_pitch.angle;
	}

	return gyroAngle;
}

RPY get_angular(Sensor type) {
	// 1차 LPF (fc≈100Hz @4kHz). y += α(x - y)  == (1-α)y + αx
	gyroAngular.x += GYRO_LPF_ALPHA * (((float) gyro.x / DPS_F) - gyroAngular.x);
	gyroAngular.y += GYRO_LPF_ALPHA * (((float) gyro.y / DPS_F) - gyroAngular.y);
	gyroAngular.z += GYRO_LPF_ALPHA * (((float) gyro.z / DPS_F) - gyroAngular.z);
	return gyroAngular;
}

float angle_deviation(float first_angle, float second_angle) {
	float angle_difference = 0.0f;
	float AD = first_angle - second_angle;
	if (fabsf(AD) <= 180.0f)
		angle_difference = AD;
	else {
		if (AD > 0)
			angle_difference = AD - 360;
		if (AD < 0)
			angle_difference = AD + 360;
	}
	return angle_difference;
}

float yaw_angle_limit(float angle) {
	if (angle < 0) {
		angle += 360;
	} else if (angle >= 360) {
		angle -= 360;
	}
	return angle;
}
