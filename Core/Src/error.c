/*
 * error.c
 *
 *  Created on: Mar 08, 2024
 *      Author: JeeGS
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 *   - WHO_AM_I 는 1초마다 1회만, 배터리 ADC 는 50Hz 논블로킹(EOC 폴링)
 *   - BARO 에러 판정 복구, RC 링크 두절 시 error.RC
 *
 *  Revised: 2026-09-25 10:50 (Claude)
 *   - 진단용으로 임시 비활성화했던 rc_link_lost() 체크를 재활성화함
 */

#include "main.h"
#include "imu.h"
#include "pid.h"
#include "error.h"
#include "rc.h"
#include "baro.h"

// 주기 설정 (메인루프 4kHz 기준)
#define WHOAMI_CHECK_LOOPS 4000   // 1s 마다 IMU WHO_AM_I 확인
#define BAT_SAMPLE_LOOPS   80     // 50Hz 로 배터리 ADC 변환
#define BAT_TIMEOUT_LOOPS  40     // 변환 시작 후 10ms 내 EOC 없으면 포기
#define BAT_FILTER_KEEP    0.92f  // 50Hz 에서 시정수 약 0.25s (기존 4kHz 0.99 와 동일)

//Locallized:
//float battery_voltage;

bool failSafe;

bool err_checker(Sensor name) {

	if (name == icm20602) { //1)
		//WHOAMI test: 매 루프(4kHz) 읽지 않고 1초마다 1회만 확인, 결과는 캐시
		static uint16_t whoami_div = 0;
		static bool whoami_err = false;
		if (whoami_div == 0) {
			whoami_err = (spi2_ll_read(0x75) != 0x12);
		}
		if (++whoami_div >= WHOAMI_CHECK_LOOPS) {
			whoami_div = 0;
		}
		if (whoami_err) {
			return true;
		}
	}

	if (name == bmp390L) { //2)
		// 칩 확인 + 워밍업이 끝나지 않았으면(또는 baro 비활성) 에러 -> FM2 이상 진입 불가
		return !baro.ready;
	}

	if (name == FS_iA6B) { //5)
		// 2026-09-25 11:20: 재활성화(진짜 원인 수정 완료, rc.c 참고).
		// get_rc()가 rc_ever_valid/rc_last_valid_ms 를 갱신하지 않던 버그 때문에
		// rc_link_lost()가 항상 true 를 반환하고 있었음 -> 지금까지는 이 체크를
		// 살리면 error.RC 가 상시 true 가 되어 아밍 자체가 막혔던 것.
		// rc.c 수정으로 원인 해결되었으므로 정상적으로 되살림.
		if (rc_link_lost()) { // 수신기 프레임이 일정 시간 없음
			return true;
		}

		if (ARMED == 0) { //Before start, RC sticks check!!!
			if (rc.ch1 > 1502 || rc.ch1 < 1498)
				return true;
			else if (rc.ch2 > 1502 || rc.ch2 < 1498)
				return true;
			else if (rc.ch3 > 1010 || rc.ch3 < 990) { // 1010!!!
				return true;
			}
			else if (rc.ch5 > 1002)
				return true;
			else if (rc.ch6 > 1002)
				return true;
			else if (rc.ch7 > 1002)
				return true;
			else if (rc.ch8 > 1002)
				return true;
			else if (rc.ch9 > 1100)
				return true;
			else if (rc.ch10 > 1100) {
				//return true;//Not used for debugging: (   )
			}
		}
	}

	if (name == BATTERY) { //6)
		// 논블로킹 ADC: 50Hz 로 변환 시작 -> EOC 가 뜨면 결과 회수 (HAL_MAX_DELAY 폴링 제거)
		static uint16_t bat_div = 0;
		static uint16_t bat_wait = 0;
		static bool bat_busy = false;

		if (!bat_busy) {
			if (++bat_div >= BAT_SAMPLE_LOOPS) {
				bat_div = 0;
				if (HAL_ADC_Start(&hadc1) == HAL_OK) {
					bat_busy = true;
					bat_wait = 0;
				}
			}
		} else if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
			//(1kk + 200k) / 200k = 6.0 ***bias 3%
			float v = (float) HAL_ADC_GetValue(&hadc1) * BAT_ADC_TO_VOLT;
			battery_voltage = battery_voltage * BAT_FILTER_KEEP
					+ v * (1.0f - BAT_FILTER_KEEP);
			bat_busy = false;
		} else if (++bat_wait > BAT_TIMEOUT_LOOPS) {
			HAL_ADC_Stop(&hadc1); // 변환 실패 -> 중단하고 다음 주기에 재시도
			bat_busy = false;
		}

		if (batteryType == _3S) { //(Min)2.8V * 3s = 8.4V, (Max)4.2V * 3s = 12.6V
			if (battery_voltage < 10.5) {
				LL_GPIO_SetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);
				LL_GPIO_SetOutputPin(LED_Green_GPIO_Port, LED_Green_Pin);
				return true;
			} else {
				LL_GPIO_ResetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);
				LL_GPIO_ResetOutputPin(LED_Green_GPIO_Port, LED_Green_Pin);
			}
		}
		if (batteryType == _4S) { //(Min)2.8V * 4s = 11.2V, (Max)4.2V * 4s = 16.8V
			if (battery_voltage < 13.5) {
				LL_GPIO_SetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);
				return true;
			} else {
				LL_GPIO_ResetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);
			}
		}
	}

	return false;
}

void make_8bit_with_errors(void) {

	bool bit0 = error.GYRO;
	bool bit1 = error.BARO;
	bool bit2 = error.COMPSS;
	bool bit3 = error.GPS;
	bool bit4 = error.RC; //
	bool bit5 = error.BATTERY;
	bool bit6 = 0;
	bool bit7 = 0;

	uint8_t num = (bit7 << 7) | (bit6 << 6) | (bit5 << 5) | (bit4 << 4)
			| (bit3 << 3) | (bit2 << 2) | (bit1 << 1) | bit0;

	//printf("8비트 숫자: %u\n", num);

	for (int i = 7; i >= 0; i--) {
		bool bit = (num >> i) & 0x01;
		printf("%d", bit); //01111011
	}
	printf(" %d", ARMED);
	printf("\n");
}

void error_handling(void) {

	//Sensors:
	error.GYRO = err_checker(icm20602); //--------------> 1)
	if (error.GYRO) {
		//Reserved another gyro_sensor activated:
	}

	// baro 미사용/미준비이면 error.BARO = true -> flightmode() 가 FM2 이상을 허용하지 않음
	error.BARO = err_checker(bmp390L); //----------------> 2)

//	error.COMPSS = err_checker(SE100_HMC5983); //-------> 3)
//
//	error.GPS = err_checker(SE100_M8N); //----------=---> 4)
//	if (error.GPS) {
//		if (FM >= 3) {
//			FM = 2; //
//		}
//	}

	//RC:
	error.RC = err_checker(FS_iA6B); //-----------------> 5)
	if (error.RC) {
		if (1/*faileSafe*/) { //------------------------------------------------ING:
//			if (FM >= 3) {
//				throttle = alt_holding_throttle;
//				FM = 4; //RTH
//			}
		}
	}

	//Battery:
	error.BATTERY = err_checker(BATTERY); ////----------> 6)
	if (error.BATTERY) { // --------------------------------------------- ING:
		//FM = 4; //RTH
	}

	//-----------------------> ING:

//	if(error.RC) error = 1;
//	else if(error.BATTERY) error = 2;
}

