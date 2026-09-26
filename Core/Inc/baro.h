/*
 * baro.h
 *
 *  Created on: Mar 08, 2024
 *      Author: JeeGS
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 *  Revised: 2026-09-26 (Claude) - 동작 확인용 진단값(baro_diag) 추가
 */

#ifndef INC_BARO_H_
#define INC_BARO_H_

#include "main.h"

#define BARO_DT 0.02f // BMP390L ODR 50Hz -> 샘플 간격 [s]

typedef struct {
	float short_pressure;    // 최신 압력 [hPa]
	float long_pressure;     // (호환용) short_pressure 와 동일
	float press_compensated; // 고도 제어에 쓰는 압력 [hPa]
	bool ready;              // 칩 확인 + 워밍업 완료 시 true
	uint32_t sample_count;   // 새 샘플이 들어올 때마다 +1 (50Hz)
} BARO;
extern BARO baro;

// 동작 확인용 진단값. CubeIDE Live Expressions 에서 baro_diag 를 보거나
// main.c 의 BARO_DEBUG_PRINT 를 1 로 해서 USART2 로 확인한다.
typedef struct {
	uint8_t chip_id;       // 0x60 이어야 정상. 0x00/0xFF = 배선(CS/MISO/전원) 문제
	uint8_t err_reg;       // ERR_REG(0x02). 0 이어야 정상 (bit0 fatal, bit1 cmd, bit2 conf)
	uint8_t pwr_ctrl;      // PWR_CTRL(0x1B) 되읽기. 0x33 이어야 정상 (압력+온도, Normal 모드)
	uint8_t osr, odr, config; // 되읽기. 0x03 / 0x02 / 0x04 이어야 정상
	int32_t press_pa_x100; // 최신 압력 [Pa x100]  (해수면 근처 약 10,132,500)
	int32_t temp_c_x100;   // 최신 칩 온도 [degC x100] (실온 + 수 도)
	uint32_t reject_count; // 범위를 벗어나 버린 샘플 수 (계속 늘면 통신 불량)
	uint32_t stale_count;  // 새 샘플이 BARO_STALE_LOOPS 동안 안 들어와 ready 를 내린 횟수
} BARO_DIAG;
extern BARO_DIAG baro_diag;

extern float brake_throttle; // 압력 변화율 [Pa/s] (0.2s 창) - 고도 PID 의 D 입력

void baro_init(Sensor type);
float get_pressure(Sensor type);

#endif /* INC_BARO_H_ */
