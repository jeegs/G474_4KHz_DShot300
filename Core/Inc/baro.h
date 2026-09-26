/*
 * baro.h
 *
 *  Created on: Mar 08, 2024
 *      Author: JeeGS
 *
 *  Revised: 2026-09-24 22:30 (Claude)
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

extern float brake_throttle; // 압력 변화율 [Pa/s] (0.2s 창) - 고도 PID 의 D 입력

void baro_init(Sensor type);
float get_pressure(Sensor type);

#endif /* INC_BARO_H_ */
