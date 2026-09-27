/*
 * rc.h
 *
 *  Created on: May 15, 2024
 *      Author: gyoos
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 */

#ifndef INC_RC_H_
#define INC_RC_H_

#include "main.h"

typedef struct {
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint16_t ch5;
	uint16_t ch6;
	uint16_t ch7;
	uint16_t ch8;
	uint16_t ch9;
	uint16_t ch10;
} RC;

extern RC rc;

extern uint32_t rc_frames_total; // 진단용(임시): iBus 프레임 시도 횟수
extern uint32_t rc_frames_valid; // 진단용(임시): 체크섬 통과(유효) 횟수

RC get_rc(Sensor type);
bool rc_link_lost(void);   // 유효 프레임이 RC_TIMEOUT_MS 이상 없으면 true
RC rc_failsafe(RC r);      // 아밍 중 링크 두절 시: 스틱 중립 + 스로틀 서서히 감소 후 디스암
uint8_t arming(RC rc);
uint8_t flightmode(RC rc);

#endif /* INC_RC_H_ */
