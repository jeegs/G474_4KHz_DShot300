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
extern uint32_t rc_frames_rejected; // 채널값이 범위 밖이라 버린 프레임 수 (0 이어야 정상)
extern uint32_t rc_spike_count;     // 한 프레임 튐으로 보류한 횟수

RC get_rc(Sensor type);
bool rc_link_lost(void);   // 유효 프레임이 RC_TIMEOUT_MS 이상 없으면 true
RC rc_failsafe(RC r);      // 아밍 중 링크 두절 시: 스틱 중립 + 스로틀 서서히 감소 후 디스암

// 페일세이프 상태 (Live Expressions 에서 rc_fs 로 확인)
typedef struct {
	bool active;     // 페일세이프 동작 중 (이 동안 flightmode 는 FM1 로 고정 = 고도 유지 끔)
	bool rising;     // true: +50 올리는 구간, false: 내리는 구간
	bool lost;       // 링크 두절 확정 (디바운스 통과)
	bool pattern;    // 이번 루프 채널값이 페일세이프 조합과 일치
	bool airborne;   // 아밍 후 스로틀이 1200 을 넘은 적 있음
	float throttle;  // 페일세이프가 만드는 스로틀 [us]
	uint32_t start_count; // 페일세이프가 새로 시작된 횟수 (한 번 끌 때 1 씩만 늘어야 정상)
} RC_FS_STATE;
extern RC_FS_STATE rc_fs;
uint8_t arming(RC rc);
uint8_t flightmode(RC rc);

#endif /* INC_RC_H_ */
