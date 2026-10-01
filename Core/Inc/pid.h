/*
 * pid.h
 *
 * Revised: 2026-09-24 22:30 (Claude)
 *  - FF 필터 상태(filtered_ff) 추가, alt_setpoint_change 반환형 int16_t
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdio.h>
#include "main.h"

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} SETPOINT;
extern SETPOINT setpoint;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float Kd_ff;         // 피드포워드(Feedforward) 게인
	float setpoint;
	bool setpointed;
	bool setpoint_changing;
	float integral;
	float prev_error;
	float prev_measurement;    // D-kick 방지용 이전 측정값
	float prev_setpoint;       // FF용 이전 목표치(FF 입력) 저장
	float filtered_derivative; // D항 필터 상태값
	float filtered_ff;         // FF항 필터 상태값 (RC 프레임 단위 스텝 -> 스파이크 방지)
	float output;
} PID;
extern PID roll, pitch, yaw, alt;

// ---- 비행 중 게인 조정 (튜닝 모드, 2026-10-01) ----
// 1: ch5(3단 스위치)로 조정할 게인을 고른다. 1단(~1000)=P, 2단(~1500)=I, 3단(~2000)=D
//    ch9(다이얼)로 고른 게인의 배율을 50~150%(5% 단위, 가운데 100%)로 바꾼다. 롤·피치에 같이 적용(요는 그대로).
//    스위치를 바꾸면 새로 고른 게인은 다이얼이 그 게인의 현재 배율 위치를 지나갈 때부터 따라간다(게인이 갑자기 튀지 않게).
//    이 동안 ch5 비행 모드(고도 유지)는 쓰지 않는다(FM1 고정). 앱 디버그 칸에 배율 표시.
//    찾은 배율을 att_pid_init() 게인에 곱해 넣은 뒤 0 으로 되돌릴 것.
// 0: 평소 비행 (ch5 = 비행 모드)
#define PID_TUNE_MODE 1

typedef struct {
	uint8_t sel;    // 1=P, 2=I, 3=D (ch5)
	uint8_t p_pct;  // 롤·피치 Kp 배율 [%]
	uint8_t i_pct;  // 롤·피치 Ki 배율 [%]
	uint8_t d_pct;  // 롤·피치 Kd 배율 [%]
	bool caught;    // 다이얼이 고른 게인의 현재 배율을 잡았음(이후 다이얼을 따라감)
} PID_TUNE;
extern PID_TUNE pid_tune;
void pid_tune_update(void); // main 루프에서 매번 호출 (PID_TUNE_MODE 0 이면 아무것도 안 함)

// 제어 주기(dt) 인자 포함
float att_PID(Which choice, PID *pid, float angle, float angular_velocity, uint16_t sp, float dt);
float restrict_max(float value, float pid_max);
void gyro_pid_reset();
void pid_gain_init(uint8_t type, float kp, float ki, float kd);
void att_pid_init(void);

float alt_PID(Which type, PID *pid, float altitude);      // altitude = baro.altitude [m]
int16_t alt_setpoint_change(PID *pid, uint16_t alt_hold_throt, float altitude);
extern uint16_t alt_throttle_out; // 고도 유지 최종 스로틀 [us] (텔레메트리 튜닝용)

#endif /* INC_PID_H_ */
