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
