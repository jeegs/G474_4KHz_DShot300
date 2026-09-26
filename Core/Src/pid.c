/*
 * pid.c
 *
 * Revised: 2026-09-24 22:30 (Claude)
 *  - D 필터 계수 재산정 (4kHz 기준 fc≈60Hz), 자이로 LPF 는 imu.c 에서 fc≈100Hz
 *  - FF 항: RC 프레임 단위 스텝을 1차 LPF 로 평활 (한 샘플 스파이크 제거)
 *  - I항 권한 제한 (PID_I_TERM_MAX)
 *  - alt_PID: 압력 갱신(50Hz)마다만 dt 적용해 계산, 음수 wrap 버그 수정, 단위 Pa
 *
 * Revised: 2026-09-26 (Claude)
 *  - 스로틀이 낮을 때(ATT_I_RESET_THROTTLE 미만: 바닥 대기/이륙 직전) 자세 I항을 0 으로 유지.
 *    모터 아이들이 생겨 아밍 중에도 PID 가 돌기 때문에, 바닥에서 기울어진 채 I항이 쌓였다가
 *    이륙 순간 한쪽으로 튀는 것을 막는다.
 *  - alt_PID 를 고도[m] 기준으로 변경 (baro.altitude). 기존 Pa 게인을 1m = 12Pa 로 환산해 유지
 *    (Kp 1.5 -> 18, Ki 0.5 -> 6, Kd 4.0 -> 48). 스로틀 1200 미만에서는 고도 유지에 들어가지 않음.
 *    D 입력 기본값을 0.2s 창 상승률(잡음이 작음)로 변경, ch10 > 1100 이면 기존 샘플 단위 필터 방식.
 */

#include "pid.h"
#include "rc.h"
#include "baro.h"

PID roll, pitch, yaw, alt;

// ---- 자세 PID 필터/제한 상수 (제어 루프 4kHz, dt=0.00025s) ----
// 1차 LPF: α = 1 - exp(-2π·fc·dt)
//   fc=100Hz -> 0.145 (imu.c GYRO_LPF_ALPHA),  fc=60Hz -> 0.09,  fc=13Hz -> 0.02
#define D_FILTER_ALPHA   0.09f   // D항 LPF (fc ≈ 60Hz)  [기존 0.02 = 13Hz]
#define FF_FILTER_ALPHA  0.02f   // FF항 LPF (fc ≈ 13Hz)
#define PID_I_TERM_MAX   150.0f  // I항 최대 출력 권한 (기존 400 = 출력 전체)
#define ATT_I_RESET_THROTTLE 1100 // 스로틀(ch3)이 이 값 미만이면 자세 I항 누적 안 함 (바닥 대기)

// FF 를 각도 피드백 성분(-angle*17/2.5)까지 미분할지 여부.
//  0: 스틱 성분만 미분(표준).  1: 기존 방식(사실상 원시 자이로 비례항이 추가됨)
#define FF_ON_ANGLE_TERM 0

// ---- 고도 PID 상수 (기압 갱신 50Hz, 단위 m) ----
// 게인 단위: Kp [us/m], Ki [us/(m*s)], Kd [us/(m/s)]  (us = 스로틀 1000~2000 스케일)
#define ALT_KP             18.0f
#define ALT_KI              6.0f
#define ALT_KD             48.0f
#define ALT_I_TERM_MAX     100.0f  // I항 최대 [us]
#define ALT_OUT_MAX        300.0f  // PID 보정 최대 [us]
#define ALT_D_FILTER_ALPHA 0.3f    // 50Hz 기준 fc ≈ 2.8Hz (ch10 > 1100 일 때만 사용)
#define ALT_ENGAGE_MIN_THROTTLE 1200 // 이 스로틀 미만(바닥)에서는 고도 유지에 들어가지 않는다
#define PA_PER_HPA         100.0f
#define AIR_SCALE_HEIGHT_M 8434.0f // 압력[Pa] / 이 값 = 1m 당 압력 차[Pa] (약 12Pa/m)

void pid_init(PID *pid, float Kp, float Ki, float Kd, float Kd_ff) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Kd_ff = Kd_ff;
	pid->prev_error = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->prev_setpoint = 0.0f;
	pid->integral = 0.0f;
	pid->filtered_derivative = 0.0f;
	pid->filtered_ff = 0.0f;
	pid->output = 0.0f;
}

void att_pid_init(void) {
	// dt 정규화가 적용된 표준 게인 설정 (FF 게인 0.02 추가)
	//pid_init(&roll, 1.5f, 5.0f, 0.025f, 0.02f);
	pid_init(&roll, 1.1f, 5.0f, 0.012f, 0.02f);

	//pid_init(&pitch, 1.5f, 5.0f, 0.025f, 0.02f);
	pid_init(&pitch, 1.1f, 5.0f, 0.012f, 0.02f);

	pid_init(&yaw, 4.0f, 20.0f, 0.0f, 0.0f);
}

void alt_pid_init(void) {
	pid_init(&alt, ALT_KP, ALT_KI, ALT_KD, 0.0f);
}

float restrict_max(float value, float pid_max) {
	if (value > pid_max)
		value = pid_max;
	else if (value < pid_max * -1)
		value = pid_max * -1;
	return value;
}

float real_setpoint(uint16_t sp) {
	float rs = 0;
	if (sp > 1503) {
		rs = sp - 1503;
	} else if (sp < 1497 && sp > 0) {
		rs = sp - 1497;
	}
	return rs;
}

float att_PID(Which choice, PID *pid, float angle, float angular_velocity, uint16_t sp, float dt) {

	// 바닥 대기/저스로틀: I항만 비운다 (P/D 는 계속 동작해 자세를 잡는다)
	if (rc.ch3 < ATT_I_RESET_THROTTLE) {
		pid->integral = 0.0f;
	}

	if (ARMED != 2) {
		pid->integral = 0.0f;
		pid->prev_error = 0.0f;
		pid->prev_measurement = angular_velocity;
		pid->prev_setpoint = 0.0f;
		pid->filtered_derivative = 0.0f;
		pid->filtered_ff = 0.0f;
	}

	// 직렬(Cascaded) 제어 구조 (Angle -> Rate 변환)
	float stick_sp = 0.0f;      // 스틱이 만드는 목표 각속도 [dps]
	float rate_setpoint = 0.0f; // 최종 목표 각속도 [dps]
	if (choice == YAW) {
		if (rc.ch3 > 1150) {
			stick_sp = real_setpoint(sp) / 2.5f;
		}
		rate_setpoint = stick_sp;
	} else {
		stick_sp = real_setpoint(sp) / 2.5f;
		rate_setpoint = (real_setpoint(sp) - (angle * 17.0f)) / 2.5f;
	}

	pid->setpoint = rate_setpoint;

	// 1. 오차 계산
	float error = pid->setpoint - angular_velocity;

	// 2. 동적 적분 와인드업 방지 (출력 포화 시 I항 누적 일시 정지)
	bool is_saturated = ((pid->output >= 400.0f && error > 0) || (pid->output <= -400.0f && error < 0));
	if (!is_saturated) {
		pid->integral += error * dt;
	}

	// I항 권한 제한: Ki*integral <= PID_I_TERM_MAX
	if (pid->Ki > 0.0000001f) {
		float i_limit = PID_I_TERM_MAX / pid->Ki;
		pid->integral = restrict_max(pid->integral, i_limit);
	}

	// 3. D항 계산 (측정값 기반 + dt 정규화)
	float raw_derivative = -(angular_velocity - pid->prev_measurement) / dt;
	pid->prev_measurement = angular_velocity;

	pid->filtered_derivative += D_FILTER_ALPHA
			* (raw_derivative - pid->filtered_derivative);

	// 4. 피드포워드(FF) 항: 스틱 변화율 기반.
	//    RC 값은 ~7ms 프레임마다 계단식으로 바뀌므로, 매 루프 미분값(스텝/dt)을 그대로 쓰면
	//    한 샘플짜리 큰 스파이크가 143Hz로 나간다 -> 1차 LPF 로 평활(면적은 보존).
	float ff_src = FF_ON_ANGLE_TERM ? rate_setpoint : stick_sp;
	float ff_raw = (ff_src - pid->prev_setpoint) / dt;
	pid->prev_setpoint = ff_src;
	pid->filtered_ff += FF_FILTER_ALPHA * (ff_raw - pid->filtered_ff);
	float ff_term = pid->Kd_ff * pid->filtered_ff;

	pid->prev_error = error;

	// 최종 PID + FF 출력 병합
	pid->output = (pid->Kp * error) +
	              (pid->Ki * pid->integral) +
	              (pid->Kd * pid->filtered_derivative) +
	              ff_term;

	return restrict_max(pid->output, 400);
}

/*
 * ALTitude:
 *  - 기압 샘플은 50Hz(baro.sample_count 증가) 로만 갱신되므로 PID 도 그 시점에만 계산한다.
 *  - altitude 인자는 baro.altitude [m] (아밍 지점 기준 상대 고도)
 *  - 오차 = 목표 고도 - 현재 고도 [m] (낮으면 +, 스로틀을 올린다)
 *  - 들어가는 순간의 스로틀(rc.ch3)을 호버 스로틀로 잡는다 -> 안정적으로 호버링 중에 전환할 것
 */
int16_t manual_Throttle;
uint16_t alt_holding_throttle;
uint16_t alt_throttle_out; // 고도 유지가 낸 최종 스로틀 [us] (텔레메트리 etc1, 튜닝용)

float alt_PID(Which type, PID *pid, float altitude) {
	static uint32_t last_sample = 0;

	if (pid->setpointed == false) {
		// 바닥/저스로틀에서 전환하면 들어가지 않고 수동 스로틀 그대로 (스틱을 올리면 그때 진입)
		if (rc.ch3 < ALT_ENGAGE_MIN_THROTTLE) {
			alt_throttle_out = rc.ch3;
			return rc.ch3;
		}
		alt_pid_init();
		pid->setpointed = true;
		pid->setpoint = altitude;
		alt_holding_throttle = rc.ch3;
		last_sample = baro.sample_count;
	}

	manual_Throttle = alt_setpoint_change(pid, alt_holding_throttle, altitude);

	if (baro.sample_count != last_sample) {
		last_sample = baro.sample_count;

		float error = pid->setpoint - altitude; // [m]

		pid->integral += error * BARO_DT;
		if (pid->Ki > 0.0000001f) {
			float i_limit = ALT_I_TERM_MAX / pid->Ki;
			pid->integral = restrict_max(pid->integral, i_limit);
		}

		// D 입력 = 오차 변화율 [m/s] = -상승률
		float derivative;
		if (rc.ch10 > 1100) {
			// (기존 방식) 샘플 단위 변화율 + 1차 LPF. 잡음이 큼
			float rate = (error - pid->prev_error) / BARO_DT;
			pid->filtered_derivative += ALT_D_FILTER_ALPHA
					* (rate - pid->filtered_derivative);
			derivative = pid->filtered_derivative;
		} else {
			// (기본) 0.2s 창 압력 변화율 [Pa/s] -> [m/s]. 압력이 오르면 하강 중 -> 오차 증가 방향
			float pa_per_m = (baro.short_pressure * PA_PER_HPA) / AIR_SCALE_HEIGHT_M;
			derivative = (pa_per_m > 1.0f) ? (brake_throttle / pa_per_m) : 0.0f;
		}
		pid->prev_error = error;

		float out = (pid->Kp * error) + (pid->Ki * pid->integral)
				+ (pid->Kd * derivative);
		pid->output = restrict_max(out, ALT_OUT_MAX); // PID 보정분만 저장
	}

	float t = alt_holding_throttle + pid->output + manual_Throttle;
	alt_throttle_out = (t < 1000.0f) ? 1000 : (t > 2000.0f) ? 2000 : (uint16_t) t;
	return t;
}

int16_t throttle_changed;
int16_t alt_setpoint_change(PID *pid, uint16_t alt_hold_throt, float altitude) {
	// 주의: 음수(하강) 값이 나오므로 반드시 부호 있는 형 사용 (uint16_t 는 65000대로 wrap 됨)
	if (rc.ch3 > (alt_hold_throt + 100)) {
		throttle_changed = (int16_t) ((rc.ch3 - (alt_hold_throt + 100)) / 5);
		pid->setpoint = altitude; // 스틱으로 오르내리는 동안은 목표가 현재 고도를 따라간다
		pid->setpoint_changing = true;
	} else if (rc.ch3 < (alt_hold_throt - 100)) {
		throttle_changed = (int16_t) ((rc.ch3 - (alt_hold_throt - 100)) / 5);
		pid->setpoint = altitude; // 스틱으로 오르내리는 동안은 목표가 현재 고도를 따라간다
		pid->setpoint_changing = true;
	} else {
		throttle_changed = 0;
		pid->setpoint_changing = false;
	}
	return throttle_changed;
}
