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

// ---- 고도 PID 상수 (기압 갱신 50Hz) ----
#define ALT_I_TERM_MAX     100.0f
#define ALT_D_FILTER_ALPHA 0.3f    // 50Hz 기준 fc ≈ 2.8Hz
#define PA_PER_HPA         100.0f

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
	pid_init(&alt, 1.5f, 0.5f, 4.0f, 0.0f);
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
 *  - 오차/미분 단위: Pa, Pa/s  (1Pa ≈ 8.3cm).  게인은 재튜닝 필요.
 *  - pressure 인자는 hPa
 */
int16_t manual_Throttle;
uint16_t alt_holding_throttle;
float alt_PID(Which type, PID *pid, float pressure) {

	static uint32_t last_sample = 0;

	if (pid->setpointed == false) {
		pid->setpointed = true;
		pid->setpoint = pressure;
		alt_holding_throttle = rc.ch3;
		alt_pid_init();
		last_sample = baro.sample_count;
	}

	manual_Throttle = alt_setpoint_change(pid, alt_holding_throttle, pressure);

	if (baro.sample_count != last_sample) {
		last_sample = baro.sample_count;

		float error = (pressure - pid->setpoint) * PA_PER_HPA; // [Pa]

		pid->integral += error * BARO_DT;

		if (pid->Ki > 0.0000001f) {
			float i_limit = ALT_I_TERM_MAX / pid->Ki;
			pid->integral = restrict_max(pid->integral, i_limit);
		}

		float rate = (error - pid->prev_error) / BARO_DT; // [Pa/s]
		float derivative;
		if (rc.ch10 > 1800) {
			derivative = brake_throttle;
		} else if (rc.ch10 > 1100) {
			derivative = rate;
		} else {
			pid->filtered_derivative += ALT_D_FILTER_ALPHA
					* (rate - pid->filtered_derivative);
			derivative = pid->filtered_derivative;
		}

		pid->prev_error = error;

		float out = (pid->Kp * error) + (pid->Ki * pid->integral)
				+ (pid->Kd * derivative);

		pid->output = restrict_max(out, 300); // PID 보정분만 저장
	}

	return alt_holding_throttle + pid->output + manual_Throttle;
}

int16_t throttle_changed;
int16_t alt_setpoint_change(PID *pid, uint16_t alt_hold_throt, float pressure) {
	// 주의: 음수(하강) 값이 나오므로 반드시 부호 있는 형 사용 (uint16_t 는 65000대로 wrap 됨)
	if (rc.ch3 > (alt_hold_throt + 100)) {
		throttle_changed = (int16_t) ((rc.ch3 - (alt_hold_throt + 100)) / 5);
		pid->setpoint = pressure;
		pid->setpoint_changing = true;
	} else if (rc.ch3 < (alt_hold_throt - 100)) {
		throttle_changed = (int16_t) ((rc.ch3 - (alt_hold_throt - 100)) / 5);
		pid->setpoint = pressure;
		pid->setpoint_changing = true;
	} else {
		throttle_changed = 0;
		pid->setpoint_changing = false;
	}
	return throttle_changed;
}
