/*
 * rc.c
 *
 *  Re_created on: Mar 08, 2024
 *      Author: GyooSuhn, JEE
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 *   - ISR 공유 변수 volatile, RC 링크 타임아웃/페일세이프 추가
 *
 *  Revised: 2026-09-25 (Claude)
 *   - 아밍 조건의 !error.BATTERY 제거(원복): 저전압 오판정 하나로 아밍이
 *     영구히 막히는 실패 모드였음. 저전압 경고는 LED/텔레메트리로만 표시.
 *   - 진단용(임시): rc_frames_total/rc_frames_valid 카운터 추가.
 *     -> "아밍이 여전히 안 됨" 원인이 iBus 체크섬이 한 번도 통과하지 못해
 *        channel(=rc)이 0으로 고정된 것인지, 아니면 다른 조건인지 구분용.
 *        (main.c 에서 printf 로 값 확인. 원인 확정되면 이 계측은 제거할 것)
 *
 *  Revised: 2026-09-25 10:50 (Claude)
 *   - !error.BATTERY 조건이 기기에서 다시 발견되어 재제거함(이전 커밋이
 *     실제로는 기기에 반영되지 않았던 것으로 확인됨). arming() 참고.
 *
 *  Revised: 2026-09-25 11:20 (Claude) *** 진짜 근본 원인 ***
 *   - get_rc()에서 체크섬이 통과해도 rc_ever_valid/rc_last_valid_ms 를
 *     한 번도 갱신하지 않던 버그 발견 및 수정. 이 때문에 rc_link_lost()가
 *     "!rc_ever_valid -> return true" 로 인해 프레임이 아무리 잘 들어와도
 *     항상 true(링크 두절)를 반환했음.
 *     -> error.RC 가 항상 true 로 고정되어 arming()의 !error.RC 조건을 막았고,
 *     -> rc_failsafe()도 ARMED==2 가 되는 즉시 "링크 두절"로 오판, 스틱을
 *        강제 중립화하고 스로틀을 감쇠시켜 armed=0 으로 되돌리고 있었음.
 *     (아밍 시도 시 armed=1까지는 가는데 ch4가 중앙(1450 이상)으로 돌아오는
 *      순간 armed=2가 되자마자 바로 0으로 풀리던 현상의 진짜 원인.)
 *     이 수정으로 rc_link_lost()/rc_failsafe()의 !return true 임시 주석
 *     처리를 모두 되돌려 원래 안전 기능으로 복구함.
 *
 *  Revised: 2026-09-25 12:10 (Claude)
 *   - "송신기 전원만 껐을 때 자동 강하 안 됨" 원인 수정.
 *     FS-iA6B(FlySky iBus) 수신기는 RF 링크가 끊겨도 전원이 켜져 있는 한
 *     iBus 시리얼 프레임을 계속 정상 체크섬으로 송출함(마지막 값 유지 또는
 *     설정된 페일세이프 값을 계속 반복 전송). 즉 iBus 프로토콜 자체에는
 *     SBUS 같은 "failsafe" 비트가 없어서, 프레임 도착 여부(rc_link_lost의
 *     타임아웃 로직)만으로는 이 상황을 감지할 수 없음.
 *
 *  Revised: 2026-09-25 12:40 (Claude)
 *   - 위 채널값 판정 방식 변경. FS-i6는 Failsafe 값을 "스틱/스위치 현재
 *     위치 캡처" 방식이라 -100%(=PWM 1000, 스로틀 물리적 최저)보다 더
 *     낮은 값을 만들 수 없음이 확인됨 -> 스로틀 채널 하나만으로는 정상
 *     상태(아이들)와 구분 불가.
 *     -> "롤/피치/요 중앙 + 스로틀 최저"가 동시에, 그리고 일정 시간
 *        (RC_FS_MATCH_HOLD_MS) 이상 계속 유지될 때만 링크 두절로 판단하는
 *        방식으로 변경(rc_matches_failsafe_pattern() 참고). 조종 중 이
 *        조합을 잠깐 스치는 것과, 송신기가 꺼져서 이 값이 고정 출력되는
 *        것을 시간으로 구분.
 *        *** 송신기 Failsafe: ch1=0%, ch2=0%, ch3=-100%, ch4=0% 로 캡처해둘 것 ***
 *
 *  Revised: 2026-09-25 13:10 (Claude)
 *   - "서서히 감소가 아니라 즉시 꺼짐" 문제 수정. rc_failsafe()가 감지 시점의
 *     r.ch3 에서부터 감소시켰는데, 위 채널값 판정 특성상 그 시점엔 이미
 *     ch3 가 페일세이프 최저값(1000)으로 고정돼 있어서 내려갈 구간이 없어
 *     즉시 디스암 조건에 걸렸던 것.
 *     -> 고정 상수 대신, 페일세이프 패턴이 아닐 때(정상 조종 중)의 마지막
 *        스로틀을 계속 기억해두는 last_good_throttle 로 변경(사용자 제안).
 *        페일세이프 확정 시 이 값 + RC_FS_RECOVERY_BOOST(50, 하강 중 자세
 *        유지용 여유)에서부터 감소시킴.
 *
 *  Revised: 2026-09-25 13:25 (Claude)
 *   - +50 올리는 것도 즉시 계단식이 아니라 "서서히" 올라가야 한다는 지적
 *     반영. rc_failsafe()에 상승 구간(fs_rising, RC_FS_RISE_US_PER_SEC)을
 *     추가: last_good_throttle 에서 정점(+50)까지 서서히 올린 뒤, 정점에서
 *     다시 서서히 감소(RC_FS_DESCENT_US_PER_SEC)시켜 디스암.
 *
 *  Revised: 2026-09-25 13:45 (Claude)
 *   - "페일세이프 시 모터가 0.5초 정도 일시적으로 죽었다가 다시 올라옴" 수정.
 *     원인: 디바운스(RC_FS_MATCH_HOLD_MS=400ms) 확정 전에는 rc_failsafe()가
 *     조기 리턴하며 원본 r을 그대로 반환했는데, 그 시점 r.ch3 는 이미 수신기가
 *     즉시 내보내는 페일세이프 최저값(1000)이었음 -> 디바운스가 끝날 때까지
 *     그 값이 그대로 모터로 흘러가 그 사이 모터가 죽었던 것.
 *     -> 채널값이 페일세이프 패턴과 "일치하는 순간부터"(디바운스 확정 여부와
 *        무관하게) 즉시 보호(상승->하강) 로직을 가동하도록 변경. 디스암 자체는
 *        여전히 "확정된" 링크 두절(lost)일 때만 실행되어, 우연히 패턴을 스친
 *        경우엔 그대로 정상 조종으로 복귀함.
 *
 *  Revised: 2026-09-25 22:10 (Claude)
 *   - arming(): 비행 중(armed==2) 스로틀 최저 + 요 좌측 끝 조합이 들어오면
 *     armed=1 로 되돌아가 모터가 멈추고, 자세각(gyroAngle)과 PID 가 초기화되던
 *     문제 수정. 이제 armed!=2 일 때만 아밍 대기(armed=1)로 진입한다.
 *   - 루프 주기 4000.0f 하드코딩 -> FC_LOOP_HZ (main.h)
 *
 *  Revised: 2026-09-25 21:29 (Claude)
 *   - 코드 변경 없음. 안전상 중요한 제약사항을 문서화만 해둠(사용자 확인/결정).
 *   *** 이 RC 페일세이프는 "고도를 전혀 모르는" 시간 기반 스로틀 램프임 ***
 *     현재 빌드는 BARO_ENABLED=0 이라 alt_PID() 등 고도 피드백이 전혀
 *     동작하지 않음. rc_failsafe()는 실제 잔여 고도와 무관하게 정해진
 *     속도(RC_FS_RISE/DESCENT_US_PER_SEC)로만 오르내리다가 fs_throttle이
 *     1050 이하가 되면 무조건 armed=0(완전 디스암, 모터 컷)으로 끝남.
 *     -> 즉 100m 같은 높은 고도에서 페일세이프가 걸리면, 지금 속도로도
 *        바닥에 닿기 훨씬 전에 모터가 완전히 꺼져 추락함. 속도 상수를
 *        아무리 튜닝해도(빠르게/느리게) 이 구조적 한계 자체는 해결 안 됨.
 *        진짜 해결하려면 (a) BARO 활성화 후 실제 고도 기반 하강 제어,
 *        또는 (b) 완전 디스암 대신 항상 낮은 완속 하강 스로틀을 유지하는
 *        방식으로 재설계해야 함.
 *     -> 사용자 결정(2026-09-25): 지금 구조를 그대로 유지하고, 이 드론/
 *        페일세이프는 저고도(실내/마당 등) 테스트 용도로만 사용하기로 함.
 *        고고도 비행 계획이 생기면 이 부분을 반드시 먼저 재설계할 것.
 */

#include "rc.h"
#include "pid.h"
#include "imu.h"
#include "error.h"

//Localized:
volatile bool uart4_rx_idle;
volatile uint8_t uart4_rx_data[32];
volatile uint8_t uart4_rx_cnt;

// RC 링크 감시
// *** 주의: 아래 페일세이프는 고도 피드백이 전혀 없는 "시간 기반" 하강임 ***
// (BARO_ENABLED=0 이라 alt_PID() 미사용) 정해진 속도로 스로틀을 낮추다가
// 무조건 완전 디스암(모터 컷)으로 끝나므로, 고고도(예: 100m)에서는 바닥에
// 닿기 전에 모터가 꺼져 추락할 수 있음. 저고도(실내/마당) 테스트 전용.
// 자세한 내용은 파일 상단 2026-09-25 21:29 리비전 기록 참고.
#define RC_TIMEOUT_MS            250     // 이 시간 동안 유효 프레임이 없으면 링크 두절
#define RC_FS_DESCENT_US_PER_SEC 50.0f   // 페일세이프 하강 구간 감소 속도 (1050까지 도달하는 시간 결정)
#define RC_FS_RISE_US_PER_SEC    50.0f   // 페일세이프 상승 구간(붐업) 증가 속도(하강과 동일) -- 기존 100의 절반
// 페일세이프 감지 시점엔 채널값 기반 판정 특성상 ch3가 이미 최저(1000)로
// 고정돼 있어서 "감지 시점 값에서부터 감소"로는 즉시 컷이 되어버림.
// 그래서 페일세이프 패턴이 아닐 때(=정상 조종 중)의 마지막 스로틀 값을 계속
// 기억해두고(last_good_throttle), 페일세이프가 확정되면 그 값에서 시작해서
// 자세 유지용 여유분(RC_FS_RECOVERY_BOOST)만큼 "서서히" 올렸다가(RC_FS_RISE_US_PER_SEC
// 속도로), 그 정점에서 다시 "서서히" 감소시킨다(RC_FS_DESCENT_US_PER_SEC 속도로).
// 두 구간 모두 즉시 계단식으로 바뀌지 않고 매 루프 조금씩 변하도록 함.
#define RC_FS_RECOVERY_BOOST     50.0f   // 하강 시 자세 유지를 위해 살짝 올리는 양
#define RC_FS_BOOSTED_MAX        1900.0f // 위 boost 후 상한(과도한 출력 방지)

// 송신기 Failsafe에 아래 값으로 캡처해둘 것: ch1=0%, ch2=0%, ch3=-100%, ch4=0%
// (FS-i6 Failsafe는 스틱 캡처 방식이라 ch3가 물리적 최저인 -100%(1000)보다
//  더 낮게는 못 내려감 -> 채널 하나가 아니라 4개 채널이 동시에 이 값과
//  일치하는 조합으로 판단해야 평상시 아이들 상태와 구분이 됨)
#define RC_FS_CH1          1500  // 롤 중앙
#define RC_FS_CH2          1500  // 피치 중앙
#define RC_FS_CH3          1000  // 스로틀 최저(캡처 가능한 한계)
#define RC_FS_CH4          1500  // 요 중앙
#define RC_FS_TOL          15    // 허용 오차 (us)
#define RC_FS_MATCH_HOLD_MS 400  // 이 조합이 이 시간 이상 유지되면 링크 두절로 판단
                                 // (조종 중 잠깐 스치는 것과 구분하기 위한 디바운스)
static uint32_t rc_last_valid_ms;
static bool rc_ever_valid = false;
static float last_good_throttle = 1500.0f; // 페일세이프 패턴이 아닐 때의 마지막 스로틀(us)

RC channel;
ERRORS error;

//Local:
uint16_t uart4_rx_checksum;

// 진단용(임시) 카운터: 프레임 시도 횟수 / 체크섬 통과 횟수
uint32_t rc_frames_total;
uint32_t rc_frames_valid;

RC get_rc(Sensor type) {

	if (uart4_rx_idle) {
		uart4_rx_idle = false;
		LL_USART_DisableIT_RXNE(UART4);

		rc_frames_total++;

		uart4_rx_checksum = 0xffff;
		for (uint8_t i = 0; i < 29 - 1; i++)
			uart4_rx_checksum -= uart4_rx_data[i];
		uart4_rx_checksum -= 0x20;
		uart4_rx_checksum -= 0x40;
		if (uart4_rx_checksum
				== (uart4_rx_data[28] | uart4_rx_data[29] << 8)) {
			rc_frames_valid++;
			channel.ch1 = uart4_rx_data[0] | uart4_rx_data[1] << 8;
			channel.ch2 = uart4_rx_data[2] | uart4_rx_data[3] << 8;
			channel.ch3 = uart4_rx_data[4] | uart4_rx_data[5] << 8;
			channel.ch4 = uart4_rx_data[6] | uart4_rx_data[7] << 8;
			channel.ch5 = uart4_rx_data[8] | uart4_rx_data[9] << 8;
			channel.ch6 = uart4_rx_data[10] | uart4_rx_data[11] << 8;
			channel.ch7 = uart4_rx_data[12] | uart4_rx_data[13] << 8;
			channel.ch8 = uart4_rx_data[14] | uart4_rx_data[15] << 8;
			channel.ch9 = uart4_rx_data[16] | uart4_rx_data[17] << 8;
			channel.ch10 = uart4_rx_data[18] | uart4_rx_data[19] << 8;

			rc_last_valid_ms = HAL_GetTick();
			rc_ever_valid = true;
		}

		LL_USART_EnableIT_RXNE(UART4);
	}
	return channel;
}

/*
 * When FM(2|3) -> ARMED(2)??? -------------> ()
 *
 */

uint8_t flightmode(RC _rc) {

	uint8_t fm = 1; //Basic!

	if (ARMED == 2) {
		if (_rc.ch5 > 1400 && !error.BARO) {
			fm = 2;
		}

		if (_rc.ch5 > 1900 && !error.BARO && !error.GPS) {

			fm = 3;

			if (_rc.ch6 > 1900) { //rtb:
				if (1/*autonomous.home_pointed*/) //---------------> ()
					fm = 4;
				else
					fm = 3;
			}

			if (_rc.ch7 > 1900) { //waypoints:
				fm = 5;
			}

			if (_rc.ch8 > 1900) { //follow me:
				fm = 6;
			}

			if (_rc.ch9 > 1100) { //circle:
				fm = 7;
			}

			if(_rc.ch10 > 1100) {
				//fm = 8 ???
			}
		}
	}

	return fm;
}

uint8_t armed;
uint8_t arming(RC _rc) {

//	if(error.BARO /*| error.GPS*/) return 0; //added in 2024-06-27
	// (2026-09-25 재확인 후 재제거) 저전압(error.BATTERY) 아밍 차단은 제거함:
	//  - 전압 분배/ADC 판정 오류 하나로 아예 이륙을 못 하게 되는 위험한 실패 모드였음.
	//  - 저전압 경고는 LED(error.c)와 텔레메트리 에러비트(bit5)로 계속 표시됨.
	//  - NOTE: 이전에도 한 번 제거했으나 기기에 실제로 반영되지 않았던 이력이 있음(커밋 동기화 문제).
	// armed != 2: 비행 중에는 이 조합(스로틀 최저 + 요 좌측 끝)으로 armed=1 이 되어
	// 모터가 멈추고 gyroAngle/PID 가 리셋되는 일이 없도록 한다(디스암은 아래 별도 조합).
	if (armed != 2 && (_rc.ch3 > 950 && _rc.ch3 < 1050) && (_rc.ch4 >  950 && _rc.ch4 < 1050) && !error.RC) {//---------> ()
		armed = 1;

		gyroAngle.x = accelAngle.x;
		gyroAngle.y = accelAngle.y;

		att_pid_init();//OK
	}

	if (armed == 1 && _rc.ch3 < 1050 && _rc.ch4 > 1450) {
		armed = 2;
	}

	//disarming:
	if (armed == 2 && _rc.ch3 < 1050 && _rc.ch4 > 1950)
		armed = 0;

	return armed;
}

/*
 * RC 링크 감시 / 페일세이프
 */
static bool rc_matches_failsafe_pattern(void) {
	int32_t d1 = (int32_t) channel.ch1 - RC_FS_CH1;
	int32_t d2 = (int32_t) channel.ch2 - RC_FS_CH2;
	int32_t d3 = (int32_t) channel.ch3 - RC_FS_CH3;
	int32_t d4 = (int32_t) channel.ch4 - RC_FS_CH4;
	if (d1 < 0) d1 = -d1;
	if (d2 < 0) d2 = -d2;
	if (d3 < 0) d3 = -d3;
	if (d4 < 0) d4 = -d4;
	return (d1 <= RC_FS_TOL) && (d2 <= RC_FS_TOL) && (d3 <= RC_FS_TOL)
			&& (d4 <= RC_FS_TOL);
}

bool rc_link_lost(void) {
	if (!rc_ever_valid)
		return true;
	if ((HAL_GetTick() - rc_last_valid_ms) > RC_TIMEOUT_MS)
		return true; // 수신기 자체가 꺼지거나 배선이 빠진 경우(프레임 자체가 끊김)

	// 송신기만 꺼진 경우: FS-iA6B는 RF 링크가 끊겨도 전원이 켜져 있으면
	// iBus 프레임을 계속 정상 체크섬으로 보내므로 위 타임아웃으로는 못 잡음.
	// 송신기 Failsafe에 캡처해둔 "롤/피치/요 중앙 + 스로틀 최저" 조합이
	// RC_FS_MATCH_HOLD_MS 이상 계속 유지되면 링크 두절로 판단한다.
	static uint32_t match_start_ms = 0;
	static bool matching = false;
	if (rc_matches_failsafe_pattern()) {
		if (!matching) {
			matching = true;
			match_start_ms = HAL_GetTick();
		}
		if ((HAL_GetTick() - match_start_ms) > RC_FS_MATCH_HOLD_MS)
			return true;
	} else {
		matching = false;
		// 페일세이프 패턴이 아닐 때(=정상 조종 중)의 스로틀을 계속 기억해둔다.
		// 링크가 끊기는 순간 ch3는 이미 페일세이프 값(최저)으로 고정되어 있으므로,
		// rc_failsafe()가 감소를 시작할 기준값은 여기서 갱신해둔 이 값을 쓴다.
		last_good_throttle = (float) channel.ch3;
	}

	return false;
}

// main 루프(4kHz)에서 get_rc() 직후 호출.
// 비행(ARMED==2) 중 링크가 끊기면 스틱을 중립으로 고정하고 스로틀을 서서히 낮춘 뒤 디스암한다.
RC rc_failsafe(RC r) {
	static bool fs_active = false;
	static bool fs_rising = false;
	static float fs_throttle = 0.0f;
	static float fs_peak = 0.0f;

	// rc_link_lost()는 내부적으로 last_good_throttle 등 상태를 갱신하므로
	// 매 루프 반드시 호출한다. "확정된"(디바운스 통과) 링크 두절 여부.
	bool lost = rc_link_lost();
	// "확정 전"이라도 이번 루프 채널값이 이미 페일세이프 패턴과 일치하는지.
	// 송신기가 꺼지는 순간 수신기는 즉시 이 패턴(스로틀 최저 등)을 내보내므로,
	// 디바운스(RC_FS_MATCH_HOLD_MS)가 끝날 때까지 그 값을 그대로 모터에
	// 흘려보내면 그 사이 모터가 일시적으로 죽어버린다. 그래서 패턴이 보이는
	// 즉시(확정 여부와 무관하게) 아래 보호 로직을 먼저 가동한다.
	bool pattern_now = rc_matches_failsafe_pattern();

	if (ARMED != 2 || (!pattern_now && !lost)) {
		fs_active = false;
		return r;
	}

	if (!fs_active) {
		fs_active = true;
		fs_rising = true;
		// 감지 시점의 r.ch3 는 이미 페일세이프 값(최저)으로 고정되어 있으므로
		// 그대로 쓰면 즉시 컷이 됨 -> 링크가 끊기기 직전의 마지막 정상 스로틀
		// (last_good_throttle)에서 시작해서 정점(fs_peak)까지 "서서히" 올렸다가,
		// 거기서부터 다시 "서서히" 감소시킨다(계단식 점프 없이 매 루프 조금씩).
		fs_throttle = last_good_throttle;
		fs_peak = last_good_throttle + RC_FS_RECOVERY_BOOST;
		if (fs_peak > RC_FS_BOOSTED_MAX)
			fs_peak = RC_FS_BOOSTED_MAX;
	}

	if (fs_rising) {
		fs_throttle += RC_FS_RISE_US_PER_SEC / FC_LOOP_HZ; // 루프 4kHz
		if (fs_throttle >= fs_peak) {
			fs_throttle = fs_peak;
			fs_rising = false; // 정점 도달 -> 이후부터는 감소 구간
		}
	} else {
		fs_throttle -= RC_FS_DESCENT_US_PER_SEC / FC_LOOP_HZ; // 루프 4kHz
	}

	r.ch1 = 1500;
	r.ch2 = 1500;
	r.ch4 = 1500;
	r.ch3 = (uint16_t) fs_throttle;

	// 디스암은 "확정된" 링크 두절(lost)일 때만 실행한다. 패턴이 디바운스
	// 시간을 못 채우고 사라지면(=우연히 스친 것) pattern_now 가 false 로
	// 돌아가서 위 가드에서 fs_active 가 풀리고 정상 조종으로 즉시 복귀한다.
	if (lost && !fs_rising && fs_throttle <= 1050.0f) {
		armed = 0; // 디스암 -> 다음 arming() 호출에서 ARMED=0
		r.ch3 = 1000;
	}
	return r;
}
