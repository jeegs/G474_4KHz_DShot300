/*
 * dshot.h
 *
 * Revised: 2026-09-24 22:30 (Claude)
 *  - DShot300 비트 주기를 실제 타이머 클럭(160MHz)에 맞게 수정 (ARR 566 -> 532)
 *  - dshot_delay_ms() 추가 (초기화 구간에도 0 프레임 유지)
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include "main.h"

// DShot300: 비트 주기 3.333us = 533 count @ TIM4/TIM5 클럭 160MHz -> ARR = 532
// (기존 ARR=566 은 170MHz 기준 값이라 실제로는 282kbps(-5.9%)였음)
#define DSHOT_ARR 532
#define DSHOT_T0  200  // 37.5% (1.25us)
#define DSHOT_T1  400  // 75.0% (2.50us)
#define DSHOT_FRAME_SIZE 18 // 16비트 데이터 + 2비트 0(리셋용)

// 최소/최대 스로틀 범위 (DShot 규격)
#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047

void dshot_init(void);
void dshot_write(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

// LL_mDelay 대용: 지연 중에도 1kHz로 0(모터 정지) 프레임을 계속 송신 -> ESC 아밍 유지
void dshot_delay_ms(uint32_t ms);

#endif /* INC_DSHOT_H_ */
