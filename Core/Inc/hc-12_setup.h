/*
 * hc-12_setup.h
 *
 *  Created on: 2026-09-23
 *  Updated: 2026-09-23 15:06
 *      Author: gyoosuhn, Jee
 */

#ifndef INC_HC_12_SETUP_H_
#define INC_HC_12_SETUP_H_

#include "main.h"
#include <stdint.h>

// USART3 송신 유틸
void UART3_SendChar(uint8_t ch);
void UART3_SendString(char *str);

// HC-12 모드 전환 (SET 핀)
void HC12_EnterATMode(void);
void HC12_EnterDataMode(void);

// AT 명령 1회 전송 + 응답/진단 출력
void HC12_SendATCommand(char *cmd);

// 보레이트 자동 탐색 -> 9600bps 설정 -> 검증 (1: 성공, 0: 실패)
uint8_t HC12_AutoBaudTo9600(void);

// 설정 루틴 진입점 (main.c에서 호출)
void HC12_Setup(void);

#endif /* INC_HC_12_SETUP_H_ */
