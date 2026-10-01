/*
 * hc-12_setup.c
 *
 *  Created on: 2026-09-23
 *  Updated: 2026-09-23 15:06
 *      Author: gyoosuhn, Jee
 *
 *  HC-12 설정 루틴 (USART3: PB10=TX, PB11=RX / SET: PB0)
 *  - 사용법: main.c USER CODE 2 영역에서 HC12_Setup(); 호출
 *  - 용도가 끝나면 main.c의 HC12_Setup(); 호출만 주석 처리
 */

#include "hc-12_setup.h"
#include <stdio.h>
#include <string.h>

// 1. 1바이트 전송 함수
void UART3_SendChar(uint8_t ch) {
	// 송신 데이터 레지스터가 비워질 때까지 대기 (TXE: Transmit Data Register Empty)
	while (!LL_USART_IsActiveFlag_TXE(USART3)) {
	}
	LL_USART_TransmitData8(USART3, ch);
}

// 2. 문자열 전송 함수
void UART3_SendString(char *str) {
	while (*str) {
		UART3_SendChar((uint8_t) (*str));
		str++;
	}
	// 모든 전송이 완료될 때까지 대기 (TC: Transmission Complete)
	while (!LL_USART_IsActiveFlag_TC(USART3)) {
	}
}

// 3. HC-12 AT 명령 모드 진입
void HC12_EnterATMode(void) {
	LL_GPIO_ResetOutputPin(HC12_SET_GPIO_Port, HC12_SET_Pin); // SET 핀 LOW
	HAL_Delay(50); // 40ms 이상 대기
}

// 4. HC-12 투과(일반 통신) 모드 진입
void HC12_EnterDataMode(void) {
	LL_GPIO_SetOutputPin(HC12_SET_GPIO_Port, HC12_SET_Pin);   // SET 핀 HIGH
	HAL_Delay(100); // 80ms 이상 대기
}

// 5. AT 명령어 전송 및 응답 확인 함수 (USART2로 출력)
// Updated: 2026-09-23 14:13 - 에러 플래그 클리어 + 진단 출력(ISR, PB11/PB0 레벨) 추가
void HC12_SendATCommand(char *cmd) {
	HC12_EnterATMode();

	// 명령어 전송 전, 에러 플래그(ORE/FE/NE/PE) 클리어 및 수신 버퍼 비우기
	LL_USART_ClearFlag_ORE(USART3);
	LL_USART_ClearFlag_FE(USART3);
	LL_USART_ClearFlag_NE(USART3);
	LL_USART_ClearFlag_PE(USART3);
	while (LL_USART_IsActiveFlag_RXNE(USART3)) {
		(void) LL_USART_ReceiveData8(USART3);
	}

	// [진단] 전송 전 핀 상태: PB0(SET)=0 이어야 AT 모드, PB11(RX)=1 이어야 정상(UART idle = HIGH)
	printf("[DIAG] SET(PB0)=%d, RX(PB11) idle=%d\r\n",
			(int) LL_GPIO_IsOutputPinSet(HC12_SET_GPIO_Port, HC12_SET_Pin),
			(int) LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_11));

	// 명령어 전송
	UART3_SendString(cmd);
	printf("Send to HC-12 : %s\r\n", cmd);
	printf("HC-12 Response: ");

	// 약 200ms 동안 HC-12의 응답을 기다리며 들어오는 족족 USART2(printf)로 출력
	uint32_t rx_count = 0;
	uint32_t start_tick = HAL_GetTick();
	while ((HAL_GetTick() - start_tick) < 200) {
		if (LL_USART_IsActiveFlag_ORE(USART3)) {
			LL_USART_ClearFlag_ORE(USART3);
		}
		if (LL_USART_IsActiveFlag_RXNE(USART3)) {
			uint8_t rx_data = LL_USART_ReceiveData8(USART3);
			printf("%c", rx_data);
			rx_count++;
		}
	}
	printf("\r\n");

	// [진단] 수신 바이트 수 + USART3 ISR 레지스터
	//  - rx=0 이고 FE/NE=0 : RX 핀으로 신호가 아예 안 들어옴 (배선/핀/HC-12 무응답)
	//  - FE 또는 NE = 1    : 신호는 들어오나 보레이트 불일치
	uint32_t isr = LL_USART_ReadReg(USART3, ISR);
	printf("[DIAG] rx=%lu, ISR=0x%08lX (ORE=%d FE=%d NE=%d)\r\n\n",
			(unsigned long) rx_count, (unsigned long) isr,
			(int) ((isr & USART_ISR_ORE) != 0), (int) ((isr & USART_ISR_FE) != 0),
			(int) ((isr & USART_ISR_NE) != 0));

	HC12_EnterDataMode();
}

// 6. USART3 보레이트 런타임 변경 (PCLK1 = USART3 클럭 소스)
// Updated: 2026-09-23 14:18
static void UART3_SetBaud(uint32_t baud) {
	while (!LL_USART_IsActiveFlag_TC(USART3)) {
	}
	LL_USART_Disable(USART3);
	LL_USART_SetBaudRate(USART3, HAL_RCC_GetPCLK1Freq(), LL_USART_PRESCALER_DIV1,
			LL_USART_OVERSAMPLING_16, baud);
	LL_USART_Enable(USART3);
	while ((!LL_USART_IsActiveFlag_TEACK(USART3))
			|| (!LL_USART_IsActiveFlag_REACK(USART3))) {
	}
}

// 7. AT 명령 전송 후 응답을 resp에 저장 (AT 모드 진입은 호출 측에서)
//    Updated: 2026-09-23 14:40 - 0x00 바이트는 문자열에서 제외(strstr/%s가 끊기는 문제),
//                               수신 원시 바이트를 HEX로 출력 + FE/NE 표시
static uint16_t HC12_Query(const char *cmd, char *resp, uint16_t max,
		uint32_t timeout_ms) {
	uint16_t n = 0, raw_n = 0;
	uint8_t raw[32];
	LL_USART_ClearFlag_ORE(USART3);
	LL_USART_ClearFlag_FE(USART3);
	LL_USART_ClearFlag_NE(USART3);
	while (LL_USART_IsActiveFlag_RXNE(USART3)) {
		(void) LL_USART_ReceiveData8(USART3);
	}
	UART3_SendString((char*) cmd);
	uint32_t t0 = HAL_GetTick();
	while ((HAL_GetTick() - t0) < timeout_ms) {
		if (LL_USART_IsActiveFlag_ORE(USART3)) {
			LL_USART_ClearFlag_ORE(USART3);
		}
		if (LL_USART_IsActiveFlag_RXNE(USART3)) {
			uint8_t c = LL_USART_ReceiveData8(USART3);
			if (raw_n < sizeof(raw)) {
				raw[raw_n++] = c;
			}
			if (c != 0x00 && n < max - 1) {
				resp[n++] = (char) c;
			}
		}
	}
	resp[n] = '\0';

	// [진단] 수신 원시 바이트(HEX) + 오류 플래그
	uint32_t isr = LL_USART_ReadReg(USART3, ISR);
	printf("    raw(%u):", raw_n);
	for (uint16_t i = 0; i < raw_n; i++) {
		printf(" %02X", raw[i]);
	}
	printf("  FE=%d NE=%d\r\n", (int) ((isr & USART_ISR_FE) != 0),
			(int) ((isr & USART_ISR_NE) != 0));
	return n;
}

// 8. HC-12의 현재 보레이트를 자동 탐색 -> 9600bps로 변경 -> 검증
//    반환: 1 = 성공(9600 확인), 0 = 실패
uint8_t HC12_AutoBaudTo9600(void) {
	const uint32_t bauds[] = { 9600, 1200, 2400, 4800, 19200, 38400, 57600,
			115200 };
	char resp[64];
	uint32_t found = 0;

	HC12_EnterATMode();

	// (1) 탐색: 각 보레이트로 "AT" 전송, "OK" 응답이 오는 속도를 찾음
	for (uint8_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
		UART3_SetBaud(bauds[i]);
		HAL_Delay(20);
		HC12_Query("AT", resp, sizeof(resp), 150);
		printf("[SCAN] %6lu bps -> \"%s\"\r\n", (unsigned long) bauds[i], resp);
		if (strstr(resp, "OK") != NULL) {
			found = bauds[i];
			break;
		}
	}

	if (found == 0) {
		printf("[HC-12] No response at any baud -> check SET pin/power/wiring\r\n");
		UART3_SetBaud(9600);
		HC12_EnterDataMode();
		return 0;
	}
	printf("[HC-12] Current baud: %lu bps\r\n", (unsigned long) found);

	// (2) 9600이 아니면 변경 명령 전송 (현재 보레이트로 보내야 함)
	if (found != 9600) {
		HC12_Query("AT+B9600", resp, sizeof(resp), 200);
		printf("[HC-12] AT+B9600 -> \"%s\"\r\n", resp);
	}

	// (3) AT 모드를 빠져나와 새 보레이트 적용 -> 9600으로 재진입해 검증
	HC12_EnterDataMode();
	UART3_SetBaud(9600);
	HC12_EnterATMode();
	HC12_Query("AT", resp, sizeof(resp), 150);
	printf("[VERIFY] 9600 bps AT -> \"%s\"\r\n", resp);
	uint8_t ok = (strstr(resp, "OK") != NULL);
	HC12_Query("AT+RX", resp, sizeof(resp), 300); // 전체 설정 확인
	printf("[VERIFY] AT+RX -> %s\r\n", resp);
	HC12_EnterDataMode();
	return ok;
}

// 9. 설정 루틴 진입점: main.c에서 이 함수만 호출
void HC12_Setup(void) {
	printf("--- HC-12 Configuration Start ---\r\n");

	if (HC12_AutoBaudTo9600()) {
		printf("[HC-12] 9600bps setup OK\r\n");
	} else {
		printf("[HC-12] Setup FAILED\r\n");
	}

	printf("--- HC-12 Configuration End ---\r\n");

	// 무선 송신 확인용 메시지 (상대편 HC-12에서 수신 확인)
	HAL_Delay(2000);
	UART3_SendString("Baudrate is set to 9600 bps!\r\n");
}
