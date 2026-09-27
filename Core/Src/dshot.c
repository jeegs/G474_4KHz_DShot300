/*
 * dshot.c
 *
 *  Created on: 2026. 9. 22.
 *      Author: 8275
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 *   - ARR/펄스폭을 160MHz 기준 DShot300 규격(3.333us)으로 수정
 *   - DShot 출력 핀 GPIO 속도 HIGH
 *   - dshot_delay_ms() 추가
 */

#include "dshot.h"

uint32_t dshot_buffer_m1[DSHOT_FRAME_SIZE]; // 모터1: TIM5_CH1 -> DMA1_Channel2
uint32_t dshot_buffer_m2[DSHOT_FRAME_SIZE]; // 모터2: TIM5_CH2 -> DMA1_Channel3
uint32_t dshot_buffer_m3[DSHOT_FRAME_SIZE]; // 모터3: TIM4_CH1 -> DMA1_Channel4
uint32_t dshot_buffer_m4[DSHOT_FRAME_SIZE]; // 모터4: TIM4_CH2 -> DMA1_Channel5

// 16비트 DShot 패킷 생성 (CRC 포함)
static uint16_t prepare_packet(uint16_t value) {
    uint16_t packet = (value << 1) | 0; // 텔레메트리 비트 0
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    return (packet << 4) | csum;
}

// 패킷을 DMA 버퍼(듀티비 배열)로 변환
static void encode_packet(uint32_t* buffer, uint16_t value) {
    uint16_t packet = prepare_packet(value);
    for (int i = 0; i < 16; i++) {
        buffer[i] = (packet & 0x8000) ? DSHOT_T1 : DSHOT_T0;
        packet <<= 1;
    }
    buffer[16] = 0; // 프레임 종료 후 LOW 유지
    buffer[17] = 0;
}

void dshot_init(void) {
    // CubeMX 재생성/설정 오류와 무관하게 DShot300 비트 주기를 보장
    LL_TIM_SetAutoReload(TIM5, DSHOT_ARR);
    LL_TIM_SetAutoReload(TIM4, DSHOT_ARR);

    // 300kbit 엣지 품질 확보를 위해 출력 핀 속도를 HIGH로 (생성 코드는 LOW)
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_2,  LL_GPIO_SPEED_FREQ_HIGH); // TIM5_CH1
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_1,  LL_GPIO_SPEED_FREQ_HIGH); // TIM5_CH2
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH); // TIM4_CH1
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH); // TIM4_CH2

    // 모든 채널의 DMA 요청 활성화
    LL_TIM_EnableDMAReq_CC1(TIM5);
    LL_TIM_EnableDMAReq_CC2(TIM5);
    LL_TIM_EnableDMAReq_CC1(TIM4);
    LL_TIM_EnableDMAReq_CC2(TIM4);

    // PWM 출력 활성화
    LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(TIM5);
    LL_TIM_EnableCounter(TIM4);
}

// DMA 전송 트리거 함수
static void trigger_dma(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t *buffer, volatile uint32_t *CCR) {
    LL_DMA_DisableChannel(DMAx, Channel);
    LL_DMA_SetDataLength(DMAx, Channel, DSHOT_FRAME_SIZE);
    LL_DMA_SetMemoryAddress(DMAx, Channel, (uint32_t)buffer);
    LL_DMA_SetPeriphAddress(DMAx, Channel, (uint32_t)CCR);
    LL_DMA_EnableChannel(DMAx, Channel);
}

void dshot_write(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4) {
    // 1. 각 모터별 스로틀 값을 18칸짜리 펄스폭 배열로 인코딩
    encode_packet(dshot_buffer_m1, motor1);
    encode_packet(dshot_buffer_m2, motor2);
    encode_packet(dshot_buffer_m3, motor3);
    encode_packet(dshot_buffer_m4, motor4);

    // 이전 프레임 전송 완료 플래그(Global Interrupt) 강제 초기화
    LL_DMA_ClearFlag_GI2(DMA1); // TIM5_CH1 (M1) 플래그 클리어
    LL_DMA_ClearFlag_GI3(DMA1); // TIM5_CH2 (M2) 플래그 클리어
    LL_DMA_ClearFlag_GI4(DMA1); // TIM4_CH1 (M3) 플래그 클리어
    LL_DMA_ClearFlag_GI5(DMA1); // TIM4_CH2 (M4) 플래그 클리어

    // 2. 인코딩된 배열을 DMA를 통해 각 타이머 채널(CCR)로 백그라운드 전송
    // (CubeMX 할당 순서: USART3_RX(Ch1) -> M1(Ch2) -> M2(Ch3) -> M3(Ch4) -> M4(Ch5))
    trigger_dma(DMA1, LL_DMA_CHANNEL_2, dshot_buffer_m1, &TIM5->CCR1);
    trigger_dma(DMA1, LL_DMA_CHANNEL_3, dshot_buffer_m2, &TIM5->CCR2);
    trigger_dma(DMA1, LL_DMA_CHANNEL_4, dshot_buffer_m3, &TIM4->CCR1);
    trigger_dma(DMA1, LL_DMA_CHANNEL_5, dshot_buffer_m4, &TIM4->CCR2);
}

// 초기화 구간(IMU 설정/오프셋 계산 등)에서 LL_mDelay 대신 사용.
// 1ms마다 0 프레임을 보내 ESC가 신호를 계속 받도록 한다.
void dshot_delay_ms(uint32_t ms) {
    while (ms--) {
        dshot_write(0, 0, 0, 0);
        LL_mDelay(1);
    }
}
