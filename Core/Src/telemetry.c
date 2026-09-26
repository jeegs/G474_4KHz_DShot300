/*
 * telemetry.c
 *
 *  Created on: Jan 24, 2024
 *      Author: gyoosuhn, Jee
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 *   - u3_rx_flag volatile, 하드코딩 더미 패킷 -> 실제 값, 에러 비트 실제 상태 반영
 *   - etc1/etc2 = 루프 사용 클럭 / 오버런 횟수 (진단용)
 *   - waypoint 배열 범위 초과 방지
 */

#include "telemetry.h"
#include "error.h"

//Local ---------------------> start:
extern uint16_t used_clocks;  // main.c (TIM7 0.25us 단위, 1000 초과 = 오버런)
extern uint32_t loop_overrun; // main.c
telemetry_tx_data tm_tx;
telemetry_rx_data tm_rx;
waypoint_data waypoint;

bool only_once = true;
uint8_t wp_number;
int32_t follow_lat;
int32_t follow_lng;

#define LAT 10000000 //???
#define LNG 100000000

//telemetry_rx:
volatile bool u3_rx_flag;
#define u3_rx_size(x) (sizeof(x) / sizeof((x)[0]))
uint8_t u3_rx[12];
uint8_t u3_rx_checksum;
//Local <----------------------- end:

void telemetry_rx_init(void) {
	//@USART3 LL DMA:

	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
	LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
	LL_DMA_MODE_NORMAL |
	LL_DMA_PERIPH_NOINCREMENT |
	LL_DMA_MEMORY_INCREMENT |
	LL_DMA_PDATAALIGN_BYTE |
	LL_DMA_MDATAALIGN_BYTE |
	LL_DMA_PRIORITY_LOW);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1,
			LL_USART_DMA_GetRegAddr(USART3,
			LL_USART_DMA_REG_DATA_RECEIVE));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) u3_rx);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, u3_rx_size(u3_rx));

	LL_USART_EnableIT_IDLE(USART3); //IDLE:
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_USART_EnableDMAReq_RX(USART3);

}

void telemetry(void) {

	tm_tx.battery = (uint8_t) (battery_voltage * 10.0f);

	// 에러 비트: [0]GYRO [1]BARO [2]COMPASS [3]GPS [4]RC [5]BATTERY (error.c make_8bit_with_errors 와 동일)
	tm_tx.error = (uint8_t) ((error.GYRO << 0) | (error.BARO << 1)
			| (error.COMPSS << 2) | (error.GPS << 3) | (error.RC << 4)
			| (error.BATTERY << 5));

	// 진단용(etc): 루프 사용 클럭, 오버런 횟수
	tm_tx.etc1 = used_clocks;
	tm_tx.etc2 = (uint16_t) loop_overrun;

	tm_tx.waypointNum = waypoint_size();

	if (ARMED == 2) {
		tm_tx.flightMode = FM;
	} else {
		tm_tx.flightMode = 0;
	}

	if (u3_rx_flag) {
		u3_rx_flag = 0;
		telemetry_rx();
	}

	// 조건문(loop_counter % 200 == 0)을 제거하고 매 0.25ms 루프마다 무조건 호출 (함수 내부에서 주기 처리)
	telemetry_tx();
}

void telemetry_rx(void) {
	//disble DMA
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

	if (u3_rx[0] == 0x57 && u3_rx[1] == 0x50) {
		tm_rx.type = Waypoint;
	} else if (u3_rx[0] == 0x46 && u3_rx[1] == 0x4D)
		tm_rx.type = FollowMe;

	else if (u3_rx[0] == 0x44 && u3_rx[1] == 0x45) {
		tm_rx.type = Delete;
		LL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
	}

	tm_rx.checksum1 = 0;
	tm_rx.checksum2 = 0;
	for (uint8_t i = 2; i < 10; i++) {
		tm_rx.checksum1 ^= u3_rx[i]; //XOR: A = A^B
		tm_rx.checksum2 |= u3_rx[i]; //OR: A = A | B
	}

	if (tm_rx.checksum1 == u3_rx[10] && tm_rx.checksum2 == u3_rx[11]
			&& tm_rx.checksum2 > 0) {

		tm_rx.lat = u3_rx[2] | u3_rx[3] << 8 | u3_rx[4] << 16 | u3_rx[5] << 24;
		tm_rx.lng = u3_rx[6] | u3_rx[7] << 8 | u3_rx[8] << 16 | u3_rx[9] << 24;

		if (tm_rx.type == Waypoint) { //37 1(10Km) 23456
			if (tm_rx.lat > LAT && tm_rx.lng > LNG) {
				if (wp_number < waypointMax) { // 최대 waypointMax 개
					waypoint_insert_at(waypoint.index++, tm_rx.lat, tm_rx.lng);
				}
			}
		}

		if (tm_rx.type == Delete) {
			if (tm_rx.lat == 127 && tm_rx.lng == 127) { //DEL's ASCII code: 127
				waypoint_delete_all();
				waypoint.index = 0;
			}
		}

		if (tm_rx.type == FollowMe) {
			if (tm_rx.lat > LAT && tm_rx.lng > LNG) {
				follow_lat = tm_rx.lat;
				follow_lng = tm_rx.lng;
				//printf("%ld %ld \r\n", follow_lat, follow_lng);
			}
		}

	}

	//For debugging:
	for (uint8_t i = 0; i < waypoint_size(); i++) {
		//printf("%u %ld %ld\n", i, waypoint.lat[i], waypoint.lng[i]);
	}

	//clear DMA TC flag: LL_DMA_CHANNEL_1!!!
	LL_DMA_ClearFlag_GI1(DMA1);
	LL_DMA_ClearFlag_HT1(DMA1);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_TE1(DMA1);

	//enable DMA
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 12);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

// --- [수정] 메인 루프를 블로킹하지 않는 상태 머신 기반 송신 로직으로 개편 ---
void telemetry_tx(void) {
	static uint8_t tx_buf[23];
	static uint8_t tx_index = 0;
	static uint8_t tx_state = 0; // 0: 패킷 생성 대기, 1: 버퍼 송신 중

	// 1. 100ms 마다(4KHz 루프 기준 400카운트) 새로운 패킷 조립 시작
	if (tx_state == 0 && (loop_counter % 400 == 0)) {
		uint8_t checksum = 0;

		tx_buf[0] = 'G';
		tx_buf[1] = 'S';
		tx_buf[2] = tm_tx.error;
		tx_buf[3] = tm_tx.battery;
		tx_buf[4] = tm_tx.flightMode;
		tx_buf[5] = (uint8_t) (tm_tx.altitude & 0xFF);
		tx_buf[6] = (uint8_t) ((tm_tx.altitude >> 8) & 0xFF);
		tx_buf[7] = tm_tx.satNum_fixType;
		tx_buf[8] = tm_tx.fc_speed;
		tx_buf[9] = tm_tx.waypointNum;
		tx_buf[10] = (uint8_t) (tm_tx.fc_gps_lat & 0xFF);
		tx_buf[11] = (uint8_t) ((tm_tx.fc_gps_lat >> 8) & 0xFF);
		tx_buf[12] = (uint8_t) ((tm_tx.fc_gps_lat >> 16) & 0xFF);
		tx_buf[13] = (uint8_t) ((tm_tx.fc_gps_lat >> 24) & 0xFF);
		tx_buf[14] = (uint8_t) (tm_tx.fc_gps_lng & 0xFF);
		tx_buf[15] = (uint8_t) ((tm_tx.fc_gps_lng >> 8) & 0xFF);
		tx_buf[16] = (uint8_t) ((tm_tx.fc_gps_lng >> 16) & 0xFF);
		tx_buf[17] = (uint8_t) ((tm_tx.fc_gps_lng >> 24) & 0xFF);
		tx_buf[18] = (uint8_t) (tm_tx.etc1 & 0xFF);
		tx_buf[19] = (uint8_t) ((tm_tx.etc1 >> 8) & 0xFF);
		tx_buf[20] = (uint8_t) (tm_tx.etc2 & 0xFF);
		tx_buf[21] = (uint8_t) ((tm_tx.etc2 >> 8) & 0xFF);

		for (int i = 2; i <= 21; i++) {
			checksum ^= tx_buf[i];
		}
		tx_buf[22] = checksum;
		tm_tx.checksum = checksum;

		tx_index = 0;
		tx_state = 1; // 송신 상태로 진입
	}

	// 2. 송신 버퍼(TXE)가 비어있을 때만 1바이트를 넣고 빠져나옴 (논블로킹)
	if (tx_state == 1) {
		if (LL_USART_IsActiveFlag_TXE(USART3)) {
			LL_USART_TransmitData8(USART3, tx_buf[tx_index++]);
			if (tx_index >= 23) {
				tx_state = 0; // 전체 전송 완료, 다음 주기 대기
			}
		}
	}
}

void waypoint_delete_at(uint8_t index) {
	if (index >= wp_number) // 범위 밖 삭제 요청 무시
		return;
	for (uint8_t i = index; i + 1 < wp_number; i++) { // lat[wp_number] 를 읽지 않도록
		waypoint.lat[i] = waypoint.lat[i + 1];
		waypoint.lng[i] = waypoint.lng[i + 1];
	}
	wp_number--;
}

void waypoint_insert_at(uint8_t index, int32_t lat, int32_t lng) {
	if (wp_number >= waypointMax) // 배열 가득 참: lat[30] 쓰기(범위 초과) 방지
		return;
	if (index > wp_number)
		index = wp_number;
	for (uint8_t i = wp_number; i > index; i--) {
		waypoint.lat[i] = waypoint.lat[i - 1];
		waypoint.lng[i] = waypoint.lng[i - 1];
	}
	waypoint.lat[index] = lat;
	waypoint.lng[index] = lng;
	wp_number++;
}

uint8_t waypoint_size(void) {
	uint8_t size = 0;
	for (uint8_t i = 0; i < wp_number; i++) {
		if (waypoint.lat[i] > LAT && waypoint.lng[i] > LNG) {
			size++;
		}
	}
	if (wp_number != size) {
		waypoint_delete_all();
		size = 0; //check: ( )
	}
	return size;
}

void waypoint_delete_all(void) {
	for (uint8_t i = 0; i < wp_number; i++) {
		waypoint.lat[i] = 0;
		waypoint.lng[i] = 0;
	}
	wp_number = 0; //added:24-01-08 (   )
}
