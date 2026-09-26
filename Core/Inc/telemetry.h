/*
 * telemetry.h
 *
 *  Created on: Jan 24, 2024
 *      Author: gyoosuhn
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

extern uint32_t loop_counter;
//extern bool u3_rx_flag;

typedef struct {
	uint8_t error; //[0]
	uint8_t battery; //[1]
	uint8_t flightMode; //[2], 4bits:FM 2bits:HL 2bits:ARMED
	uint16_t altitude; //[3][4], into_cm
	uint8_t satNum_fixType; //[5], satellite 5bit(max 31)_fix_type 3bit(max 7)
	uint8_t fc_speed; //[6]
	uint8_t waypointNum; //[7]
	int32_t fc_gps_lat; //[8][9][10][11]
	int32_t fc_gps_lng; //[12][13][14][15]
	uint16_t etc1; //[16][17] //for debug
	uint16_t etc2; //[18][19] //for debug
	uint8_t checksum; //[20]
} telemetry_tx_data;
extern telemetry_tx_data tm_tx;

typedef enum {
	Waypoint, FollowMe, Delete
} RX_type;

typedef struct {
	RX_type type;
	int32_t lat;
	int32_t lng;
	uint8_t checksum1;
	uint8_t checksum2;
} telemetry_rx_data;

#define waypointMax 30
typedef struct {
	int32_t lat[waypointMax];
	int32_t lng[waypointMax];
	uint8_t index;
} waypoint_data;

void telemetry_rx_init(void);
void telemetry(void);
void telemetry_rx(void);
// --- [수정] 매개변수 제거 ---
void telemetry_tx(void);
void waypoint_insert_at(uint8_t index, int32_t lat, int32_t lon);
void waypoint_delete_at(uint8_t index);
void waypoint_delete_all(void);
uint8_t waypoint_size(void);

#endif /* INC_TELEMETRY_H_ */
