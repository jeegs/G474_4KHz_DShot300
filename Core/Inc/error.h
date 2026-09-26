/*
 * error.h
 *
 *  Created on: Mar 08, 2024
 *      Author: JeeGS
 *
 *  Revised: 2026-09-24 22:30 (Claude)
 */


#ifndef INC_ERROR_H_
#define INC_ERROR_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern float battery_voltage;
extern Battery batteryType;

typedef struct {
	bool GYRO;
	bool BARO;
	bool COMPSS;
	bool GPS;
	bool BATTERY;
	bool RC;
} ERRORS;
extern ERRORS error;

// 배터리 분압 ADC -> 전압 [V]  (Vref 3.3V, 12bit, 분압비 6.18)
#define BAT_ADC_TO_VOLT   (3.3f / 4096.0f * 6.18f)

void error_handling(void);
bool checker(Sensor name);
void battery_type(Battery type);

#endif /* INC_ERROR_H_ */
