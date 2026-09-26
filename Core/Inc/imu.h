/*
 * imu.h
 *
 *  Created on: 2024. 03. 08.
 *  Author: JeeGS
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"

//#define DPS 131.0 //+-250
#define DPS 65.5 //+-500
//#define DPS 32.8 //+-1000
//#define DPS 16.4 //+-2000

#define WHO_AM_I          0x75 //0x12?
#define PWR_MGMT_1        0x6B
#define PWR_MGMT_2        0x6C
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define SMPLRT_DIV        0x19
#define ACCEL_XOUT_H      0x3B
#define GYRO_XOUT_H       0x43
#define SIGNAL_PATH_RESET 0x68

typedef struct {
	float x;
	float y;
	float z;
} RPY;
extern RPY gyroAngle;
extern RPY accelAngle;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
	int32_t x_offset;
	int32_t y_offset;
	int32_t z_offset;
	int32_t vector_sum;
	bool offset_calculated;
} IMU;

//Kalman_test:
typedef struct {
    float q_angle; // 각도 과정 노이즈 공분산 (Process noise covariance for the angle)
    float q_bias;  // 바이어스 과정 노이즈 공분산 (Process noise covariance for the bias)
    float r_measure; // 측정 노이즈 공분산 (Measurement noise covariance)
    float angle; // 각도 추정값 (Angle estimate)
    float bias;  // 바이어스 추정값 (Bias estimate)
    float rate;  // 비율 (Rate)
    float p[2][2]; // 오차 공분산 (Error covariance matrix)
} KalmanFilter;
extern KalmanFilter kf_roll;
extern KalmanFilter kf_pitch;

void KalmanInit(KalmanFilter *kf, float q_angle, float q_bias, float r_measure);
void KalmanUpdate(KalmanFilter *kf, float newAngle, float newRate, float dt);


void spi2_ll_open();
void spi2_ll_close(void);
uint8_t spi2_ll_transfer(uint8_t data);
uint8_t spi2_ll_read(uint8_t address);
void spi2_ll_write(uint8_t address, uint8_t data);
void spi2_ll_reads(uint8_t address, uint8_t *data, uint8_t length);
void imu_config(void);
void imu_init(Sensor type);
void gyro_offset_calculate(void);
void accel_offset_calculate(void);
void imu_read(void);
//void gyro_read(void);
//void imu_raw_read(void);
RPY get_angle(Sensor type);
RPY get_angular(Sensor type);
float yaw_angle_limit(float angle);
float angle_deviation(float first_angle, float second_angle);


#endif /* INC_IMU_H_ */
