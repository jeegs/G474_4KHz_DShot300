/*
 * imu.h
 *
 *  Created on: 2024. 03. 08.
 *  Author: JeeGS
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"

// 자이로 측정 범위. GYRO_FS_SEL 과 DPS 를 반드시 짝으로 바꿀 것.
//  GYRO_FS_SEL 0x00: +-250  (131.0)   0x08: +-500 (65.5)
//              0x10: +-1000 (32.8)    0x18: +-2000 (16.4)
// 2026-09-26: +-500 -> +-1000 (거친 기동에서 포화 방지). 분해능 0.03dps 로 제어에는 충분.
#define GYRO_FS_SEL 0x10
#define DPS 32.8 //+-1000

// 1: 자이로를 FIFO 로 읽는다. 센서는 8kHz 로 샘플을 만들고 루프(4kHz)는 1번에 보통 2개를 읽어
//    평균한다 -> 샘플을 버리지 않고, 읽는 시점과 센서 샘플 시점이 어긋나 생기던 지터가 없어진다.
// 0: 기존 방식(데이터 레지스터 직접 읽기, 8kHz 샘플 중 4kHz 로 1개만 읽음)
#define IMU_USE_FIFO 1

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
#define ICM_FIFO_EN       0x23 // bit4 GYRO_FIFO_EN, bit3 ACCEL_FIFO_EN
#define USER_CTRL         0x6A // bit6 FIFO_EN, bit2 FIFO_RST
#define I2C_IF            0x70 // bit6 I2C_IF_DIS (SPI 전용)
#define FIFO_COUNTH       0x72
#define FIFO_R_W          0x74
#define IMU_FIFO_PACKET   14   // 가속도 6 + 온도 2 + 자이로 6 (레지스터 0x3B~0x48 과 같은 순서)
#define IMU_FIFO_SIZE     1008
#define IMU_FIFO_MAX_READ 16   // 한 번에 읽는 최대 샘플 수 (16*14 = 224 바이트)

// FIFO 동작 확인용 (Live Expressions 에서 imu_fifo 를 본다)
typedef struct {
	uint8_t last_samples;   // 직전 읽기에서 평균한 샘플 수. 비행 루프에서는 대부분 2
	uint32_t empty_count;   // 읽을 샘플이 없던 횟수 (가끔은 정상, 계속 늘면 FIFO 가 안 차는 것)
	uint32_t overflow_count;// FIFO 가 넘쳐 리셋한 횟수 (부팅 직후 외에는 0 이어야 정상)
	uint32_t fallback_count;// FIFO 가 연속으로 비어 레지스터 직접 읽기로 대신한 횟수 (0 이어야 정상)
	uint8_t count_h, count_l; // 마지막으로 읽은 FIFO_COUNTH/L 원시값
	// 설정 직후 되읽기 (정상값): CONFIG 0x40, GYRO_CONFIG 0x10, FIFO_EN 0x18, USER_CTRL 0x40,
	//                            PWR_MGMT_1 0x01, I2C_IF 0x40, WHO_AM_I 0x12
	uint8_t rb_config, rb_gyro_config, rb_fifo_en, rb_user_ctrl, rb_pwr_mgmt_1, rb_i2c_if, rb_whoami;
} IMU_FIFO_DIAG;
#define IMU_FIFO_EMPTY_FALLBACK 8 // 연속으로 이만큼(2ms) 비어 있으면 레지스터를 직접 읽는다
extern IMU_FIFO_DIAG imu_fifo;

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
void imu_fifo_reset(void);
//void gyro_read(void);
//void imu_raw_read(void);
RPY get_angle(Sensor type);
RPY get_angular(Sensor type);
float yaw_angle_limit(float angle);
float angle_deviation(float first_angle, float second_angle);


#endif /* INC_IMU_H_ */
