/*
	mpu6050.h
	创建日期：2019年11月13日
	作者：Bulanov Konstantin
*/

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* GY521 头文件 */

#include <stdint.h>
#include "i2c.h"

// MPU6050 结构体
typedef struct
{

    int16_t Accel_X_RAW;	// X 轴加速度原始数据
    int16_t Accel_Y_RAW;	// Y 轴加速度原始数据
    int16_t Accel_Z_RAW;	// Z 轴加速度原始数据
    double Ax;						// 转换后的 X 轴加速度值（单位：g）
    double Ay;						// 转换后的 Y 轴加速度值（单位：g）
    double Az;						// 转换后的 Z 轴加速度值（单位：g）

    int16_t Gyro_X_RAW;		// X 轴陀螺仪原始数据
    int16_t Gyro_Y_RAW;		// Y 轴陀螺仪原始数据
    int16_t Gyro_Z_RAW;		// Z 轴陀螺仪原始数据
    double Gx;						// 转换后的 X 轴角速度（单位：°/s）
    double Gy;						// 转换后的 Y 轴角速度（单位：°/s）
    double Gz;						// 转换后的 Z 轴角速度（单位：°/s）

    float Temperature;		// 转换后的温度值（单位：℃）

    double KalmanAngleX;	// 经卡尔曼滤波计算出的 X 轴角度
    double KalmanAngleY;	// 经卡尔曼滤波计算出的 Y 轴角度
} MPU6050_t;

// 卡尔曼结构体
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
