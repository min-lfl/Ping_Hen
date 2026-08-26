/*
文件名：mpu6050.c
创建日期：2019年11月13日
作者：Bulanov Konstantin
联系方式
电子邮箱：leech001@gmail.com
*/
/*
|---------------------------------------------------------------------------------
| 版权所有 (C) Bulanov Konstantin, 2021
|
| 本程序属于自由软件：你可以根据自由软件基金会（Free Software Foundation）所发布的
| GNU 通用公共许可证（GPL）第 3 版或（随你选择）更新的版本，来重新分发或修改它。
|
| 我们发布这个程序的目的是希望它能对他人有用，
| 但我们“不提供任何保证”；甚至不包含对“可销售性”或“特定用途适用性”的暗示性保证。
| 详情请参阅 GNU 通用公共许可证。
|
| 你应该已经随程序收到了一份 GNU 通用公共许可证的副本。
| 如果没有，请参阅 http://www.gnu.org/licenses/。
|
| 本代码中使用的卡尔曼滤波（Kalman filter）算法参考自：
| https://github.com/TKJElectronics/KalmanFilter
|---------------------------------------------------------------------------------
*/

#include <math.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_CONFIG_2_REG 0x1D
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_READ_TIMEOUT_MS 3U

// Setup MPU6050
#define MPU6050_ADDR 0xD0									// 传感器的 I2C 设备地址
const uint16_t i2c_timeout = 100;					// I2C 通讯超时时间（单位：毫秒）
const double Accel_Z_corrector = 16384.0;	// Z 轴加速度计校准比例因子（用于转换原始值）

// X 轴卡尔曼滤波器参数
static double gyro_bias_x;
static double gyro_bias_y;
static double gyro_bias_z;
static uint8_t filter_initialized;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,										// 角度过程噪声协方差：代表对预测角度的信任程度
    .Q_bias = 0.003f,											// 陀螺仪漂移噪声协方差：代表对陀螺仪偏移量的信任程度
    .R_measure = 0.03f};									// 测量噪声协方差：代表对加速度计测量值的信任程度

// Y 轴卡尔曼滤波器参数
Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // 检查设备ID

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);
		
    if (check == 0x68) // 如果一切顺利，传感器将返回0x70
    {
				
        // 电源管理寄存器 0X6B，需写入全 0 以唤醒传感器
        Data = 0x01;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
				
        HAL_Delay(100);
				
        // CONFIG=2：陀螺仪数字低通约92Hz，响应延迟约3~4ms。
        Data = 0x02;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // 基准采样率为1kHz，分频值为2，传感器输出约333Hz。
        // MPU6500兼容芯片的加速度计低通也设置为约92Hz。
        Data = 0x02;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // 在 ACCEL_CONFIG 寄存器中设置加速度计配置
        // 设置 X、Y、Z 轴自检禁用，量程选择 FS_SEL=0，即量程为 ±2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        Data = 0x02;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_2_REG, 1, &Data, 1, i2c_timeout);

        // 在 GYRO_CONFIG 寄存器中设置陀螺仪配置
        // 设置 X、Y、Z 轴自检禁用，量程选择 FS_SEL=0，即量程为 ±250 °/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
				
        return 0;
    }
    return 1;
}

uint8_t MPU6050_CalibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t sample_count)
{
    uint8_t Rec_Data[6];
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;
    uint16_t sample;

    if (sample_count == 0)
        return 1;

    // 校准期间必须保持传感器静止，先等待器件和温度稳定。
    HAL_Delay(500);

    // 对静止陀螺仪原始值求平均，得到三个轴的零偏。
    for (sample = 0; sample < sample_count; sample++)
    {
        if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1,
                             Rec_Data, 6, i2c_timeout) != HAL_OK)
            return 2;

        sum_x += (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        sum_y += (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        sum_z += (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        HAL_Delay(10);
    }

    gyro_bias_x = (double)sum_x / sample_count;
    gyro_bias_y = (double)sum_y / sample_count;
    gyro_bias_z = (double)sum_z / sample_count;

    // 清空卡尔曼滤波器状态，下一次读取时用加速度计角度初始化。
    KalmanX.angle = 0.0;
    KalmanX.bias = 0.0;
    KalmanX.P[0][0] = 0.0;
    KalmanX.P[0][1] = 0.0;
    KalmanX.P[1][0] = 0.0;
    KalmanX.P[1][1] = 0.0;
    KalmanY.angle = 0.0;
    KalmanY.bias = 0.0;
    KalmanY.P[0][0] = 0.0;
    KalmanY.P[0][1] = 0.0;
    KalmanY.P[1][0] = 0.0;
    KalmanY.P[1][1] = 0.0;
    filter_initialized = 0;

    return 0;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // 从 ACCEL_XOUT_H 寄存器开始读取 6 字节数据

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** 	将原始数值转换为以 'g' 为单位的加速度值。
					根据 FS_SEL 设置的满量程范围，需要除以相应的比例因子。
					此处配置 FS_SEL = 0，故除以 16384.0。
					更多详情请查阅 ACCEL_CONFIG 寄存器。 ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // 从 GYRO_XOUT_H 寄存器开始读取 6 字节数据

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** 	将原始数值转换为度/秒 (°/s)。
					根据 FS_SEL 设置的满量程范围，需要除以相应的比例因子。
					此处配置 FS_SEL = 0，故除以 131.0。
					更多详情请查阅 GYRO_CONFIG 寄存器。 ****/

    DataStruct->Gx = (DataStruct->Gyro_X_RAW - gyro_bias_x) / 131.0;
    DataStruct->Gy = (DataStruct->Gyro_Y_RAW - gyro_bias_y) / 131.0;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW - gyro_bias_z) / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    
		// 从 TEMP_OUT_H_REG 寄存器开始读取 2 字节数据

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

	
		//	把温度值存入结构体（单位°C）
    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

//哪个口,结构体,频率
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,double Frequency)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // 从 ACCEL_XOUT_H 寄存器开始读取 14 字节数据

    if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1,
                         Rec_Data, 14, MPU6050_READ_TIMEOUT_MS) != HAL_OK)
        return;

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = (DataStruct->Gyro_X_RAW - gyro_bias_x) / 131.0;
    DataStruct->Gy = (DataStruct->Gyro_Y_RAW - gyro_bias_y) / 131.0;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW - gyro_bias_z) / 131.0;

    
		// 卡尔曼角度解算
    // 姿态融合按目标周期计算dt。
    double dt = 1.0 / Frequency;
    double roll;
    double accel_x = DataStruct->Accel_X_RAW;
    double accel_z = DataStruct->Accel_Z_RAW;
    double roll_sqrt = sqrt(accel_x * accel_x + accel_z * accel_z);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;

    if (!filter_initialized)
    {
        KalmanX.angle = roll;
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleX = roll;
        DataStruct->KalmanAngleY = pitch;
        filter_initialized = 1;
        return;
    }

    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
