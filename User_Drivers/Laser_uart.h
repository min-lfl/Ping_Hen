/**
 * @file    Laser_uart.h
 * @brief   STP-23L 激光测距传感器的 UART DMA + IDLE 接收接口。
 *
 * 传感器上电后会主动连续发送数据，本模块不向传感器发送初始化命令。
 * 应用层只需要：
 * 1. 在对应 UART 和 DMA 完成 CubeMX 初始化后调用 Laser_UART_Init()；
 * 2. 需要距离时调用 Laser_UART_GetDistance() 获取后台刷新的最新值。
 */

#ifndef LASER_UART_H
#define LASER_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/** STP-23L 官方例程使用的固定串口波特率。 */
#define LASER_UART_BAUD_RATE    230400U

/**
 * @brief  启动 STP-23L 的 DMA + IDLE 后台接收。
 * @param  huart 专用于 STP-23L 的 UART 句柄。
 * @retval HAL_OK    接收已经启动。
 * @retval HAL_ERROR 参数、波特率或 RX DMA 配置不正确。
 * @retval HAL_BUSY  该 UART 正被其他接收任务占用。
 *
 * @note 调用前应由 CubeMX 完成以下硬件配置：
 *       - UART：230400, 8 data bits, no parity, 1 stop bit；
 *       - UART RX DMA：外设到内存、字节对齐、内存地址递增；
 *         普通模式和环形模式均支持；
 *       - 使能 UART 全局中断和对应的 DMA 通道中断。
 * @note 一个 UART 只能有一个接收拥有者，不能与其他协议共用 RX。
 */
HAL_StatusTypeDef Laser_UART_Init(UART_HandleTypeDef *huart);

/**
 * @brief  获取后台最近一次校验正确的平均距离。
 * @param  distance_mm 输出距离，单位为毫米，浮点数类型。
 * @retval true  已收到过有效数据，distance_mm 中是最新距离。
 * @retval false 尚未收到有效数据，或 distance_mm 是空指针。
 *
 * @note 本函数只复制缓存，不会访问串口、不会等待新数据，也不会
 *       消耗已有数据；连续调用会返回同一个最新值，直到后台更新。
 */
bool Laser_UART_GetDistance(float *distance_mm);

#ifdef __cplusplus
}
#endif

#endif /* LASER_UART_H */
