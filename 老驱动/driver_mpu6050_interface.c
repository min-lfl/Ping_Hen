#include "driver_mpu6050_interface.h"
#include "i2c.h"    // 确保包含了你的 i2c 句柄
#include "usart.h"  
#include <stdarg.h>
#include <stdio.h>
#include "Printf_DMA.H"
/**
 * @brief  IIC 初始化接口 (改成 iic)
 */
uint8_t mpu6050_interface_iic_init(void) {
    return 0;
}

/**
 * @brief  IIC 去初始化接口 (改成 iic)
 */
uint8_t mpu6050_interface_iic_deinit(void) {
    return 0;
}

/**
 * @brief  IIC 读接口 (改成 iic)
 */
uint8_t mpu6050_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    if (HAL_I2C_Mem_Read(&hi2c2, (addr), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 1000) == HAL_OK) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * @brief  IIC 写接口 (改成 iic)
 */
uint8_t mpu6050_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    if (HAL_I2C_Mem_Write(&hi2c2, (addr), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 1000) == HAL_OK) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * @brief  毫秒延时接口
 */
void mpu6050_interface_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

/**
 * @brief  调试打印接口
 */
void mpu6050_interface_debug_print(const char *const fmt, ...) {
    char str[256];
    va_list args;
    
    va_start(args, fmt);
    vsnprintf((char *)str, sizeof(str), fmt, args); // 删除了未使用的 len 变量以消除 Warning #550-D
    va_end(args);
    
    printf_dma("%s", str);
}

/**
 * @brief  接收回调接口
 */
void mpu6050_interface_receive_callback(uint8_t type) {
    // 留空即可
}

