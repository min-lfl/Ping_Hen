#ifndef __Printf_DMA_H__
#define __Printf_DMA_H__

#include "main.h"
#include "usart.h"

#include <stdarg.h> // 需要包含这个头文件来处理可变参数
#include <stdio.h>

// 定义一个全局的发送缓冲区，保证DMA发送期间数据不会被销毁
#define UART_TX_BUF_SIZE 256
static uint8_t UartTxBuf[UART_TX_BUF_SIZE];


void printf_dma(const char *format, ...);				// 自定义的 DMA 打印函数

//使用示例
//printf_dma("value = %d\r\n", val);


#endif
