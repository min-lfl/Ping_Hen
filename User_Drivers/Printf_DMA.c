#include "Printf_DMA.H"


// 自定义的 DMA 打印函数
void printf_dma(const char *format, ...) {
    // 1. 等待上一次 DMA 发送完成（防止覆盖正在发送的数据）
    // 判断串口是否处于就绪状态
    while (huart1.gState != HAL_UART_STATE_READY) {
        // 等待，或者加上超时处理
    }

    // 2. 将传入的参数格式化为字符串，存入全局缓冲区 UartTxBuf
    va_list args;
    va_start(args, format);
    // vsnprintf 会自动把 \n 之类的格式化好，返回实际写入的长度
    int length = vsnprintf((char *)UartTxBuf, UART_TX_BUF_SIZE, format, args);
    va_end(args);

    // 3. 启动 DMA 发送
    if (length > 0) {
        HAL_UART_Transmit_DMA(&huart1, UartTxBuf, length);
    }
}



