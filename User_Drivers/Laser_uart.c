/**
 * @file    Laser_uart.c
 * @brief   STP-23L 激光测距传感器的 DMA + IDLE 接收与协议解析。
 *
 * 设计说明：
 * - DMA 一次接收一段数据，UART 空闲时才进入一次中断，不再逐字节中断；
 * - 协议解析器允许一帧跨越多次 DMA 回调，也能从半帧或坏帧后重新同步；
 * - 每个正确数据帧包含 12 个测距点，沿用官方算法：剔除 0 后求平均；
 * - 中断后台只覆盖“最新距离”，应用层读取不会阻塞后台刷新。
 */

#include "Laser_uart.h"

#include <stddef.h>

/* STP-23L 数据帧固定字段。 */
#define LASER_FRAME_HEADER_BYTE          0xAAU
#define LASER_FRAME_HEADER_SIZE          4U
#define LASER_DEVICE_ADDRESS             0x00U
#define LASER_DISTANCE_COMMAND           0x02U
#define LASER_CHUNK_OFFSET               0x00U

/*
 * 一帧布局：
 *   4 字节帧头 + 设备地址/命令/偏移/长度共 6 字节
 *   + 12 点 * 15 字节 + 4 字节时间戳 + 1 字节校验和
 */
#define LASER_POINT_COUNT                12U
#define LASER_POINT_SIZE                 15U
#define LASER_TIMESTAMP_SIZE             4U
#define LASER_PAYLOAD_SIZE               ((LASER_POINT_COUNT * LASER_POINT_SIZE) + \
                                          LASER_TIMESTAMP_SIZE)
#define LASER_FRAME_PREFIX_SIZE          10U
#define LASER_FRAME_SIZE                 (LASER_FRAME_PREFIX_SIZE + \
                                          LASER_PAYLOAD_SIZE + 1U)

#define LASER_DISTANCE_LOW_OFFSET        0U
#define LASER_CHECKSUM_START_OFFSET      LASER_FRAME_HEADER_SIZE
#define LASER_CHECKSUM_OFFSET            (LASER_FRAME_SIZE - 1U)

/*
 * 256 字节大于完整的 195 字节数据帧。正常 10 Hz 输出时，每帧间隔会
 * 触发一次 IDLE；即使没有 IDLE，DMA 缓冲区装满也会回调，解析器仍可
 * 正确处理跨缓冲区数据。
 */
#define LASER_DMA_BUFFER_SIZE            256U

static UART_HandleTypeDef *s_laser_uart;
static uint8_t s_dma_rx_buffer[LASER_DMA_BUFFER_SIZE];
static uint16_t s_dma_read_position;
static bool s_dma_is_circular;

/* 协议帧组装缓存。s_frame_length 小于 4 时同时表示已匹配的连续帧头数。 */
static uint8_t s_frame_buffer[LASER_FRAME_SIZE];
static uint16_t s_frame_length;

/*
 * 该影子值只由 UART 中断更新，由应用层读取。
 * getter 使用极短临界区同时复制有效标志和值，避免读写交叉。
 */
static volatile float s_latest_distance_mm;
static volatile bool s_distance_valid;

static HAL_StatusTypeDef Laser_StartDmaReception(void);
static void Laser_ProcessCircularDmaData(uint16_t write_position);
static void Laser_ParseBytes(const uint8_t *data, uint16_t length);
static void Laser_ParseByte(uint8_t byte);
static void Laser_ResyncParser(void);
static bool Laser_IsFramePrefixValid(void);
static bool Laser_ProcessCompleteFrame(void);
static bool Laser_IsChecksumValid(void);
static bool Laser_CalculateAverageDistance(float *distance_mm);
static uint16_t Laser_ReadUint16LittleEndian(const uint8_t *data);

/**
 * @brief 启动一轮 Receive-to-IDLE DMA，并关闭不需要的半传输中断。
 */
static HAL_StatusTypeDef Laser_StartDmaReception(void)
{
    HAL_StatusTypeDef status;

    if ((s_laser_uart == NULL) || (s_laser_uart->hdmarx == NULL)) {
        return HAL_ERROR;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(s_laser_uart,
                                          s_dma_rx_buffer,
                                          LASER_DMA_BUFFER_SIZE);
    if (status == HAL_OK) {
        s_dma_read_position = 0U;

        /*
         * 本模块在 IDLE 或 DMA 满缓冲区时处理数据即可。关闭 HT 可避免
         * 每收半个缓冲区额外进入一次中断。
         */
        __HAL_DMA_DISABLE_IT(s_laser_uart->hdmarx, DMA_IT_HT);
    }

    return status;
}

HAL_StatusTypeDef Laser_UART_Init(UART_HandleTypeDef *huart)
{
    uint32_t saved_primask;

    if ((huart == NULL) ||
        (huart->hdmarx == NULL) ||
        (huart->Init.BaudRate != LASER_UART_BAUD_RATE) ||
        (huart->Init.WordLength != UART_WORDLENGTH_8B) ||
        (huart->Init.Parity != UART_PARITY_NONE) ||
        (huart->Init.StopBits != UART_STOPBITS_1) ||
        ((huart->Init.Mode & UART_MODE_RX) == 0U) ||
        (huart->hdmarx->Init.Direction != DMA_PERIPH_TO_MEMORY) ||
        (huart->hdmarx->Init.PeriphInc != DMA_PINC_DISABLE) ||
        (huart->hdmarx->Init.MemInc != DMA_MINC_ENABLE) ||
        (huart->hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE) ||
        (huart->hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE) ||
        ((huart->hdmarx->Init.Mode != DMA_NORMAL) &&
         (huart->hdmarx->Init.Mode != DMA_CIRCULAR))) {
        return HAL_ERROR;
    }

    /*
     * 不在模块内部偷偷修改波特率或重新初始化 GPIO，避免破坏同一 UART
     * 上已有的设备。初始化函数只接管调用者明确传入的 UART 接收通道。
     */
    if (huart->RxState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    saved_primask = __get_PRIMASK();
    __disable_irq();

    s_laser_uart = huart;
    s_dma_is_circular = (huart->hdmarx->Init.Mode == DMA_CIRCULAR);
    s_dma_read_position = 0U;
    s_frame_length = 0U;
    s_latest_distance_mm = 0.0F;
    s_distance_valid = false;

    __DMB();
    if (saved_primask == 0U) {
        __enable_irq();
    }

    {
        HAL_StatusTypeDef status = Laser_StartDmaReception();

        if (status != HAL_OK) {
            /* 启动失败时释放模块所有权，允许修正配置后再次初始化。 */
            s_laser_uart = NULL;
        }
        return status;
    }
}

bool Laser_UART_GetDistance(float *distance_mm)
{
    uint32_t saved_primask;
    bool is_valid;

    if (distance_mm == NULL) {
        return false;
    }

    /* 临界区只包含一个 bool 和一个 32 位 float 的复制，耗时极短。 */
    saved_primask = __get_PRIMASK();
    __disable_irq();

    is_valid = s_distance_valid;
    if (is_valid) {
        *distance_mm = s_latest_distance_mm;
    }

    __DMB();
    if (saved_primask == 0U) {
        __enable_irq();
    }

    return is_valid;
}

/**
 * @brief HAL 的 Receive-to-IDLE 事件回调。
 *
 * IDLE 和 DMA 缓冲区接收完成都会进入这里。普通 DMA 在解析后重新挂起；
 * 环形 DMA 始终运行，只解析上次位置到本次位置之间的新数据。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != s_laser_uart) {
        return;
    }
		
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

		
		
    if (size > LASER_DMA_BUFFER_SIZE) {
        size = LASER_DMA_BUFFER_SIZE;
    }

    if (s_dma_is_circular) {
        Laser_ProcessCircularDmaData(size);
    } else {
        Laser_ParseBytes(s_dma_rx_buffer, size);
        (void)Laser_StartDmaReception();
    }
}

/**
 * @brief UART 出错后的恢复回调。
 *
 * 发生溢出、噪声或帧错误时丢弃正在组装的坏帧，并重新启动后台 DMA。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_laser_uart) {
        return;
    }

    s_frame_length = 0U;
    (void)Laser_StartDmaReception();
}

/**
 * @brief 解析环形 DMA 中尚未处理的新数据。
 *
 * HAL 在环形 Receive-to-IDLE 模式下传入的 size 是 DMA 当前写入位置，
 * 而不是本次新增字节数。因此必须保存上次位置，并在回卷时分两段解析。
 */
static void Laser_ProcessCircularDmaData(uint16_t write_position)
{
    uint16_t normalized_position = write_position;

    if (normalized_position == LASER_DMA_BUFFER_SIZE) {
        /* DMA 传输完成事件位于缓冲区末尾，随后硬件会从索引 0 继续。 */
        normalized_position = 0U;
    }

    if (write_position > s_dma_read_position) {
        Laser_ParseBytes(&s_dma_rx_buffer[s_dma_read_position],
                         (uint16_t)(write_position - s_dma_read_position));
    } else if (write_position < s_dma_read_position) {
        Laser_ParseBytes(&s_dma_rx_buffer[s_dma_read_position],
                         (uint16_t)(LASER_DMA_BUFFER_SIZE - s_dma_read_position));
        Laser_ParseBytes(s_dma_rx_buffer, write_position);
    }

    s_dma_read_position = normalized_position;
}

static void Laser_ParseBytes(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; ++index) {
        Laser_ParseByte(data[index]);
    }
}

/**
 * @brief 向流式解析器送入一个字节。
 *
 * 注意：这里是在一次 DMA/IDLE 回调中遍历内存，并不是每字节触发一次
 * UART 中断。按字节推进解析器可以自然支持半帧启动和跨 DMA 缓冲区帧。
 */
static void Laser_ParseByte(uint8_t byte)
{
    if (s_frame_length < LASER_FRAME_HEADER_SIZE) {
        if (byte == LASER_FRAME_HEADER_BYTE) {
            s_frame_buffer[s_frame_length] = byte;
            ++s_frame_length;
        } else {
            s_frame_length = 0U;
        }
        return;
    }

    s_frame_buffer[s_frame_length] = byte;
    ++s_frame_length;

    if ((s_frame_length <= LASER_FRAME_PREFIX_SIZE) &&
        !Laser_IsFramePrefixValid()) {
        Laser_ResyncParser();
        return;
    }

    if (s_frame_length == LASER_FRAME_SIZE) {
        if (Laser_ProcessCompleteFrame()) {
            /* 正确帧已完整消费，校验字节不能兼作下一帧帧头。 */
            s_frame_length = 0U;
        } else {
            /* 坏帧可能来自中途接收，保留末尾潜在的新帧头以快速恢复。 */
            Laser_ResyncParser();
        }
    }
}

/**
 * @brief 坏帧后保留末尾连续的 0xAA，防止丢掉下一帧的部分帧头。
 */
static void Laser_ResyncParser(void)
{
    uint16_t header_bytes_to_keep = 0U;

    while ((header_bytes_to_keep < LASER_FRAME_HEADER_SIZE) &&
           (header_bytes_to_keep < s_frame_length) &&
           (s_frame_buffer[s_frame_length - header_bytes_to_keep - 1U] ==
            LASER_FRAME_HEADER_BYTE)) {
        ++header_bytes_to_keep;
    }

    s_frame_length = header_bytes_to_keep;
    while (header_bytes_to_keep > 0U) {
        --header_bytes_to_keep;
        s_frame_buffer[header_bytes_to_keep] = LASER_FRAME_HEADER_BYTE;
    }
}

/**
 * @brief 在固定字段到齐时尽早拒绝非测距帧或损坏帧。
 */
static bool Laser_IsFramePrefixValid(void)
{
    uint16_t payload_length;

    if ((s_frame_length >= 5U) &&
        (s_frame_buffer[4] != LASER_DEVICE_ADDRESS)) {
        return false;
    }

    if ((s_frame_length >= 6U) &&
        (s_frame_buffer[5] != LASER_DISTANCE_COMMAND)) {
        return false;
    }

    if ((s_frame_length >= 7U) &&
        (s_frame_buffer[6] != LASER_CHUNK_OFFSET)) {
        return false;
    }

    if ((s_frame_length >= 8U) &&
        (s_frame_buffer[7] != LASER_CHUNK_OFFSET)) {
        return false;
    }

    if (s_frame_length == LASER_FRAME_PREFIX_SIZE) {
        payload_length = Laser_ReadUint16LittleEndian(&s_frame_buffer[8]);
        if (payload_length != LASER_PAYLOAD_SIZE) {
            return false;
        }
    }

    return true;
}

static bool Laser_ProcessCompleteFrame(void)
{
    float average_distance_mm;

    if (!Laser_IsChecksumValid()) {
        return false;
    }

    if (!Laser_CalculateAverageDistance(&average_distance_mm)) {
        return false;
    }

    /* 中断是唯一写入者；getter 会在关中断临界区内读取这两个变量。 */
    s_latest_distance_mm = average_distance_mm;
    __DMB();
    s_distance_valid = true;
    return true;
}

/**
 * @brief 校验规则与官方例程一致：地址到时间戳逐字节累加，取低 8 位。
 */
static bool Laser_IsChecksumValid(void)
{
    uint16_t index;
    uint8_t checksum = 0U;

    for (index = LASER_CHECKSUM_START_OFFSET;
         index < LASER_CHECKSUM_OFFSET;
         ++index) {
        checksum = (uint8_t)(checksum + s_frame_buffer[index]);
    }

    return (checksum == s_frame_buffer[LASER_CHECKSUM_OFFSET]);
}

/**
 * @brief 对一帧中的 12 个距离点去除 0 值后求均值。
 */
static bool Laser_CalculateAverageDistance(float *distance_mm)
{
    uint16_t point_index;
    uint16_t point_offset;
    uint16_t raw_distance;
    int16_t signed_distance;
    int32_t distance_sum = 0;
    uint8_t valid_point_count = 0U;

    for (point_index = 0U; point_index < LASER_POINT_COUNT; ++point_index) {
        point_offset = (uint16_t)(LASER_FRAME_PREFIX_SIZE +
                                 (point_index * LASER_POINT_SIZE) +
                                 LASER_DISTANCE_LOW_OFFSET);
        raw_distance = Laser_ReadUint16LittleEndian(&s_frame_buffer[point_offset]);
        signed_distance = (int16_t)raw_distance;

        /* 与官方算法保持一致：只剔除距离为 0 的无效点。 */
        if (signed_distance != 0) {
            distance_sum += signed_distance;
            ++valid_point_count;
        }
    }

    if (valid_point_count == 0U) {
        return false;
    }

    *distance_mm = (float)distance_sum / (float)valid_point_count;
    return true;
}

static uint16_t Laser_ReadUint16LittleEndian(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] |
                      ((uint16_t)data[1] << 8U));
}
