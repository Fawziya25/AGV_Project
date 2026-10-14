#include "radar_uart_port.h"

#include "radar_client.h"   /* RadarClient_OnUartTxCompleteInterrupt / OnUartErrorInterrupt */

/*
 * 雷达串口适配（UART4 + DMA1_Stream2/CH4 循环接收 + 中断发送）
 * ========================================================
 * 为什么 DMA 在这里自建而不是在 usart.c：
 *   本机没装 CubeMX，而 .ioc 里 UART4 只有时钟+GPIO、没有 DMA/NVIC；把初始化写进
 *   Core/Src/usart.c 会被下次 CubeMX 重新生成冲掉。放在本文件里自包含，也符合
 *   hwt101.c 自己实现 UART5_IRQHandler 的既有做法。
 *   如果以后你在 CubeMX 里给 UART4 配了 RX DMA，本文件会检测到 huart4.hdmarx != NULL
 *   并跳过自建（见 RadarUartPort_StartReceive）。
 *
 * 中断优先级取 6：比 TIM7(0)/CAN/USART1(0) 低，和 HWT101 的 UART5 同级，
 * 避免雷达收发压住视觉和电机回帧（改动记录.md:1343-1346 的教训）。
 */

#define RADAR_UART_IRQ_PRIORITY   6U

static DMA_HandleTypeDef s_hdma_uart4_rx;
static bool s_dma_ready;

/* ======================== 内部工具 ======================== */

static bool rx_dma_hardware_enabled(const DMA_HandleTypeDef *dma)
{
    return dma != NULL && dma->Instance != NULL &&
           (dma->Instance->CR & DMA_SxCR_EN) != 0U;
}

static void clear_rx_dma_flags(DMA_HandleTypeDef *dma)
{
    uint32_t flags = __HAL_DMA_GET_TC_FLAG_INDEX(dma) |
                     __HAL_DMA_GET_HT_FLAG_INDEX(dma) |
                     __HAL_DMA_GET_TE_FLAG_INDEX(dma) |
                     __HAL_DMA_GET_DME_FLAG_INDEX(dma) |
                     __HAL_DMA_GET_FE_FLAG_INDEX(dma);
    __HAL_DMA_CLEAR_FLAG(dma, flags);
}

static bool force_rx_dma_reinit(UART_HandleTypeDef *uart,
                                DMA_HandleTypeDef *dma)
{
    CLEAR_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    (void)HAL_DMA_Abort(dma);
    __HAL_DMA_DISABLE(dma);
    if (rx_dma_hardware_enabled(dma)) return false;

    /* 硬件已停止时才修正可能滞留在BUSY/ABORT/ERROR的HAL状态。 */
    if (HAL_DMA_GetState(dma) == HAL_DMA_STATE_BUSY)
        dma->State = HAL_DMA_STATE_READY;
    if (HAL_DMA_DeInit(dma) != HAL_OK || HAL_DMA_Init(dma) != HAL_OK)
        return false;

    uart->RxState = HAL_UART_STATE_READY;
    uart->RxXferCount = 0U;
    uart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
    return true;
}

/*
 * 自建 UART4 的 RX DMA 与两个 NVIC：
 *   DMA1_Stream2 / CHANNEL_4 = UART4_RX（F407 的固定映射）
 *   TX 用中断发送，所以不需要第二个 DMA 流，只需要 UART4 自己的中断。
 * 只执行一次（s_dma_ready）且只在 huart4.hdmarx 为空时执行。
 */
static bool ensure_dma_ready(void)
{
    UART_HandleTypeDef *uart = RADAR_UART_HANDLE;

    if (uart->hdmarx != NULL) return true;      /* CubeMX 已配好，直接用 */
    if (s_dma_ready) return true;

    __HAL_RCC_DMA1_CLK_ENABLE();

    s_hdma_uart4_rx.Instance = DMA1_Stream2;
    s_hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    s_hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    s_hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    s_hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    s_hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    s_hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    s_hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
    s_hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_hdma_uart4_rx) != HAL_OK) return false;

    __HAL_LINKDMA(uart, hdmarx, s_hdma_uart4_rx);

    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, RADAR_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_SetPriority(UART4_IRQn, RADAR_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    s_dma_ready = true;
    return true;
}

/* ======================== 对外接口 ======================== */

bool RadarUartPort_StartReceive(uint8_t *buffer, uint16_t length)
{
    UART_HandleTypeDef *uart = RADAR_UART_HANDLE;
    DMA_HandleTypeDef *dma;
    HAL_StatusTypeDef status;
    bool receive_dirty;

    if (buffer == NULL || length == 0U) return false;
    if (!ensure_dma_ready()) return false;

    dma = uart->hdmarx;
    if (dma == NULL) return false;
    if (dma->Init.Mode != DMA_CIRCULAR) return false;

    receive_dirty = uart->RxState != HAL_UART_STATE_READY ||
                    HAL_DMA_GetState(dma) != HAL_DMA_STATE_READY ||
                    HAL_DMA_GetError(dma) != HAL_DMA_ERROR_NONE ||
                    (uart->Instance->CR3 & USART_CR3_DMAR) != 0U ||
                    rx_dma_hardware_enabled(dma);

    if (receive_dirty && HAL_UART_AbortReceive(uart) != HAL_OK) {
        if (!force_rx_dma_reinit(uart, dma)) return false;
    } else if (HAL_DMA_GetState(dma) != HAL_DMA_STATE_READY ||
               HAL_DMA_GetError(dma) != HAL_DMA_ERROR_NONE ||
               (uart->Instance->CR3 & USART_CR3_DMAR) != 0U ||
               rx_dma_hardware_enabled(dma)) {
        if (!force_rx_dma_reinit(uart, dma)) return false;
    }

    /* 读取SR/DR一次会同时清除PE、FE、NE和ORE。 */
    __HAL_UART_CLEAR_PEFLAG(uart);
    clear_rx_dma_flags(dma);
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    dma->ErrorCode = HAL_DMA_ERROR_NONE;

    status = HAL_UART_Receive_DMA(uart, buffer, length);
    if (status == HAL_OK)
        __HAL_DMA_DISABLE_IT(dma, DMA_IT_HT);   /* 循环接收不需要半满中断 */
    return status == HAL_OK;
}

bool RadarUartPort_ReceiveIsActive(void)
{
    DMA_HandleTypeDef *dma = RADAR_UART_HANDLE->hdmarx;
    return dma != NULL &&
           RADAR_UART_HANDLE->RxState == HAL_UART_STATE_BUSY_RX &&
           HAL_DMA_GetState(dma) == HAL_DMA_STATE_BUSY &&
           HAL_DMA_GetError(dma) == HAL_DMA_ERROR_NONE &&
           (RADAR_UART_HANDLE->Instance->CR3 & USART_CR3_DMAR) != 0U &&
           rx_dma_hardware_enabled(dma);
}

uint16_t RadarUartPort_GetReceivePosition(uint16_t buffer_length)
{
    uint16_t position;
    if (RADAR_UART_HANDLE->hdmarx == NULL || buffer_length == 0U) return 0U;
    position = (uint16_t)(buffer_length -
               __HAL_DMA_GET_COUNTER(RADAR_UART_HANDLE->hdmarx));
    return position >= buffer_length ? 0U : position;
}

/* 一帧一次，非阻塞；发完由 HAL_UART_TxCpltCallback 通知客户端继续下一帧。 */
bool RadarUartPort_StartTransmit(uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) return false;
    if (!ensure_dma_ready()) return false;      /* 确保 UART4_IRQn 已使能 */
    return HAL_UART_Transmit_IT(RADAR_UART_HANDLE, data, length) == HAL_OK;
}

void RadarUartPort_AbortTransmit(void)
{
    (void)HAL_UART_AbortTransmit(RADAR_UART_HANDLE);
}

uint32_t RadarUartPort_GetTick(void)
{
    return HAL_GetTick();      /* 必须是毫秒；客户端所有超时都基于它 */
}

uint32_t RadarUartPort_EnterCritical(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

void RadarUartPort_ExitCritical(uint32_t previous_primask)
{
    if (previous_primask == 0U) __enable_irq();
}

/* ======================== 中断入口 ========================
 * 按 hwt101.c 的做法由本模块自己实现中断服务函数：
 *   UART4_IRQHandler      —— 发送完成 + 各类错误（PE/FE/NE/ORE）
 *   DMA1_Stream2_IRQHandler —— RX DMA 的传输错误（循环模式下不会用 TC/HT）
 * 中断里只让 HAL 登记标志，恢复和重挂都留给主循环的 RadarClient_Poll()。
 */

void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(RADAR_UART_HANDLE);
}

void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_uart4_rx);
}

/* HAL 的完成/错误回调：项目里目前只有雷达用这两个，直接在这里分发。
   以后别的外设也要用，就把这里改成先判断句柄再转调各自模块。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == RADAR_UART_HANDLE) RadarClient_OnUartTxCompleteInterrupt();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == RADAR_UART_HANDLE) RadarClient_OnUartErrorInterrupt(huart->ErrorCode);
}
