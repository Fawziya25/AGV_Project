#include "QRcode.h"
#include "camera.h"             /* Cam_SwitchTo / CAM_CMD_TASKCODE */
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>

// 串口触发扫码命令
uint8_t Starscan_CMD[9] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

/* 扫码等待上限(秒): 到点还没收全15字节就用保底串, 不让主流程卡在扫码上 */
#define QRC_WAIT_SEC  1U
/* 一帧的字节数: xxx+xxx+xxx+xxx */
#define QRC_LEN       15U
/* 保底结果: 15秒内没扫到码时整串换成它, 保证 Pick/Place_Color 一定有值可用。
   位置分隔符解析时不校验(QRcode_ParseResult 只按下标取值), '+' 和 ',' 效果一样;
   内容按 123 循环, 方便在 OLED 上一眼认出"这次是保底值"。423 */
static const uint8_t QRC_FALLBACK[16] = "213+213+213+213";
// 二维码扫描结果缓冲区（15字节数据 + 1字节'\0'结束符）
uint8_t QRcode_Buffer[16];
// 第一轮抓取物料的颜色顺序
uint8_t Pick_Color1[3];
// 第一轮物料放置顺序
uint8_t Place_Color1[3];
// 第二轮抓取物料的颜色顺序
uint8_t Pick_Color2[3];
// 第二轮物料放置顺序
uint8_t Place_Color2[3];

void QRcode_ParseResult(void);

/* ================= 扫码接收：开机后全程中断收 =================
 * 扫码模块一直开着，可能在 AGV_QRcode_WaitForScan() 被调用之前就把码发完了。
 * 原来 WaitForScan() 自己调 HAL_UART_Receive 读串口 —— 来晚一步，字节早就丢了，
 * 只能干等 15 秒超时、走保底串。
 * 现在 USART3 从开机起就逐字节中断接收，收到完整一帧先锁存到 s_readyBuf；
 * WaitForScan() 只查 s_ready：之前已经扫到就立刻用，没扫到才阻塞等。
 */
static uint8_t          qrc_rxByte;             /* 中断接收的单字节落点 */
static uint8_t          s_win[QRC_LEN];         /* 滑动窗口：最近 15 个字节 */
static uint8_t          s_n;                    /* 窗口里已有多少字节 */
static uint8_t          s_readyBuf[QRC_LEN];    /* 最近一次完整帧 */
static volatile uint8_t s_ready;                /* 1 = s_readyBuf 里有可用的码 */

/* 校验 15 字节是否成一帧：'+' 固定在 3/7/11，其余位置必须是数字 */
static uint8_t QRC_IsFrame(const uint8_t *f)
{
    for(uint8_t i = 0U; i < QRC_LEN; i++)
    {
        if((i == 3U) || (i == 7U) || (i == 11U))
        {
            if(f[i] != (uint8_t)'+')
            {
                return 0U;
            }
        }
        else if((f[i] < (uint8_t)'0') || (f[i] > (uint8_t)'9'))
        {
            return 0U;
        }
    }
    return 1U;
}

/* 中断里每收到一个字节调用。
   用滑动窗口而不是"从头数满 15 个"：扫码模块可能连续重发，也可能我们开机晚了
   漏掉开头几个字节，滑动窗口能自己重新对齐，不会一步错步步错。 */
static void QRC_FeedByte(uint8_t b)
{
    uint8_t i;

    if(s_n < QRC_LEN)
    {
        s_win[s_n++] = b;
    }
    else
    {
        for(i = 1U; i < QRC_LEN; i++)
        {
            s_win[i - 1U] = s_win[i];
        }
        s_win[QRC_LEN - 1U] = b;
    }

    if((s_n == QRC_LEN) && QRC_IsFrame(s_win))
    {
        for(i = 0U; i < QRC_LEN; i++)
        {
            s_readyBuf[i] = s_win[i];
        }
        s_ready = 1U;
        s_n = 0U;       /* 本帧已锁存，后面的重复帧从头重新拼 */
    }
}

/* 中断回调里调用（hwt101.c 的 HAL_UART_RxCpltCallback 分发过来）：
   喂一个字节，然后立刻重新挂上接收 */
void QRcode_RxISR(UART_HandleTypeDef *huart)
{
    QRC_FeedByte(qrc_rxByte);
    HAL_UART_Receive_IT(huart, &qrc_rxByte, 1U);
}

/* USART3 中断入口（startup_stm32f407xx.s 里是弱定义，这里覆盖） */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

/* 开串口 3 的接收中断。开机尽早调一次（main 里 MX_USART3_UART_Init 之后），
   之后扫码模块发的所有字节都有人接，不再依赖 WaitForScan 的调用时机。 */
void AGV_QRcode_Init(void)
{
    s_n     = 0U;
    s_ready = 0U;

    /* 先挂接收、再开 NVIC。顺序反过来（先开中断再 Receive_IT）的话，中间来一个字节
       会让 RXNE 一直挂着、而 HAL 还没进 BUSY_RX 状态，回调里清不掉标志 → 中断风暴。
       HAL_UART_Receive_IT 内部会自己使能 RXNE 中断，不用再手工 __HAL_UART_ENABLE_IT。 */
    HAL_UART_Receive_IT(&huart3, &qrc_rxByte, 1U);
    HAL_NVIC_SetPriority(USART3_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

// 开始扫描二维码
void AGV_QRcode_StartScan(void)
{
    HAL_UART_Transmit(&huart3, Starscan_CMD, 9, HAL_MAX_DELAY);
}

// 等二维码（格式 xxx+xxx+xxx+xxx 共15字节）
// **先查有没有已经扫到的码**：调用之前中断就收到完整帧的话，这里直接取用、不等；
// 没有才阻塞等，最多 QRC_WAIT_SEC 秒（期间 OLED 显示倒计时），到点仍没有就用保底串。
void AGV_QRcode_WaitForScan(void)
{
    uint8_t  i;
    uint32_t t0    = HAL_GetTick();
    uint32_t shown = QRC_WAIT_SEC + 1U;   /* 故意与第一拍不等，保证首帧必画 */

    /* 中断一直在收，这里 10ms 查一次标志：收到完整帧最多晚 10ms 返回。
       调用前就已经扫到的话，这个循环一次都不会进。 */
    while((s_ready == 0U) && ((HAL_GetTick() - t0) < (QRC_WAIT_SEC * 1000U)))
    {
        uint32_t left = QRC_WAIT_SEC - (HAL_GetTick() - t0) / 1000U;
        if(left != shown)
        {
            shown = left;
            OLED_ShowString(0, 0, (uint8_t *)"Scan QR...", 16, 1);
            OLED_ShowNum(0, 16, left, 2U, 16U, 1);
            OLED_Refresh();
        }
        HAL_Delay(10);
    }

    if(s_ready != 0U)       /* 已扫到（不管是刚才还是调用之前）: 直接取用 */
    {
        for(i = 0U; i < QRC_LEN; i++)
        {
            QRcode_Buffer[i] = s_readyBuf[i];
        }
        /* 不清标志：留着最后一次扫到的码，之后再调用 WaitForScan 也直接跳过。
           扫码模块发来新码时 s_readyBuf 会被覆盖，不用管。 */
    }
    else                    /* 等满 QRC_WAIT_SEC 还没有: 整串换成保底值 */
    {
        for(i = 0U; i < QRC_LEN; i++)
        {
            QRcode_Buffer[i] = QRC_FALLBACK[i];
        }
    }
    QRcode_Buffer[QRC_LEN] = '\0';   /* 补字符串结束符，保证可作C字符串使用 */
    QRcode_ParseResult();
}

// 解析二维码扫描结果
void QRcode_ParseResult(void)
{
    for (int i = 0; i < 3; i++)
    {
        Pick_Color1[i] = QRcode_Buffer[i] - '0';
    }
    for(int i = 4; i < 7; i++)
    {
        Place_Color1[i - 4] = QRcode_Buffer[i] - '0';
    }


    for(int i = 8; i < 11; i++)
    {
        Pick_Color2[i - 8] = QRcode_Buffer[i] - '0';
    }
    for(int i = 12; i < 15; i++)
    {
        Place_Color2[i - 12] = QRcode_Buffer[i] - '0';
    }
}

/**
 * @brief 扫码 + 显示 + 把第一轮任务码下发给相机
 * @note  扫码结果是整条流程的输入：这里扫完就顺手把任务码发出去，
 *        调用方（task.c 的 case QRcode）不用再单独发一次。
 *        下发是阻塞的，等相机回 0x01 的 flag=1(任务码已收)才算完；超时 10s 就往下走。
 *        返回值当前未判：相机没收到任务码的话后面所有颜色都对不上且不会报错，
 *        要收紧就在这儿接返回值。第二轮任务码 Pick_Color2 仍未下发（见改动记录待办 9）。
 */
void QRcode_Task(void)
{
    OLED_Clear();
    /* 扫码模块一直开着，不再发 AGV_QRcode_StartScan() 触发；
       码可能在我们走到这一步之前就发完了，WaitForScan() 会先查已经收到的缓存。 */
    AGV_QRcode_WaitForScan();
    uint8_t sep = QRcode_Buffer[8];
    QRcode_Buffer[8] = '\0';
    OLED_ShowString(0, 0, (uint8_t *)QRcode_Buffer, 32, 1);
    QRcode_Buffer[8] = sep;
    OLED_ShowString(0, 32, (uint8_t *)&QRcode_Buffer[8], 32, 1);
    OLED_Refresh();

    Cam_SwitchTo(01);   /* 0x01 任务码：三色不用传，Cam_SwitchTo 自己从 Pick_Color1 取 */
}