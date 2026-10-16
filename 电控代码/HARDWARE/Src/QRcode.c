#include "QRcode.h"
#include "camera.h"             /* Cam_SwitchTo / CAM_CMD_TASKCODE */
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>

// 串口触发扫码命令
uint8_t Starscan_CMD[9] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

/* 扫码等待上限(秒): 到点还没收全15字节就用保底串, 不让主流程卡在扫码上 */
#define QRC_WAIT_SEC  15U
/* 保底结果: 15秒内没扫到码时整串换成它, 保证 Pick/Place_Color 一定有值可用。
   位置分隔符解析时不校验(QRcode_ParseResult 只按下标取值), '+' 和 ',' 效果一样;
   内容按 123 循环, 方便在 OLED 上一眼认出"这次是保底值"。423 */
static const uint8_t QRC_FALLBACK[16] = "542+312+452+123";
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

// 开始扫描二维码
void AGV_QRcode_StartScan(void)
{
    HAL_UART_Transmit(&huart3, Starscan_CMD, 9, HAL_MAX_DELAY);
}

// 阻塞式等待二维码扫描完成（格式 xxx,xxx,xxx,xxx 共15字节）
// 最多等 QRC_WAIT_SEC 秒: 期间 OLED 显示倒计时(第2行), 收全15字节立刻返回;
// 超时或只收到半截 → 整串换成 QRC_FALLBACK, 再照常解析
void AGV_QRcode_WaitForScan(void)
{
    uint8_t got = 0U;       /* 已收到的字节数 */

    for(uint8_t sec = QRC_WAIT_SEC; (sec > 0U) && (got < 5U); sec--)
    {
        OLED_ShowString(0, 0, (uint8_t *)"Scan QR...", 16, 1);
        OLED_ShowNum(0, 16, (uint32_t)sec, 2U, 16U, 1);
        OLED_Refresh();

        /* 逐字节收(不是一次要15字节): "收了几字节"明确, 秒到点能干净退出。
           9600下字节间隔约1ms, 远大于两次调用之间几十us的取tick开销,
           不会漏字节; 两次OLED_Refresh之间的盲区由"超时就用保底串"兜底。 */
        uint32_t tSec = HAL_GetTick();
        while(((HAL_GetTick() - tSec) < 1000U) && (got < 15U))
        {
            uint32_t left = 1000U - (HAL_GetTick() - tSec);
            if(left == 0U)
            {
                break;
            }
            if(HAL_UART_Receive(&huart3, &QRcode_Buffer[got], 1U, left) == HAL_OK)
            {
                got++;
            }
        }
    }

    if(got < 15U)       /* 超时或只收到半截: 整串换成保底值 */
    {
        for(uint8_t i = 0U; i < 15U; i++)
        {
            QRcode_Buffer[i] = QRC_FALLBACK[i];
        }
    }
    QRcode_Buffer[15] = '\0';   /* 补字符串结束符，保证可作C字符串使用 */
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

    // 校赛暂时不解析第二轮颜色顺序

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
    AGV_QRcode_StartScan();
    AGV_QRcode_WaitForScan();
    uint8_t sep = QRcode_Buffer[8];
    QRcode_Buffer[8] = '\0';
    OLED_ShowString(0, 0, (uint8_t *)QRcode_Buffer, 32, 1);
    QRcode_Buffer[8] = sep;
    OLED_ShowString(0, 32, (uint8_t *)&QRcode_Buffer[8], 32, 1);
    OLED_Refresh();

    Cam_SwitchTo(01);   /* 0x01 任务码：三色不用传，Cam_SwitchTo 自己从 Pick_Color1 取 */
}