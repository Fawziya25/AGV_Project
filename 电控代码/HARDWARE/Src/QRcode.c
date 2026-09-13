#include "QRcode.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>

// 串口触发扫码命令
uint8_t Starscan_CMD[9] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
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
    HAL_UART_Transmit(&huart2, Starscan_CMD, 9, HAL_MAX_DELAY);
}

// 阻塞式等待二维码扫描完成（格式 xxx,xxx,xxx,xxx 共15字节）
void AGV_QRcode_WaitForScan(void)
{
    HAL_UART_Receive(&huart2, QRcode_Buffer, 15, HAL_MAX_DELAY);
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
}