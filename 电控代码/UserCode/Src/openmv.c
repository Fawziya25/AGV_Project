/**
 * @file openmv.c
 * @brief OpenMV 视觉模块：串口接收并解析 5 个字段
 *        数据格式："block_dx,block_dy,ring_dx,ring_dy,openmv_cmd\n"
 *        无数据时各字段为 -999（openmv_cmd 无数据为 -1）
 *        数据源：USART1 + DMA（空闲中断），OpenMV 通过串口发送
 *        调用方只需读 OpenMV_RxFlag 及各字段变量。
 */
#include "openmv.h"
#include "usart.h"
#include "oled.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

volatile uint8_t OpenMV_RxFlag = 0;   /* 1=已收到并解析完一帧新数据 */
volatile int16_t block_dx = -999;     /* 物块X偏差(像素) */
volatile int16_t block_dy = -999;     /* 物块Y偏差(像素) */
volatile int16_t ring_dx  = -999;     /* 圆环X偏差(像素) */
volatile int16_t ring_dy  = -999;     /* 圆环Y偏差(像素) */
volatile int16_t openmv_cmd = -1;     /* OpenMV 命令字段，-1 表示无数据 */

static uint8_t RxBuffer[50];          /* DMA接收缓冲 */
static uint8_t RxParseBuffer[50];     /* 接收缓冲的拷贝，供解析用，防止解析期间被新数据覆盖 */
static uint16_t RxLen = 0;            /* 本帧有效接收长度 */

/**
 * @brief 解析逗号分隔的 5 个整数字段，写回全局变量
 *        格式："block_dx,block_dy,ring_dx,ring_dy,openmv_cmd"，行尾为\r\n
 * @param buf 待解析字符串缓冲区
 * @param len 有效字符数
 * @retval 1 解析成功；0 解析失败(格式错误)
 */
static uint8_t OpenMV_ParseData(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    volatile int16_t *out[5] = { &block_dx, &block_dy, &ring_dx, &ring_dy, &openmv_cmd };

    for(uint8_t field = 0; field < 5; field++)
    {
        int32_t v = 0;
        int8_t sign = 1;
        uint8_t digitCnt = 0;

        /* 符号位 */
        if(i < len && (buf[i] == '-' || buf[i] == '+'))
        {
            if(buf[i] == '-') sign = -1;
            i++;
        }

        /* 数字位，遇逗号/行尾结束 */
        while(i < len && buf[i] != ',' && buf[i] != '\r' && buf[i] != '\n')
        {
            if(buf[i] < '0' || buf[i] > '9')
            {
                return 0;
            }
            v = v * 10 + (buf[i] - '0');
            if(v > 32767 + (sign < 0))
            {
                return 0;   /* 超出int16_t范围 */
            }
            digitCnt = 1;
            i++;
        }
        if(digitCnt == 0)
        {
            return 0;       /* 缺数字 */
        }

        *out[field] = (int16_t)(sign * v);

        /* 前4个字段后必须有逗号；最后一个字段可到行尾 */
        if(field < 4)
        {
            if(i >= len || buf[i] != ',')
            {
                return 0;   /* 缺逗号分隔 */
            }
            i++;            /* 跳过逗号 */
        }
    }
    return 1;
}

/**
 * @brief 启动串口 DMA 接收（在 AGV_Init 中调用一次）
 */
void OpenMV_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, 50);
}

/**
 * @brief OLED 显示全部 5 个视觉参数（调试用，多处可调用）
 *        3 行：B:物块X,Y / R:圆环X,Y / ID:openmv_cmd
 */
void OpenMV_ShowParams(void)
{
    char buf1[32], buf2[32], buf3[32];
    sprintf(buf1, "B:%d,%d", block_dx, block_dy);
    sprintf(buf2, "R:%d,%d", ring_dx,  ring_dy);
    sprintf(buf3, "ID:%d",   openmv_cmd);
    OLED_Clear();
    OLED_ShowString(0, 0,  (uint8_t *)buf1, 16, 0);
    OLED_ShowString(0, 16, (uint8_t *)buf2, 16, 0);
    OLED_ShowString(0, 32, (uint8_t *)buf3, 16, 0);
    OLED_Refresh();
}

/**
 * @brief 串口空闲中断回调：收到一帧数据后解析，任务读取
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        if(Size <= 50)
        {
            for(uint16_t i = 0; i < Size; i++)
            {
                RxParseBuffer[i] = RxBuffer[i];   /* 拷贝后立即重启DMA接收，避免解析期间被覆盖 */
            }
            RxLen = Size;

            if(OpenMV_ParseData(RxParseBuffer, RxLen))
            {
                OpenMV_RxFlag = 1;                /* 解析成功，置标志供任务读取 */
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, 50);
    }
}
