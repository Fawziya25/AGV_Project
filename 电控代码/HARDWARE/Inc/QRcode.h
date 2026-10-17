#ifndef __QRCODE_H__
#define __QRCODE_H__

#include "main.h"
#include "usart.h"
#include "oled.h"

/* 开 USART3 接收中断：开机尽早调用一次。
   扫码模块一直开着，从这时起收到的字节都会被拼帧并锁存，
   之后 AGV_QRcode_WaitForScan() 先查缓存，已经扫到就不阻塞。 */
void AGV_QRcode_Init(void);

/* 中断回调转发（HAL_UART_RxCpltCallback 里按 USART3 分发过来），一般不用外部调用 */
void QRcode_RxISR(UART_HandleTypeDef *huart);

void AGV_QRcode_StartScan(void);
void AGV_QRcode_WaitForScan(void);
void QRcode_Task(void);

// 二维码扫描结果缓冲区（15字节数据 + 1字节'\0'结束符）
extern uint8_t QRcode_Buffer[16];
// 第一轮抓取物料的颜色顺序
extern uint8_t Pick_Color1[3];
// 第一轮物料放置顺序
extern uint8_t Place_Color1[3];
// 第二轮抓取物料的颜色顺序
extern uint8_t Pick_Color2[3];
// 第二轮物料放置顺序
extern uint8_t Place_Color2[3];

#endif