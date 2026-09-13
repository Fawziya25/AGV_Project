#ifndef __QRCODE_H__
#define __QRCODE_H__

#include "main.h"
#include "usart.h"
#include "oled.h"

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