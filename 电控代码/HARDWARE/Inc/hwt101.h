#ifndef __HWT101_H
#define __HWT101_H

/**********************************************************
*** HWT101 单轴航向角 IMU（维特智能）驱动程序
*** 通信接口：UART5（115200bps, 8N1, PC12-TX / PD2-RX）
*** 数据输出：0x55 + TYPE + 8字节数据 + SUM（共11字节）
*** 寄存器写：0xFF 0xAA ADDR DATAL DATAH（共5字节）
*** 协议文档：HWT101协议.md
**********************************************************/

#include "main.h"
#include <stdbool.h>

/* 数据帧定义 */
#define HWT101_FRAME_LEN        11U     /* 帧总长度 */
#define HWT101_FRAME_HEAD       0x55U   /* 帧头 */
#define HWT101_FRAME_GYRO       0x52U   /* 角速度帧 */
#define HWT101_FRAME_ANGLE      0x53U   /* 角度帧 */

/* 寄存器地址 */
#define HWT101_REG_SAVE         0x00U   /* 保存/重启/恢复出厂 */
#define HWT101_REG_RRATE        0x03U   /* 输出速率 */
#define HWT101_REG_BAUD         0x04U   /* 串口波特率 */
#define HWT101_REG_KEY          0x69U   /* 解锁 */
#define HWT101_REG_CALIYAW      0x76U   /* Z轴角度归零/设置 */
#define HWT101_REG_MANUALCALI   0xA6U   /* 手动获取零偏 */
#define HWT101_REG_NOAUTOCALI   0xA7U   /* 自动校准开关 */

/* 物理量换算系数 */
#define HWT101_GYRO_SCALE       (2000.0f / 32768.0f)    /* °/s per LSB */
#define HWT101_ANGLE_SCALE      (180.0f / 32768.0f)     /* ° per LSB */

typedef struct
{
    UART_HandleTypeDef *huart;          /* 绑定的串口句柄 */
    uint8_t frame[HWT101_FRAME_LEN];    /* 组帧缓冲 */
    uint8_t frameIndex;                 /* 组帧游标 */
    __IO float yaw;                     /* 航向角（°） */
    __IO float wz;                      /* 校准后Z轴角速度（°/s） */
    __IO float rawWz;                   /* 原始Z轴角速度（°/s） */
    __IO uint16_t version;              /* 固件版本号 */
    __IO bool dataReady;                /* 新数据标志（主循环读取后清除） */
} HWT101_TypeDef;

extern HWT101_TypeDef hwt101;

void HWT101_Init(UART_HandleTypeDef *huart);
void HWT101_FeedByte(uint8_t byte);
void HWT101_WriteReg(uint8_t addr, uint16_t data);
void HWT101_Unlock(void);
void HWT101_Save(void);
void HWT101_SetYawZero(void);
void HWT101_SetYaw(float yawDeg);
void HWT101_SetRate(uint16_t hz);
void HWT101_ManualCalStart(void);
void HWT101_ManualCalStop(void);
void HWT101_SetAutoCal(bool enable);
bool HWT101_IsDataReady(void);
void HWT101_ClearDataReady(void);
float HWT101_GetYaw(void);
float HWT101_GetWz(void);
void HWT101_ManualCal(void);

#endif
