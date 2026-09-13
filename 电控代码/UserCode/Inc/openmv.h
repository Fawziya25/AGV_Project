#ifndef __OPENMV_H
#define __OPENMV_H

#include <stdint.h>

/**********************************************************
 * OpenMV 视觉模块：串口接收 "block_dx,block_dy,ring_dx,ring_dy,openmv_cmd"
 * 并解析（逗号分隔、换行结尾，无数据显示 -999）
 * 依赖：USART1 + DMA（空闲中断）
 * task 只需读 OpenMV_RxFlag 及各字段变量
 **********************************************************/

/* 1=已收到并解析完一帧新数据；读取后需手动清零 */
extern volatile uint8_t OpenMV_RxFlag;
/* 物块 X/Y 偏差(像素)，-999 表示当前无物块 */
extern volatile int16_t block_dx;
extern volatile int16_t block_dy;
/* 圆环 X/Y 偏差(像素)，-999 表示当前无圆环 */
extern volatile int16_t ring_dx;
extern volatile int16_t ring_dy;
/* OpenMV 命令字段：-1 表示无数据 */
extern volatile int16_t openmv_cmd;

/* 启动串口 DMA 接收（在 AGV_Init 中调用一次） */
void OpenMV_Init(void);

/* OLED 显示全部 5 个视觉参数（调试用，多处可调用） */
void OpenMV_ShowParams(void);

#endif
