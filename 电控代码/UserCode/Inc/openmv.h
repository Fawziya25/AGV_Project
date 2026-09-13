#ifndef __OPENMV_H
#define __OPENMV_H

#include <stdint.h>

/**********************************************************
 * OpenMV 视觉模块：串口接收 "target_dx,target_dy,openmv_cmd"
 * 并解析（逗号分隔、换行结尾，无目标 -999，cmd 空闲 -1）
 * 目标(物块/圆环)由 OpenMV 自动判定后上报
 * 依赖：USART1 + DMA（空闲中断）
 * task 只需读 OpenMV_RxFlag 及各字段变量
 **********************************************************/

/* 1=已收到并解析完一帧新数据；读取后需手动清零 */
extern volatile uint8_t OpenMV_RxFlag;
/* 目标 X/Y 偏差(像素)，-999 表示当前无目标(物块/圆环由OpenMV判定) */
extern volatile int16_t target_dx;
extern volatile int16_t target_dy;
/* OpenMV 命令字段：-1 空闲；颜色命令后回显当前颜色号 */
extern volatile int16_t openmv_cmd;
/* OpenMV 启动成功标志：1=OpenMV_Init 已收到首帧数据 */
extern volatile uint8_t openmv_start;

/* 启动串口 DMA 接收（在 AGV_Init 中调用一次） */
void OpenMV_Init(void);

/* 接收链自愈：周期性调用（主循环/调试循环每100ms一次）。
   串口错误在DMA接收模式下会被HAL中止接收且不会自动重启，必须靠它恢复，
   否则 openmv_cmd 可能永远停在 -1。 */
void OpenMV_Service(void);

/* OLED 显示视觉参数 target_dx/dy 与 openmv_cmd（调试用） */
void Openmv_OLED_Show(void);

void OpenMV_WaitForStill(void);
void OpenMV_WaitForNearStill(void);

void OpenMV_SwitchCloseColor(void);
/* 发送颜色切换命令给OpenMV并等待回显确认(成功返回1,3s超时返回0) */
uint8_t OpenMV_SwitchTo(uint8_t color);

#endif
