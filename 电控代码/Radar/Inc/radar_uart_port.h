#ifndef __RADAR_UART_PORT_H
#define __RADAR_UART_PORT_H

/*
 * 雷达链路板级适配层（Radar 模块的 HAL 边界）
 * ==========================================
 * 这条链路上层是 radar_client.c，它只调用下面这 8 个函数，不直接碰 HAL。
 * 换 MCU / 换串口时只需要改这个文件和 .c 里的实现。
 *
 * 硬件分工（改动记录.md:668 的串口表）：
 *   UART4 / PC10(TX) / PC11(RX) / 115200 8N1，从 2026-10-03 起就是给雷达预留的空闲口
 *   （CubeMX 已生成 MX_UART4_Init() 与 GPIO，但 .ioc 里没有 DMA/NVIC，
 *   所以 RX DMA 由 radar_uart_port.c 在第一次 StartReceive 时自建）
 *
 * 收发方式：
 *   RX：DMA1_Stream2 / CHANNEL_4，循环模式，关掉半满中断，只靠"写指针"取数据
 *   TX：HAL_UART_Transmit_IT 中断发送（一帧最大 237 字节，115200 下约 21ms）
 *
 * 注意：雷达 TX 是开漏输出，接线时"雷达 PA9 → 本机 PC11"这条线上要一个
 *       4.7kΩ 上拉到 3.3V，且必须共地（说明书 §1）。
 */

#include "usart.h"

#include <stdbool.h>
#include <stdint.h>

#define RADAR_UART_HANDLE (&huart4)

bool RadarUartPort_StartReceive(uint8_t *buffer, uint16_t length);
bool RadarUartPort_ReceiveIsActive(void);
uint16_t RadarUartPort_GetReceivePosition(uint16_t buffer_length);
bool RadarUartPort_StartTransmit(uint8_t *data, uint16_t length);
void RadarUartPort_AbortTransmit(void);
uint32_t RadarUartPort_GetTick(void);
uint32_t RadarUartPort_EnterCritical(void);
void RadarUartPort_ExitCritical(uint32_t previous_primask);

#endif /* __RADAR_UART_PORT_H */
