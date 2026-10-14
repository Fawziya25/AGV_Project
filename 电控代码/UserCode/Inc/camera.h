#ifndef __CAM_H
#define __CAM_H

#include <stdint.h>

/**********************************************************
 * 相机视觉模块：二进制定长帧协议（2026-10-03 新版）
 * （原名 openmv.h；函数前缀 Cam_，宏前缀 CAM_）
 *
 *   帧格式：AA <cmd> <payload...> BB      （AA 帧头，BB 帧尾）
 *
 *   电控 -> 相机（命令）：
 *     0x00  AA 00 BB                 空闲
 *     0x01  AA 01 c1 c2 c3 BB        下发任务码（任务码前三个数字）
 *     0x02  AA 02 BB                 请求圆盘中心位置偏差
 *     0x03  AA 03 BB                 请求三个框内颜色
 *     0x04  AA 04 BB                 请求圆环
 *     0x05  AA 05 color BB           请求指定颜色 color
 *
 *   相机 -> 电控（循环回，flag=0 未完成、flag!=0 完成）：
 *     0x00  AA 00 flag BB                     flag 随意
 *     0x01  AA 01 flag BB                     flag=1 任务码已收
 *     0x02  AA 02 flag dxH dxL dyH dyL BB     flag=1 圆盘中心已识别
 *     0x03  AA 03 flag c1 c2 c3 BB            flag=1 三框从无到有
 *     0x04  AA 04 flag dxH dxL dyH dyL BB     flag=圆环编号(1/2/3)，0=未识别
 *     0x05  AA 05 flag dxH dxL dyH dyL BB     flag=1 识别到对应颜色
 *
 *   dx / dy：有符号 int16，高字节在前
 *   依赖：USART1 + DMA（空闲中断）
 *
 *   请求式 API 的等待分两段（见 camera.c Cam_WaitSettle），0x01~0x05 全走同一条路：
 *     第一段 等相机回一帧「第二字节(命令码) == 刚发出去的命令」，确认命令被采用；
 *            每 100ms 没动静就补发一次，10s 超时失败。相机在任何模式下都会持续回数据，
 *            所以"收到帧"不能当握手成功，必须按命令码匹配。
 *     第二段 等这同一条命令回 flag!=0，即相机把活干完；超时由调用方传，也是 10s。
 **********************************************************/

/* ========================== 协议常量 ========================== */
#define CAM_HEAD        0xAAU
#define CAM_TAIL        0xBBU

#define CAM_CMD_IDLE        0x00U    /* 空闲 */
#define CAM_CMD_TASKCODE    0x01U    /* 下发任务码 */
#define CAM_CMD_DISC        0x02U    /* 请求圆盘中心 */
#define CAM_CMD_BOXCOLOR    0x03U    /* 请求三框颜色 */
#define CAM_CMD_RING        0x04U    /* 请求圆环 */
#define CAM_CMD_COLOR       0x05U    /* 请求指定颜色 */

/* 请求式 API 的 settle_ms 统一用这个：第二段(等相机回 flag!=0)的等待上限。
   第一段(等相机回报本命令)固定 10s，在 camera.c 里，不受这里影响 —— 两段都是 10s。
   调用方想单独调某一条的耐心就自己传别的值。 */
#define CAM_REQ_TMO_MS  10000U

/* ========================== 解析结果 ==========================
   每收到一帧合法帧就整体刷新一次；cam_receive_flag 是"有新帧待读"的握手位，
   读取方（等待函数 / AGV_Correct 轮询）读完自行清零。 */
/* 最近一帧的 X / Y 偏差(像素)，仅 0x02 / 0x04 帧会更新；初值 0。
   注意 dx/dy 没有"无效值"哨兵 —— 有没有识别到目标一律看 cam_flag，
   cam_flag==0 时 dx/dy 无意义、不该拿去闭环。 */
extern volatile int16_t dx;
extern volatile int16_t dy;
/* 最近一帧里的 flag 字段（0x04 时即圆环编号 1/2/3）；0 = 没识别到目标 */
extern volatile uint8_t cam_flag;
/* 最近一帧的命令码，0xFF = 还没收到过任何帧 */
extern volatile uint8_t cam_mode;
/* 1=已解析完一帧新帧，读取方自行清零 */
extern volatile uint8_t cam_receive_flag;
/* 0x03 帧回报的三个框内颜色（c1 c2 c3） */
extern volatile uint8_t box_color[3];

/* OpenMV 启动成功标志：1=Cam_Init 已建立链路 */
extern volatile uint8_t cam_start;

/* ========================== 初始化 / 维护 ========================== */
/* 启动串口 DMA 接收并发一次空闲命令探链路（在 AGV_Init 中调用一次） */
void Cam_Init(void);
/* 接收链自愈：周期性调用（主循环/调试循环每 100ms 一次）。
   串口错误在 DMA 接收模式下会被 HAL 中止接收且不会自动重启，必须靠它恢复。 */
void Cam_Service(void);
/* OLED 显示最近一帧的 cmd/flag 与偏差（调试用） */

/* ========================== 新协议 API ========================== */
/* 发送空闲命令（**不等待返回**：它是上电探链路的，链路还没通时等不了） */
void Cam_SendIdle(void);

/* 发送一条命令帧、**不等回复**（AA <cmd> <payload...> BB）。
   payload 传 NULL 表示无参数；plen：0x01 传 3、0x05 传 1、其余传 0。
   相机被唤醒后会持续回发当前模式的数据，所以发完直接看 cam_mode / cam_flag /
   dx / dy / box_color[] 即可。给"每 N 秒轮询一遍所有命令"这类扫描测试用 ——
   Cam_RequestXxx() 那套是阻塞的（一条最多卡 10s+settle_ms），轮询用不了。 */
void Cam_SendCommand(uint8_t cmd, const uint8_t *payload, uint8_t plen);

/* 以下五个请求式 API 的返回值含义统一：
     1 = 两段都过(命令被采用 且 相机回了 flag!=0)，出参有效；
     0 = 第一段(10s 内没收到本命令的回帧)或第二段(settle_ms 内没等到 flag!=0)超时，
         此时出参**没有意义**，调用方必须判返回值，别拿旧值往下走。
   settle_ms 只作用于第二段(等相机干完活)，第一段固定 10s。 */

/* 下发任务码三色，等相机回 0x01 的 flag=1(任务码已收)。无出参，返回值即结果 */
uint8_t Cam_RequestTaskCode(uint8_t c1, uint8_t c2, uint8_t c3, uint32_t settle_ms);
/* 请求圆盘中心偏差。成功(flag=1)返回 1 并写出 dx/dy */
uint8_t Cam_RequestDisc(int16_t *px, int16_t *py, uint32_t settle_ms);
/* 请求三个框内颜色。成功(flag=1)返回 1 并写出 c1/c2/c3 */
uint8_t Cam_RequestBoxColor(uint8_t *c1, uint8_t *c2, uint8_t *c3, uint32_t settle_ms);
/* 请求圆环。成功返回 1，*ring=圆环编号(1/2/3)、px/py=偏差 */
uint8_t Cam_RequestRing(int16_t *px, int16_t *py, uint8_t *ring, uint32_t settle_ms);
/* 请求指定颜色(0x05)。成功(flag=1)返回 1 并写出 dx/dy。
   color 用任务码里的颜色编码(1~5 等)，0 不是有效颜色号。 */
uint8_t Cam_RequestColor(uint8_t color, int16_t *px, int16_t *py, uint32_t settle_ms);

/* 旧 ASCII 协议的兼容层(Cam_SwitchTo / Cam_SwitchCloseColor / Cam_WaitFor*)
   已随 pick_task.c 重写一并删除，调用方改用上面的请求式 API。 */
#endif
