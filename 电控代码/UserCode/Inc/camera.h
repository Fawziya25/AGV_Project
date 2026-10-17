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
 *     0x02  AA 02 BB                 请求最近物块的位置偏差和边沿判断
 *     0x03  AA 03 BB                 请求三个框内颜色
 *     0x04  AA 04 BB                 请求圆环
 *     0x05  AA 05 color BB           请求指定颜色 color
 *
 *   相机 -> 电控（循环回，flag=0 未完成、flag!=0 完成）：
 *     0x00  AA 00 flag BB                     flag 随意
 *     0x01  AA 01 flag BB                     flag=1 任务码已收
 *     0x02  AA 02 flag dxH dxL dyH dyL BB     flag=最近物块的颜色编号(1~6)，
 *                                             边沿静止时才发（flag!=0 即"已静止"）
 *     0x03  AA 03 flag c1 c2 c3 BB            flag=1 三框从无到有
 *     0x04  AA 04 flag dxH dxL dyH dyL BB     flag=圆环编号(1/2/3)，0=未识别
 *     0x05  AA 05 flag dxH dxL dyH dyL BB     flag=1 识别到对应颜色
 *
 *   ⚠ 0x02 在 2026-10-16 改过语义：原来报的是"圆盘中心"、flag 只有 0/1，
 *     现在报的是"最近物块"、flag 就是那个物块的颜色号 1~6（见工程根目录的
 *     视觉-电控通信.txt）。所以收到 0x02 的 flag!=0 只代表"物块边缘已经静止"，
 *     **不代表颜色就是要抓的那一色** —— 颜色得自己拿 flag 去比对。
 *     0x03（三框颜色）仍保留，Pick_RAW_PLUS 那条备用路线还在用。
 *
 *   dx / dy：有符号 int16，高字节在前
 *   依赖：USART1 + DMA（空闲中断）
 *
 *   请求式 API 的等待分两段（见 camera.c Cam_WaitSettle），0x01~0x05 全走同一条路：
 *     第一段 等相机回一帧「第二字节(命令码) == 刚发出去的命令」，确认命令被采用；
 *            每 100ms 没动静就补发一次，10s 超时失败。相机在任何模式下都会持续回数据，
 *            所以"收到帧"不能当握手成功，必须按命令码匹配。
 *     第二段 等这同一条命令回 flag!=0，即相机把活干完；超时 CAM_REQ_TMO_MS(20s)。
 *           两句都不由调用方传：Cam_SwitchTo() 只挑模式，耐心是 camera 模块自己的事。
 **********************************************************/

/* ========================== 协议常量 ========================== */
#define CAM_HEAD        0xAAU
#define CAM_TAIL        0xBBU

#define CAM_CMD_IDLE        0x00U    /* 空闲 */
#define CAM_CMD_TASKCODE    0x01U    /* 下发任务码 */
#define CAM_CMD_DISC        0x02U    /* 请求最近物块位置偏差 + 边沿判断(flag=颜色号 1~6) */
#define CAM_CMD_BOXCOLOR    0x03U    /* 请求三框颜色 */
#define CAM_CMD_RING        0x04U    /* 请求圆环 */
#define CAM_CMD_COLOR       0x05U    /* 请求指定颜色 */

/* 请求式等待【第二段】(等相机回 flag!=0)的等待上限，20s。
   只有 camera.c 的 Cam_WaitSettle() 在用，**不是参数** —— 调用方(Cam_SwitchTo)不用传。
   第一段(等相机回报本命令)是 CAM_ACK_TMO_MS=10s，写在 camera.c 里。
   两段加起来最坏 30s：等回帧 10s + 等相机把活干完 20s。
   ⚠ 改这个数只影响"等相机干完活"这一段；相机没接时每条命令卡的最长时间也由它决定。 */
#define CAM_REQ_TMO_MS  20000U

/* ========================== 解析结果 ==========================
   每收到一帧合法帧就整体刷新一次；cam_receive_flag 是"有新帧待读"的握手位，
   读取方（等待函数 / AGV_Correct 轮询）读完自行清零。 */
/* 最近一帧的 X / Y 偏差(像素)，仅 0x02 / 0x04 帧会更新；初值 0。
   注意 dx/dy 没有"无效值"哨兵 —— 有没有识别到目标一律看 cam_flag，
   cam_flag==0 时 dx/dy 无意义、不该拿去闭环。 */
extern volatile int16_t dx;
extern volatile int16_t dy;
/* 最近一帧里的 flag 字段：0x02 时即最近物块的颜色号(1~6)、0x04 时即圆环编号(1/2/3)；
   0 = 没识别到目标。⚠ 这是「最近一帧」的值，会被紧随其后的 flag=0 帧冲掉，
   所以只在 Cam_SwitchTo() 刚返回的那一刻读它才有意义（见 camera.h 末尾的说明）。 */
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
   Cam_SwitchTo() 那套是阻塞的（一条最多卡 10s+CAM_REQ_TMO_MS=30s），轮询用不了。 */
void Cam_SendCommand(uint8_t cmd, const uint8_t *payload, uint8_t plen);

/* ========================== 统一切换相机模式 ==========================
   六个模式就一条命令，参数只有模式号；带参数的两条(0x01 / 0x05)由本函数自己
   从任务码里取，调用方不用管 —— 原来那五个 Cam_RequestXxx() 已删掉。

   @param mode 直接写命令码的**八进制字面量**：01/02/03/04/05（= 十六进制 0x01~0x05）。
          ⚠ 必须带前导 0 —— 写 2 也是同一个值，但 010 这种会被当八进制 8，看着像笔误；
            而 08 / 09 根本不是合法字面量，编译器直接报错。不用 CAM_CMD_xxx 宏，就是为了
            调用处一眼能看出发的是几号命令。

   **没有出参**：结果全在全局量里，函数返回后直接读：
     dx / dy          0x02 / 0x04 / 0x05 的位置偏差(px)
     box_color[0..2]  0x03 的三个框内颜色
     cam_flag         这条命令的 flag：0x02 时是最近物块的颜色号(1~6)、
                      0x04 时是圆环编号(1/2/3)

   为什么返回后能直接读 cam_flag：本函数是在"相机回完成帧"的那一拍当场返回的
   （Cam_WaitSettle 一发现完成就 return，中间没再收帧），所以返回点上 cam_flag
   就是那条完成帧的 flag。**读它之前别插会跑 Cam_Service() 的动作**
   （HAL_Delay / 再接一次 Cam_SwitchTo），否则可能被新来的 flag=0 帧冲掉。

   返回值：1 = 命令被采用且相机回了 flag!=0，全局量已刷新；
           0 = 超时(第一段 10s 没等到本命令的回帧，或第二段 20s 没等到 flag!=0)，
               此时全局量还是旧值，别拿它往下走。
           0x05 还有个特殊情况：任务码里没有哪一槽进过 2 号环 → 没颜色可发，
               直接返回 0，一帧都不发。 */
uint8_t Cam_SwitchTo(uint8_t mode);

/* 旧 ASCII 协议的兼容层(Cam_SwitchTo / Cam_SwitchCloseColor / Cam_WaitFor*)
   已随 pick_task.c 重写一并删除，调用方改用上面的请求式 API。 */
#endif
