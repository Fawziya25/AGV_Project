#ifndef __RADAR_TASK_H
#define __RADAR_TASK_H

/*
 * 雷达任务的 AGV 侧胶水层
 * ==========================================================
 * 职责划分：
 *   Radar/Src/link_protocol.c     —— COBS + CRC16 + v6 帧编解码（纯 C，与板子无关）
 *   Radar/Src/radar_client.c      —— 非阻塞客户端：握手/心跳/选区/事务/超时/事件重组
 *   Radar/Src/launch_mission.c    —— 启动一次性任务：选区→设位姿→方格扫描→四段规划→STOP
 *   Radar/Src/radar_uart_port.c   —— UART4 + DMA 板级适配
 *   本文件                        —— 与整车流程的接口：初始化、轮询、菜单起点格、结果与显示
 *
 * 时序（见 task.c 的 AGV_Init）：
 *   Radar_Init()     初始化客户端（不发帧）+ 装默认启停区的一套值，放在 Menu_Init() 之前
 *   Menu_Init()      阻塞等 A8；期间 C6 选整车路线，A8 后进入"选启停区"（C6 改 / A8 确认）
 *   Radar_RunOnce()  阻塞跑完整个雷达任务（约 2~10s），屏幕显示进度，结束后显示四段摘要
 *   TaskCode=Depart  之后整车流程完全不用雷达（Radar_Service 不必再调用）
 *
 * 术语：**启停区 = 雷达的工作区域 zone(1/2)**，两区位姿独立保存，用 SELECT_ZONE 切换；
 *       **出发点方格(0~24)** 是 PLAN_PATH 的起点参数，本项目按"每个启停区一个固定出发点"
 *       处理，跟着区域一起自动设好（见 radar_config.h）。
 *
 * 如需在别处使用：Radar_Service() 是非阻塞的，放在主循环里反复调用即可，
 * 但必须保证调用间隔 ≤10ms（客户端的心跳/超时依赖它）。
 */

#include "radar_client.h"

#include <stdbool.h>
#include <stdint.h>

/* 初始化客户端与四段任务（不发任何帧）；在 AGV_Init 里、Menu_Init 之前调用一次。 */
void Radar_Init(void);

/* 非阻塞服务入口：RadarClient_Poll + Launch_Poll。 */
void Radar_Service(void);

/* 阻塞跑完整个雷达任务并显示结果；返回 true 表示四段都规划到了。 */
bool Radar_RunOnce(void);

/*
 * 选启停区（工作区域）1 或 2 —— 菜单阶段2 里 C6/A8 调的就是它。
 * 一次调用同时做三件事：把选区请求登记给客户端、按该区设雷达位姿、按该区设出发点方格。
 * 区域不是 1/2 或已经进入规划阶段时返回 false。
 */
bool Radar_SetZone(uint8_t zone);
uint8_t Radar_GetZone(void);

/* 当前启停区对应的出发点方格（0~24，只读，用于显示）。 */
uint8_t Radar_GetStartGrid(void);

/* 临时改本次的出发点方格（一般不用：它跟着启停区自动设好了）；
   规划已经开始后返回 false。 */
bool Radar_SetStartGrid(uint8_t grid);

/* ======================== 四段路径点（全局，整车流程可直接取） ========================
 * 每段一个数组：Radar_Path[i].points[0 .. count-1] 就是该段按行驶顺序的方格编号
 * （不含起点、含终点，元素是雷达方格编号 0~24）。想按"四个独立数组"写，
 * 用下面的 Radar_Path1..4 宏即可（它们就是那四个数组本身，不是拷贝）。
 *
 * 生命周期：Radar_RunOnce() 结束时自动同步一次；之后整车流程随时可读，掉电即失。
 * 未跑到的段：status = 0xFF、count = 0、points[] 全 0xFF。
 */
#define RADAR_LEG_COUNT       4U      /* 本项目固定四段：起点→14、14→10、10→22、22→14 */
#define RADAR_PATH_MAX_POINTS 24U     /* 单段最多 24 个路径点（雷达限制，不含起点） */
#define RADAR_PATH_INVALID    0xFFU   /* "这段没跑到" / "该位置无路径点"的标记 */

typedef struct {
    uint8_t status;                        /* 0 成功；其他见 radar_client.h 的 status 表 */
    uint8_t start;                         /* 该段起点方格 0~24 */
    uint8_t goal;                          /* 该段终点方格 0~24 */
    uint8_t count;                         /* points[] 的有效项数 */
    uint8_t points[RADAR_PATH_MAX_POINTS]; /* 路径点：方格编号，行驶顺序 */
} radar_path_t;

extern radar_path_t Radar_Path[RADAR_LEG_COUNT];   /* 四段路径点（全局，RAM） */
extern uint8_t Radar_PathCount;                    /* 实际拿到的段数，正常 4 */

/* 四个数组的直接视图：Radar_Path1[j] 等价于 Radar_Path[0].points[j] */
#define Radar_Path1  Radar_Path[0].points
#define Radar_Path2  Radar_Path[1].points
#define Radar_Path3  Radar_Path[2].points
#define Radar_Path4  Radar_Path[3].points

/* 把 launch_mission 的结果同步到上面的数组；Radar_RunOnce() 末尾会自动调一次，
   手动调用也安全（幂等）。 */
void Radar_SyncPathArrays(void);

/* 逐段结果（index 0 = 第一段）。status: 0 成功 / 1 无有效方格 / 2 起点被占用 /
   3 终点被占用 / 4 不可达 / 5 参数错误 / 6 容量超限。 */
uint8_t Radar_GetLegCount(void);
bool Radar_GetLeg(uint8_t index, radar_client_path_result_t *path);

/* 0 = 无失败；1 连接超时 / 2 位姿 / 3 方格扫描 / 4 读保存状态 /
   5 雷达Flash无有效方格 / 6 路径规划 / 7 雷达拒绝或协议错。 */
uint8_t Radar_GetFailure(void);

/* bit i = 第 i 段 status != 0（业务失败：占用/不可达等）。 */
uint8_t Radar_GetFailureMask(void);

/* OLED 显示四段摘要（不刷新调试画面，调用方负责 Dbg 的挂起/恢复）。 */
void Radar_ShowSummary(void);

#endif /* __RADAR_TASK_H */
