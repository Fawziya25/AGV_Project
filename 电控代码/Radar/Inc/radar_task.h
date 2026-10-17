#ifndef __RADAR_TASK_H
#define __RADAR_TASK_H

/*
 * 雷达任务的 AGV 侧胶水层
 * ==========================================================
 * 职责划分：
 *   Radar/Src/link_protocol.c     —— COBS + CRC16 + v6 帧编解码（纯 C，与板子无关）
 *   Radar/Src/radar_client.c      —— 非阻塞客户端：握手/心跳/选区/事务/超时/事件重组
 *   Radar/Src/launch_mission.c    —— 启动一次性任务：选区→设位姿→方格扫描→六段规划→STOP
 *   Radar/Src/radar_uart_port.c   —— UART4 + DMA 板级适配
 *   本文件                        —— 与整车流程的接口：初始化、轮询、菜单起点格、结果与显示
 *
 * 时序（见 task.c 的 AGV_Init）：
 *   Radar_Init()     初始化客户端（不发帧）+ 装默认启停区的一套值，放在 Menu_Init() 之前
 *   Menu_Init()      阻塞等 A8；期间 C6 选整车路线，A8 后进入"选启停区"（C6 改 / A8 确认）
 *   Radar_RunOnce()  阻塞跑完整个雷达任务（约 2~10s），屏幕显示进度，结束后显示六段摘要
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

/* 初始化客户端与六段任务（不发任何帧）；在 AGV_Init 里、Menu_Init 之前调用一次。 */
void Radar_Init(void);

/* 非阻塞服务入口：RadarClient_Poll + Launch_Poll。 */
void Radar_Service(void);

/* 阻塞跑完整个雷达任务并显示结果；返回 true 表示六段都规划到了。 */
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

/* ======================== 雷达收放（收放舵机） ========================
 * 机构：舵机 **0° = 雷达伸出**（扫描时用）、**300° = 雷达收入车内**（行驶时用）。
 * （2026-10-14：收起角 360°→180°(保险)→**300°**(用户确认)；0°↔300° 是 300° 大行程。）
 * 舵机是第 4 路 `SERVO_RADAR`（TIM4_CH1 = PD12），角度/平滑参数在 radar_config.h 的
 * RADAR_SERVO_* 那组。
 *
 * 用法（典型顺序）：
 *   Radar_SetDeployed(true);            // 伸出，内部阻塞到舵机到位
 *   Radar_Init(); Radar_RunOnce();      // 扫描 + 六段规划
 *   Radar_SetDeployed(false);           // 收回车内，内部阻塞到到位
 *   ...开跑...
 *
 * 注意：
 * 1) **第一次调用会自己初始化这一路舵机**（`Servo_InitId(SERVO_RADAR)`），
 *    不碰机械臂那三路 —— 测试模式里 AGV_Init()/Pick_Init() 没跑也能用。
 * 2) 若之后又调了 `Servo_Init()`（`Pick_Init()` 内部会调），会把雷达舵机复位到
 *    300°(收起)：生产流程里请保证"伸出 → 扫描 → 收回"都排在 Pick_Init 之后，
 *    或者复位后重新调一次 Radar_SetDeployed(true)。
 * 3) 运动是阻塞的（约 0.7~1.2s），不要在非阻塞轮询里频繁调用。
 */
void Radar_SetDeployed(bool extended);
bool Radar_IsDeployed(void);   /* 最近一次下发的状态（伸出 = true） */

/* ======================== 完整扫描的原始坐标（诊断用，2026-10-14 加） ========================
 * 背景：障碍方格"位置和数量都不对"时，光看格子分不清是**朝向转了多少**还是 **X/Y 平移了多少**。
 * 完整扫描（`SCAN_LOCKED 0x07`）会回传每个障碍的**场地毫米坐标** `(x_mm, y_mm)` + `grid_id`
 * + 置信度，而且**不写雷达 Flash**（区别于方格扫描 0x08）。拿它和一个卷尺量出来的真实坐标比，
 * 就能直接算出"转了多少度、移了多少毫米"，不用猜任何上位机约定。
 *
 *   Radar_RunFullScan(timeout_ms)  阻塞跑一次完整扫描（内部自己 Poll），true = 拿到结果
 *   Radar_GetScanCount()           结果里的障碍个数
 *   Radar_GetScanObstacle(i, &o)   取第 i 个（x/y/grid_id/confidence）
 */
bool Radar_RunFullScan(uint32_t timeout_ms);
uint8_t Radar_GetScanCount(void);
bool Radar_GetScanObstacle(uint8_t index, radar_client_obstacle_t *out);

/* ======================== 位姿对照（排查"扫出来的格子不对"用） ========================
 * 现场最常踩的坑：本机 SET_POSE 把上位机标定的位姿覆盖成 radar_config.h 里那套，
 * 两者朝向差一点，障碍方格的位置和数量就全错。下面几条用来在屏上/程序里对照：
 *
 *   Radar_GetRadarPose()   雷达自报的位姿（HELLO）。选完区后 = 该区 Flash 里那份
 *                          = 上位机标定的值。zone=0（区域未选定）时返回 false。
 *   Radar_GetConfigPose()  本机 radar_config.h 里那套值；LAUNCH_USE_SAVED_POSE=0 时
 *                          才会被 SET_POSE 发出去，=1 时不发、仅作对照。
 *   Radar_IsUsingSavedPose() true = 当前不覆盖雷达那份（LAUNCH_USE_SAVED_POSE=1）。
 */
bool Radar_GetRadarPose(radar_client_pose_t *out);
bool Radar_GetConfigPose(radar_client_pose_t *out);
bool Radar_IsUsingSavedPose(void);

/* ======================== 方格几何与逐格展开 ========================
 * ⚠ 雷达回的是**拐点 + 终点**，不是逐格序列（说明书 §7：直线路径只返回终点；
 *   移植示例 §6：路径结果不包含起点、包含拐点和终点）。所以一段 "4→14" 只回
 *   1 个点（14）是**正常**的 —— 两点同列，本来就是一条直线，不是丢数据。
 *   要**逐格**路线（驱车、或想在屏上看清走哪几格）用 Radar_ExpandLeg() 展开。
 *
 * 方格编号几何（说明书 §9，0~24 从 (0,0) 沿 Y 蛇形）：
 *      20 21 22 23 24        row = id / 5
 *      19 18 17 16 15        col = 奇数行 ? 4 - id%5 : id%5
 *      10 11 12 13 14
 *       9  8  7  6  5
 *       0  1  2  3  4
 */
uint8_t Radar_GridRow(uint8_t grid);   /* 方格 → 行 0~4 */
uint8_t Radar_GridCol(uint8_t grid);   /* 方格 → 列 0~4 */

/*
 * 把第 leg 段展开成**逐格**编号序列（不含起点、含终点），返回写入的格数；
 * 0 = 无法展开：该段没跑到 / status≠0 / 缓冲不够 / 折点既不共行也不共列。
 * 依据：段落 = 起点 → 拐点 → … → 终点，相邻两点必同行或同列，沿行/列逐格走过去。
 * 缓冲 24 字节就够（5×5 网格里除起点外最多 24 格 = RADAR_PATH_MAX_POINTS）。
 */
uint8_t Radar_ExpandLeg(uint8_t leg, uint8_t *cells, uint8_t max_cells);

/* ======================== 障碍方格（方格扫描的直接结果） ========================
 * 来自那一次 14 圈**方格扫描**（EVENT_OBSTACLE_GRID）：它就是路径规划的输入，
 * 也是"雷达认为哪几格被占用"的原始答案。雷达回传前已去重；**count=0 是有效空
 * 结果（没有障碍），不是失败**（见说明书 §6/§7 方格结果）。
 * 与路径结果一样只在本机 RAM，掉电即失；雷达 Flash 里另存着一份，
 * 下次上电 PERSISTED_STATE 能读出来（那是规划用的，不是本函数这份）。
 */
uint8_t Radar_GetObstacleCount(void);
bool Radar_GetObstacle(uint8_t index, uint8_t *grid_id);   /* index 0..count-1 */
bool Radar_GridIsObstacle(uint8_t grid);                   /* 画 5×5 障碍图用 */

/* ======================== 逐段路径点（全局，整车流程可直接取） ========================
 * 每段一个数组：Radar_Path[i].points[0 .. count-1] 就是该段按行驶顺序的方格编号
 * （不含起点、含终点，元素是雷达方格编号 0~24）。想按"独立数组"写，
 * 用下面的 Radar_Path1..5 宏即可（它们就是那些数组本身，不是拷贝）。
 *
 * 生命周期：Radar_RunOnce() 结束时自动同步一次；之后整车流程随时可读，掉电即失。
 * 未跑到的段：status = 0xFF、count = 0、points[] 全 0xFF。
 */
#define RADAR_LEG_COUNT       6U      /* 本项目固定六段：
                                         起点→2、2→14、14→10、10→22、22→14、**22→起点**。
                                         注意第 6 段**不是接着第 5 段的**：第 5 段结束在 14，
                                         回程段却从 RADAR_RETURN_FROM_GRID(22) 出发
                                         （用户 2026-10-14 要求）→ 路线不是一条链，
                                         由 radar_task.c 的 Radar_ComposeRoute() 用
                                         Launch_SetLegs() 逐段显式给出；回程终点 = 出发点方格
                                         （区2=0 / 区1=4）。RADAR_RETURN_TO_START=0 时只跑 5 段、
                                         第 6 槽显示 "--"。改段数只需改这里 +
                                         radar_config.h 的 LAUNCH_ROUTE_GOALS。 */
#define RADAR_PATH_MAX_POINTS 24U     /* 单段最多 24 个路径点（雷达限制，不含起点） */
#define RADAR_PATH_INVALID    0xFFU   /* "这段没跑到" / "该位置无路径点"的标记 */

typedef struct {
    uint8_t status;                        /* 0 成功；其他见 radar_client.h 的 status 表 */
    uint8_t start;                         /* 该段起点方格 0~24 */
    uint8_t goal;                          /* 该段终点方格 0~24 */
    uint8_t count;                         /* points[] 的有效项数 */
    uint8_t points[RADAR_PATH_MAX_POINTS]; /* 路径点：方格编号，行驶顺序 */
} radar_path_t;

extern radar_path_t Radar_Path[RADAR_LEG_COUNT];   /* 六段路径点（全局，RAM） */
extern uint8_t Radar_PathCount;                    /* 实际拿到的段数，正常 5 */

/* 各段数组的直接视图：Radar_Path1[j] 等价于 Radar_Path[0].points[j] */
#define Radar_Path1  Radar_Path[0].points
#define Radar_Path2  Radar_Path[1].points
#define Radar_Path3  Radar_Path[2].points
#define Radar_Path4  Radar_Path[3].points
#define Radar_Path5  Radar_Path[4].points

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

/* OLED 显示六段摘要（不刷新调试画面，调用方负责 Dbg 的挂起/恢复）。 */
void Radar_ShowSummary(void);

#endif /* __RADAR_TASK_H */
