#ifndef LAUNCH_MISSION_H
#define LAUNCH_MISSION_H

/*
 * 启动一次性任务：设置发车区域与雷达位姿 -> 方格扫描 -> 路径规划 -> 结束雷达阶段
 * ==============================================================================
 *
 * 流程（全部非阻塞，由 Launch_Poll 推进）：
 *   1. 等待 READY：客户端完成 v6 HELLO 握手 + 工作区域确认 + 读取雷达保存状态。
 *      选区请求由调用方在 RadarClient_Init 之后单独提交：
 *          (void)RadarClient_RequestSelectZone(LAUNCH_ZONE);
 *   2. 设置雷达位姿：默认 CMD_SET_POSE(0x06)，只改本次上电的会话位姿；
 *      把 LAUNCH_SAVE_POSE_TO_FLASH 置 1 则改用 CMD_SAVE_POSE(0x11) 写入雷达 Flash。
 *   3. 方格扫描：CMD_SINGLE_MEASURE(0x08)。必须用方格扫描，因为只有它会把去重后的
 *      方格结果保存到雷达 Flash，而路径规划只认 Flash 里最近一次有效的方格结果。
 *      （CMD_SCAN_LOCKED(0x07) 返回坐标和方格，但不写 Flash，不能作为规划前置。）
 *   4. （可选）重新读取保存状态，确认 grid_result_valid=1。
 *   5. 路径规划：按配置的路线依次发送 CMD_PLAN_PATH(0x14)。默认路线由
 *      LAUNCH_START_GRID + LAUNCH_ROUTE_GOALS 给出，例如
 *      {14, 10, 22, 14} 表示"起点->14、14->10、10->22、22->14"共四段；
 *      起点可在规划前用 Launch_SetStart(当前方格) 在运行时覆盖，
 *      整条路线也可用 Launch_SetRoute() 覆盖。逐段结果用 Launch_GetLeg() 读取。
 *   6. 收尾：发送高优先级定向 CMD_STOP，等待确认或超时后结束雷达阶段。
 *
 * 结束判据：Launch_IsFinished() == true。此后调用方可以永久停止调用
 * RadarClient_Poll()，不再使用雷达；结果用 Launch_GetPath()/Launch_GetGrid*() 读取。
 *
 * main.c 集成示例（初始化顺序不可交换）：
 *
 *   radar_client_callbacks_t cb;
 *   DemoConsole_Init();
 *   Launch_InitCallbacks(&cb);                          // 取代 Demo_InitCallbacks
 *   RadarClient_Init(&cb, LAUNCH_NODE_ID);              // 只初始化通信，不选区
 *   (void)RadarClient_RequestSelectZone(LAUNCH_ZONE);   // 未连接时先登记，握手后自动发出
 *   Launch_Init();
 *
 *   for (;;) {
 *       if (!Launch_IsFinished()) {
 *           RadarClient_Poll();        // 雷达阶段必须高频、非阻塞
 *           DemoConsole_Poll();        // 仅 LAUNCH_USE_DEMO_CONSOLE=1 时需要
 *           Launch_Poll();
 *       } else {
 *           App_Run();                 // 之后完全不用雷达
 *       }
 *   }
 *
 * 约束：
 *   - 全流程只提交一次 SELECT_ZONE。业务前重复选区会重新载入该区 Flash 位姿，
 *     覆盖第 2 步设置的会话位姿；断链重连时客户端会自动重新选区，同样会覆盖，
 *     此时需要重新设置位姿，或改用 LAUNCH_SAVE_POSE_TO_FLASH=1。
 *   - 所有回调都在 RadarClient_Poll() 上下文执行；本模块回调只复制数据并置标志，
 *     不延时、不重入 Poll、不打印超过队列能力的日志。
 *   - 雷达坐标与角度的范围由客户端本地校验，超范围时 Request 直接返回
 *     RADAR_CLIENT_REQUEST_BAD_PARAMETER 且不发出任何帧；本头文件的编译期检查
 *     会在编译阶段就拦住常见越界。
 */

#include "radar_client.h"
#include "radar_config.h"   /* 本项目的参数（区域/位姿/路线/超时）——见该文件头 */

#include <stdbool.h>
#include <stdint.h>

/* ======================== 编译期配置：只改这一段 ======================== */

/* 雷达节点 ID，必须与雷达内保存的一致（单设备默认 1）。 */
#ifndef LAUNCH_NODE_ID
#define LAUNCH_NODE_ID            1U
#endif

/* 发车区域（启停区）：1 或 2。 */
#ifndef LAUNCH_ZONE
#define LAUNCH_ZONE               1U
#endif

/*
 * 雷达自身在场地中的坐标与朝向。
 * 客户端范围校验：两区 X 都必须 -200~500；
 * 区域1 的 Y 必须 1900~2600，区域2 的 Y 必须 -200~500。
 */
#ifndef LAUNCH_POSE_X_MM
#define LAUNCH_POSE_X_MM          150
#endif
#ifndef LAUNCH_POSE_Y_MM
#define LAUNCH_POSE_Y_MM          2200
#endif
/* 朝向角 0~359 度；内部换算 heading_q6 = round(角度*64) mod 23040。 */
#ifndef LAUNCH_POSE_HEADING_DEG
#define LAUNCH_POSE_HEADING_DEG   0U
#endif
#define LAUNCH_POSE_HEADING_Q6    \
    ((uint16_t)(((uint32_t)(LAUNCH_POSE_HEADING_DEG) * 64U) % 23040U))

/* 0 = CMD_SET_POSE，只改本次上电的会话位姿；1 = CMD_SAVE_POSE，写入雷达 Flash。 */
#ifndef LAUNCH_SAVE_POSE_TO_FLASH
#define LAUNCH_SAVE_POSE_TO_FLASH 0
#endif

/*
 * 默认路线：LAUNCH_START_GRID 是起点（运行时可用 Launch_SetStart 用车辆当前
 * 所在方格覆盖），LAUNCH_ROUTE_GOALS 按顺序列出每一段的终点；段数 = 终点个数。
 * 默认值对应"起点->14、14->10、10->22、22->14"四段。
 */
#ifndef LAUNCH_START_GRID
#define LAUNCH_START_GRID         0U
#endif
#ifndef LAUNCH_ROUTE_GOALS
#define LAUNCH_ROUTE_GOALS        { 14U, 10U, 22U, 14U }
#endif

/* 某一段规划业务失败（status != 0）时是否继续规划后面的段：1 = 继续并逐段上报。 */
#ifndef LAUNCH_CONTINUE_AFTER_LEG_FAILURE
#define LAUNCH_CONTINUE_AFTER_LEG_FAILURE 1
#endif

/* 一次任务最多 7 段（8 个端点），受 8 位失败掩码限制。 */
#ifndef LAUNCH_MAX_LEGS
#define LAUNCH_MAX_LEGS           7U
#endif

/* 1 = 方格扫描后重新读取保存状态，确认 grid_result_valid；0 = 直接规划。 */
#ifndef LAUNCH_VERIFY_SAVED_GRID
#define LAUNCH_VERIFY_SAVED_GRID  1
#endif

/*
 * 1 = 使用例程的 demo_console/demo_common 打印调试日志（需要这两个模块）；
 * 0 = 本模块不依赖任何调试输出，可单独放进你自己的工程。
 * 本项目为 0：AGV 工程没有那两个模块，进度/结果都通过 OLED 显示（见 radar_task.c）。
 */
#ifndef LAUNCH_USE_DEMO_CONSOLE
#define LAUNCH_USE_DEMO_CONSOLE   0
#endif

/* ======================== 超时（单位 ms） ======================== */

/* 首连接宽限；期间客户端每 200 ms 重发 HELLO。 */
#ifndef LAUNCH_T_CONNECT_MS
#define LAUNCH_T_CONNECT_MS       15000U
#endif
/* 普通命令 ACK（SET_POSE / 读保存状态 / 路径）。 */
#ifndef LAUNCH_T_ACK_MS
#define LAUNCH_T_ACK_MS           1500U
#endif
/* SAVE_POSE 需要 ACK + 保存状态 + HELLO 三者齐全。 */
#ifndef LAUNCH_T_POSE_MS
#define LAUNCH_T_POSE_MS          2000U
#endif
/* 14 圈方格扫描（客户端自身 8 s 超时通常先触发，这里是外层兜底）。 */
#ifndef LAUNCH_T_SCAN_MS
#define LAUNCH_T_SCAN_MS          9000U
#endif
/* 路径结果。 */
#ifndef LAUNCH_T_PATH_MS
#define LAUNCH_T_PATH_MS          1500U
#endif
/* 定向 STOP：说明书建议最多等 400 ms。 */
#ifndef LAUNCH_T_STOP_MS
#define LAUNCH_T_STOP_MS          500U
#endif

/* ======================== 编译期参数检查 ======================== */

#if LAUNCH_ZONE != 1U && LAUNCH_ZONE != 2U
#error "LAUNCH_ZONE must be 1 or 2"
#endif

#if LAUNCH_NODE_ID < 1U || LAUNCH_NODE_ID > 254U
#error "LAUNCH_NODE_ID must be 1..254"
#endif

#if LAUNCH_POSE_X_MM < -200 || LAUNCH_POSE_X_MM > 500
#error "LAUNCH_POSE_X_MM must be -200..500 for both zones"
#endif

#if LAUNCH_ZONE == 1U
#if LAUNCH_POSE_Y_MM < 1900 || LAUNCH_POSE_Y_MM > 2600
#error "Zone 1 requires LAUNCH_POSE_Y_MM in 1900..2600"
#endif
#else
#if LAUNCH_POSE_Y_MM < -200 || LAUNCH_POSE_Y_MM > 500
#error "Zone 2 requires LAUNCH_POSE_Y_MM in -200..500"
#endif
#endif

#if LAUNCH_POSE_HEADING_DEG > 359U
#error "LAUNCH_POSE_HEADING_DEG must be 0..359"
#endif

#if LAUNCH_START_GRID > 24U
#error "LAUNCH_START_GRID must be 0..24"
#endif

#if LAUNCH_VERIFY_SAVED_GRID != 0 && LAUNCH_VERIFY_SAVED_GRID != 1
#error "LAUNCH_VERIFY_SAVED_GRID must be 0 or 1"
#endif

#if LAUNCH_SAVE_POSE_TO_FLASH != 0 && LAUNCH_SAVE_POSE_TO_FLASH != 1
#error "LAUNCH_SAVE_POSE_TO_FLASH must be 0 or 1"
#endif

/* ======================== 对外接口 ======================== */

/*
 * 注册本模块需要的回调。必须在 RadarClient_Init() 之前调用。
 * LAUNCH_USE_DEMO_CONSOLE=1 时先调用 DemoCommon_InitCallbacks() 清表并注册公共回调；
 * =0 时只覆盖 grid/persisted/path/error 四个成员，其余成员保持调用方原有值，
 * 因此调用方自己的回调表如果不为空，需要先自行 memset。
 */
void Launch_InitCallbacks(radar_client_callbacks_t *callbacks);

/*
 * 启动状态机并开始计时。在 RadarClient_Init() 与 RequestSelectZone() 之后调用。
 * 本函数不发送任何帧。
 */
void Launch_Init(void);

/* 放在主循环里反复调用；内部完全非阻塞。 */
void Launch_Poll(void);

/* true = 雷达阶段已结束（成功或失败），此后可以不再调用任何 RadarClient_* 接口。 */
bool Launch_IsFinished(void);

/* 0 = 无失败；非 0 为 LAUNCH_FAIL_*（见 launch_mission.c）。 */
unsigned Launch_GetFailure(void);

/* 本次方格扫描到的障碍方格（count=0 是有效空结果，不是失败）。 */
uint8_t Launch_GetGridCount(void);
bool Launch_GetGridId(uint8_t index, uint8_t *grid_id);

/*
 * 运行时指定本次要设定的区域（启停区）与雷达位姿。
 * 区域 1/2 的位姿窗口不同（区域1 的 Y 1900~2600，区域2 的 Y -200~500），
 * 所以位姿必须跟着区域走 —— RADAR 模块的 Radar_SetZone() 就是干这个的。
 * 必须在 Launch_Init() 之后、PLAN_PATH 提交之前调用；越界或阶段已过返回 false。
 */
bool Launch_SetPose(int16_t x_mm, int16_t y_mm, uint16_t heading_q6, uint8_t zone);

/*
 * 运行时路线配置。必须在 Launch_Init() 之后、PLAN_PATH 提交之前调用；
 * 参数越界或规划阶段已经开始时返回 false 且不生效。
 *
 *   Launch_SetStart(grid)         —— 只改起点。最常见用法：起点 = 车辆当前所在方格
 *   Launch_SetRoute(nodes, count) —— 整条路线，nodes = {起点, 第1段终点, 第2段终点, ...}
 *   Launch_SetPath(start, goal)   —— 只规划一段，等价 Launch_SetRoute({start,goal}, 2)
 */
bool Launch_SetStart(uint8_t start_grid);
bool Launch_SetRoute(const uint8_t *nodes, uint8_t node_count);
bool Launch_SetPath(uint8_t start, uint8_t goal);

/* 当前生效的路线：节点数 = 段数 + 1，节点 0 是起点。 */
uint8_t Launch_GetRouteLength(void);
bool Launch_GetRouteNode(uint8_t index, uint8_t *grid_id);

/*
 * 逐段结果（index 0 = 第一段）。
 * radar_client_path_result_t.status：0 成功、1 无有效方格、2 起点被占用、
 * 3 终点被占用、4 不可达、5 参数错误、6 容量超限；
 * waypoints[0..count-1] 按行驶顺序，不含起点、含终点。
 */
uint8_t Launch_GetLegCount(void);
bool Launch_GetLeg(uint8_t index, radar_client_path_result_t *path);
/* bit i = 第 i 段 status != 0（业务失败，例如被占用或不可达），最多 7 段。 */
uint8_t Launch_GetLegFailureMask(void);

/* 等价于 Launch_GetLeg(0U, path)：单段路线或整条路线的第一段。 */
bool Launch_GetPath(radar_client_path_result_t *path);

/* 当前已确认的工作区域（1/2）；0 表示还没确认。 */
uint8_t Launch_GetZone(void);

#endif
