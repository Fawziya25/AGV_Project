#include "launch_mission.h"

#include "radar_uart_port.h"

#include <string.h>

#if LAUNCH_USE_DEMO_CONSOLE
#include "demo_common.h"
#include "demo_console.h"
#define LAUNCH_LOG(...)            DemoConsole_Printf(__VA_ARGS__)
#define LAUNCH_PRINT_REQUEST(n, r) DemoCommon_PrintRequest((n), (r))
#else
#define LAUNCH_LOG(...)            ((void)0)
#define LAUNCH_PRINT_REQUEST(n, r) ((void)0)
#endif

/* ======================== 失败原因（Launch_GetFailure 返回值） ======================== */

#define LAUNCH_FAIL_NONE          0U  /* 成功 */
#define LAUNCH_FAIL_CONNECT       1U  /* 15 s 内没有进入 READY */
#define LAUNCH_FAIL_POSE          2U  /* 设置/保存位姿被拒绝或超时 */
#define LAUNCH_FAIL_SCAN          3U  /* 方格扫描被拒绝或超时 */
#define LAUNCH_FAIL_STATE         4U  /* 读取保存状态失败 */
#define LAUNCH_FAIL_GRID_INVALID  5U  /* 扫描后雷达 Flash 里没有有效方格结果 */
#define LAUNCH_FAIL_PATH          6U  /* 路径规划被拒绝或超时 */
#define LAUNCH_FAIL_DEVICE        7U  /* 雷达拒绝 / 协议错 / 事务超时 / UART 故障 */

/* ======================== 内部状态 ======================== */

typedef enum {
    STEP_CONNECT = 0,   /* 等 HELLO 握手 + 区域确认 + 保存状态读取 */
    STEP_SET_POSE,      /* 提交 SET_POSE 或 SAVE_POSE */
    STEP_WAIT_POSE,     /* 等 ACK（保存版还等 PERSISTED + HELLO） */
    STEP_GRID_SCAN,     /* 提交 SINGLE_MEASURE 方格扫描 */
    STEP_WAIT_GRID,     /* 等方格结果（客户端还会等锁定状态） */
    STEP_READ_STATE,    /* 可选：重新读取保存状态 */
    STEP_WAIT_STATE,    /* 可选：等保存状态 */
    STEP_PLAN_PATH,     /* 提交 PLAN_PATH */
    STEP_WAIT_PATH,     /* 等路径结果 */
    STEP_STOP,          /* 定向 STOP 收尾 */
    STEP_FINISHED       /* 雷达阶段结束 */
} launch_step_t;

static launch_step_t step;
static uint32_t deadline_ms;
static bool fault;           /* error 回调登记的故障 */
static bool stop_submitted;
static bool finished;
static unsigned failure;
static uint8_t confirmed_zone; /* 进入 READY 时缓存，避免结束后再访问客户端 */

/* ---- 本次要设定的位姿（区域 + 坐标 + 朝向）----
 * 默认取编译期配置，调用方可在规划前用 Launch_SetPose() 覆盖。
 * 之所以做成运行时：区域(启停区)是启动菜单里选的，而两区的位姿窗口不同
 * （区域1 的 Y 1900~2600，区域2 的 Y -200~500），位姿必须跟着区域走。 */
static uint8_t pose_zone;
static int16_t pose_x_mm;
static int16_t pose_y_mm;
static uint16_t pose_heading_q6;

static bool grid_ready;
static uint8_t grid_count;
static uint8_t grid_ids[RADAR_CLIENT_MAX_GRID_IDS];

#if LAUNCH_VERIFY_SAVED_GRID
static bool state_ready;
static bool saved_grid_valid;
#endif

static bool path_ready;
static radar_client_path_result_t path_copy;

/* ---- 路线与逐段结果 ----
 * 路线改成**逐段显式指定**：`leg_starts[i] -> leg_goals[i]`，段数 = leg_total。
 * 这样既能表达"首尾相连"的普通路线（Launch_SetRoute/默认配置），也能表达
 * **不连的段**（例如"22→14 之后，另外再规划一段 22→起点"）——用户 2026-10-14 要求的就是这种。
 * 默认来自编译期配置 LAUNCH_START_GRID / LAUNCH_ROUTE_GOALS，
 * 可在规划前用 Launch_SetRoute() / Launch_SetLegs() / Launch_SetStart() 覆盖。 */
static const uint8_t default_goals[] = LAUNCH_ROUTE_GOALS;
static uint8_t leg_starts[LAUNCH_MAX_LEGS];
static uint8_t leg_goals[LAUNCH_MAX_LEGS];
static uint8_t leg_total;                     /* 段数 */
static uint8_t leg_index;                     /* 下一段待规划的序号 */
static uint8_t leg_done;                      /* 已完成规划的段数 */
static uint8_t leg_failure_mask;              /* bit i = 第 i 段 status != 0 */
static radar_client_path_result_t leg_results[LAUNCH_MAX_LEGS];

/* 编译期保证配置的终点个数不超过容量。 */
typedef char launch_route_goals_fit
    [(sizeof(default_goals) / sizeof(default_goals[0]) <= LAUNCH_MAX_LEGS) ? 1 : -1];

/* 有符号差值比较时间，HAL tick 回绕后仍然有效。 */
static bool time_reached(uint32_t now_ms, uint32_t deadline)
{
    return (int32_t)(now_ms - deadline) >= 0;
}

/* ======================== 回调：只复制数据并置标志 ======================== */

static void on_grid(const radar_client_grid_result_t *data)
{
    uint8_t index;
#if LAUNCH_USE_DEMO_CONSOLE
    DemoCommon_PrintGrid(data);
#endif
    /* count=0 也是有效结果（本次没有障碍方格），雷达会把它保存为有效空结果。 */
    grid_count = data->count;
    for (index = 0U; index < data->count && index < RADAR_CLIENT_MAX_GRID_IDS; ++index)
        grid_ids[index] = data->grid_ids[index];
    grid_ready = true;
}

static void on_persisted(const radar_client_persisted_state_t *data)
{
    (void)data;
#if LAUNCH_USE_DEMO_CONSOLE
    DemoCommon_PrintPersisted(data);
#endif
#if LAUNCH_VERIFY_SAVED_GRID
    saved_grid_valid = data->grid_result_valid;
    state_ready = true;
#endif
}

static void on_path(const radar_client_path_result_t *data)
{
#if LAUNCH_USE_DEMO_CONSOLE
    DemoCommon_PrintPath(data);
#endif
    path_copy = *data;   /* data 只在本次回调内有效，必须立即复制 */
    path_ready = true;
}

static void on_error(const radar_client_error_t *data)
{
#if LAUNCH_USE_DEMO_CONSOLE
    DemoConsole_Printf("ERROR kind=%u operation=%u code=%u detail=%u\r\n",
                       (unsigned int)data->kind,
                       (unsigned int)data->operation,
                       data->code, data->detail);
#else
    (void)data;
#endif
    fault = true;
}

void Launch_InitCallbacks(radar_client_callbacks_t *callbacks)
{
    if (callbacks == NULL) return;
#if LAUNCH_USE_DEMO_CONSOLE
    /* 先清表并注册公共的 hello/state/persisted/error 打印，再覆盖本模块需要的成员。 */
    DemoCommon_InitCallbacks(callbacks);
#endif
    callbacks->grid = on_grid;
    callbacks->persisted = on_persisted;
    callbacks->path = on_path;
    callbacks->error = on_error;
}

/* ======================== 收尾 ======================== */

static void enter_stop(unsigned reason, const char *text)
{
    if (failure == LAUNCH_FAIL_NONE) failure = reason;
    LAUNCH_LOG("LAUNCH abort: %s\r\n", (text != NULL) ? text : "");
    (void)text;
    step = STEP_STOP;
    stop_submitted = false;
    deadline_ms = RadarUartPort_GetTick() + LAUNCH_T_STOP_MS;
}

static void finish(void)
{
    step = STEP_FINISHED;
    finished = true;
    if (failure == LAUNCH_FAIL_NONE)
        LAUNCH_LOG("LAUNCH done: radar stage finished, radar no longer used.\r\n");
    else
        LAUNCH_LOG("LAUNCH failed: reason=%u, radar stage finished.\r\n", failure);
}

/* ======================== 初始化 ======================== */

/* 用编译期配置填充路线（链式：起点 -> 第1段终点 -> 第2段终点 …）；
   返回 false 表示配置越界（不发起任何规划）。 */
static bool set_route_defaults(void)
{
    uint8_t index;
    uint8_t goals = (uint8_t)(sizeof(default_goals) / sizeof(default_goals[0]));
    uint8_t prev = (uint8_t)LAUNCH_START_GRID;

    if (goals > LAUNCH_MAX_LEGS) goals = LAUNCH_MAX_LEGS;
    if (goals == 0U) return false;
    if (prev > 24U) return false;

    for (index = 0U; index < goals; ++index) {
        if (default_goals[index] > 24U) return false;
        leg_starts[index] = prev;
        leg_goals[index] = default_goals[index];
        prev = default_goals[index];
    }
    leg_total = goals;
    return true;
}

void Launch_Init(void)
{
    step = STEP_CONNECT;
    deadline_ms = RadarUartPort_GetTick() + LAUNCH_T_CONNECT_MS;
    fault = false;
    stop_submitted = false;
    finished = false;
    failure = LAUNCH_FAIL_NONE;
    confirmed_zone = 0U;
    grid_ready = false;
    grid_count = 0U;
    memset(grid_ids, 0, sizeof(grid_ids));
#if LAUNCH_VERIFY_SAVED_GRID
    state_ready = false;
    saved_grid_valid = false;
#endif
    path_ready = false;
    memset(&path_copy, 0, sizeof(path_copy));
    leg_index = 0U;
    leg_done = 0U;
    leg_failure_mask = 0U;
    memset(leg_results, 0, sizeof(leg_results));
    /* 默认位姿/区域来自编译期宏；调用方可在规划前用
       Launch_SetPose() / Launch_SetStart() / Launch_SetRoute() 覆盖。 */
    pose_zone = (uint8_t)LAUNCH_ZONE;
    pose_x_mm = (int16_t)LAUNCH_POSE_X_MM;
    pose_y_mm = (int16_t)LAUNCH_POSE_Y_MM;
    pose_heading_q6 = LAUNCH_POSE_HEADING_Q6;
    if (!set_route_defaults()) {
        enter_stop(LAUNCH_FAIL_PATH, "invalid route configuration");
        return;
    }

    LAUNCH_LOG("LAUNCH init: zone=%u pose=(%d,%d,%u/64deg) legs=%u\r\n",
               pose_zone, pose_x_mm, pose_y_mm,
               pose_heading_q6, (unsigned)leg_total);
}

/* ======================== 主状态机 ======================== */

void Launch_Poll(void)
{
    radar_client_state_t link;
    radar_client_request_result_t result;
    uint32_t now_ms = RadarUartPort_GetTick();

    if (finished) return;

    (void)RadarClient_GetState(&link);

    /* 雷达拒绝、协议错、事务超时或 UART 故障：立即进入收尾流程。 */
    if (fault && step != STEP_STOP) {
        enter_stop(LAUNCH_FAIL_DEVICE, "device or protocol error");
        return;
    }

    switch (step) {
    case STEP_CONNECT:
        /* ready = 已收到本节点 v6 HELLO + 工作区域确认 + 保存状态读取完成。 */
        if (link.ready) {
            confirmed_zone = link.selected_zone;
            step = STEP_SET_POSE;
        } else if (time_reached(now_ms, deadline_ms)) {
            enter_stop(LAUNCH_FAIL_CONNECT, "connect timeout");
        }
        break;

    case STEP_SET_POSE:
#if LAUNCH_USE_SAVED_POSE
        /* 用雷达 Flash 里那份位姿（上位机标定的）：一发 SET_POSE 就把它覆盖成我们自己的值，
           现场标定过的朝向/坐标就白标了 —— 因此这里直接跳到方格扫描。 */
        step = STEP_GRID_SCAN;
        break;
#else
        if (!RadarClient_IsBusy()) {
            radar_client_pose_t pose;
            memset(&pose, 0, sizeof(pose));
            pose.x_mm = pose_x_mm;
            pose.y_mm = pose_y_mm;
            pose.heading_q6 = pose_heading_q6;
            pose.zone = pose_zone;   /* 必须等于当前已确认的区域（= 选中的启停区） */
#if LAUNCH_SAVE_POSE_TO_FLASH
            result = RadarClient_RequestSavePose(&pose);
            LAUNCH_PRINT_REQUEST("save_pose", result);
            deadline_ms = now_ms + LAUNCH_T_POSE_MS;
#else
            result = RadarClient_RequestSetPose(&pose);
            LAUNCH_PRINT_REQUEST("set_pose", result);
            deadline_ms = now_ms + LAUNCH_T_ACK_MS;
#endif
            if (result != RADAR_CLIENT_REQUEST_ACCEPTED) {
                /* BAD_PARAMETER=3 表示位姿越界或区域不匹配，此时没有发出任何帧。 */
                enter_stop(LAUNCH_FAIL_POSE, "pose request rejected");
                break;
            }
            step = STEP_WAIT_POSE;
        }
        break;
#endif  /* LAUNCH_USE_SAVED_POSE */

    case STEP_WAIT_POSE:
        /* 成功 ACK（保存版还需 PERSISTED+HELLO）后客户端会清 busy；失败走 error 回调。 */
        if (!RadarClient_IsBusy()) {
            step = STEP_GRID_SCAN;
        } else if (time_reached(now_ms, deadline_ms)) {
            enter_stop(LAUNCH_FAIL_POSE, "pose timeout");
        }
        break;

    case STEP_GRID_SCAN:
        if (!RadarClient_IsBusy()) {
            /* 必须是方格扫描：只有它把结果保存到雷达 Flash，路径规划才可用。 */
            result = RadarClient_RequestGridScan();
            LAUNCH_PRINT_REQUEST("grid_scan", result);
            if (result != RADAR_CLIENT_REQUEST_ACCEPTED) {
                enter_stop(LAUNCH_FAIL_SCAN, "grid_scan rejected");
                break;
            }
            grid_ready = false;
            deadline_ms = now_ms + LAUNCH_T_SCAN_MS;
            step = STEP_WAIT_GRID;
        }
        break;

    case STEP_WAIT_GRID:
        if (grid_ready) {
#if LAUNCH_VERIFY_SAVED_GRID
            step = STEP_READ_STATE;   /* 先确认新方格确实写进了雷达 Flash */
#else
            step = STEP_PLAN_PATH;
#endif
        } else if (time_reached(now_ms, deadline_ms)) {
            enter_stop(LAUNCH_FAIL_SCAN, "grid scan timeout");
        }
        break;

#if LAUNCH_VERIFY_SAVED_GRID
    case STEP_READ_STATE:
        if (!RadarClient_IsBusy()) {
            result = RadarClient_RequestReadPersistedState();
            LAUNCH_PRINT_REQUEST("read_state", result);
            if (result != RADAR_CLIENT_REQUEST_ACCEPTED) {
                enter_stop(LAUNCH_FAIL_STATE, "read_state rejected");
                break;
            }
            state_ready = false;
            deadline_ms = now_ms + LAUNCH_T_ACK_MS;
            step = STEP_WAIT_STATE;
        }
        break;

    case STEP_WAIT_STATE:
        if (state_ready) {
            if (!saved_grid_valid) {
                /* 方格扫描成功却没有有效保存结果，不能拿旧方格去规划。 */
                enter_stop(LAUNCH_FAIL_GRID_INVALID, "no valid grid in radar flash");
            } else {
                step = STEP_PLAN_PATH;
            }
        } else if (time_reached(now_ms, deadline_ms)) {
            enter_stop(LAUNCH_FAIL_STATE, "read_state timeout");
        }
        break;
#endif /* LAUNCH_VERIFY_SAVED_GRID */

    case STEP_PLAN_PATH:
        if (!RadarClient_IsBusy()) {
            if (leg_index >= leg_total) {
                /* 所有路段都规划完毕，进入收尾。 */
                step = STEP_STOP;
                stop_submitted = false;
                deadline_ms = now_ms + LAUNCH_T_STOP_MS;
                break;
            }
            LAUNCH_LOG("LAUNCH leg %u: %u -> %u\r\n", (unsigned)(leg_index + 1U),
                       leg_starts[leg_index], leg_goals[leg_index]);
            result = RadarClient_RequestPlanPath(leg_starts[leg_index],
                                                 leg_goals[leg_index]);
            LAUNCH_PRINT_REQUEST("plan_path", result);
            if (result != RADAR_CLIENT_REQUEST_ACCEPTED) {
                enter_stop(LAUNCH_FAIL_PATH, "plan_path rejected");
                break;
            }
            path_ready = false;
            deadline_ms = now_ms + LAUNCH_T_PATH_MS;
            step = STEP_WAIT_PATH;
        }
        break;

    case STEP_WAIT_PATH:
        if (path_ready) {
            /* status 由业务侧判读：0 成功，1 无有效方格，2/3 起终点被占用，4 不可达… */
            leg_results[leg_index] = path_copy;
            if (path_copy.status != 0U)
                leg_failure_mask |= (uint8_t)(1U << leg_index);
            ++leg_index;
            ++leg_done;
            path_ready = false;
#if !LAUNCH_CONTINUE_AFTER_LEG_FAILURE
            if (leg_results[leg_index - 1U].status != 0U) {
                enter_stop(LAUNCH_FAIL_PATH, "leg planning failed");
                break;
            }
#endif
            step = STEP_PLAN_PATH;   /* 继续下一段；是否结束由 STEP_PLAN_PATH 判定 */
        } else if (time_reached(now_ms, deadline_ms)) {
            enter_stop(LAUNCH_FAIL_PATH, "path result timeout");
        }
        break;

    case STEP_STOP:
        if (fault && failure == LAUNCH_FAIL_NONE) failure = LAUNCH_FAIL_DEVICE;
        if (!link.connected) {
            finish();   /* 从未连上或已经断开，没有可停止的对象 */
            break;
        }
        if (!stop_submitted) {
            if (RadarClient_IsBusy()) break;      /* 等当前事务自己结束或超时 */
            result = RadarClient_RequestStop();   /* 高优先级独立发送槽位 */
            LAUNCH_PRINT_REQUEST("stop", result);
            (void)result;
            stop_submitted = true;
            deadline_ms = now_ms + LAUNCH_T_STOP_MS;
            break;
        }
        if (!RadarClient_IsBusy()) {
            finish();
        } else if (time_reached(now_ms, deadline_ms)) {
            LAUNCH_LOG("LAUNCH warning: STOP not confirmed.\r\n");
            finish();
        }
        break;

    case STEP_FINISHED:
    default:
        break;
    }
}

/* ======================== 运行时配置（区域/位姿/路线） ======================== */

/*
 * 指定本次要设定的区域（启停区）与雷达位姿。
 * 区域就是"启停区"：1 或 2，两区位姿独立保存；菜单里选的就是它。
 * heading_q6 = round(角度 × 64) mod 23040。
 * 越界不会在此拦下（客户端 RequestSetPose 会返回 BAD_PARAMETER），
 * 但推荐用 RADAR 模块的 Radar_SetZone() —— 它会按区域取对应的一套位姿。
 */
bool Launch_SetPose(int16_t x_mm, int16_t y_mm, uint16_t heading_q6, uint8_t zone)
{
    if (zone != 1U && zone != 2U) return false;
    if (step >= STEP_PLAN_PATH) return false;
    pose_x_mm = x_mm;
    pose_y_mm = y_mm;
    pose_heading_q6 = heading_q6;
    pose_zone = zone;
    return true;
}

/*
 * 只改起点（最常见：起点 = 车辆当前所在方格）。
 * 一旦进入 PLAN_PATH 阶段（请求已提交或已完成）就不再接受修改，返回 false。
 */
bool Launch_SetStart(uint8_t start_grid)
{
    if (start_grid > 24U) return false;
    if (step >= STEP_PLAN_PATH) return false;
    leg_starts[0] = start_grid;   /* 只改第 1 段起点 */
    return true;
}

/* 整条路线：nodes = {起点, 第1段终点, 第2段终点, ...}，2 ~ LAUNCH_MAX_LEGS+1 个节点。 */
bool Launch_SetRoute(const uint8_t *nodes, uint8_t node_count)
{
    uint8_t index;

    if (nodes == NULL) return false;
    if (node_count < 2U || node_count > (uint8_t)(LAUNCH_MAX_LEGS + 1U)) return false;
    if (step >= STEP_PLAN_PATH) return false;
    for (index = 0U; index < node_count; ++index) {
        if (nodes[index] > 24U) return false;
    }
    for (index = 0U; index + 1U < node_count; ++index) {
        leg_starts[index] = nodes[index];
        leg_goals[index] = nodes[index + 1U];
    }
    leg_total = (uint8_t)(node_count - 1U);
    return true;
}

/*
 * 逐段显式指定：第 i 段 = starts[i] -> goals[i]，共 leg_count 段（1 ~ LAUNCH_MAX_LEGS）。
 * **段与段之间不必首尾相连** —— 用于"22→14 之后再另外规划一段 22→起点"这类需求
 * （用户 2026-10-14 的第 6 段就是这种：第 5 段结束在 14，第 6 段却从 22 出发）。
 * 一旦进入 PLAN_PATH 阶段（请求已提交或已完成）就不再接受修改，返回 false。
 */
bool Launch_SetLegs(const uint8_t *starts, const uint8_t *goals, uint8_t leg_count)
{
    uint8_t index;

    if (starts == NULL || goals == NULL || leg_count == 0U) return false;
    if (leg_count > LAUNCH_MAX_LEGS) return false;
    if (step >= STEP_PLAN_PATH) return false;
    for (index = 0U; index < leg_count; ++index) {
        if (starts[index] > 24U || goals[index] > 24U) return false;
    }
    for (index = 0U; index < leg_count; ++index) {
        leg_starts[index] = starts[index];
        leg_goals[index] = goals[index];
    }
    leg_total = leg_count;
    return true;
}

/* 只规划一段，等价于 Launch_SetRoute({start, goal}, 2)。 */
bool Launch_SetPath(uint8_t start, uint8_t goal)
{
    const uint8_t nodes[2] = { start, goal };
    return Launch_SetRoute(nodes, 2U);
}

uint8_t Launch_GetLegTotal(void)
{
    return leg_total;
}

bool Launch_GetLegEndpoints(uint8_t leg, uint8_t *start, uint8_t *goal)
{
    if (start == NULL || goal == NULL || leg >= leg_total) return false;
    *start = leg_starts[leg];
    *goal = leg_goals[leg];
    return true;
}

/* ======================== 结果查询（结束后也可安全调用） ======================== */

bool Launch_IsFinished(void)
{
    return finished;
}

unsigned Launch_GetFailure(void)
{
    return failure;
}

uint8_t Launch_GetZone(void)
{
    return confirmed_zone;
}

uint8_t Launch_GetGridCount(void)
{
    return grid_count;
}

bool Launch_GetGridId(uint8_t index, uint8_t *grid_id)
{
    if (grid_id == NULL || index >= grid_count) return false;
    *grid_id = grid_ids[index];
    return true;
}

uint8_t Launch_GetLegCount(void)
{
    return leg_done;
}

bool Launch_GetLeg(uint8_t index, radar_client_path_result_t *path)
{
    if (path == NULL || index >= leg_done) return false;
    *path = leg_results[index];
    return true;
}

uint8_t Launch_GetLegFailureMask(void)
{
    return leg_failure_mask;
}

bool Launch_GetPath(radar_client_path_result_t *path)
{
    return Launch_GetLeg(0U, path);
}
