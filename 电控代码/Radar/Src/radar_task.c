#include "radar_task.h"

#include "launch_mission.h"
#include "radar_config.h"
#include "radar_uart_port.h"

#include "oled.h"
#include "test.h"           /* Dbg_Suspend / Dbg_Resume —— 雷达进度期间不让调试服务抢屏 */

#include <stdio.h>
#include <string.h>

/* ======================== 角度换算 ======================== */

/*
 * 角度（度，可带小数）→ heading_q6。
 * 雷达线上字段是 uint16 定点：heading_q6 = round(角度 × 64) mod 23040，
 * 即分辨率 1/64° = 0.015625°，一圈正好 23040 个格点（说明书 §9）。
 * 所以小数角度只在 0.015625° 的格点上精确，这里四舍五入到最近格点：
 *   12.5f → 800   12.502f → 800   12.508f → 801   -0.01f → 23039   360.0f → 0
 * 用 float 而不是 double：本工程 FPU 是单精度（-mfpu=fpv4-sp-d16），f 后缀省一次转换。
 */
static uint16_t Radar_HeadingDegToQ6(float deg)
{
    float q6 = deg * 64.0f;
    int32_t rounded = (int32_t)((q6 >= 0.0f) ? (q6 + 0.5f) : (q6 - 0.5f));
    int32_t wrapped = rounded % 23040;
    if (wrapped < 0) wrapped += 23040;
    return (uint16_t)wrapped;
}

/* ======================== 内部状态 ======================== */

static uint8_t s_zone;              /* 选中的启停区（工作区域）1/2 */
static uint8_t s_start_grid;        /* 该启停区对应的出发点方格 */
static uint8_t s_link_zone;         /* 最近一次 HELLO 报的区域，仅用于显示 */
static uint8_t s_link_protocol;
static uint8_t s_link_fw_major;
static uint8_t s_link_fw_minor;

/* 收到 v6 HELLO 时刷新链路信息（握手、探测每 2s、选区后都会来）。 */
static void on_hello(const radar_client_hello_t *data)
{
    s_link_zone = data->pose.zone;
    s_link_protocol = data->protocol;
    s_link_fw_major = data->firmware_major;
    s_link_fw_minor = data->firmware_minor;
}

/* ======================== 显示 ======================== */

static void Radar_ShowProgress(void)
{
    char buf[24];

    OLED_Clear();
    sprintf(buf, "RADAR zone %u", (unsigned)s_zone);
    OLED_ShowString(0, 0, (uint8_t *)buf, 16, 1);

    sprintf(buf, "start %u", (unsigned)s_start_grid);
    OLED_ShowString(0, 16, (uint8_t *)buf, 16, 1);

    sprintf(buf, "leg %u/4 f%u", (unsigned)Launch_GetLegCount(), (unsigned)Launch_GetFailure());
    OLED_ShowString(0, 32, (uint8_t *)buf, 16, 1);

    if (s_link_protocol == 0U)
        sprintf(buf, "no hello");
    else
        sprintf(buf, "fw%u.%u p%u z%u", (unsigned)s_link_fw_major, (unsigned)s_link_fw_minor,
                (unsigned)s_link_protocol, (unsigned)s_link_zone);
    OLED_ShowString(0, 48, (uint8_t *)buf, 16, 1);

    OLED_Refresh();
}

/*
 * 四段摘要：一行一段，格式 "L1 3>14 s0 n2"
 *   Lx 段号 / 起点>终点 / s = status（0 成功）/ n = 路径节点数（不含起点、含终点）
 * 段数不足或某段失败时照样显示，方便现场一眼看出卡在哪一段。
 */
void Radar_ShowSummary(void)
{
    char buf[24];
    uint8_t index;
    uint8_t legs = Launch_GetLegCount();

    OLED_Clear();
    for (index = 0U; index < 4U; ++index) {
        radar_client_path_result_t path;
        if (index < legs && Launch_GetLeg(index, &path)) {
            sprintf(buf, "L%u %u>%u s%u n%u", (unsigned)(index + 1U),
                    (unsigned)path.start, (unsigned)path.goal,
                    (unsigned)path.status, (unsigned)path.waypoint_count);
        } else {
            sprintf(buf, "L%u --", (unsigned)(index + 1U));
        }
        OLED_ShowString(0, (uint8_t)(index * 16U), (uint8_t *)buf, 16, 1);
    }
    OLED_Refresh();
}

/* ======================== 四段路径点（全局） ======================== */

radar_path_t Radar_Path[RADAR_LEG_COUNT];
uint8_t Radar_PathCount;

/*
 * 把 launch_mission 的逐段结果搬到四段数组里。
 * 未跑到的段统一填 RADAR_PATH_INVALID，避免整车流程误用残值。
 */
void Radar_SyncPathArrays(void)
{
    uint8_t legs = Launch_GetLegCount();
    uint8_t index;

    if (legs > (uint8_t)RADAR_LEG_COUNT) legs = (uint8_t)RADAR_LEG_COUNT;
    Radar_PathCount = legs;

    for (index = 0U; index < (uint8_t)RADAR_LEG_COUNT; ++index) {
        radar_client_path_result_t path;
        uint8_t i;

        Radar_Path[index].status = RADAR_PATH_INVALID;
        Radar_Path[index].start = RADAR_PATH_INVALID;
        Radar_Path[index].goal = RADAR_PATH_INVALID;
        Radar_Path[index].count = 0U;
        for (i = 0U; i < (uint8_t)RADAR_PATH_MAX_POINTS; ++i)
            Radar_Path[index].points[i] = RADAR_PATH_INVALID;

        if (index >= legs || !Launch_GetLeg(index, &path)) continue;

        Radar_Path[index].status = path.status;
        Radar_Path[index].start = path.start;
        Radar_Path[index].goal = path.goal;
        Radar_Path[index].count = (path.waypoint_count > (uint8_t)RADAR_PATH_MAX_POINTS)
                                  ? (uint8_t)RADAR_PATH_MAX_POINTS : path.waypoint_count;
        for (i = 0U; i < Radar_Path[index].count; ++i)
            Radar_Path[index].points[i] = path.waypoints[i];
    }
}

/* ======================== 对外接口 ======================== */

void Radar_Init(void)
{
    radar_client_callbacks_t callbacks;

    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.hello = on_hello;
    /* launch_mission 只覆盖 grid/persisted/path/error 四个成员，其余保持上面注册的 */
    Launch_InitCallbacks(&callbacks);
    RadarClient_Init(&callbacks, LAUNCH_NODE_ID);
    Launch_Init();

    s_link_zone = 0U;
    s_link_protocol = 0U;
    s_link_fw_major = 0U;
    s_link_fw_minor = 0U;

    /* 先把默认启停区的一套（选区请求 + 位姿 + 出发点方格）装好；
       菜单阶段2 若改成另一个区会再调一次 Radar_SetZone() 覆盖。 */
    (void)Radar_SetZone((uint8_t)RADAR_ZONE_DEFAULT);
}

void Radar_Service(void)
{
    RadarClient_Poll();
    Launch_Poll();
    /* 每规划完一段就同步到四段数组：整车流程不必等四段全跑完才能用第一段 */
    if (Launch_GetLegCount() != Radar_PathCount) Radar_SyncPathArrays();
}

uint8_t Radar_GetZone(void)
{
    return s_zone;
}

uint8_t Radar_GetStartGrid(void)
{
    return s_start_grid;
}

bool Radar_SetZone(uint8_t zone)
{
    int16_t x_mm;
    int16_t y_mm;
    uint16_t heading_q6;
    uint8_t start_grid;

    if (zone == 1U) {
        x_mm = (int16_t)RADAR_ZONE1_POSE_X_MM;
        y_mm = (int16_t)RADAR_ZONE1_POSE_Y_MM;
        heading_q6 = Radar_HeadingDegToQ6(RADAR_ZONE1_POSE_HEADING_DEG);
        start_grid = (uint8_t)RADAR_ZONE1_START_GRID;
    } else if (zone == 2U) {
        x_mm = (int16_t)RADAR_ZONE2_POSE_X_MM;
        y_mm = (int16_t)RADAR_ZONE2_POSE_Y_MM;
        heading_q6 = Radar_HeadingDegToQ6(RADAR_ZONE2_POSE_HEADING_DEG);
        start_grid = (uint8_t)RADAR_ZONE2_START_GRID;
    } else {
        return false;
    }

    /* 选区请求：未连接时客户端只登记 desired_zone，握手完成后自动发出（这里不等） */
    if (RadarClient_RequestSelectZone(zone) != RADAR_CLIENT_REQUEST_ACCEPTED) return false;
    /* 位姿必须跟着区域走：两区的位姿窗口不同，用错区会被客户端本地拦下 */
    if (!Launch_SetPose(x_mm, y_mm, heading_q6, zone)) return false;
    (void)Launch_SetStart(start_grid);

    s_zone = zone;
    s_start_grid = start_grid;
    return true;
}

bool Radar_SetStartGrid(uint8_t grid)
{
    if (grid > 24U) return false;
    if (!Launch_SetStart(grid)) return false;   /* 已经进入规划阶段 */
    s_start_grid = grid;
    return true;
}

bool Radar_RunOnce(void)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_draw_ms = start_ms;

    Dbg_Suspend();                 /* 进度画面归雷达；Dbg_Resume 时会补一次重画 */
    Radar_ShowProgress();

    while (!Launch_IsFinished() && (HAL_GetTick() - start_ms) < RADAR_RUN_TMO_MS) {
        Radar_Service();
        if ((HAL_GetTick() - last_draw_ms) >= 200U) {
            last_draw_ms = HAL_GetTick();
            Radar_ShowProgress();
        }
    }

    Radar_SyncPathArrays();      /* 存进四段数组（Radar_Path[]），整车流程可直接读 */
    Radar_ShowSummary();
    Dbg_Resume();

    return (Launch_GetFailure() == 0U) && (Launch_GetLegCount() > 0U);
}

uint8_t Radar_GetLegCount(void)
{
    return Launch_GetLegCount();
}

bool Radar_GetLeg(uint8_t index, radar_client_path_result_t *path)
{
    return Launch_GetLeg(index, path);
}

uint8_t Radar_GetFailure(void)
{
    return (uint8_t)Launch_GetFailure();
}

uint8_t Radar_GetFailureMask(void)
{
    return Launch_GetLegFailureMask();
}
