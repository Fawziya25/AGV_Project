#include "radar_task.h"

#include "launch_mission.h"
#include "radar_config.h"
#include "radar_uart_port.h"
#include "servo.h"          /* 第 4 路舵机 SERVO_RADAR：雷达收放 */

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
static radar_client_pose_t s_radar_pose;   /* 最近一次 HELLO 自报的位姿：
                                              选区后 = 该区 Flash 里那份（也就是上位机标定的） */
static radar_client_pose_t s_config_pose;  /* 本机 config 里那套值（LAUNCH_USE_SAVED_POSE=0 时会被发出去） */
static bool s_config_valid;

/* 收到 v6 HELLO 时刷新链路信息（握手、探测每 2s、选区后都会来）。 */
static void on_hello(const radar_client_hello_t *data)
{
    s_radar_pose = data->pose;           /* 雷达自报：区域 + 实际会话位姿（zone 可为 0） */
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

    sprintf(buf, "leg %u/%u f%u", (unsigned)Launch_GetLegCount(), (unsigned)RADAR_LEG_COUNT,
            (unsigned)Launch_GetFailure());
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
 * 六段摘要：一行一段，格式 "L1 3>14 s0 n2"
 *   Lx 段号 / 起点>终点 / s = status（0 成功）/ n = 路径节点数（不含起点、含终点）
 * 段数不足或某段失败时照样显示，方便现场一眼看出卡在哪一段。
 */
void Radar_ShowSummary(void)
{
    char buf[24];
    uint8_t index;
    uint8_t legs = Launch_GetLegCount();

    OLED_Clear();
    /* 5 段用 6x8 小字：每段一行占 8 像素（16 像素字体一屏只能放 4 行，5 段放不下）。
       y = 0/8/16/24/32；失败码放 y=48，别和段行挤在一起。 */
    for (index = 0U; index < (uint8_t)RADAR_LEG_COUNT; ++index) {
        radar_client_path_result_t path;
        if (index < legs && Launch_GetLeg(index, &path)) {
            sprintf(buf, "L%u %u>%u s%u n%u", (unsigned)(index + 1U),
                    (unsigned)path.start, (unsigned)path.goal,
                    (unsigned)path.status, (unsigned)path.waypoint_count);
        } else {
            sprintf(buf, "L%u --", (unsigned)(index + 1U));
        }
        OLED_ShowString(0, (uint8_t)(index * 8U), (uint8_t *)buf, 8, 1);
    }
    if (Launch_GetFailure() != 0U) {
        sprintf(buf, "fail f%u", (unsigned)Launch_GetFailure());
        OLED_ShowString(0, 48, (uint8_t *)buf, 8, 1);
    }
    OLED_Refresh();
}

/* ======================== 雷达收放（舵机） ========================
 * 机构：0° = 雷达伸出（扫描时用）、340° = 雷达收入车内（行驶时用）。
 * 第 4 路舵机 SERVO_RADAR（TIM4_CH1 = PD12），参数在 radar_config.h 的 RADAR_SERVO_*。
 * 第一次调用才初始化这一路（Servo_InitId），不碰机械臂那三路 ——
 * 测试模式里 AGV_Init()/Pick_Init() 没跑也能单独收放雷达。
 */
static bool s_deployed;

void Radar_SetDeployed(bool extended)
{
    if (!Servo_IsReady(SERVO_RADAR)) {
        /* 只有真正没初始化时才初始化（servo.c 里按 SERVO_R_START_ANG = 340°(收起) 起 PWM）。
           若上电已经由 Test_HwInit() 初始化过，这里就不会重复动作。 */
        Servo_InitId(SERVO_RADAR);
    }
    /* 平滑分步转到位（阻塞到命令发完），再等物理到位裕量 */
    Servo_SetAngleSmooth(SERVO_RADAR,
                         extended ? (float)RADAR_SERVO_OUT_DEG : (float)RADAR_SERVO_IN_DEG,
                         (float)RADAR_SERVO_STEP_DEG, (uint16_t)RADAR_SERVO_STEP_MS);
    HAL_Delay((uint32_t)RADAR_SERVO_SETTLE_MS);
    s_deployed = extended;
}

bool Radar_IsDeployed(void)
{
    return s_deployed;
}
/* ======================== 完整扫描的原始坐标（诊断用） ========================
 * 只看方格编号分不清"朝向转了多少"和"X/Y 平移了多少"；完整扫描 0x07 会回传
 * 每个障碍的场地毫米坐标 + 方格号 + 置信度，而且**不写雷达 Flash**（0x08 才写）。
 * 这里存下最近一次结果，屏上打出来与卷尺实测的坐标一比，旋转/平移量就出来了。
 */
static radar_client_obstacle_result_t s_scan_result;
static bool s_scan_ready;

static void on_obstacles(const radar_client_obstacle_result_t *data)
{
    s_scan_result = *data;      /* 回调指针只在回调内有效，整体拷一份 */
    s_scan_ready = true;
}

bool Radar_RunFullScan(uint32_t timeout_ms)
{
    uint32_t start = RadarUartPort_GetTick();

    s_scan_ready = false;
    if (RadarClient_RequestScanOnce() != RADAR_CLIENT_REQUEST_ACCEPTED) return false;

    /* 内部自己 Poll：这段时间不返回；调用方（测试模式）本来就是阻塞式的 */
    while (!s_scan_ready) {
        RadarClient_Poll();
        if ((int32_t)(RadarUartPort_GetTick() - (start + timeout_ms)) >= 0) return false;
    }
    return true;
}

uint8_t Radar_GetScanCount(void)
{
    return s_scan_ready ? s_scan_result.obstacle_count : 0U;
}

bool Radar_GetScanObstacle(uint8_t index, radar_client_obstacle_t *out)
{
    if (out == NULL || !s_scan_ready || index >= s_scan_result.obstacle_count) return false;
    *out = s_scan_result.obstacles[index];
    return true;
}

/* ======================== 方格几何与逐格展开 ========================
 * 说明：雷达回的是**拐点 + 终点**（直线段只回终点，见说明书 §7 路径结果），
 * 不是逐格序列 —— 一段 4→14 只回 1 个点并不奇怪。要驱车/要看清走哪几格，
 * 就在本地把直线段展开。方格编号几何来自说明书 §9（0~24 从 (0,0) 沿 Y 蛇形）：
 *
 *      20 21 22 23 24        row = id / 5
 *      19 18 17 16 15        col = 奇数行 ? 4 - id%5 : id%5
 *      10 11 12 13 14
 *       9  8  7  6  5
 *       0  1  2  3  4
 */

uint8_t Radar_GridRow(uint8_t grid)
{
    return (uint8_t)(grid / 5U);
}

uint8_t Radar_GridCol(uint8_t grid)
{
    uint8_t row = (uint8_t)(grid / 5U);
    uint8_t col = (uint8_t)(grid % 5U);
    return ((row & 1U) != 0U) ? (uint8_t)(4U - col) : col;   /* 奇数行反向 */
}

/* (行,列) → 方格编号（Radar_GridRow/Col 的逆） */
static uint8_t grid_id_of(uint8_t row, uint8_t col)
{
    return (uint8_t)(row * 5U + (((row & 1U) != 0U) ? (uint8_t)(4U - col) : col));
}

/*
 * 把第 leg 段展开成**逐格**编号序列（不含起点、含终点），返回写入的格数；
 * 0 = 无法展开（该段没跑到 / status≠0 / 缓冲不够 / 折点不共行也不共列）。
 * 依据：段落 = 起点 → 拐点 → … → 终点，相邻两点必同行或同列（4 邻接），
 *       沿行/列逐格走过去即可；不共行不共列说明数据不合预期，直接拒绝而不是硬猜。
 */
uint8_t Radar_ExpandLeg(uint8_t leg, uint8_t *cells, uint8_t max_cells)
{
    uint8_t pts[RADAR_PATH_MAX_POINTS + 2U];   /* {起点, 拐点…, 终点} */
    uint8_t n = 0U;
    uint8_t i;
    uint8_t out = 0U;

    if (cells == NULL || leg >= Radar_PathCount) return 0U;
    if (Radar_Path[leg].status != 0U) return 0U;

    pts[n++] = Radar_Path[leg].start;
    for (i = 0U; i < Radar_Path[leg].count && i < (uint8_t)RADAR_PATH_MAX_POINTS; ++i)
        pts[n++] = Radar_Path[leg].points[i];
    if (pts[n - 1U] != Radar_Path[leg].goal) pts[n++] = Radar_Path[leg].goal;
    if (n < 2U) return 0U;                     /* 起点 = 终点：没有路径点 */

    for (i = 1U; i < n; ++i)
    {
        uint8_t r0 = Radar_GridRow(pts[i - 1U]);
        uint8_t c0 = Radar_GridCol(pts[i - 1U]);
        uint8_t r1 = Radar_GridRow(pts[i]);
        uint8_t c1 = Radar_GridCol(pts[i]);

        if (r0 == r1)                          /* 同行：沿列走 */
        {
            while (c0 != c1)
            {
                c0 = (uint8_t)((c1 > c0) ? (c0 + 1U) : (c0 - 1U));
                if (out >= max_cells) return 0U;
                cells[out++] = grid_id_of(r0, c0);
            }
        }
        else if (c0 == c1)                     /* 同列：沿行走 */
        {
            while (r0 != r1)
            {
                r0 = (uint8_t)((r1 > r0) ? (r0 + 1U) : (r0 - 1U));
                if (out >= max_cells) return 0U;
                cells[out++] = grid_id_of(r0, c0);
            }
        }
        else
        {
            return 0U;                         /* 折点不共线：不猜 */
        }
    }
    return out;
}

/* ======================== 障碍方格 ========================
 * 薄封装：真正的数据在 launch_mission 里（方格扫描的 grid 回调存下来的）。
 * 这里对上只暴露"数量 / 按序号取 / 某个格子是不是障碍"三件，
 * 整车流程和屏幕显示都不用直接碰 launch_mission。
 */

uint8_t Radar_GetObstacleCount(void)
{
    return Launch_GetGridCount();
}

bool Radar_GetObstacle(uint8_t index, uint8_t *grid_id)
{
    return Launch_GetGridId(index, grid_id);
}

/* 画 5×5 障碍图用：逐个比对（最多 25 项，开销可以忽略）。 */
bool Radar_GridIsObstacle(uint8_t grid)
{
    uint8_t count = Launch_GetGridCount();
    uint8_t i;

    for (i = 0U; i < count; ++i) {
        uint8_t id;
        if (Launch_GetGridId(i, &id) && id == grid) return true;
    }
    return false;
}

/* ======================== 六段路径点（全局） ======================== */

radar_path_t Radar_Path[RADAR_LEG_COUNT];
uint8_t Radar_PathCount;

/*
 * 把 launch_mission 的逐段结果搬到六段数组里。
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
    callbacks.obstacles = on_obstacles;   /* 完整扫描的原始坐标（诊断用，见下面 Radar_RunFullScan） */
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
    memset(&s_radar_pose, 0, sizeof(s_radar_pose));   /* HELLO 还没来，别留着上次的值 */
    (void)Radar_SetZone((uint8_t)RADAR_ZONE_DEFAULT);
}

void Radar_Service(void)
{
    RadarClient_Poll();
    Launch_Poll();
    /* 每规划完一段就同步到六段数组：整车流程不必等六段全跑完才能用第一段 */
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

/* 雷达自报的位姿（HELLO）：选完区后就是该区 Flash 里那份，即上位机标定的值。
   zone=0 表示工作区域还没选定（此时位姿字段为 0，说明书 §6），当作"还没拿到"。 */
bool Radar_GetRadarPose(radar_client_pose_t *out)
{
    if (out == NULL || s_radar_pose.zone == 0U) return false;
    *out = s_radar_pose;
    return true;
}

/* 本机 config 里那套值（radar_config.h）。LAUNCH_USE_SAVED_POSE=0 时它会被 SET_POSE 发出去，
   =1 时不发，仅用于在屏上与雷达那份对照。 */
bool Radar_GetConfigPose(radar_client_pose_t *out)
{
    if (out == NULL || !s_config_valid) return false;
    *out = s_config_pose;
    return true;
}

/* true = 不覆盖雷达里保存的位姿（LAUNCH_USE_SAVED_POSE=1） */
bool Radar_IsUsingSavedPose(void)
{
    return (LAUNCH_USE_SAVED_POSE != 0U);
}

/* 按当前起点格把整条路线交给 launch_mission（**逐段显式**，不用节点链）。
 * 原因：第 5 段是 `22→14`（结束在 14），而第 6 段按要求是 **从 22 回起点格** ——
 * "上一段终点"和"下一段起点"对不上（14 ≠ 22），链式模型表达不了，
 * 所以用 Launch_SetLegs(starts, goals, n) 把每段的起终点都写明。
 * 回程起点取 RADAR_RETURN_FROM_GRID（22），回程终点取当前起点格
 * （区2 = 0、区1 = 4）→ 换启停区、用 Radar_SetStartGrid() 改起点都会自动跟上。
 * 段数 6 ≤ LAUNCH_MAX_LEGS(7)；两个数组按 LAUNCH_MAX_LEGS 开，容量够。 */
static void Radar_ComposeRoute(uint8_t start_grid)
{
    static const uint8_t goals[] = LAUNCH_ROUTE_GOALS;
    uint8_t starts[LAUNCH_MAX_LEGS];
    uint8_t ends[LAUNCH_MAX_LEGS];
    uint8_t n = 0U;
    uint8_t i;
    uint8_t prev = start_grid;

    /* 去程（链式）：起点→2→14→10→22→14 */
    for (i = 0U; i < (uint8_t)(sizeof(goals) / sizeof(goals[0])); ++i) {
        starts[n] = prev;
        ends[n] = goals[i];
        prev = goals[i];
        ++n;
    }
#if RADAR_RETURN_TO_START
    /* 第 6 段（非链式）：从 22 回到启停区出发点方格 */
    starts[n] = (uint8_t)RADAR_RETURN_FROM_GRID;
    ends[n] = start_grid;
    ++n;
#endif
    /* 此时还没进入规划阶段，正常不会失败；失败只影响路线，不影响选区/位姿 */
    (void)Launch_SetLegs(starts, ends, n);
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
    /* 路线也按这个区重新拼：回程段终点 = 本区的出发点方格（见 Radar_ComposeRoute 的说明） */
    Radar_ComposeRoute(start_grid);

    s_zone = zone;
    s_start_grid = start_grid;

    /* 记下 config 里这套值：LAUNCH_USE_SAVED_POSE=0 时上面那句会把它发出去，
       =1 时只用于屏上对照（看它和雷达自报的差多少）。 */
    s_config_pose.x_mm = x_mm;
    s_config_pose.y_mm = y_mm;
    s_config_pose.heading_q6 = heading_q6;
    s_config_pose.zone = zone;
    s_config_valid = true;
    return true;
}

bool Radar_SetStartGrid(uint8_t grid)
{
    if (grid > 24U) return false;
    if (!Launch_SetStart(grid)) return false;   /* 已经进入规划阶段 */
    s_start_grid = grid;
    Radar_ComposeRoute(grid);                   /* 起点变了，回程终点也得跟着变 */
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

    Radar_SyncPathArrays();      /* 存进六段数组（Radar_Path[]），整车流程可直接读 */
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
