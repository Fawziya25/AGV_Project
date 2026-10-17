#ifndef __RADAR_CONFIG_H
#define __RADAR_CONFIG_H

/*
 * 雷达参数 —— 本项目唯一配置处
 * ==========================================================
 * 术语（容易混，先分清）：
 *   启停区 / 工作区域 zone —— 雷达自己的 1 或 2，两区的位姿**独立保存**；
 *                             启动菜单里选的就是它，对应命令 SELECT_ZONE(0x05)。
 *   出发点方格 0~24        —— PLAN_PATH(0x14) 的第一个参数，是 5×5 方格图上
 *                             "车现在停在哪个格"。雷达不会自己知道车停在哪，必须由我们给；
 *                             本项目按"每个启停区一个固定出发点"处理。
 *
 * ⚠ 「待填」的值只有装机现场才知道：雷达安装坐标/朝向、两个启停区各自的出发点方格。
 *    区域填错/位姿越界不会损坏硬件，但客户端会在本地拦下（一帧都不发），
 *    现象是"雷达任务刚开始就结束"、屏幕第 3 行显示 f2。
 *
 * 方格编号 0~24 是雷达给的 5×5 蛇形编号，不是车的场地坐标：
 *   20 21 22 23 24 / 19 18 17 16 15 / 10 11 12 13 14 / 9 8 7 6 5 / 0 1 2 3 4
 */

/* ---- 身份 ---- */

#define LAUNCH_NODE_ID            1U      /* 雷达节点ID，必须与雷达内保存的一致（单设备默认 1） */

/* ---- 默认启停区（菜单阶段2 里可以用 C6 改成另一个）---- */

#define RADAR_ZONE_DEFAULT        2U      /* 1 或 2 */

/* ---- 上位机"朝向偏移" → 协议 heading 的关系（2026-10-14）----
 *
 * ⚠⚠ 2026-10-14 更新：**位姿已在上位机里设置好并写入雷达 Flash**，
 *      本程序 `LAUNCH_USE_SAVED_POSE = 1`，下面这一整套值**不再下发**，
 *      只作"万一以后要改回程序设位姿"时的参考。**别指望改这里能影响扫描结果。**
 *
 * 用户结论：**雷达位姿 = 180° + 上位机里的朝向偏移**。
 * 用现场那两组值验一遍：
 *      区1: 180 + 107.2  = 287.2°  → heading_q6 = 18381
 *      区2: 180 + (−72.8) = 107.2°  → heading_q6 =  6861
 * （287.2° 与 −72.8° 是同一个角度，两区正好互换，符合"两区雷达机械上反装 180°"。）
 *
 * 做成"关系 + 偏移"两个量：以后关系要改（比如实际是 −偏移、或还有别的基准），
 * **只动 RADAR_HEADING_BASE_DEG 这一行**；上面那两个偏移值保持上位机里显示的原样，
 * 方便和上位机界面对照。 */
#define RADAR_HEADING_BASE_DEG     180.0f   /* 关系里的那个常数项（"180°+偏移"里的 180°） */

/* 上位机里各启停区显示/设定的"朝向偏移"（原样照抄，别在这里换算） */
#define RADAR_ZONE1_OFFSET_DEG     107.2f
#define RADAR_ZONE2_OFFSET_DEG     (-72.8f)

/* ---- 启停区 1：雷达位姿（**参考值，当前不下发**，见下方 LAUNCH_USE_SAVED_POSE）+ 出发点方格 ---- */

#define RADAR_ZONE1_POSE_X_MM          330     /* 参考值（不下发）：本位姿以上位机写入雷达 Flash 的为准 */
#define RADAR_ZONE1_POSE_Y_MM          2200    /* 参考值（不下发） */
#define RADAR_ZONE1_POSE_HEADING_DEG   (RADAR_HEADING_BASE_DEG + RADAR_ZONE1_OFFSET_DEG)  /* = 287.2f */
#define RADAR_ZONE1_START_GRID         4U      /* 待填：车停在启停区1 时所在的方格 0~24 */

/* ---- 启停区 2：雷达位姿（**参考值，当前不下发**）+ 出发点方格 ---- */

#define RADAR_ZONE2_POSE_X_MM          330     /* 参考值（不下发） */
#define RADAR_ZONE2_POSE_Y_MM          200     /* 参考值（不下发） */
#define RADAR_ZONE2_POSE_HEADING_DEG   (RADAR_HEADING_BASE_DEG + RADAR_ZONE2_OFFSET_DEG)  /* = 107.2f */
#define RADAR_ZONE2_START_GRID         0U      /* 待填：车停在启停区2 时所在的方格 0~24 */

/* ---- 路线（去程）：起点 → 2 → 14 → 10 → 22 → 14，之后再加一段"回程"回启停区 ----
 * 这里只列**去程**各段的终点；第 6 段（回程）的终点是"起点格"本身，由
 * radar_task.c 的 Radar_ComposeRoute() 组路线时自动补上（见下面的 RADAR_RETURN_TO_START）。
 * 起点 = 出发点方格，跟着启停区走（区2 = 0、区1 = 4，见上面 RADAR_ZONEx_START_GRID）。 */
#define LAUNCH_ROUTE_GOALS        { 2U, 14U, 10U, 22U, 14U }

/* 1 = 去程走完后再规划第 6 段：**从 22（RADAR_RETURN_FROM_GRID）回到启停区出发点方格**。
 *     注意这一段**不是接着第 5 段的**：第 5 段是 22→14（结束在 14），而回程按要求
 *     从 22 出发 —— 两段的端点对不上，所以回程起点必须显式给出，不能靠"上一段终点"推。
 * 0 = 只规划去程 5 段（第 6 个槽在屏上显示 "--"）。 */
#define RADAR_RETURN_TO_START     1
#define RADAR_RETURN_FROM_GRID    22U   /* 第 6 段起点（改路线时记得一起改） */

/* ---- 雷达收放舵机（第 4 路 SERVO_RADAR，TIM4_CH1 = PD12，见 servo.c）----
 * 机构：**舵机 0° = 雷达伸出（扫描时用），300° = 雷达收入车内（行驶时用）**。
 * （2026-10-14 变动：360° → 180°（保险）→ **300°**（用户确认）；舵机量程仍是 0~360°。）
 * ⚠ 0°↔300° 是 300° 的大行程（走长边），单程约 0.9s；如果实际机构只允许短边(60°)，
 *   需要给平滑函数加『最短路径』逻辑 —— 现在按 300° 长边执行。
 * 时序封装在 radar_task 的 Radar_SetDeployed(true/false)（内部阻塞到舵机到位）：
 *     伸出 → Radar_RunOnce()（方格扫描 + 六段规划）→ 收回 → 再开跑。
 * 舵机上电初值是 300°（收起，= 2167us），避免一上电雷达就弹出来。 */
#define RADAR_SERVO_OUT_DEG       0.0f     /* 伸出角（扫描时） */
#define RADAR_SERVO_IN_DEG        300.0f   /* 收起角（行驶时）；2026-10-14：360°→180°(保险)→300°(用户确认) */
#define RADAR_SERVO_STEP_DEG      1.5f     /* 平滑分步：每步角度(°)，0°↔300° 行程约 200 步(≈400ms) */
#define RADAR_SERVO_STEP_MS       2U       /* 每步间隔(ms)：1.5°/2ms ≈ 750°/s，整程约 720ms */
#define RADAR_SERVO_SETTLE_MS     500U     /* 转完后的物理到位裕量(ms)，之后才扫描/开跑 */

/* ---- 行为开关 ---- */

/* ★ 位姿由谁设（2026-10-14 定稿）★
 *   **用户已在上位机里把位姿设好并写入雷达 Flash** → 本程序**不做任何位姿修改**，故取 1。
 *
 *   1 = **不设**（当前）：全程不碰雷达位姿。
 *       - 需要换区时由 SELECT_ZONE 载入该区 Flash 里那份；
 *       - 区域已经一致时，客户端连 SELECT_ZONE 都不发（radar_client.c:477），位姿原样保留；
 *       - 整个流程不会出现 SET_POSE / SAVE_POSE 任何一个字节。
 *       **前提**：对应启停区的位姿已在上位机里保存进雷达（两个区是**独立保存**的，
 *       换区前要各自确认存过）。
 *   0 = 程序自己设（旧做法，**已停用**）：每次上电用下面各区的 X/Y/朝向发 SET_POSE 覆盖会话位姿。
 *       当时踩过坑：值对不上 → 整片坐标系旋转 → 障碍方格位置/数量全错（改动记录 第 62/65 节）。
 *
 * 想读雷达里实际生效的位姿（排查用）：Radar_GetRadarPose()（HELLO 自报）。
 * 下面各区的 X/Y/朝向 现在只是**参考值**（不下发）。 */
#define LAUNCH_USE_SAVED_POSE     1

#define LAUNCH_SAVE_POSE_TO_FLASH 0       /* 0 = 只改本次上电的会话位姿（推荐，不写雷达 Flash）
                                             1 = 写雷达 Flash（掉电保留，但每次上电都会重写）
                                             ※ LAUNCH_USE_SAVED_POSE=1 时本开关不生效（压根不发位姿命令） */
#define LAUNCH_VERIFY_SAVED_GRID  1       /* 1 = 方格扫描后回读保存状态确认 grid_result_valid */

/* ---- 超时（毫秒）---- */

#define LAUNCH_T_CONNECT_MS       8000U   /* 首连接宽限；握手期间客户端每 200ms 重发 HELLO */
#define RADAR_RUN_TMO_MS          30000U  /* 整个雷达任务的外层兜底（5 段：方格扫描 8s + 5×1.5s，留足余量） */

#endif /* __RADAR_CONFIG_H */
