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

#define RADAR_ZONE_DEFAULT        1U      /* 1 或 2 */

/* ---- 启停区 1：雷达位姿 + 出发点方格 ---- */

#define RADAR_ZONE1_POSE_X_MM          150     /* 待填：雷达自身在场地中的 X（mm），两区都 -200~500 */
#define RADAR_ZONE1_POSE_Y_MM          2200    /* 待填：区域 1 的 Y 必须 1900~2600 */
#define RADAR_ZONE1_POSE_HEADING_DEG   0U      /* 待填：雷达朝向 0~359°（0° 定义见说明书 §9） */
#define RADAR_ZONE1_START_GRID         4U      /* 待填：车停在启停区1 时所在的方格 0~24 */

/* ---- 启停区 2：雷达位姿 + 出发点方格 ---- */

#define RADAR_ZONE2_POSE_X_MM          150     /* 待填：两区都 -200~500 */
#define RADAR_ZONE2_POSE_Y_MM          0       /* 待填：区域 2 的 Y 必须 -200~500 */
#define RADAR_ZONE2_POSE_HEADING_DEG   0U      /* 待填 */
#define RADAR_ZONE2_START_GRID         0U     /* 待填：车停在启停区2 时所在的方格 0~24 */

/* ---- 四段路线：起点 -> 14 -> 10 -> 22 -> 14 ---- */

#define LAUNCH_ROUTE_GOALS        { 14U, 10U, 22U, 14U }

/* ---- 行为开关 ---- */

#define LAUNCH_SAVE_POSE_TO_FLASH 0       /* 0 = 只改本次上电的会话位姿（推荐，不写雷达 Flash）
                                             1 = 写雷达 Flash（掉电保留，但每次上电都会重写） */
#define LAUNCH_VERIFY_SAVED_GRID  1       /* 1 = 方格扫描后回读保存状态确认 grid_result_valid */

/* ---- 超时（毫秒）---- */

#define LAUNCH_T_CONNECT_MS       8000U   /* 首连接宽限；握手期间客户端每 200ms 重发 HELLO */
#define RADAR_RUN_TMO_MS          20000U  /* 整个雷达任务的外层兜底（内部各步还有自己的超时） */

#endif /* __RADAR_CONFIG_H */
