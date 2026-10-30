#ifndef __TEST_H
#define __TEST_H

#include <stdint.h>              /* Dbg_ShowScreen 的 uint8_t 参数 */

/**********************************************************
 * 桌面测试 + 运行时调试（2026-10-04 新建；原 dbg.c/dbg.h 已并进来）
 *
 * 三个互相独立的开关：
 *   TEST_MODE       上电跑哪个测试（0=不测试，跑正式流程）
 *   DBG_ENABLE      运行时调试服务：TIM7 周期刷 OLED + C8 短按暂停/继续
 *                   （C8 只管暂停；A8 是启动键，见下面第一节。2026-10-14 前
 *                     这两个角色由 A8 一个人兼，启动瞬间会跟暂停打架，已拆开）
 *   TEST_CAN_ENABLE 台架上接不接步进电机（0=没接，跳过 CAN 初始化）
 *
 * **正式比赛 TEST_MODE / DBG_ENABLE 都设 0**（TEST_CAN_ENABLE 保持 1）。
 * DBG_ENABLE=0 时 Dbg_Init() / Dbg_Service() 会被
 * 替换成空语句 —— 别的模块里那些调用点（task.c / positionPIDcontroller.c /
 * Emm_V5.c）**一个字都不用删**，编译出来也不占空间、也不覆盖 HAL_Delay。
 **********************************************************/

/* ==========================================================================
 * 一、测试模式
 * ==========================================================================
 * main() 在 AGV_Init() **之前** 调 Test_Switch()：
 *   TEST_MODE == 0  → 立刻返回，正常走 AGV_Init() + TaskProc() 正式流程
 *   TEST_MODE != 0  → 测试自备硬件初始化
 *                     → **阻塞等 A8 短按（一键启动）**
 *                     → 跑对应测试 → 死循环
 *                      程序不会走到 AGV_Init()，也不会进正式流程
 *
 * **加了那道 A8 等待是为了手上还没准备好时不用赶着上电**：先从容上电、把臂挪到
 * 行程中段、把车摆正、尺子拿在手上，再按一下 A8 才开始跑。等待期间 OLED 显示
 * "MODE n / press A8 / to start"，看得见自己跑的是哪个模式。
 * **A8 是启动键，C8 才是暂停键**（2026-10-14 之前两个角色都由 A8 兼，按下去
 * 既"启动"又"暂停"，启动那一下就打架；现在拆成两个物理键）。
 * 等待期间这段用 Dbg_Suspend() 把调试服务整个挡住（见 test.c 的
 * Test_WaitStartKey），取消暂停之外的副作用。
 * TEST_MODE == 0（正式流程）**不经过**这道等待，正式流程的等待还是 AGV_Init()
 * 末尾的 Menu_Init()。
 *
 * 模式表（test.c 的 Test_Switch 按这个分发）：
 *   0   Test_Switch 直接返回，跑正式流程
 *   1   Test_AGV        路径：**雷达舵机伸出(0°) → 一次 14 圈方格扫描 + 六段路径规划
 *                       → 雷达舵机收回(340°) → 直接开跑**下面原有的固定路线各段。
 *                       收放舵机是第 4 路 `SERVO_RADAR`（TIM4_CH1 = PD12），
 *                       `Radar_SetDeployed(true/false)` 封装（内部阻塞到舵机到位）。
 *                       `TEST_AGV_WITH_RADAR=0` 跳过整段雷达（含舵机收放，台架只验底盘用）。
 *                       注意：开跑走的是原有 `AGV_Position_xxx` 固定路线，**不是**把雷达
 *                       规划出的方格路径喂给底盘（那还缺"方格→场地坐标"映射）
 *   2   Test_Correct    视觉修正：夹爪摆观察位 + 问圆盘中心(0x02) + AGV_Correct 闭环
 *   3   Test_Pick_Scale 夹爪开合死循环(POS_RUN 不动)，调 PICK_GRAB/PICK_OBSERVE 角度用
 *   4   Test_Pick_Raw   原料 A/B/C 框 → 车上 1/2/3 槽（纯动作，不扫码不碰相机）
 *   5   Test_Pick_Place 车上 1/2/3 槽 → 圆环 1/2/3 号（纯动作，不扫码不碰相机）
 *   6   Test_Func       空壳，留给以后单步测带视觉的完整流程
 *   7   Test_Camera     相机指令轮询：每 5s 依次发 0x00~0x05，屏上看相机回的帧
 *                       （0x01 参数发 1,2,3，0x05 发颜色号 2，其余无参数）
 *   8   Test_OLED       OLED 自检：甲段(TIM7 不启动)黑白闪 3 下 + 标记字，
 *                       乙段(手工起 TIM7)看调试画面会不会刷出来 —— 用来分辨
 *                       "屏/接线坏了"和"TIM7 刷屏把屏冲没了"。详见 test.c 里
 *                       那个函数头上的一整段判读说明。
 *   9   Test_Motor      臂测试，五个子测试一次跑一个（换 test.c 里的注释切换）：
 *                       高度电机(地址5) 0<->30mm 往返 —— pick.c 原来写"一圈=8mm"，
 *                         机械上是 60mm/圈，改完要拿这个循环验方向+比例；
 *                       半径电机(地址6) 0<->30mm 往返 —— 100mm/圈 还是纸面值；
 *                       转盘舵机(PA5) A(30°)/B(150°)/C(270°) 循环 —— 2026-10-14
 *                         接线和 Z轴对调过，换完要验，顺带看三个槽的停位；
 *                       Z轴舵机(PE9) 0°/100°/50° 循环 —— 同上，对调的另一半；
 *                       夹爪舵机(PA6) 观察/抓紧/放开 走 3 轮，停在抓紧
 *                         —— 三个角度一直没单独验过。这个是**唯一会返回**的子测试。
 *                       函数体在 pick.c 的 Pick_Test_MotorZ / MotorR / TurnTable /
 *                       ZServo / Gripper，判读方法写在各自函数头上。
 *                       **高度/半径要 CAN**：必须 TEST_CAN_ENABLE=1 且被测电机真的
 *                       接在总线上；跑之前把臂放行程中段。**三个舵机不用 CAN**，
 *                       但摆 Z 轴之前要先**把臂抬起来**（运输位高度只有 10mm）。
 *                       夹爪那个只碰夹爪，不用管臂的位置。
 *  10  Test_Radar      雷达（UART4 / PC10=TX PC11=RX / 115200）：
 *                       只碰雷达 + OLED，不初始化相机/机械臂/陀螺仪（CAN 仍由
 *                       Test_HwInit 按 TEST_CAN_ENABLE 决定，但雷达测试用不上）。
 *                       一次 A8 后自动跑完：选默认启停区 → 握手 → 设位姿 →
 *                       **一次 14 圈方格扫描** → **六段**路径规划 → STOP
 *                       （2026-10-14：起点→2→14→10→22→14 共 5 段，另加第 6 段**22→起点**回启停区，
 *                        所以是 6 段；第 6 段不接第 5 段的 14，起点显式取 RADAR_RETURN_FROM_GRID）。
 *                       跑完分页显示（C6 下一页 / C8 上一页 / A8 重跑）：
 *                         页0 障碍方格：5×5 图，"XX" = 有障碍、数字 = 空闲（方格扫描的结果）
 *                         页1 六段概览：标题 "legs n/6 f码" + 每段一行 Lx 起点>终点 s状态 n拐点数
 *                         页2~7 第 1~6 段的**逐格**路线（雷达只回"拐点+终点"、直线段
 *                              只回终点，屏上按说明书 §9 展开成逐格编号，6x8 小字）
 *                       参数在 Radar/Inc/radar_config.h（`LAUNCH_USE_SAVED_POSE=0` 时程序
 *                       每次上电自己发 SET_POSE；段数由 RADAR_LEG_COUNT + LAUNCH_ROUTE_GOALS 决定）；
 *                       障碍读 Radar_GetObstacleCount() / Radar_GetObstacle() / Radar_GridIsObstacle()，
 *                       位姿读 Radar_GetRadarPose() / Radar_GetConfigPose()，
 *                       路径在 Radar_Path[5]（都在 Radar/Inc/radar_task.h）。**不返回。**
 *  11  Test_Place_Ring  最简单的放置过程：只跑「车上取料位 POS_CAR → 圆环 1/2/3」，
 *                       转盘固定在 A 槽不动，不扫码、不碰相机、不做视觉修正。
 *                       每个圆环都是 POS_CAR 取料 → 夹紧 → POS_x → 放开，
 *                       三次跑完收到 POS_RUN。用来单独验绝对位置模式的
 *                       "取-放"动作链。**前提：上电前两轴人工放 0 位。**
 *
 * **测试之间的分界线在初始化上**：Test_HwInit() 只做共用的那几样
 * （OLED / 调试服务 / CAN / DWT / 航向PID），各外设由每个测试函数在自己开头
 * 按需初始化 —— 只测臂的就不用白等相机。所以选模式 3~5 时相机、陀螺仪都是
 * 没起来的，正常。
 *
 * CAN 之所以也算"共用"、并进 Test_HwInit：C8 暂停在**任何**模式下都会生效，
 * 而暂停第一件事就是 AGV_Position_Stop() 往 CAN 上发四轮停车帧。哪个模式漏
 * 了 AGV_Can_Init()，暂停就会每帧空等到 EMM_V5_ACK_TMO_MS(200ms) 才返回。
 * 它自带 HAL_Delay(500) 等 CAN 稳定，放最前面正好。
 * 【但台架上没接电机时要关掉，见下面第三个开关 TEST_CAN_ENABLE。】
 *
 * 加新测试：这里占一个号 → test.c 里写一个 static 测试函数 → 在
 *          Test_Switch 的 switch 里加一个 case。步骤在 test.c 顶部再写一遍。
 * 三个开关都可以从编译命令行覆盖（#ifndef 挡住重复定义），比如临时编一版
 * 测路径的：cmake ... -DCMAKE_C_FLAGS="-DTEST_MODE=1"。不传就按下面的默认值。
 */
#ifndef TEST_MODE
#define TEST_MODE   1      /* 2026-10-14：调原料→车上槽位(模式 4)，测完改回 0 */
#endif

#ifndef DBG_ENABLE
#define DBG_ENABLE  0
#endif


void Test_Switch(void);

#if DBG_ENABLE
/* 启动 TIM7 中断（MX_TIM7_Init 已经把 10ms 配置好了，这里只负责 Start_IT）。
   必须在 OLED_Init() 之后调 —— 中断里要驱动软件 I2C 刷屏。 */
void Dbg_Init(void);
/* 调试服务：暂停判断 + 到点刷屏。在中断上下文里调用会直接返回 */
void Dbg_Service(void);
/* 挂起 / 恢复刷屏：屏幕被别的模块占用期间（目前只有 Menu_Init 的启动菜单）
   调 Suspend，用完调 Resume。挂起期间 Dbg_Service 整个让路 —— 不刷屏也不
   响应 C8 暂停（菜单里 A8 才是启动键）。Resume 会立刻补一次重画。 */
void Dbg_Suspend(void);
void Dbg_Resume(void);
#else
/* 关掉时调用点原地变成空语句，别的模块（menu.c 等）不用改 */
#define Dbg_Init()     ((void)0)
#define Dbg_Service()  ((void)0)
#define Dbg_Suspend()  ((void)0)
#define Dbg_Resume()   ((void)0)
#endif

/* 整屏显示：相机 mode/flag/dx/dy + 任务码 12 位 + C8 提示。
   **不随 DBG_ENABLE 走** —— Cam_Init() 里上电诊断也要用它（原 Cam_OLED_Show()
   已删除，由它取代）。paused=1 显示暂停画面。 */
void Dbg_ShowScreen(uint8_t paused);

#endif /* __TEST_H */
