/**
 * @file pick.c
 * @brief 物料抓取模块：3 舵机（Z轴旋转/夹爪/车上转盘）+ 2 步进电机（高度/半径）
 *        当前版本：视觉默认已对准，只做机械臂动作控制。
 * @note  步进电机与四轮完全同协议(Emm_V5)，相对位置模式(raF=2)，地址5/6。
 *        移植到新工程：只改本文件顶部"可移植配置区"。
 */
#include "pick.h"
#include "Emm_V5.h"
#include "servo.h"
#include "oled.h"
#include "test.h"          /* Dbg_Service —— Pick_Test_Gripper() 末尾那个空转保底循环
                              是纯自旋，不经过被 test.c 盖掉的 HAL_Delay，得自己调。
                              （Emm_V5.c / positionPIDcontroller.c 同样这么干） */
#include "stm32f4xx_hal.h"
#include <math.h>          /* fabsf */

/* ===================== 可移植配置区 ===================== */
#define PICK_MOTOR_Z_ADDR   5     /* 高度步进电机 CAN 地址 */
#define PICK_MOTOR_R_ADDR   6     /* 半径步进电机 CAN 地址 */
/* 各电机正方向对应的dir值(0/1)：两个轴的方向定义可以不同，反了改各自的值(取反)。
   当前 Z=0 / R=1。**这两个数都还没实测过** —— 模式 9 的 0<->30mm 往返测试就是
   用来实测它们的：看臂往哪边走，反了就取反。 */
#define PICK_MOTOR_Z_POS_DIR  0
#define PICK_MOTOR_R_POS_DIR  0
/* 各电机速度/加速度(RPM)：可分别调节 */
#define PICK_MOTOR_Z_VEL    1000   /* 高度电机速度；注意是原值100的10倍，不是注释里写的2倍 */
#define PICK_MOTOR_Z_ACC    230  /* 高度电机加速度 */
#define PICK_MOTOR_R_VEL    100   /* 半径电机速度，保持原样 */
#define PICK_MOTOR_R_ACC    20    /* 半径电机加速度 */
#define PICK_MOTOR_DONE_TMO 2000  /* 电机回执等待超时(ms) */
#define PICK_SAFE_HEIGHT_MM 2.0f  /* 安全高度(mm)：每次动作先升到此高度再水平移动+转向 */

/* ---- 失败恢复参数 ---- */
#define PICK_MOTOR_RETRY_TMO    2000  /* 失败后降速重试的等待超时(ms) */
#define PICK_READ_POS_TMO        300  /* 读 S_CPOS 实时位置的超时(ms) */
#define PICK_MOTOR_Z_RETRY_VEL   300  /* 高度轴重试速度，低于正常值，减小顶限位冲击 */
#define PICK_MOTOR_Z_RETRY_ACC    30
#define PICK_MOTOR_R_RETRY_VEL   100
#define PICK_MOTOR_R_RETRY_ACC    20

/* 读回的 S_CPOS(驱动器绝对位置) 与 MCU 跟踪值的符号关系：
   +1=同向，-1=反向。若实测读回位置与 mm 坐标增减相反，把这里改成 (-1)。 */
#define PICK_MOTOR_Z_CPOS_SIGN   (+1)

/* ---- 上电碰撞回零（可选，默认关闭） ----
   0 = 现状：上电前人工把两轴放到 0 位，Pick_Axis_Init() 发 0x0A 把该位置设为绝对零点。
   1 = 改为上电自动回零：写回零参数 → 触发 0x9A 模式2(多圈无限位碰撞回零) → 等结束。
       两轴分别朝各自的机械限位顶，驱动器检测到堵转后把限位位置设成零点。
   ⚠ 打开前必须确认两件事，否则会撞坏机构或让所有 POS_* 坐标整体偏移：
     1) 那个机械限位就是标定 POS_* 时用的 0 位（不是的话所有高度/半径值要重标）；
     2) 碰撞检测电流/时间按实际机构调，别用小电流快速撞。
   回零方向不用实测：回零永远是"从 >0 的坐标往 0(限位)走"，即位置命令的负方向，
   dir = 1 - POS_DIR。位置命令里的正方向宏改过，这里会自动跟着变。
   回零参数：速度 RPM、超时 ms、检测电流 mA、检测时间 ms。svF=false 只写 RAM，
   不改驱动器 EEPROM；确认参数稳定后可以改成 true 存一次。 */
#define PICK_AXIS_AUTO_HOME      1
#define PICK_HOME_Z_DIR          (1 - PICK_MOTOR_Z_POS_DIR)  /* 朝 0/限位：位置命令的负方向 */
#define PICK_HOME_R_DIR          (1 - PICK_MOTOR_R_POS_DIR)  /* 同上 */
#define PICK_HOME_VEL            100   /* 回零速度 RPM（碰撞回零建议先慢） */
#define PICK_HOME_TMO            10000 /* 驱动器侧回零超时 ms */
#define PICK_HOME_STALL_VEL      30    /* 碰撞检测转速 RPM */
#define PICK_HOME_STALL_MA       800   /* 碰撞检测电流 mA（按电机/机构调） */
#define PICK_HOME_STALL_MS       300   /* 碰撞检测时间 ms */
#define PICK_HOME_Z_WAIT_MS      2000 /* MCU 侧等 Z 回零完成的上限 ms */
#define PICK_HOME_R_WAIT_MS      6000  /* 半径不回到位帧，只能固定等这么久 */

/* 一次步进动作的结果 */
#define PICK_MOVE_TIMEOUT  0U   /* 没等到到位帧 */
#define PICK_MOVE_OK       1U   /* 真到位(0xFD 0x9F) */
#define PICK_MOVE_REJECT   2U   /* 驱动器拒收(0xFD 0xE2)：堵转保护/未使能 */
#define PICK_MOVE_UNKNOWN  3U   /* 失败后位置又读不回来，模型不可信 */

/* 诊断计数：只给调试器 watch / OLED 看，不参与控制 */
volatile uint32_t Pick_Diag_TimeoutCnt = 0U;   /* Z 轴等待超时次数 */
volatile uint32_t Pick_Diag_RejectCnt  = 0U;   /* Z 轴收到 0xFD 0xE2 次数 */
volatile uint8_t  Pick_Diag_LastStatus = PICK_MOVE_OK;  /* 最近一次 Z 动作结果 */
volatile int32_t  Pick_Diag_LastCpos   = 0;    /* 最近一次读回的 Z 位置(脉冲) */
/* 舵机运动参数 SERVO_Z_* / SERVO_G_* / SERVO_T_* 都定义在 servo.h，三路独立 */

/* 高度电机(地址5) 行程配置: 一圈=60mm, 一圈3200脉冲 → 53脉冲/mm
   2026-10-11 更正：原来这里写的是"逆时针一圈=8mm"(400脉冲/mm)，机械上量出来
   是 60mm/圈，差了 7.5 倍。旧数字的后果：调 MoveToMM(30) 想走 3cm，实际会发
   12000 脉冲 = 3.75 圈 = 实走 225mm。用 POS_* 那批位置宏验证一下也能看出来旧
   数字不对 —— POS_OBSERVE_CIRCLE 高度写 140mm，按 8mm/圈 是 17.6 圈(不可能)，
   按 60mm/圈 是 2.3 圈(合理)。 */
#define PICK_MOTOR_Z_MM_PER_REV   60.0f   /* 一圈对应 60mm */
#define PICK_MOTOR_Z_PUL_PER_REV  3200    /* 一圈脉冲数(16细分×1.8°) */
#define PICK_MOTOR_Z_MAX_MM       130.0f  /* 高度行程上限 0~130mm */
#define PICK_MOTOR_Z_MM_PUL       ((int32_t)(PICK_MOTOR_Z_PUL_PER_REV / PICK_MOTOR_Z_MM_PER_REV)) /* 53脉冲/mm */
/* ↑ 3200/60=53.33 被整数截断成 53，换算偏小约 0.6%(走 140mm 差 0.8mm)。
   要更准就得把这个宏和下面两处乘法的中间量改成浮点，等行程实测确认了再说。 */

/* 半径电机(地址6) 行程配置: 一圈=100mm, 一圈3200脉冲 → 32脉冲/mm */
#define PICK_MOTOR_R_MM_PER_REV   100.0f  /* 一圈对应 100mm */
#define PICK_MOTOR_R_PUL_PER_REV  3200    /* 一圈脉冲数 */
#define PICK_MOTOR_R_MAX_MM       85.0f   /* 半径行程上限 0~85mm */
#define PICK_MOTOR_R_MM_PUL       ((int32_t)(PICK_MOTOR_R_PUL_PER_REV / PICK_MOTOR_R_MM_PER_REV)) /* 32脉冲/mm */
/* ======================================================= */

/**
 * @brief 等步进电机回帧，并把"到位 / 拒收 / 超时"三种结果分开
 * @retval PICK_MOVE_OK(0xFD 0x9F) / PICK_MOVE_REJECT(0xFD 0xE2) / PICK_MOVE_TIMEOUT
 * @note  不能直接用 Emm_V5_WaitArrive()：它只认 0x9F，会把 0xE2 当成"继续等"，
 *        最后只能报一个笼统的超时。0xE2 的含义是"命令没被执行"（堵转保护/未使能），
 *        和"走到一半超时"的处置方式完全不同，必须分开。
 */
static uint8_t Pick_WaitArriveEx(uint32_t tmo_ms)
{
    uint32_t t0 = HAL_GetTick();

    while((HAL_GetTick() - t0) < tmo_ms)
    {
        if(can.rxFrameFlag)
        {
            can.rxFrameFlag = false;            /* 消费本帧 */
            if(can.rxData[0] == 0xFD)
            {
                if(can.rxData[1] == 0x9F) return PICK_MOVE_OK;
                if(can.rxData[1] == 0xE2) return PICK_MOVE_REJECT;
            }
        }
    }
    return PICK_MOVE_TIMEOUT;
}

/**
 * @brief 失败后的驱动器恢复：解除堵转保护 + 重新使能
 * @note  Emm_V5 的堵转保护是**锁存**的：顶到机械限位后不发 0x0E 解除，
 *        后续位置命令会一直以 0xFD 0xE2 拒收，直到断电重启。
 *        本工程原来从未调用过 Emm_V5_Reset_Clog_Pro()，这是"卡住后只能断电"的直接原因。
 */
static void Pick_MotorRecover(uint8_t addr)
{
    Emm_V5_Reset_Clog_Pro(addr);
    HAL_Delay(20);
    Emm_V5_En_Control(addr, true, false);
    HAL_Delay(20);
    Emm_V5_ClearRxFrame();
}

/**
 * @brief 读回电机实时位置(S_CPOS / 0x36)，用于失败后重同步 MCU 的跟踪值
 * @retval 1=读成功；0=超时或回帧布局不符
 * @note  按标准回帧 [0xFD, 0x36, p3, p2, p1, p0] 解析。若实际固件回帧不同，
 *        本函数只会返回 0，随后走"位置未知→本次动作中止"的安全路径，不会乱发命令。
 */
static uint8_t Pick_ReadAxisPos(uint8_t addr, int32_t *pos)
{
    uint32_t t0;
    uint32_t u;

    Emm_V5_ClearRxFrame();
    HAL_Delay(2);
    Emm_V5_Read_Sys_Params(addr, S_CPOS);

    t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < PICK_READ_POS_TMO)
    {
        Dbg_Service();
        if(can.rxFrameFlag)
        {
            can.rxFrameFlag = false;
            if(can.rxData[0] == 0xFD && can.rxData[1] == 0x36)
            {
                u = ((uint32_t)can.rxData[2] << 24) |
                    ((uint32_t)can.rxData[3] << 16) |
                    ((uint32_t)can.rxData[4] << 8)  |
                    ((uint32_t)can.rxData[5]);
                *pos = (int32_t)u;
                return 1;
            }
        }
    }
    return 0;
}

/* 半径电机当前位置(脉冲，绝对坐标) */
static int32_t pick_motorRPos = 0;

/* 两轴驱动器是否已经建立零点/使能 */
static uint8_t pick_axisReady = 0;

#if PICK_AXIS_AUTO_HOME
/**
 * @brief 写一轴的回零参数（模式2：多圈无限位碰撞回零）
 * @note  svF=false：只作用于本次运行，不写驱动器 EEPROM（确认参数后再改 true 存一次）。
 *        potF=false：不启用驱动器自己的"上电自动回零"，由 MCU 显式触发，时序可控。
 */
static void Pick_Home_WriteParams(uint8_t addr, uint8_t dir)
{
    Emm_V5_Origin_Modify_Params(addr, false, 2, dir,
                                PICK_HOME_VEL, PICK_HOME_TMO,
                                PICK_HOME_STALL_VEL, PICK_HOME_STALL_MA,
                                PICK_HOME_STALL_MS, false);
    HAL_Delay(20);
    Emm_V5_ClearRxFrame();
}

/**
 * @brief 触发一轴碰撞回零并等它结束
 * @param has_frame 1=该轴回到位帧(高度轴)，等到 0xFD 0x9F；0=半径轴，只能固定等待
 * @retval PICK_MOVE_OK / PICK_MOVE_REJECT / PICK_MOVE_TIMEOUT
 * @note  失败会清一次堵转再重触发一次。半径轴收不到回帧，固定等
 *        PICK_HOME_R_WAIT_MS；真正的结束由驱动器 PICK_HOME_TMO 兜底。
 */
static uint8_t Pick_Home_Axis(uint8_t addr, uint8_t dir, uint8_t has_frame, uint32_t wait_ms)
{
    uint8_t r;

    Pick_Home_WriteParams(addr, dir);

    Emm_V5_Origin_Trigger_Return(addr, 2, false);
    HAL_Delay(20);

    if(has_frame)
    {
        int32_t pos;

        r = Pick_WaitArriveEx(wait_ms);
        if(r != PICK_MOVE_OK)
        {
            /* 兜底判据：有的固件回零结束不发 0x9F，但零点已经写成 0，
               这时读回 S_CPOS 应在 0 附近，直接算成功，不要盲目再撞一次。 */
            if(Pick_ReadAxisPos(addr, &pos) && (pos > -100) && (pos < 100))
                return PICK_MOVE_OK;

            Pick_MotorRecover(addr);
            Emm_V5_Origin_Trigger_Return(addr, 2, false);
            HAL_Delay(20);
            r = Pick_WaitArriveEx(wait_ms);
        }
        return r;
    }

    HAL_Delay(wait_ms);
    Emm_V5_ClearRxFrame();
    return PICK_MOVE_OK;
}
#endif

/**
 * @brief 步进电机两轴驱动器初始化：建立绝对零点，并清堵转/使能
 * @note  **必须在 AGV_Can_Init() 之后调用**（本函数会通过 CAN 发命令）。
 *        零点来源由 PICK_AXIS_AUTO_HOME 决定：
 *          0(默认)：**上电前人工把两轴放在 0 位**，本函数发 0x0A 把该位置设为 0mm；
 *          1：不收人工 0 位，两轴各自朝机械限位做碰撞回零(0x9A 模式2)，限位即 0mm。
 *        现在两轴都走绝对位置，**所有动作都依赖这个零点** —— 零点错，臂的
 *        目标高度/半径就整体偏移，所以上电前的人工 0 位必须保持一致。
 *        正常流程里四个 Pick_* 入口会在第一次动作时自动补调一次（见 Pick_Axis_EnsureInit），
 *        所以 task.c 里那句显式调用是"把时序写明"，不是唯一触发点。
 */
void Pick_Axis_Init(void)
{
    Emm_V5_En_Control(PICK_MOTOR_Z_ADDR, true, false);
    HAL_Delay(20);
    Emm_V5_En_Control(PICK_MOTOR_R_ADDR, true, false);
    HAL_Delay(20);

    Emm_V5_Reset_Clog_Pro(PICK_MOTOR_Z_ADDR);
    HAL_Delay(20);
    Emm_V5_Reset_Clog_Pro(PICK_MOTOR_R_ADDR);
    HAL_Delay(20);

#if PICK_AXIS_AUTO_HOME
    /* 先 Z 后 R：Z 先到限位停稳，再做水平方向的回零，避免低位横扫。
       回零成功后驱动器的零点就在机械限位上，**不再发 0x0A** ——
       万一回零没成功，0x0A 会把"当时随便一个位置"错当成零点，比不清更危险。 */
    Pick_Diag_LastStatus = Pick_Home_Axis(PICK_MOTOR_Z_ADDR, PICK_HOME_Z_DIR, 1, PICK_HOME_Z_WAIT_MS);
    (void)Pick_Home_Axis(PICK_MOTOR_R_ADDR, PICK_HOME_R_DIR, 0, PICK_HOME_R_WAIT_MS);
#else
    /* 人工 0 位模式：先使能、清堵转，最后再清零位置，
       保证 0x0A 一定被执行，且清零后不会被使能瞬间的微小动作带偏。 */
    Emm_V5_Reset_CurPos_To_Zero(PICK_MOTOR_Z_ADDR);
    HAL_Delay(20);
    Emm_V5_Reset_CurPos_To_Zero(PICK_MOTOR_R_ADDR);
    HAL_Delay(20);
#endif

    Emm_V5_ClearRxFrame();

    pick_motorRPos = 0;
    pick_axisReady = 1;
}

/* 第一次机械臂动作前自动建立零点；之后是空操作 */
static void Pick_Axis_EnsureInit(void)
{
    if(!pick_axisReady) Pick_Axis_Init();
}

/**
 * @brief 机械臂舵机初始化（只启动 PWM 舵机，不发任何 CAN 命令）
 * @note  本函数在 AGV_Can_Init() **之前**被调用，所以不能在这里碰步进电机。
 *        步进电机零点和使能由 Pick_Axis_Init() 负责，必须在 CAN 起来之后调用。
 *        （旧注释说本函数会把臂收到 POS_START，函数体里其实早就没有这句了。）
 */
void Pick_Init(void)
{
    Servo_Init();
}

/* ==================== 绝对位置模式：高度轴 + 半径轴 ==================== */

/**
 * @brief 发一条绝对位置命令(raF=1)，不等回执
 * @note  绝对模式下方向由"目标位置 vs 当前位置"决定，dir 固定 0。
 *        半径电机(地址6)不回到位帧，只能走这个"只发不等"的版本。
 */
static void Pick_StepperIssue(uint8_t addr, int32_t target, uint16_t vel, uint8_t acc)
{
    Emm_V5_ClearRxFrame();
    HAL_Delay(2);
    Emm_V5_Pos_Control(addr, 0, vel, acc, (uint32_t)target, 1, 0);   /* raF=1: 绝对位置 */
    HAL_Delay(2);
}

/**
 * @brief 绝对位置模式移动一个轴（raF=1）并等待到位，失败自动清堵转并降速重试一次
 * @retval PICK_MOVE_OK / PICK_MOVE_REJECT / PICK_MOVE_TIMEOUT
 * @note  绝对目标**天然幂等**：同一个目标重发不会多走。丢一帧、超时一次都不会
 *        让"接下来该走多少"算错 —— 这是它和相对模式(raF=2)最本质的区别，
 *        也是开源工程不出现本问题的原因（它们上电回零后一直发绝对目标）。
 *        只给**会回到位帧**的轴用（高度轴）；半径轴用 Pick_StepperIssue。
 */
static uint8_t Pick_StepperMove(uint8_t addr, int32_t target, uint16_t vel, uint8_t acc)
{
    uint8_t  r;
    uint16_t retryVel = (addr == PICK_MOTOR_Z_ADDR) ? PICK_MOTOR_Z_RETRY_VEL : PICK_MOTOR_R_RETRY_VEL;
    uint8_t  retryAcc = (addr == PICK_MOTOR_Z_ADDR) ? PICK_MOTOR_Z_RETRY_ACC : PICK_MOTOR_R_RETRY_ACC;

    Pick_StepperIssue(addr, target, vel, acc);

    r = Pick_WaitArriveEx(PICK_MOTOR_DONE_TMO);
    if(r == PICK_MOVE_OK) return r;

    Pick_MotorRecover(addr);

    /* 绝对目标重发安全；第二次降速，减小顶着限位时的冲击 */
    Pick_StepperIssue(addr, target, retryVel, retryAcc);

    return Pick_WaitArriveEx(PICK_MOTOR_RETRY_TMO);
}

/**
 * @brief 高度电机移动到指定 mm（绝对位置模式）
 * @note  绝对坐标以 Pick_Axis_Init() 建立的零点为准（上电时人工放好的 0 位）。
 *        失败时会读回 S_CPOS 修正跟踪值；即使读不回，下一次绝对目标仍然正确，
 *        因为目标不依赖"上次走了多少"。
 */
static uint8_t Pick_MotorZ_MoveToMM(float mm)
{
    int32_t target, cpos;
    uint8_t r;

    if(mm < 0.0f)                mm = 0.0f;
    if(mm > PICK_MOTOR_Z_MAX_MM) mm = PICK_MOTOR_Z_MAX_MM;
    target = (int32_t)(mm * PICK_MOTOR_Z_MM_PUL + 0.5f);

    r = Pick_StepperMove(PICK_MOTOR_Z_ADDR, target, PICK_MOTOR_Z_VEL, PICK_MOTOR_Z_ACC);

    if(r == PICK_MOVE_OK)
    {
        Pick_Diag_LastStatus = PICK_MOVE_OK;
        return PICK_MOVE_OK;
    }

    if(r == PICK_MOVE_REJECT)       Pick_Diag_RejectCnt++;
    else if(r == PICK_MOVE_TIMEOUT) Pick_Diag_TimeoutCnt++;

    /* 失败后读回真实位置只用于诊断；绝对目标本身不依赖"上次走了多少" */
    if(Pick_ReadAxisPos(PICK_MOTOR_Z_ADDR, &cpos))
    {
        Pick_Diag_LastCpos = PICK_MOTOR_Z_CPOS_SIGN * cpos;
    }

    Pick_Diag_LastStatus = r;
    return r;
}

/**
 * @brief 半径电机移动到指定 mm（绝对位置）：只发命令不等待
 * @note  半径电机(地址6)**不回到位帧**（硬件实测），所以不能等回执，
 *        只把目标发出去，和舵机并行 —— 等回执只会每次白等满 7 秒超时。
 *        绝对目标的好处仍然成立：丢一帧不会累积偏差，下一趟动作再发同一个目标
 *        就会自动纠回。零点由 Pick_Axis_Init() 在上电时建立（见该函数说明）。
 */
static void Pick_MotorR_MoveToMM_Issue(float mm)
{
    int32_t target;

    if(mm < 0.0f)                mm = 0.0f;
    if(mm > PICK_MOTOR_R_MAX_MM) mm = PICK_MOTOR_R_MAX_MM;

    target = (int32_t)(mm * PICK_MOTOR_R_MM_PUL + 0.5f);
    Pick_StepperIssue(PICK_MOTOR_R_ADDR, target, PICK_MOTOR_R_VEL, PICK_MOTOR_R_ACC);
    pick_motorRPos = target;                  /* 只用于轨迹短路判断，不用于算增量 */
}

/* ==================== 轨迹：JumpTo / MoveTo 共用 ==================== */

/**
 * @brief 两条基本轨迹的共用实现
 * @param liftFirst 1=门字形(先到安全高度)；0=直接到位
 * @note  JumpTo 与 MoveTo 的唯一差别就是 liftFirst，共用一份代码，避免两边改漏。
 *        两轴都走**绝对位置(raF=1)**，所以轨迹里不需要再做"走了多少"的增量运算。
 *        安全高度那一步失败时**直接中止本次动作**：臂的高度未知时继续转向/走半径
 *        比停下更危险。旧代码不检查这一步的结果，是"顶着限位还要继续做动作"的来源。
 */
static void Pick_Traj_Move(float servoZ, float motorZ_mm, float motorR_mm, uint8_t liftFirst)
{
    float   rClamped = motorR_mm;
    int32_t rTarget;
    uint8_t zMoved, rMoved;

    if(rClamped < 0.0f)                rClamped = 0.0f;
    if(rClamped > PICK_MOTOR_R_MAX_MM) rClamped = PICK_MOTOR_R_MAX_MM;
    rTarget = (int32_t)(rClamped * PICK_MOTOR_R_MM_PUL + 0.5f);

    zMoved = (fabsf(Servo_GetAngle(SERVO_Z) - servoZ) >= 0.5f);  /* 舵机是否真的需要转 */
    rMoved = (rTarget != pick_motorRPos);                        /* 半径是否真的需要走 */
    /* 半径和旋转角都在目标：不需要门字形，直接改高度到目标高度 */
    if(!rMoved && !zMoved)
    {
        (void)Pick_MotorZ_MoveToMM(motorZ_mm);
        return;
    }

    /* 1. 门字形：先到安全高度；失败不中止*/
    if(liftFirst)
    {
        Pick_MotorZ_MoveToMM(PICK_SAFE_HEIGHT_MM);
    }

    /* 2. 并行：半径命令只发不等回执，舵机平滑耗时盖住半径运动。
       绝对目标幂等：即使 rMoved 为假也重发一次，
       万一上一次命令丢帧，这一趟就把半径拉回正确位置。 */
    Pick_MotorR_MoveToMM_Issue(rClamped);
    if(zMoved)
    {
        Servo_SetAngleSmooth(SERVO_Z, servoZ, SERVO_Z_STEP_DEG, SERVO_Z_STEP_MS);
        HAL_Delay(SERVO_Z_SETTLE_MS);              /* 舵机物理到位裕量 */
    }

    /* 3. 舵机运行结束后，再运行到输入的目标高度 */
    (void)Pick_MotorZ_MoveToMM(motorZ_mm);
}

/**
 * @brief 基本动作：门字形轨迹 —— 先升到安全高度，再水平移动+转向，最后降高度
 * @param servoZ   Z轴旋转舵机目标角(°)，量程 0~270
 * @param motorZ_mm 高度电机目标位置(mm)，量程 0~130（越界自动钳位）
 * @param motorR_mm 半径电机目标位置(mm)，量程 0~85（越界自动钳位）
 * @note  若半径和旋转角都已到位，则省略门字形，直接改高度（省一次无谓升降）。
 *        否则：先升到安全高度并等高度电机到位返回标志(0xFD 0x9F)，再并行
 *        (半径只发指令不等回执 + 舵机CPU平滑转动)，舵机运行结束后再降到目标高度。
 *        **两轴都走绝对位置模式(raF=1)**：前提是 Pick_Axis_Init() 已把"上电时
 *        人工放好的 0 位"写为两轴驱动器零点（入口第一次动作会自动补调）。
 *        若上电时臂不在 0 位，绝对坐标会整体偏移 —— 这是唯一的使用约束。
 *        高度轴：绝对目标 + 等到位；失败清堵转、降速重试一次。
 *        半径轴：不回到位帧，只发绝对目标不等回执（丢帧靠下一趟重发自动纠正）。
 *        好处：超时/丢帧/顶限位一次，不会污染后续动作的坐标。
 *        夹爪不在此控制，由调用方按需开合。
 */
void Pick_JumpTo(float servoZ, float motorZ_mm, float motorR_mm)
{
    Pick_Traj_Move(servoZ, motorZ_mm, motorR_mm, 1U);
}

/**
 * @brief 基本动作：直接到位 —— 不上升到安全高度，直接转向+走半径，最后改高度
 * @param servoZ   Z轴旋转舵机目标角(°)，量程 0~270
 * @param motorZ_mm 高度电机目标位置(mm)，量程 0~130（越界自动钳位）
 * @param motorR_mm 半径电机目标位置(mm)，量程 0~85（越界自动钳位）
 * @note  与 Pick_JumpTo 的区别只有一个：省掉"先升到 PICK_SAFE_HEIGHT_MM"那一步和
 *        其回执等待，因此更快，但臂是贴着当前高度横扫过去的 —— 只有当起点到终点
 *        之间确实没有障碍、不会蹭到料台或已放的物料时才可以用。
 *        行程中间有东西就老老实实用 Pick_JumpTo。
 *        夹爪不在此控制，由调用方按需开合。
 */
void Pick_MoveTo(float servoZ, float motorZ_mm, float motorR_mm)
{
    //Pick_Axis_EnsureInit();
    Pick_Traj_Move(servoZ, motorZ_mm, motorR_mm, 0U);
}

/* ==================== 夹爪控制 ==================== */
void Pick_Gripper_Observe(void)
{
    Servo_SetAngleSmooth(SERVO_GRIP, PICK_OBSERVE, SERVO_G_STEP_DEG, SERVO_G_STEP_MS);
}

void Pick_Gripper_Grab(void)
{
    Servo_SetAngleSmooth(SERVO_GRIP, PICK_GRAB, SERVO_G_STEP_DEG, SERVO_G_STEP_MS);
}

void Pick_Gripper_Release(void)
{
    Servo_SetAngleSmooth(SERVO_GRIP, PICK_RELEASE, SERVO_G_STEP_DEG, SERVO_G_STEP_MS);
}


/* ==================== 车上转盘 ==================== */

/**
 * @brief 转盘选槽：把车上转盘转到第 slot 个槽位，阻塞到停稳
 * @param slot 0=A(30°) / 1=B(150°) / 2=C(270°)，越界按 A 处理
 * @note  **平滑转动**（Servo_SetAngleSmooth）+ SERVO_T_SETTLE_MS 到位裕量，
 *        与 Pick_JumpTo 对 Z 舵机的处理一致。2026-10-14 中间短暂改成过直给
 *        (Servo_SetAngle)，已按设计撤回到平滑 —— 参数在 servo.h 的 SERVO_T_* 一组。
 *        Servo_SetAngleSmooth 只保证"命令角发出去了"，舵机追命令还有滞后，
 *        转盘带着料没停稳臂就下去会撞 —— **下面的 HAL_Delay 不能删**。
 *        角度钳位由 Servo_SetAngleSmooth 自己做（超出 0~270 会被钳住）。
 *        时序约定：先本函数选槽，再 Pick_JumpTo POS_CAR —— 转盘转动时臂不在车体上方。
 */
void Pick_TurnTable_Set(uint8_t slot)
{
    static const float ang[3] = { PICK_TURN_ANG_A, PICK_TURN_ANG_B, PICK_TURN_ANG_C };
    if(slot > 2U) slot = 0U;
    Servo_SetAngleSmooth(SERVO_TURN, ang[slot], SERVO_T_STEP_DEG, SERVO_T_STEP_MS);
    HAL_Delay(SERVO_T_SETTLE_MS);
}

/* ==================== 测试函数 ==================== */

/**
 * @brief 高度步进电机(地址5)行程测试：0 → 30mm → 停3s → 0 → 停3s，死循环
 * @note  目的是把高度电机的**方向和比例**实测出来，判读方法：
 *
 *        方向：从 0 走到 30mm 时臂是**上升**还是**下降**。
 *              往下去 = PICK_MOTOR_Z_POS_DIR 反了，改成 1。
 *              改完重新烧一次再看一遍（方向对了这条测试就算过）。
 *        比例：拿卡尺量 0→30mm 实际走了多少。量到 30mm = 60mm/圈 换算对了；
 *              量到 4mm 左右 = 还是旧的 8mm/圈；量到别的数按比例回推，
 *              再去改 PICK_MOTOR_Z_MM_PER_REV。
 *        响应：每一步卡满 5 秒(PICK_MOTOR_DONE_TMO)才进下一步 = 电机没在线/
 *              没使能/地址不对 —— 正常应该几百毫秒就走完。
 *
 *        用 mm 绝对位置接口 Pick_MotorZ_MoveToMM()：零点固定是**上电位置**
 *        (记作 0mm)，先到 30mm 再回 0，一正一反都在同一个坐标里，不会越走越偏。
 *        30mm 按现在的 60mm/圈 是 1590 脉冲 ≈ 半圈。
 *
 *        **跑之前把臂放到行程中段**（比如上下各留出 5cm 以上）再上电：方向没
 *        确认前，30mm 万一往限位那侧走，贴着边起跑就直接顶死。
 *        另外 A8 暂停在这个测试里救不了 —— Dbg_Service 的暂停只给四个轮发停车，
 *        不停臂的高度/半径电机。要停只能断电。
 *
 * @note  依赖 AGV_Can_Init() 已就绪；电机上电默认就是使能态，不需要额外使能。
 */
void Pick_Test_MotorZ(void)
{
    Pick_Axis_EnsureInit();            /* 以当前（上电前的 0 位）建立两轴零点 */
    for(;;)
    {
        (void)Pick_MotorZ_MoveToMM(30.0f);   /* → 30mm，阻塞到高度电机到位返回标志 */
        HAL_Delay(3000);                     /* 停 3 秒：量尺寸 / 看方向 */
        (void)Pick_MotorZ_MoveToMM(0.0f);    /* → 回 0mm(上电位置) */
        HAL_Delay(3000);
    }
}

/**
 * @brief 半径步进电机(地址6)行程测试：0 → 30mm → 停3s → 0 → 停3s，死循环
 * @note  和 Pick_Test_MotorZ() 一个套路，只是换到半径轴。判读方法：
 *
 *        方向：0 → 30mm 时臂是**向外伸**还是**向内收**。
 *              反了 = 改 PICK_MOTOR_R_POS_DIR（当前 1，改成 0）。
 *        比例：拿尺子量 0→30mm 实际走了多少。量到 30mm = 100mm/圈 换算对了；
 *              别的数按比例回推，去改 PICK_MOTOR_R_MM_PER_REV。
 *              **100mm/圈 这个数一次都没实测过**，只有纸面值 —— 量出来不是
 *              30mm 很正常，按实测的改就行。
 *        响应：半径电机**不回到位帧**，所以本测试固定停 3 秒观察、不等回执；
 *              不能靠"卡满 5 秒"判断掉线，只能看实物有没有动。
 *
 *        30mm 按现在的 100mm/圈 是 960 脉冲 = 0.3 圈。零点同样是**上电位置**
 *        (记作 0mm)，先到 30mm 再回 0，一正一反在同一坐标里，不会越走越偏。
 *
 *        **跑之前把臂放到半径行程中段**：方向没确认前，30mm 万一往外顶到限位
 *        就直接卡死。
 *        另外 A8 暂停在这个测试里救不了 —— Dbg_Service 的暂停只给四个轮发停车，
 *        不停臂的高度/半径电机。要停只能断电。
 *
 *        判读：每轮回到 0 后，臂应停在**同一个机械位置** —— 绝对目标幂等，
 *        几十轮下来不会越走越偏。
 *
 * @note  依赖 AGV_Can_Init() 已就绪。用只发不等的 Pick_MotorR_MoveToMM_Issue()。
 */
void Pick_Test_MotorR(void)
{
    Pick_Axis_EnsureInit();
    for(;;)
    {
        Pick_MotorR_MoveToMM_Issue(30.0f);   /* 绝对 30mm，只发不等 */
        HAL_Delay(3000);                     /* 停 3 秒：量尺寸 / 看方向 */
        Pick_MotorR_MoveToMM_Issue(0.0f);    /* 绝对 0mm = 上电时的机械 0 位 */
        HAL_Delay(3000);
    }
}

/**
 * @brief 车上转盘舵机测试：A槽(30°) → B槽(150°) → C槽(270°) → A槽 … 每处停 3 秒，死循环
 * @note  一次验三件事：
 *          1) **换线对不对** —— 转盘是唯一挂在 PA5/TIM2 上的舵机（2026-10-14 刚从
 *             PE9/TIM1 换过来）。它转起来就说明 PA5/TIM2 这条链是通的；要是纹丝不动
 *             而别的舵机正常，先回去查 PA5 的线和 TIM2 的预分频（要 83，见 servo.c）。
 *          2) **三个槽的角度对不对** —— PICK_TURN_ANG_A/B/C = 30/150/270（pick.h）。
 *             看转盘是不是正好停在三个料位上、料有没有偏；要微调就改那三个宏。
 *          3) **方向和一圈走不走得完** —— 从上电的 A 槽出发转一圈回 A 槽，
 *             看有没有卡住/丢步。
 *        每步之间停 3 秒，方便量角度、对位置。
 *
 *        **跑之前确认臂不在车体上方。** Pick_TurnTable_Set() 的时序约定是"先选槽，
 *        再 Pick_JumpTo POS_CAR"—— 转盘带着料转的时候臂压在上面会把料扫飞。
 *        Test_Motor() 里 Pick_Init() 已经把臂收到 POS_START(运输位)，是安全的；
 *        要单独调这个函数就自己先把臂挪开。
 *
 * @note  依赖 Servo_Init() 已就绪（Servo_SetAngleSmooth 要求 PWM 已经起着）。
 *        **不需要 CAN** —— 纯舵机测试，跟步进电机没关系。
 */
void Pick_Test_TurnTable(void)
{
    uint8_t slot = 0U;

    for(;;)
    {
        Pick_TurnTable_Set(slot);      /* 平滑转到 A/B/C 槽，内部自带 SERVO_T_SETTLE_MS 裕量 */
        HAL_Delay(3000);               /* 停 3 秒：看停位准不准、量角度 */

        slot++;
        if(slot > 2U) slot = 0U;       /* 0→1→2→0 循环 */
    }
}

/**
 * @brief Z轴旋转舵机测试：0° → 100° → 50° → 循环，每处停 3 秒，死循环
 * @note  一次验三件事：
 *          1) **换线对不对** —— Z轴舵机 2026-10-14 从 PA5/TIM2 换到了
 *             **PE9/TIM1**（见 servo.c 顶部）。它摆起来 = PE9/TIM1 这条链通了；
 *             不动而别的舵机正常，先查 PE9 的线和 TIM1 的预分频 —— 要 **167**，
 *             配成 83 会得到 100Hz，舵机抖动或不动作。
 *          2) **三个角对不对** —— 看 0°/100°/50° 停到位时的姿态是不是你要的。
 *             要调就改下面 ang[] 里那三个数，或者改 pick.h 里 POS_* 那批宏的
 *             第一个值(角度z)，两者是同一个坐标系。
 *          3) **方向和行程够不够** —— 0→100→50 每步跨的角度都不小，看有没有
 *             反向、卡住、舵机拉不动（拉不动会听到持续的堵转声）。
 *        每步之间停 3 秒，方便对位姿、量角度。
 *
 *        **臂要抬起来，而且不能在车体上方。** Pick_Init() 留下的运输位
 *        (POS_START) 高度只有 10mm，0°/100°/50° 这么大范围摆 Z 轴很容易蹭到
 *        桌面或车体 —— 跑之前先把臂手动抬到行程中上部，或者把 pick.h 里
 *        POS_START 的高度(现在是 10)改大一点。
 *
 * @note  依赖 Servo_Init() 已就绪。**不需要 CAN** —— 纯舵机测试。
 *        （这是本函数故意不自己抬臂的原因：一调 Pick_JumpTo 就又要 CAN 了。）
 */
void Pick_Test_ZServo(void)
{
    /* 三个测试角(°)：0°/100°/50°。换位置直接改这里，别处不用动。 */
    static const float ang[3] = { 0.0f, 100.0f, 50.0f };
    uint8_t i = 0U;

    for(;;)
    {
        Servo_SetAngleSmooth(SERVO_Z, ang[i], SERVO_Z_STEP_DEG, SERVO_Z_STEP_MS);
        HAL_Delay(SERVO_Z_SETTLE_MS);   /* 舵机物理到位裕量(命令发完 ≠ 转到位) */
        HAL_Delay(3000);                /* 停 3 秒：对位姿、量角度 */

        i++;
        if(i > 2U) i = 0U;              /* 0°→100°→50°→0° 循环 */
    }
}



void Pick_Test_Gripper(void)
{
    uint8_t round;

    for(round = 0U; round < 3U; round++)          /* 三个状态走 3 轮 */
    {
        Pick_Gripper_Observe();   /* → PICK_OBSERVE  张最开 */
        HAL_Delay(1000);          /* 停 1 秒：看爪口张到底没有 */
        Pick_Gripper_Grab();      /* → PICK_GRAB     收紧 */
        HAL_Delay(1000);
        Pick_Gripper_Release();   /* → PICK_RELEASE  松到放开 */
        HAL_Delay(1000);
    }

    Pick_Gripper_Grab();          /* 第 3 轮停在"放开"，再补一次"抓紧"就收工 */
    HAL_Delay(1000);

    for(;;)
    {
        Dbg_Service();            /* 见函数头：这个自旋不经过被盖掉的 HAL_Delay */
    }
}