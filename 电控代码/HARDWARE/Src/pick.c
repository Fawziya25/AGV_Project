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
#define PICK_POS_MODE       2     /* 与四轮一致: 2=相对当前实时位置 */
/* 各电机正方向对应的dir值(0/1)：两个轴的方向定义可以不同，反了改各自的值(取反)。
   当前 Z=0 / R=1。**这两个数都还没实测过** —— 模式 9 的 0<->30mm 往返测试就是
   用来实测它们的：看臂往哪边走，反了就取反。 */
#define PICK_MOTOR_Z_POS_DIR  0
#define PICK_MOTOR_R_POS_DIR  0
/* 各电机速度/加速度(RPM)：可分别调节 */
#define PICK_MOTOR_Z_VEL    1000   /* 高度电机速度，原100的2倍 */
#define PICK_MOTOR_Z_ACC    230  /* 高度电机加速度 */
#define PICK_MOTOR_R_VEL    100   /* 半径电机速度，保持原样 */
#define PICK_MOTOR_R_ACC    20    /* 半径电机加速度 */
#define PICK_MOTOR_DONE_TMO 5000  /* 电机回执等待超时(ms) */
#define PICK_SAFE_HEIGHT_MM 10.0f  /* 安全高度(mm)：每次动作先升到此高度再水平移动+转向 */
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
 * @brief 发送电机相对移动命令但不等待（返回本次脉冲数，0=不移动）
 * @note  只发指令不等回执，供 Pick_JumpTo 与舵机并行；半径不需要确认返回。
 */
static uint32_t Pick_StepperMove_Issue(uint8_t addr, int32_t pos)
{
    uint32_t mag = (pos >= 0) ? (uint32_t)pos : (uint32_t)(-pos);
    if(mag == 0) return 0;                      /* 0 脉冲不发指令 */
    uint8_t posDir = (addr == PICK_MOTOR_Z_ADDR) ? PICK_MOTOR_Z_POS_DIR : PICK_MOTOR_R_POS_DIR;
    uint16_t vel   = (addr == PICK_MOTOR_Z_ADDR) ? PICK_MOTOR_Z_VEL : PICK_MOTOR_R_VEL;
    uint8_t  acc   = (addr == PICK_MOTOR_Z_ADDR) ? PICK_MOTOR_Z_ACC : PICK_MOTOR_R_ACC;
    uint8_t dir = (pos >= 0) ? posDir : (1 - posDir);
    Emm_V5_Pos_Control(addr, dir, vel, acc, mag, PICK_POS_MODE, 0);
    HAL_Delay(1);                               /* 防粘包；只发指令，不等回执 */
    return mag;
}

/**
 * @brief 驱动单个步进电机相对移动 pos 个脉冲并等它真正到位（正=dir0, 负=dir1）
 * @note  发送前清掉历史回执帧，发送后等电机到位返回标志(0xFD 0x9F)，参考chassis.c
 *        电机到达目标位置后才回这一帧，等到它就等于物理到位；带超时防卡死。
 */
static uint32_t Pick_StepperMove(uint8_t addr, int32_t pos)
{
    uint32_t mag = (pos >= 0) ? (uint32_t)pos : (uint32_t)(-pos);
    if(mag == 0) return 0;                      /* 0 脉冲不发指令 */
    uint8_t posDir = (addr == PICK_MOTOR_Z_ADDR) ? PICK_MOTOR_Z_POS_DIR : PICK_MOTOR_R_POS_DIR;
    uint16_t vel   = (addr == PICK_MOTOR_Z_ADDR) ? PICK_MOTOR_Z_VEL : PICK_MOTOR_R_VEL;
    uint8_t  acc   = (addr == PICK_MOTOR_Z_ADDR) ? PICK_MOTOR_Z_ACC : PICK_MOTOR_R_ACC;
    uint8_t dir = (pos >= 0) ? posDir : (1 - posDir);

    Emm_V5_ClearRxFrame();                      /* 清历史回执帧，确保等的是本次电机返回的帧 */
    HAL_Delay(10);
    Emm_V5_Pos_Control(addr, dir, vel, acc, mag, PICK_POS_MODE, 0);
    HAL_Delay(10);

    /* 等电机到位返回标志(0xFD 0x9F)，参考chassis.c；带超时防止堵转时卡死 */
    Emm_V5_WaitArrive(PICK_MOTOR_DONE_TMO);
    return mag;
}

/* 高度电机当前位置(脉冲)，上电位置记作零点0 */
static int32_t pick_motorZPos = 0;

/**
 * @brief 高度电机移动到指定 mm 位置（绝对位置，上电位置=0）
 * @note  目标先钳位到 [0, PICK_MOTOR_Z_MAX_MM]，再算相对增量走，
 *        配合 MCU 跟踪当前位置；发送后阻塞到高度电机到位返回标志。
 * @param mm 目标位置(mm)
 */
static void Pick_MotorZ_MoveToMM(float mm)
{
    if(mm < 0.0f)                    mm = 0.0f;
    if(mm > PICK_MOTOR_Z_MAX_MM)     mm = PICK_MOTOR_Z_MAX_MM;

    int32_t target = (int32_t)(mm * PICK_MOTOR_Z_MM_PUL + 0.5f);
    int32_t delta  = target - pick_motorZPos;
    if(delta != 0)
    {
        Pick_StepperMove(PICK_MOTOR_Z_ADDR, delta);
        pick_motorZPos = target;
    }
}

/* 半径电机当前位置(脉冲)，上电位置记作零点0 */
static int32_t pick_motorRPos = 0;

/**
 * @brief 半径电机移动到指定 mm：只发命令不等待，返回本次脉冲数(0=已在目标)
 * @note  供 Pick_JumpTo 与舵机并行；只发指令，不等回执
 * @param mm 目标位置(mm)，越界自动钳位
 */
static uint32_t Pick_MotorR_MoveToMM_Issue(float mm)
{
    if(mm < 0.0f)                    mm = 0.0f;
    if(mm > PICK_MOTOR_R_MAX_MM)     mm = PICK_MOTOR_R_MAX_MM;

    int32_t target = (int32_t)(mm * PICK_MOTOR_R_MM_PUL + 0.5f);
    int32_t delta  = target - pick_motorRPos;
    uint32_t mag = 0;
    if(delta != 0)
    {
        mag = Pick_StepperMove_Issue(PICK_MOTOR_R_ADDR, delta);
        pick_motorRPos = target;
    }
    return mag;
}

/**
 * @brief 半径电机移动到指定 mm 并**等它真正到位**（与 Pick_MotorZ_MoveToMM 对称）
 * @note  正式流程用的是上面那个 Issue 版（不等回执，好跟舵机并行）。这个阻塞版
 *        只有 Pick_Test_MotorR() 在用 —— 桌面测试需要两个东西：
 *          1) 走到位再开始计时，3 秒停顿量出来的尺寸才准；
 *          2) 电机没在线时能卡满超时（"卡 5 秒"就是掉线的信号），Issue 版直接
 *             发完就走，这个诊断就没了。
 *        两个版本共用同一份钳位/增量/跟踪逻辑，改一个记得改另一个。
 * @param mm 目标位置(mm)，越界自动钳位
 */
static void Pick_MotorR_MoveToMM(float mm)
{
    if(mm < 0.0f)                    mm = 0.0f;
    if(mm > PICK_MOTOR_R_MAX_MM)     mm = PICK_MOTOR_R_MAX_MM;

    int32_t target = (int32_t)(mm * PICK_MOTOR_R_MM_PUL + 0.5f);
    int32_t delta  = target - pick_motorRPos;
    if(delta != 0)
    {
        Pick_StepperMove(PICK_MOTOR_R_ADDR, delta);
        pick_motorRPos = target;
    }
}

/**
 * @brief 机械臂初始化：启动两个PWM舵机(输出并归位到启动角)，随后把臂收到
 *        运输位 POS_START，使整车移动前尺寸在框内。
 * @note  Pick_JumpTo 走步进电机，依赖 AGV_Can_Init() 已就绪；且本函数在
 *        HWT101_Init 的20s手动校准之后调用，臂运动不会干扰零偏窗口。
 */
void Pick_Init(void)
{
    Servo_Init();
}

/**
 * @brief 基本动作：门字形轨迹 —— 先升到安全高度，再水平移动+转向，最后降高度
 *        固定顺序：先升到 PICK_SAFE_HEIGHT_MM 并等高度电机到位返回标志 → 并行(转向+半径) → 再降到目标高度
 * @param servoZ   Z轴旋转舵机目标角(°)，量程 0~270
 * @param motorZ_mm 高度电机目标位置(mm)，量程 0~100（越界自动钳位）
 * @param motorR_mm 半径电机目标位置(mm)，量程 0~74（越界自动钳位）
 * @note  若半径和旋转角都已到位，则省略门字形，直接改高度（省一次无谓升降）。
 *        否则：先升到安全高度并等高度电机到位返回标志(0xFD 0x9F)，再并行
 *        (半径只发指令不等回执 + 舵机CPU平滑转动)，舵机运行结束后再降到目标高度；
 *        舵机实际发生转动时才加 SERVO_Z_SETTLE_MS 裕量。
 *        夹爪不在此控制，由调用方按需开合。
 */
void Pick_JumpTo(float servoZ, float motorZ_mm, float motorR_mm)
{
    /* 半径目标先钳位（与内部一致），用于短路判断 */
    if(motorR_mm < 0.0f)                motorR_mm = 0.0f;
    if(motorR_mm > PICK_MOTOR_R_MAX_MM) motorR_mm = PICK_MOTOR_R_MAX_MM;
    int32_t rTarget = (int32_t)(motorR_mm * PICK_MOTOR_R_MM_PUL + 0.5f);
    float   zCur    = Servo_GetAngle(SERVO_Z);
    uint8_t zMoved  = (fabsf(zCur - servoZ) >= 0.5f);  /* 舵机是否真的需要转 */
    uint8_t rMoved  = (rTarget != pick_motorRPos);     /* 半径是否真的需要走 */

    /* 半径和旋转角都在目标：不需要门字形，直接改高度到目标高度 */
    if(!rMoved && !zMoved)
    {
        Pick_MotorZ_MoveToMM(motorZ_mm);
        return;
    }

    /* 1. 有水平/旋转动作：先升到安全高度，等高度电机到位返回标志 */
    Pick_MotorZ_MoveToMM(PICK_SAFE_HEIGHT_MM);

    /* 2. 并行：半径命令只发不等回执，舵机平滑耗时盖住半径运动 */
    if(rMoved) Pick_MotorR_MoveToMM_Issue(motorR_mm);
    if(zMoved) Servo_SetAngleSmooth(SERVO_Z, servoZ, SERVO_Z_STEP_DEG, SERVO_Z_STEP_MS);
    if(zMoved) HAL_Delay(SERVO_Z_SETTLE_MS);               /* 舵机物理到位裕量 */

    /* 3. 舵机运行结束后，再运行到输入的目标高度 */
    Pick_MotorZ_MoveToMM(motorZ_mm);
}

/**
 * @brief 基本动作：直接到位 —— 不上升到安全高度，直接转向+走半径，最后改高度
 * @param servoZ   Z轴旋转舵机目标角(°)，量程 0~270
 * @param motorZ_mm 高度电机目标位置(mm)，量程 0~100（越界自动钳位）
 * @param motorR_mm 半径电机目标位置(mm)，量程 0~74（越界自动钳位）
 * @note  与 Pick_JumpTo 的区别只有一个：省掉"先升到 PICK_SAFE_HEIGHT_MM"那一步和
 *        其回执等待，因此更快，但臂是贴着当前高度横扫过去的 —— 只有当起点到终点
 *        之间确实没有障碍、不会蹭到料台或已放的物料时才可以用。
 *        行程中间有东西就老老实实用 Pick_JumpTo。
 *        夹爪不在此控制，由调用方按需开合。
 */
void Pick_MoveTo(float servoZ, float motorZ_mm, float motorR_mm)
{
    /* 半径目标先钳位（与内部一致），用于短路判断 */
    if(motorR_mm < 0.0f)                motorR_mm = 0.0f;
    if(motorR_mm > PICK_MOTOR_R_MAX_MM) motorR_mm = PICK_MOTOR_R_MAX_MM;
    int32_t rTarget = (int32_t)(motorR_mm * PICK_MOTOR_R_MM_PUL + 0.5f);
    float   zCur    = Servo_GetAngle(SERVO_Z);
    uint8_t zMoved  = (fabsf(zCur - servoZ) >= 0.5f);  /* 舵机是否真的需要转 */
    uint8_t rMoved  = (rTarget != pick_motorRPos);     /* 半径是否真的需要走 */

    /* 半径和旋转角都在目标：没什么可动，直接改高度到目标高度 */
    if(!rMoved && !zMoved)
    {
        Pick_MotorZ_MoveToMM(motorZ_mm);
        return;
    }

    /* 并行：半径命令只发不等回执，舵机平滑耗时盖住半径运动 */
    if(rMoved) Pick_MotorR_MoveToMM_Issue(motorR_mm);
    if(zMoved) Servo_SetAngleSmooth(SERVO_Z, servoZ, SERVO_Z_STEP_DEG, SERVO_Z_STEP_MS);
    if(zMoved) HAL_Delay(SERVO_Z_SETTLE_MS);               /* 舵机物理到位裕量 */

    /* 舵机运行结束后，再运行到输入的目标高度 */
    Pick_MotorZ_MoveToMM(motorZ_mm);
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

/* --- 快动版：直接设角，一步到位 --- */
void Pick_Gripper_Quick_Observe(void)
{
    Servo_SetAngle(SERVO_GRIP, PICK_OBSERVE);
}

void Pick_Gripper_Quick_Grab(void)
{
    Servo_SetAngle(SERVO_GRIP, PICK_GRAB);
}

void Pick_Gripper_Quick_Release(void)
{
    Servo_SetAngle(SERVO_GRIP, PICK_RELEASE);
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
    for(;;)
    {
        Pick_MotorZ_MoveToMM(50.0f);   /* → 30mm，阻塞到高度电机到位返回标志 */
        HAL_Delay(3000);               /* 停 3 秒：量尺寸 / 看方向 */
        Pick_MotorZ_MoveToMM(30.0f);    /* → 回 0mm(上电位置) */
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
 *        响应：每步卡满 5 秒(PICK_MOTOR_DONE_TMO)才进下一步 = 电机没在线/
 *              没使能/地址不对。这条能测到，靠的是下面那个阻塞版接口。
 *
 *        30mm 按现在的 100mm/圈 是 960 脉冲 = 0.3 圈。零点同样是**上电位置**
 *        (记作 0mm)，先到 30mm 再回 0，一正一反在同一坐标里，不会越走越偏。
 *
 *        **跑之前把臂放到半径行程中段**：方向没确认前，30mm 万一往外顶到限位
 *        就直接卡死。
 *        另外 A8 暂停在这个测试里救不了 —— Dbg_Service 的暂停只给四个轮发停车，
 *        不停臂的高度/半径电机。要停只能断电。
 *
 * @note  依赖 AGV_Can_Init() 已就绪。用的是阻塞版 Pick_MotorR_MoveToMM()
 *        （不是正式流程那个 Issue 版），理由见那个函数头上的说明。
 */
void Pick_Test_MotorR(void)
{
    for(;;)
    {
        Pick_MotorR_MoveToMM(30.0f);   /* → 30mm，阻塞到半径电机到位返回标志 */
        HAL_Delay(3000);               /* 停 3 秒：量尺寸 / 看方向 */
        Pick_MotorR_MoveToMM(0.0f);    /* → 回 0mm(上电位置) */
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

/**
 * @brief 夹爪三态测试：观察 → 抓紧 → 放开，走 3 轮，最后停在**抓紧**
 * @note  一次验三件事：
 *          1) **三个角度对不对** —— 三个状态就是 pick.h 里那三个宏：
 *             `PICK_OBSERVE`(张最开) / `PICK_GRAB`(收紧夹住) /
 *             `PICK_RELEASE`(松到放开)，**具体度数看 pick.h，别在这儿抄**
 *             （2026-10-14 夹爪换 180° 舵机后正在重对，抄一份就得跟着改一份）。
 *             看每个状态爪口开合到不到位、会不会夹不紧或者顶死在物料上。
 *             要调就改那三个宏，别处不用动。
 *          2) **行程够不够、有没有卡住** —— 这三个宏沿角度轴从小到大是
 *             `PICK_OBSERVE < PICK_RELEASE < PICK_GRAB`（张开→半开→夹紧；
 *             ⚠ 换舵机重对之后如果 PICK_RELEASE 不再落在另两个中间，下面那段
 *             "每轮扫过整个行程"的说法就不成立了，跟着改）。
 *             本测试按 观察 → 抓紧 → 放开 走，每轮都扫过整个行程再从放开回观察，
 *             哪一段发涩/丢步/拉不动（有持续堵转声）一眼能看出来。
 *          3) **换线对不对** —— 夹爪是唯一在 PA6/TIM3 上的舵机（没跟着 2026-10-14
 *             那次 Z轴↔转盘 对调动过）。它动 = 这条链是通的。
 *
 *        走的是 `Pick_Gripper_*`（平滑分步版），不是 `Pick_Gripper_Quick_*`：
 *        测试要看的就是真实动作手感，得和正式流程用同一套。
 *        三个动作之间各停 1 秒 —— `Pick_Gripper_*` 内部**不额外等待**，
 *        且分步只保证"命令发完了"，舵机追命令还有滞后，不留停顿看不清停位。
 *
 *        **跑完停在"抓紧"不再动**：舵机靠 PWM 保持角度，松手也不会掉。
 *        末尾那个空转循环里**必须手工调 Dbg_Service()** —— 自旋不经过被 test.c
 *        盖掉的 HAL_Delay，不调的话 A8 暂停和 OLED 刷新全都失效（两者都住在
 *        Dbg_Service 里）。DBG_ENABLE=0 时它是个空语句，这个循环就纯空转。
 *
 * @note  依赖 Servo_Init() 已就绪。**不需要 CAN** —— 纯舵机测试。
 *        **臂可以不动**：这个测试只碰夹爪，不改高度/半径/转向，所以不用先抬臂
 *        （整爪口对着空气开合就行；要试夹物料就自己先摆好）。
 */
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


/* 原料区抓一块放到车上第 slot 槽：槽位靠转盘选，臂位姿三次都是 POS_CAR */
static void Pick_Test_RawToCar(uint8_t slot)
{
    Pick_Gripper_Release();
    Pick_JumpTo POS_OBSERVE_RAW;
    Pick_JumpTo POS_RAW_A;
    Pick_Gripper_Grab();
    Pick_TurnTable_Set(slot);       /* 先选槽 */
    Pick_JumpTo POS_CAR;            /* 再摆臂 */
    Pick_Gripper_Release();
}

void Pick_Test_Transfer(void)
{
    Pick_Test_RawToCar(0);          /* → A 槽(30°) */
    Pick_Test_RawToCar(1);          /* → B 槽(150°) */
    Pick_Test_RawToCar(2);          /* → C 槽(270°) */

    // ========== 回到运行位置 ==========
    Pick_JumpTo POS_RUN;
}


void Pick_DebugTest(void)
{
    Pick_JumpTo POS_RAW_A;
    Pick_Gripper_Release();
    Pick_JumpTo POS_1;
    Pick_Gripper_Grab();
    Pick_JumpTo POS_3;
    // // 夹爪控制 + 机械臂移动逻辑


    // /* 全部完成，回到运行位置 */
    // Pick_JumpTo POS_RUN;
}
