/**
 * @file pick_ctrl.c
 * @brief 物料抓取模块：2 舵机（Z轴旋转/夹爪）+ 2 步进电机（高度/半径）
 *        当前版本：视觉默认已对准，只做机械臂动作控制。
 * @note  步进电机与四轮完全同协议(Emm_V5)，相对位置模式(raF=2)，地址5/6。
 *        移植到新工程：只改本文件顶部"可移植配置区"。
 */
#include "pick_ctrl.h"
#include "Emm_V5.h"
#include "servo.h"
#include "oled.h"
#include "stm32f4xx_hal.h"
#include <math.h>          /* fabsf */

/* ===================== 可移植配置区 ===================== */
#define PICK_MOTOR_Z_ADDR   5     /* 高度步进电机 CAN 地址 */
#define PICK_MOTOR_R_ADDR   6     /* 半径步进电机 CAN 地址 */
#define PICK_POS_MODE       2     /* 与四轮一致: 2=相对当前实时位置 */
/* 各电机正方向对应的dir值(0/1)：实测两电机均需dir=1，反了改各自的值 */
#define PICK_MOTOR_Z_POS_DIR  1
#define PICK_MOTOR_R_POS_DIR  1
/* 各电机速度/加速度(RPM)：可分别调节 */
#define PICK_MOTOR_Z_VEL    1000   /* 高度电机速度，原100的2倍 */
#define PICK_MOTOR_Z_ACC    245  /* 高度电机加速度 */
#define PICK_MOTOR_R_VEL    100   /* 半径电机速度，保持原样 */
#define PICK_MOTOR_R_ACC    20    /* 半径电机加速度 */
#define PICK_MOTOR_DONE_TMO 2500  /* 电机回执等待超时(ms) */
#define PICK_SAFE_HEIGHT_MM 7.0f  /* 安全高度(mm)：每次动作先升到此高度再水平移动+转向 */
/* 舵机平滑转动参数 SERVO_STEP_DEG / SERVO_STEP_MS / SERVO_SETTLE_MS 定义在 servo.h */

/* 高度电机(地址5) 行程配置: 逆时针一圈=8mm, 一圈3200脉冲 → 400脉冲/mm */
#define PICK_MOTOR_Z_MM_PER_REV   8.0f    /* 一圈对应 8mm */
#define PICK_MOTOR_Z_PUL_PER_REV  3200    /* 一圈脉冲数(16细分×1.8°) */
#define PICK_MOTOR_Z_MAX_MM       150.0f  /* 高度行程上限 0~100mm */
#define PICK_MOTOR_Z_MM_PUL       ((int32_t)(PICK_MOTOR_Z_PUL_PER_REV / PICK_MOTOR_Z_MM_PER_REV)) /* 400脉冲/mm */

/* 半径电机(地址6) 行程配置: 一圈=100mm, 一圈3200脉冲 → 32脉冲/mm */
#define PICK_MOTOR_R_MM_PER_REV   100.0f  /* 一圈对应 100mm */
#define PICK_MOTOR_R_PUL_PER_REV  3200    /* 一圈脉冲数 */
#define PICK_MOTOR_R_MAX_MM       74.0f   /* 半径行程上限 0~74mm */
#define PICK_MOTOR_R_MM_PUL       ((int32_t)(PICK_MOTOR_R_PUL_PER_REV / PICK_MOTOR_R_MM_PER_REV)) /* 32脉冲/mm */
/* ======================================================= */

/**
 * @brief 发送电机相对移动命令但不等待（返回本次脉冲数，0=不移动）
 * @note  只发指令不等回执，供 Pick_MoveTo 与舵机并行；半径不需要确认返回。
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

    can.rxFrameFlag = false;                    /* 清历史回执帧，确保等的是本次电机返回的帧 */

    Emm_V5_Pos_Control(addr, dir, vel, acc, mag, PICK_POS_MODE, 0);
    HAL_Delay(1);

    /* 等电机到位返回标志(0xFD 0x9F)，参考chassis.c；带超时防止堵转时卡死 */
    uint32_t t0 = HAL_GetTick();
    while(!(can.rxFrameFlag && can.rxData[0] == 0xFD && can.rxData[1] == 0x9F))
    {
        if((HAL_GetTick() - t0) > PICK_MOTOR_DONE_TMO) break;
    }
    can.rxFrameFlag = false;
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
 * @note  供 Pick_MoveTo 与舵机并行；只发指令，不等回执
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
 * @brief 机械臂初始化：启动两个PWM舵机(输出并归位到启动角)，随后把臂收到
 *        运输位 POS_START，使整车移动前尺寸在框内。
 * @note  Pick_MoveTo 走步进电机，依赖 AGV_Can_Init() 已就绪；且本函数在
 *        HWT101_Init 的20s手动校准之后调用，臂运动不会干扰零偏窗口。
 */
void Pick_Init(void)
{
    Servo_Init();
    //Pick_MoveTo POS_START;   /* 收臂到运输位(断电/上电复位位)，控制整体尺寸 */
    //Pick_Gripper_Grab();
    //HAL_Delay(2000);
    //Pick_Gripper_Release(); 
    //HAL_Delay(2000);
    //Pick_Gripper_Observe(); /* 夹爪张开观察位，避免碰撞 */
    Pick_MoveTo POS_START; 
}

/**
 * @brief 基本动作：门字形轨迹 —— 先升到安全高度，再水平移动+转向，最后降高度
 *        固定顺序：先升到 PICK_SAFE_HEIGHT_MM 并等高度电机到位返回标志 → 并行(转向+半径) → 再降到目标高度
 * @param servoZ   Z轴旋转舵机目标角(°)，量程 0~360
 * @param motorZ_mm 高度电机目标位置(mm)，量程 0~100（越界自动钳位）
 * @param motorR_mm 半径电机目标位置(mm)，量程 0~74（越界自动钳位）
 * @note  若半径和旋转角都已到位，则省略门字形，直接改高度（省一次无谓升降）。
 *        否则：先升到安全高度并等高度电机到位返回标志(0xFD 0x9F)，再并行
 *        (半径只发指令不等回执 + 舵机CPU平滑转动)，舵机运行结束后再降到目标高度；
 *        舵机实际发生转动时才加 SERVO_SETTLE_MS 裕量。
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
    if(zMoved) Servo_SetAngleSmooth(SERVO_Z, servoZ, SERVO_STEP_DEG, SERVO_STEP_MS);
    if(zMoved) HAL_Delay(SERVO_SETTLE_MS);                 /* 舵机物理到位裕量 */

    /* 3. 舵机运行结束后，再运行到输入的目标高度 */
    Pick_MotorZ_MoveToMM(motorZ_mm);
}

//不需要上升到安全高度
// void Pick_Direct_MoveTo(float servoZ, float motorZ_mm, float motorR_mm)
// {
//     /* 半径目标先钳位（与内部一致），用于短路判断 */
//     if(motorR_mm < 0.0f)                motorR_mm = 0.0f;
//     if(motorR_mm > PICK_MOTOR_R_MAX_MM) motorR_mm = PICK_MOTOR_R_MAX_MM;
//     int32_t rTarget = (int32_t)(motorR_mm * PICK_MOTOR_R_MM_PUL + 0.5f);
//     float   zCur    = Servo_GetAngle(SERVO_Z);
//     uint8_t zMoved  = (fabsf(zCur - servoZ) >= 0.5f);  /* 舵机是否真的需要转 */
//     uint8_t rMoved  = (rTarget != pick_motorRPos);     /* 半径是否真的需要走 */

//     /* 半径和旋转角都在目标：不需要门字形，直接改高度到目标高度 */
//     if(!rMoved && !zMoved)
//     {
//         Pick_MotorZ_MoveToMM(motorZ_mm);
//         return;
//     }
//     /* 2. 并行：半径命令只发不等回执，舵机平滑耗时盖住半径运动 */
//     if(rMoved) Pick_MotorR_MoveToMM_Issue(motorR_mm);
//     if(zMoved) Servo_SetAngleSmooth(SERVO_Z, servoZ, SERVO_STEP_DEG, SERVO_STEP_MS);
//     if(zMoved) HAL_Delay(SERVO_SETTLE_MS);                 /* 舵机物理到位裕量 */

//     /* 3. 舵机运行结束后，再运行到输入的目标高度 */
//     Pick_MotorZ_MoveToMM(motorZ_mm);
// }

/* ==================== 夹爪控制 ==================== */
void Pick_Gripper_Observe(void)
{
    Servo_SetAngleSmooth(SERVO_GRIP, PICK_OBSERVE, SERVO_STEP_DEG, SERVO_STEP_MS);
}

void Pick_Gripper_Grab(void)
{
    Servo_SetAngleSmooth(SERVO_GRIP, PICK_GRAB, SERVO_STEP_DEG, SERVO_STEP_MS);
}

void Pick_Gripper_Release(void)
{
    Servo_SetAngleSmooth(SERVO_GRIP, PICK_RELEASE, SERVO_STEP_DEG, SERVO_STEP_MS);
}

/* ==================== 测试函数 ==================== */



void Pick_Test_Transfer(void)
{
   Pick_Gripper_Release();
    Pick_MoveTo POS_OBSERVE;
    Pick_MoveTo POS_RAW;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_A;
    Pick_Gripper_Release();

    // ========== 第2次：抓取放到 POS_B ==========
    Pick_Gripper_Release();
    Pick_MoveTo POS_OBSERVE;
    Pick_MoveTo POS_RAW;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_B;
    Pick_Gripper_Release();

    // ========== 第3次：抓取放到 POS_C ==========
    Pick_Gripper_Release();
    Pick_MoveTo POS_OBSERVE;
    Pick_MoveTo POS_RAW;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_C;
    Pick_Gripper_Release();

    // ========== 回到运行位置 ==========
    Pick_MoveTo POS_RUN;
}


void Pick_DebugTest(void)
{
    Pick_MoveTo POS_RAW;
    Pick_Gripper_Release();
    Pick_MoveTo POS_1;
    Pick_Gripper_Grab();
    Pick_MoveTo POS_3;
    // // 夹爪控制 + 机械臂移动逻辑

    // /* 第1次：释放夹爪，移动到POS_A */
    // Pick_Gripper_Release();
    // Pick_MoveTo POS_OBSERVE_RAW;
    // Pick_MoveTo POS_RAW;
    // Pick_MoveTo POS_A;
    // Pick_Gripper_Release();

    // /* 第2次：抓取夹爪，移动到POS_B */
    // Pick_MoveTo POS_OBSERVE_RAW;
    // Pick_MoveTo POS_RAW;
    // Pick_Gripper_Grab();
    // Pick_MoveTo POS_B;
    // Pick_Gripper_Release();

    // /* 第3次：释放夹爪，移动到POS_C */
    // Pick_Gripper_Release();
    // Pick_MoveTo POS_OBSERVE_RAW;
    // Pick_MoveTo POS_RAW;
    // Pick_Gripper_Grab();
    // Pick_MoveTo POS_C;
    // Pick_Gripper_Release();

    // /* 全部完成，回到运行位置 */
    // Pick_MoveTo POS_RUN;
}
