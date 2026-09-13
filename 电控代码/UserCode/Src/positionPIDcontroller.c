#include "positionPIDcontroller.h"
#include "openmv.h"         /* 视觉修正顶层循环依赖 OpenMV 全局变量与函数 */
#include "oled.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>          /* abs */

/* ==================== 位置修正PID 调参区 ====================
   结构参考角度PID(yawPIDcontroller.c)：误差(px)→P+I+D→输出=轮速RPM。
   KP   误差(px)→RPM 比例。误差10px≈KP×10 RPM。
        相机帧率约10~20Hz：KP偏大(>3)会一帧冲过头再来回抖(实测1/3轮剧烈抖动)，
        KP偏小(<0.5)则太慢甚至电机爬行。现场标定。
   KI/KD 量级参考角度PID，先不动；抖动就降KP。
   限幅/积分限幅/死区同角度PID习惯。 */
#define POS_PID_KP      0.8f   /* 参考角度PID的KP：误差1px→1.25RPM */
#define POS_PID_KI      0.0f
#define POS_PID_KD      0.03f
#define POS_PID_OUT_MAX 200.0f  /* 输出限幅：最大轮速RPM */
#define POS_PID_INT_LIM 80.0f   /* 积分限幅(px) */
/* 相机分辨率放大倍数：误差进PID前先除回旧像素比例。
   分辨率翻倍→同样物理偏差像素值×2，若直接进PID则P/I/D三项都×2，
   必须在入口统一缩放，物理行为才与原来一致。改分辨率时同步修改。 */
#define POS_PX_SCALE    1.0f
/* ============================================================= */
/* ============ 视觉闭环：控制节拍与对准判据 ============
   控制节拍 AGV_CTRL_TICK_MS：相机实测帧率~40fps，但 KP 按 10~20Hz 标定。
   若每帧都算一轮PID，等效控制率翻倍需重调 KP；故锁 ~20Hz(50ms)下发，
   相机只是"每次用最新一帧"。 */
#define AGV_CTRL_TICK_MS   40U    /* 控制节拍：距上次下发>=此值才做一轮(≈20Hz) */
#define AGV_ALIGN_WIN_PX   1      /* |dx|,|dy| 低于该值视为已对准(px) */
#define AGV_ALIGN_HOLD_CNT 4      /* 连续 N 个节拍在窗口内才真退出(≈200ms, 防过冲早停) */
/* ============================================================= */

PosPID_TypeDef posPID_X;
PosPID_TypeDef posPID_Y;

/**
 * @brief 初始化位置修正PID（两轴共用一组参数，清空历史状态）
 */
void PositionPID_Init(void)
{
    posPID_X.kp = POS_PID_KP;  posPID_X.ki = POS_PID_KI;  posPID_X.kd = POS_PID_KD;
    posPID_X.error = 0.0f;  posPID_X.prevError = 0.0f;  posPID_X.integral = 0.0f;  posPID_X.output = 0.0f;
    posPID_Y.kp = POS_PID_KP;  posPID_Y.ki = POS_PID_KI;  posPID_Y.kd = POS_PID_KD;
    posPID_Y.error = 0.0f;  posPID_Y.prevError = 0.0f;  posPID_Y.integral = 0.0f;  posPID_Y.output = 0.0f;
}

/**
 * @brief 单轴PID计算（参考 yawPID_Calculate）
 * @param pid  轴PID
 * @param error 本轴误差(px)
 */
static void PosPID_Calc(PosPID_TypeDef *pid, float error)
{
    pid->error = error;
    /* 比例项 */
    float P_term = pid->kp * pid->error;
    /* 积分项：小误差才积分，大误差停积防饱和 */
    if(pid->error <= 30.0f && pid->error >= -30.0f)
    {
        pid->integral += pid->error;
    }
    if(pid->integral > POS_PID_INT_LIM)       { pid->integral =  POS_PID_INT_LIM; }
    else if(pid->integral < -POS_PID_INT_LIM) { pid->integral = -POS_PID_INT_LIM; }
    float I_term = pid->ki * pid->integral;
    /* 微分项：误差差分 */
    float D_term = pid->kd * (pid->error - pid->prevError);
    pid->prevError = pid->error;
    /* 总输出并限幅 */
    pid->output = P_term + I_term + D_term;
    if(pid->output > POS_PID_OUT_MAX)       { pid->output =  POS_PID_OUT_MAX; }
    else if(pid->output < -POS_PID_OUT_MAX) { pid->output = -POS_PID_OUT_MAX; }
}

/**
 * @brief 麦轮逆解 + 速度模式下发（下发节奏参考 AGV_SetOmega）
 * @param rpmX X直行分量(RPM)
 * @param rpmY Y平移分量(RPM)
 */
static void AGV_Correct_Vel(float rpmX, float rpmY)
{
    /* 麦轮逆解：1/3轮 = X直行+Y平移；2/4轮 = Y平移-X直行 */
    float w13 = rpmX + rpmY;
    float w24 = rpmY - rpmX;

    /* 轮速限幅 */
    if(w13 > POS_PID_OUT_MAX)       { w13 =  POS_PID_OUT_MAX; }
    else if(w13 < -POS_PID_OUT_MAX) { w13 = -POS_PID_OUT_MAX; }
    if(w24 > POS_PID_OUT_MAX)       { w24 =  POS_PID_OUT_MAX; }
    else if(w24 < -POS_PID_OUT_MAX) { w24 = -POS_PID_OUT_MAX; }

    /* 提取方向与幅值 */
    uint8_t sign13 = (w13 >= 0.0f) ? 0U : 1U;
    uint8_t sign24 = (w24 >= 0.0f) ? 0U : 1U;
    uint16_t vel13 = (uint16_t)((w13 >= 0.0f) ? w13 : -w13);
    uint16_t vel24 = (uint16_t)((w24 >= 0.0f) ? w24 : -w24);

    Emm_V5_Vel_Control(1, Wheel1PositiveDir ^ sign13, vel13, 0, true);
    delay_us(250);
    Emm_V5_Vel_Control(3, Wheel3PositiveDir ^ sign13, vel13, 0, true);
    delay_us(250);
    Emm_V5_Vel_Control(2, Wheel2PositiveDir ^ sign24, vel24, 0, true);
    delay_us(250);
    Emm_V5_Vel_Control(4, Wheel4PositiveDir ^ sign24, vel24, 0, true);
    delay_us(250);

    Emm_V5_Synchronous_motion(0);   /* 广播触发多机同步 */
    delay_us(250);

    while(can.rxFrameFlag == false){};
    can.rxFrameFlag = false;
}

/**
 * @brief 位置修正单拍：输入相机偏差 dx/dy(px)，算一轮PID并下发一轮轮速
 * @note  相机帧与车体帧旋转90°：dy→小车X(直行)、dx→小车Y(平移)；负号=方向取反(实测)
 */
void AGV_Position_Correct(int16_t dx, int16_t dy)
{
    /* 分辨率放大后像素值同步放大，先除回旧像素比例再进PID（见 POS_PX_SCALE） */
    PosPID_Calc(&posPID_X, -(float)dy / POS_PX_SCALE);
    PosPID_Calc(&posPID_Y, -(float)dx / POS_PX_SCALE);
    AGV_Correct_Vel(posPID_X.output, posPID_Y.output);
}

/**
 * @brief 立即停车（对准/超时退出时调用，参考 AGV_StopNow）
 */
void AGV_Position_Stop(void)
{
    Emm_V5_Stop_Now(1, true);
    delay_us(250);
    Emm_V5_Stop_Now(2, true);
    delay_us(250);
    Emm_V5_Stop_Now(3, true);
    delay_us(250);
    Emm_V5_Stop_Now(4, true);
    delay_us(250);

    Emm_V5_Synchronous_motion(0);
    delay_us(250);

    while(can.rxFrameFlag == false){};
    can.rxFrameFlag = false;
}

/**
 * @brief 视觉闭环修正主流程：持续修正直到"真对准"或超时（阻塞），退出前自动停车
 * @param timeout_ms 本段修正允许的总时长(ms)，超时强制退出并停车
 * @note  原 Pick_Correct 整体搬移至此（pick_task 只负责调度）。
 *        - 控制节拍锁 ~20Hz(AGV_CTRL_TICK_MS)：只按节拍消费一帧，控制率不随
 *          相机帧率(实测~40fps)漂移，与 KP 标定的 10~20Hz 环境一致 → 无需重调 KP。
 *        - 对准判据：连续 AGV_ALIGN_HOLD_CNT 个节拍都在 |dx|,|dy|<=AGV_ALIGN_WIN_PX
 *          内才退出；单帧出窗/目标丢失即清零 → 防"过冲经过0点"或抖动造成的假对准。
 *        - 目标丢失(-999)时不下发(保持当前轮速)、等目标回来；全程无帧则空转到超时。
 *        - OLED 刷新(~20ms软I2C整屏)已移出本循环，避免拖乱 50ms 控制节拍。
 *        全局变量 target_dx/target_dy 等在 openmv.h 声明。
 */
void AGV_Correct(uint32_t timeout_ms)
{
    PositionPID_Init();                 /* 每次修正重新初始化PID状态 */
    uint32_t start_time = HAL_GetTick();    // 超时计时起点
    uint32_t last_ctrl = 0;                 /* 上次真正下发轮速的时刻(0=首个节拍立即执行) */
    uint8_t  align_cnt = 0;                 /* 连续在窗口内的节拍数 */

    while((HAL_GetTick() - start_time) < timeout_ms)
    {
        Openmv_OLED_Show();               /* 调试:显示当前帧数据 */
        OpenMV_Service();       /* 接收链自愈：错误中止后重挂DMA，否则OpenMV_RxFlag永不刷新 */

        /* 控制节拍门：距上次下发不足 AGV_CTRL_TICK_MS 不处理(控制率锁~20Hz) */
        if((HAL_GetTick() - last_ctrl) < AGV_CTRL_TICK_MS) continue;

        if(!OpenMV_RxFlag)
        {
            last_ctrl = HAL_GetTick();      /* 无新帧也推进节拍，避免空转忙等 */
            continue;
        }
        OpenMV_RxFlag = 0;
        last_ctrl = HAL_GetTick();

        /* 无目标：不下发(保持上次轮速)、清零对准计数，等目标回来 */
        if(target_dx == -999 || target_dy == -999)
        {
            align_cnt = 0;
            continue;
        }

        /* 对准判定：在窗口内→计数；出窗/抖动→清零(防过冲经过0点时误停) */
        if(target_dx <=  AGV_ALIGN_WIN_PX && target_dx >= -AGV_ALIGN_WIN_PX &&
           target_dy <=  AGV_ALIGN_WIN_PX && target_dy >= -AGV_ALIGN_WIN_PX)
        {
            if(++align_cnt >= AGV_ALIGN_HOLD_CNT) break;   /* 连续N节拍对准=完成 */
        }
        else
        {
            align_cnt = 0;
        }

        AGV_Position_Correct(target_dx, target_dy);  /* 一轮PID：轴映射+麦轮逆解+下发 */
    }
    AGV_Position_Stop();    /* 退出(真对准或超时)立即停车 */
}
