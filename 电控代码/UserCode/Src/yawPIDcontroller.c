#include "yawPIDcontroller.h"
#include "hwt101.h"
#include <stdint.h>

static YawPID_TypeDef yawPID;        /* 角度PID参数与状态(仅本文件使用) */
static float omega;                  /* 小车整体角速度 */
static uint8_t TurnCpltFlag = 0;     /* 转弯完成标志位 */

/* 小车朝向: 0:X+ 1:Y+ 2:X- 3:Y- (逆时针递增)，AGV_Position_Go() 靠它把
   目标点差量拆成"直行/平移"两段；每次转弯后由 TurnLeft/TurnRight 更新 */
uint8_t AGV_Direction = 3;

/* ================= 航向基准与偏差学习（2026-11-18 重构） =================
 * 旧框架(MovingOffset/RotatingOffset 进转弯目标)是纯相对导航：任何没被
 * 补偿项覆盖的误差(摆车歪、视觉对正、异常残差、读数漂移)都原样带进下一段。
 * 新框架：**转弯直接瞄准名义航向**，target = Nominal ± 90 + 转向×turnLag。
 *   yawNominal  名义航向(°)：车"应该"朝哪。转弯 ±90/180 更新；
 *               视觉对正后 Yaw_NominalSync() 重设(否则对正成果会被下个转弯抹掉)。
 *               当前读数与 Nominal 的差 = 全部历史误差，瞄准 Nominal 即一次性修掉。
 *   turnLag     停位残差学习(°, 停早为正)：死区停车 + 车轮滑行让每次转弯都
 *               少转一点，是机器属性；按本次转向加进目标(停稳后正好落回名义角)。
 *   yawLastResid 上次转弯停稳后的带符号残差(°)：观测/学习源。
 *   movingDelta 自上次转弯以来的行进偏差累计(°)：观测量，不进控制。
 * 软件零偏估计器(ZeroBias_*)已整套删除，零偏由 HWT101 模块固件自校准
 * (HWT101_Init → ManualCal 写 0xA6，协议对 AGV 的推荐用法)。 */
static float yawNominal;     /* 名义航向(°) */
static float turnLag;        /* 停位残差学习(°)：停早为正 */
static float yawLastResid;   /* 上次转弯停稳后的残差(°)：读数 − 目标 */
static float movingDelta;    /* 行进偏差观测累计(°) */
static float movStart;       /* 行进偏差测量的起点读数(°) */

static void AGV_StopNow(void);
static void AGV_SetOmega(float omega);
static float Yaw_WaitSettled(void);

/*
 * 把任意角度归一到 (-180, 180]。
 * 为什么需要：转弯目标角、名义角、残差都可能落在 [-180,180] 之外。
 * **只用一个 if 补 360 只能补一侧**，另一侧漏掉时误差按"数值差"算，
 * 车会朝长边转(看起来像转反/多转)。用 while 双向归一，允许 ±360 整数倍。
 */
static float Yaw_Normalize180(float angle)
{
    while(angle > 180.0f)   angle -= 360.0f;
    while(angle <= -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 装载增益 + 清 PID 运行状态（每次转弯前调）
 * @note  不碰名义角/turnLag：那是跨段要用的导航状态，清掉补偿就白学了。
 *        integral / prevError 每次转弯开头清一次无害（死区里本来就把 integral 清了）。
 */
static void YawPID_Prepare(void)
{
    yawPID.kp = YAW_PID_KP;
    yawPID.ki = YAW_PID_KI;
    yawPID.kd = YAW_PID_KD;
    yawPID.error = 0.0f;
    yawPID.prevError = 0.0f;
    yawPID.integral = 0.0f;
    yawPID.output = 0.0f;
}

/**
 * @brief 初始化航向控制与导航基准（测试模式上电时调；正式流程走 AGV_Init 里的
 *        Yaw_NominalSync()，不依赖本函数）
 * @retval None
 */
void YawPID_Init(void)
{
    YawPID_Prepare();
    yawPID.target = 0.0f;
    yawPID.current = 0.0f;
    Yaw_NominalSync();        /* 当前实际朝向即名义基准 */
    turnLag      = 0.0f;
    yawLastResid = 0.0f;
}

/**
 * @brief 计算角度PID输出
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
static void yawPID_Calculate(void)
{
    yawPID.error = yawPID.target - yawPID.current;
    yawPID.error = (yawPID.error >= 180.0f) ? yawPID.error - 360.0f : yawPID.error;
    yawPID.error = (yawPID.error <= -180.0f) ? yawPID.error + 360.0f : yawPID.error;
    // 误差死区：|误差|<0.2°视为已到达，避免到位后小幅度震荡
    if(yawPID.error < 0.2f && yawPID.error > -0.2f)
    {
        AGV_StopNow();
        yawPID.error = 0.0f;
        yawPID.integral = 0.0f;
        TurnCpltFlag = 1;
        /* 真残差不在这里记：StopNow 刚发出，车轮还在滑行；
           AGV_SetYaw() 的停稳采样里记。 */
    }
    float P_term, I_term, D_term;
    // 比例项
    P_term = yawPID.kp * yawPID.error;
    // 积分项累加并限幅（抗积分饱和，限的是积分本身）
    if(yawPID.error <= 30.0f && yawPID.error >= -30.0f)
    {
        yawPID.integral += yawPID.error;
    }
    if(yawPID.integral > YAW_PID_INT_LIM)
    {
        yawPID.integral = YAW_PID_INT_LIM;
    }
    else if(yawPID.integral < -YAW_PID_INT_LIM)
    {
        yawPID.integral = -YAW_PID_INT_LIM;
    }
    I_term = yawPID.ki * yawPID.integral;
    // 微分项(对测量值微分:prevError 里存的是上一拍 current,prevError-current=-Δcurrent;
    // 常目标下与标准误差微分等价,只起阻尼;差值做±180归一防±360尖峰)
    {
        float d_delta = yawPID.prevError - yawPID.current;
        d_delta = (d_delta >= 180.0f) ? d_delta - 360.0f : d_delta;
        d_delta = (d_delta <= -180.0f) ? d_delta + 360.0f : d_delta;
        D_term = yawPID.kd * d_delta;
    }
    yawPID.prevError = yawPID.current;
    // 总输出
    yawPID.output = P_term + I_term + D_term;
    // 输出限幅（由 YAW_WHEEL_MAX_RPM 反算，见 yawPIDcontroller.h）
    if(yawPID.output > YAW_PID_OUT_LIM)
    {
        yawPID.output = YAW_PID_OUT_LIM;
    }
    else if(yawPID.output < -YAW_PID_OUT_LIM)
    {
        yawPID.output = -YAW_PID_OUT_LIM;
    }
}

/**
 * @brief 阻塞转弯到目标航向（100Hz，每帧 IMU 数据走一拍角度闭环）
 * @param yaw 目标角度(单位:度,范围:-180~180)
 * @retval None
 * @note  转弯结束后等车停稳，把**真残差**记到 yawLastResid 并学习 turnLag。
 *        导航大转角(>YAW_LAG_MIN_TURN)才学；视觉对正那种几度的小修正
 *        残差里噪声占比大，学了反而污染 turnLag。
 */
void AGV_SetYaw(float yaw)
{
    float turnDelta;

    YawPID_Prepare();              /* 自带初始化：不依赖上电时调过 YawPID_Init() */
    yaw = Yaw_Normalize180(yaw);   /* 入口先归一，调用方传什么都安全 */
    turnDelta = Yaw_Normalize180(yaw - HWT101_GetYaw());   /* 本次转角(带转向符号) */

    while(TurnCpltFlag == 0)
    {
        yawPID.target = yaw;
        if(!HWT101_IsDataReady())
        {
            continue; /* 无新数据原地等下帧:不能提前退出,否则本次转弯会被跳过 */
        }
        HWT101_ClearDataReady();
        yawPID.current = Yaw_Normalize180(HWT101_GetYaw());
        yawPID_Calculate();
        if(TurnCpltFlag)break;   /* 完成拍已StopNow,不再下发速度,防止停车后车轮继续转动 */
        omega = yawPID.output*YAW_OMEGA_PER_OUT;
        AGV_SetOmega(omega);
    }
    TurnCpltFlag = 0;

    /* ---- 停稳采样：StopNow 后车轮还要滑几十 ms，完成拍的读数不是真残差 ---- */
    yawLastResid = Yaw_Normalize180(Yaw_WaitSettled() - yaw);

    /* ---- 学停位残差：lag = −转向×残差，停早为正(与转向无关的机器属性) ---- */
    if(turnDelta > YAW_LAG_MIN_TURN || turnDelta < -YAW_LAG_MIN_TURN)
    {
        float lag = ((turnDelta >= 0.0f) ? -1.0f : 1.0f) * yawLastResid;
        if(lag > -YAW_LAG_LIMIT && lag < YAW_LAG_LIMIT)
        {
            turnLag = YAW_LAG_GAIN*lag + (1.0f - YAW_LAG_GAIN)*turnLag;
        }
        /* 超限(打滑/被撞)不学，保住已学的值 */
    }
}

/**
 * @brief 等车停稳并返回停稳后的航向读数
 * @note  相邻两拍读数差 < YAW_SETTLE_EPS 判停稳；最多等 YAW_SETTLE_TICKS 拍。
 *        IMU 掉线时纯自旋把 guard 耗尽后返回当前值，不卡死。
 * @retval float 停稳后的航向(°)
 */
static float Yaw_WaitSettled(void)
{
    float prev = HWT101_GetYaw();
    uint16_t guard = 0U;
    uint8_t stable = 0U;

    while(!stable && ++guard < YAW_SETTLE_TICKS)
    {
        float now, d;

        if(!HWT101_IsDataReady()) continue;   /* 无新帧：guard 仍计数，不会死等 */
        HWT101_ClearDataReady();
        now = HWT101_GetYaw();
        d = Yaw_Normalize180(now - prev);
        prev = now;
        stable = (uint8_t)(d < YAW_SETTLE_EPS && d > -YAW_SETTLE_EPS);
    }
    return prev;
}

/* ---- AGV_TrimYaw() 的参数 ----
   半度级微调用一套独立的小闭环，**不复用 yawPID/AGV_SetYaw**，原因有两个：
     ① AGV_SetOmega() 把角速度按 (uint16_t) 截断成 RPM，|ω| < 0.98°/s 直接变 0；
        小误差下前几十拍根本不发速度，只能靠积分慢慢往上顶；
     ② AGV_SetYaw() 的 while(TurnCpltFlag==0) 没有超时，顶不起来就是死循环。
   这里显式给最低角速度 + 总时长上限：0.5° 一定动得起来，也一定出得来。 */
#define YAW_TRIM_KP         8.0f    /* 比例增益：0.5° 误差 → 4°/s */
#define YAW_TRIM_OMEGA      15.0f   /* 角速度上限(°/s)：0.5° 约 130ms 走完 */
#define YAW_TRIM_OMEGA_MIN  3.0f    /* 角速度下限(°/s)：≈3RPM，躲开 (uint16_t) 截断黑洞 */
#define YAW_TRIM_TOL        0.05f   /* 到位死区(°) */
#define YAW_TRIM_TICK_MS    10U     /* 控制节拍(ms) */
#define YAW_TRIM_TMO_MS     1500U   /* 总时长上限(ms)：到点强制停车退出，绝不卡死 */

/**
 * @brief 相对转角微调（闭环、带最低角速度、带超时）
 * @param delta_deg 相对转角(°)，正负号与 AGV_SetYaw 的目标角同向
 * @retval None
 * @note  给"半度级"前馈补偿用（例：粗加工区低位对正后的固定 0.5° 补角）。
 *        和 AGV_SetYaw 的分工：
 *          - 不学 turnLag、不改 yawNominal —— 想让它进名义基准由调用方 Yaw_NominalSync()；
 *          - 不写 yawLastResid —— 那仍是"上次 AGV_SetYaw 的停稳残差"；
 *          - 小误差也有速度（YAW_TRIM_OMEGA_MIN），超时必定退出并停车。
 *        退出前一定 AGV_StopNow，所以调用返回时车是停住的。
 *        ⚠ 转向正负跟 AGV_SetYaw 一致；实车方向反了就让调用方传负值。
 */
void AGV_TrimYaw(float delta_deg)
{
    const float target = Yaw_Normalize180(HWT101_GetYaw() + delta_deg);
    uint32_t start = HAL_GetTick();

    /* 本来就不需要补：什么都不发，连停车命令都不用（省一次 CAN 往返） */
    if(delta_deg <= YAW_TRIM_TOL && delta_deg >= -YAW_TRIM_TOL)
    {
        return;
    }

    while((HAL_GetTick() - start) < YAW_TRIM_TMO_MS)
    {
        float err = Yaw_Normalize180(target - HWT101_GetYaw());
        float mag;

        if(err < YAW_TRIM_TOL && err > -YAW_TRIM_TOL)
        {
            break;                      /* 到位 */
        }

        mag = YAW_TRIM_KP * err;        /* 比例 */
        if(mag >  YAW_TRIM_OMEGA) mag =  YAW_TRIM_OMEGA;
        if(mag < -YAW_TRIM_OMEGA) mag = -YAW_TRIM_OMEGA;
        /* 下限：|ω| 太小会被 AGV_SetOmega() 截断成 0（步进压根不动），
           这里按误差符号把幅度抬到 YAW_TRIM_OMEGA_MIN */
        if(mag > -YAW_TRIM_OMEGA_MIN && mag < YAW_TRIM_OMEGA_MIN)
        {
            mag = (err >= 0.0f) ? YAW_TRIM_OMEGA_MIN : -YAW_TRIM_OMEGA_MIN;
        }

        AGV_SetOmega(mag);
        HAL_Delay(YAW_TRIM_TICK_MS);
    }

    AGV_StopNow();                      /* 到位/超时都停住，绝不带着速度返回 */
}

/**
 * @brief 左转 90°
 * @note  目标 = 名义角 + (90 + turnLag)。**当前读数不进目标**——
 *        行进偏差/陀螺漂移/摆车误差全在「读数−名义角」里，瞄准名义角即一次性修掉。
 *        turnLag(停早为正)让停稳后正好落回名义角。
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_TurnLeft(void)
{
    yawNominal = Yaw_Normalize180(yawNominal + (90.0f + turnLag));
    AGV_SetYaw(yawNominal);
    AGV_Direction = (AGV_Direction+1)%4;
    movingDelta = 0.0f;   /* 行进偏差已随本次瞄准修掉，重新累计下一段 */
}

/**
 * @brief 右转 90°
 * @note  同 TurnLeft，方向取反：−(90 + turnLag)。
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_TurnRight(void)
{
    yawNominal = Yaw_Normalize180(yawNominal - (90.0f + turnLag));
    AGV_SetYaw(yawNominal);
    AGV_Direction = (AGV_Direction+3)%4;
    movingDelta = 0.0f;
}

/**
 * @brief 掉头 180°
 * @note  与 TurnLeft 同向(+180)。turnLag 直接复用 90° 学到的值：
 *        末段逼近的动力学只由误差决定，与总转角无关。
 * @retval None
 */
void AGV_TurnBack(void)
{
    yawNominal = Yaw_Normalize180(yawNominal + (180.0f + turnLag));
    AGV_SetYaw(yawNominal);
    AGV_Direction = (AGV_Direction+2)%4;
    movingDelta = 0.0f;
}

void AGV_Turn(uint8_t dir)
{
    /* ⚠ 必须是**有符号**类型：dir - AGV_Direction ∈ [-3, 3]，右转/-1、掉头/-2、
       跨 0/3 的 -3 都是负数。原来写 uint8_t，-1/-2/-3 会被截成 255/254/253，
       后面所有 == -1 / == -2 / == -3 的判断永远不成立 → **右转、跨0/3转、部分掉头
       全部静默不执行**（12 种场景里 7 种），AGV_Direction 也不更新 → 后面走位全错。 */
    int8_t turnDir = (int8_t)(dir - AGV_Direction);
    if(turnDir == 3)turnDir = -1;      /* 3 等价于 -1（右转 90°） */
    else if (turnDir == -3)turnDir = 1;/* -3 等价于 +1（左转 90°） */
    if(turnDir == 2||turnDir == -2)
    {
        AGV_TurnBack();
    }
    else if(turnDir == 1)
    {
        AGV_TurnLeft();
    }
    else if(turnDir == -1)
    {
        AGV_TurnRight();
    }
}

/**
 * @brief 把当前实际航向设为新的名义基准
 * @note  视觉对正/人工挪车后必须调——对正是"有意"改航向，不调的话下一次转弯
 *        会把车拧回旧的名义角，对正成果被抹掉。上电 AGV_Init() 里也调一次
 *        (HWT101_Init 之后)：摆好车，当前朝向即基准。
 *        AGV_Correct() 不用调：它末尾 AGV_SetYaw(temp_yaw) 恢复原航向。
 * @retval None
 */
void Yaw_NominalSync(void)
{
    yawNominal = HWT101_GetYaw();
    movingDelta = 0.0f;
}

/*
 * @brief 开始记录行进偏差(AGV_Position_GoStraight/Translate 内部成对调用)
 * @note  平移/直行理论上不改变航向：期间读数变化 = 真实跑偏 + 陀螺漂移。
 *        新框架里它**不进转弯目标**——行进偏差随"瞄准名义角"被自动修掉，
 *        这里只作观测量：watch Yaw_GetMovingDelta() 看每段直行的航向稳定性，
 *        数值持续偏大说明打滑/配重问题(机械)，控制层修不了根因。
 */
void Yaw_MovingOffsetStart(void)
{
    movStart = HWT101_GetYaw();
}

/*
 * @brief 结束记录行进偏差(累加，支持一趟 Go 的直行+平移两段)
 */
void Yaw_MovingOffsetEnd(void)
{
    movingDelta += Yaw_Normalize180(HWT101_GetYaw() - movStart);
}

/* ---- 观测口：debugger watch / OLED / 上位机 ---- */
float Yaw_GetNominal(void)     { return yawNominal;   }
float Yaw_GetTurnLag(void)     { return turnLag;      }
float Yaw_GetLastResid(void)   { return yawLastResid; }
float Yaw_GetMovingDelta(void) { return movingDelta;  }

/**
 * @brief 设置目标角速度（车体°/s -> 轮子RPM，四轮同向原地旋转）
 * 
 * @param omega 目标角速度(单位:度/秒,范围:-500~500)
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_SetOmega(float omega)
{
    /* 正负只决定转向dir，转速统一取绝对值；omega==0时dir=1但vel=0，等价停车指令 */
    uint8_t dir = (omega >= 0.0f) ? 1U : 0U;
    float mag  = (omega < 0.0f) ? -omega : omega;
    uint16_t vel = (uint16_t)(YAW_RPM_PER_OMEGA * mag);

    Emm_V5_Vel_Control(1, dir, vel, 0, 1);
    delay_us(250);
    Emm_V5_Vel_Control(2, dir, vel, 0, 1);
    delay_us(250);
    Emm_V5_Vel_Control(3, dir, vel, 0, 1);
    delay_us(250);
    Emm_V5_Vel_Control(4, dir, vel, 0, 1);
    delay_us(250);

    Emm_V5_Synchronous_motion(0);    // 广播地址0触发
    delay_us(250);

    Emm_V5_WaitRxFrame(EMM_V5_ACK_TMO_MS);   /* 等命令回帧(原为死等，加超时防卡死) */
}

/**
 * @brief 停车!
 * @retval None
 * @author Fawziya
 * @date 2026-08-18
 */
void AGV_StopNow(void)
{
    Emm_V5_Stop_Now(1, 1);
	delay_us(250);
	Emm_V5_Stop_Now(2, 1);
	delay_us(250);
	Emm_V5_Stop_Now(3, 1);
	delay_us(250);
	Emm_V5_Stop_Now(4, 1);
	delay_us(250);
	
	Emm_V5_Synchronous_motion(0); 								 // 广播地址0触发
	delay_us(250);	
	
	Emm_V5_WaitRxFrame(EMM_V5_ACK_TMO_MS);		// 等命令回帧(原为死等，加超时防卡死)
}
   
/**
 * @brief 将航向角格式化为字符串（nano.specs下sprintf不支持%f，改用整数拆分）
 * @param yaw 航向角（°，-180~180）
 * @param buf 输出缓冲区（至少8字节）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void FormatYaw(float yaw, char *buf)
{
    int sign = (yaw < 0.0f) ? -1 : 1;
    int v = (int)(yaw * 100.0f * (float)sign + 0.5f);   /* 绝对值放大100倍并四舍五入 */
    if(sign < 0)
    {
        sprintf(buf, "-%d.%02d", v / 100, v % 100);
    }
    else
    {
        sprintf(buf, "%d.%02d", v / 100, v % 100);
    }
}

/**
 * @brief 主循环周期调用：在OLED固定行(80,48)刷新当前航向角(调试显示)
 * @retval None
 */
void Yaw_OLED_Show(void)
{
    char buf[16];
    FormatYaw(HWT101_GetYaw(), buf);
    OLED_ShowString(80, 48, (uint8_t *)buf, 16, 0);
    OLED_Refresh();
}
