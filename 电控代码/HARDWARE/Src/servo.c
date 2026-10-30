/**
 * @file servo.c
 * @brief PWM 舵机驱动，手写 TIM 初始化，不依赖 CubeMX 重新生成。
 *        50Hz（20ms），脉宽换算由每路舵机的角度/脉宽配置决定。
 * @note  Z轴舵机: TIM1_CH1=PE9, 量程0°~270° (500~2500us)
 *        夹爪舵机: TIM3_CH1=PA6, 量程0°~180° (500~2500us)
 *        转盘舵机: TIM2_CH1=PA5, 量程0°~270° (500~2500us)
 *
 *        2026-10-14 接线调整：**Z轴 与 车上转盘 这两路对调**（原 Z=PA5/TIM2、
 *        转盘=PE9/TIM1），夹爪 PA6 没动。
 *        **对调的是"定时器+通道+引脚+AF+预分频"这一组，不是整个配置块**：
 *          * 预分频必须按**定时器**取 —— TIM1 在 APB2 要 167、TIM2 在 APB1 要 83。
 *            跟着引脚走的话 TIM1 会被配成 83，PWM 变成 10ms/100Hz，舵机抖动或不动作。
 *          * 角度量程、脉宽、上电初始角、加减速参数是跟着**哪一路舵机**走的，
 *            没跟着引脚跑（Z轴 0~270°、转盘 0~270°，起始角都是 0°）。
 *            注意量程里的 360→270 是**换舵机**改的（同一天），跟这次对调无关。
 *            【2026-10-14 先后改了两处，别混：①两路引脚对调 ②Z轴量程 360→270。】
 *
 *        全工程的舵机引脚**只有本文件在配**：生成的 stm32f4xx_hal_msp.c 里
 *        只有 HAL_MspInit()，TIM1/2/3 的 MspInit 一个都没有；htim1/htim2/htim3
 *        除声明无人使用。所以改接线只需要动这个文件。
 *        改接线/换舵机：只改本文件顶部配置区。
 */
#include "servo.h"
#include "stm32f4xx_hal.h"
#include "pick.h"      /* SERVO_Z_BASE_OFFSET / SERVO_GRIP_BASE_OFFSET（和姿态宏放一起调） */
#include <math.h>      /* fabsf */

/* ==================== 可移植配置区 ====================
   预分频为什么每路不一样：
   TIMxCLK 是"APBx 分频系数≠1 时取 2×PCLKx"。
     HCLK=168MHz, APB1=DIV4 → PCLK1=42MHz → TIM2/3/4 时钟 = 84MHz → 预分频 83 得 1MHz
     HCLK=168MHz, APB2=DIV2 → PCLK2=84MHz → TIM1   时钟 = 168MHz → 预分频 167 得 1MHz
   按 83 配 TIM1 会让 PWM 周期变成 10ms(100Hz)，舵机会抖动/不动作。
   改主频或改 APBx 分频时，这几路预分频必须跟着重算。 */
/* Z轴旋转舵机: 量程 0°~270°, 对应脉宽 500~2500us
   引脚 PE9 = TIM1_CH1（2026-10-14 由 PA5/TIM2 换过来，与转盘对调）
   2026-10-14：量程 360° → **270°**（换舵机）。脉宽还是 500~2500us，所以每度对应
   的脉宽从 5.56us/° 变成 7.41us/°，**同样的"命令角"现在物理转得更远**。
   后果：pick.h 里 POS_* 那批宏的第一个值(角度z)全都指向了新的物理位置 ——
   数值本身不用改，但每个姿态实际偏了，正式跑之前得重新对一遍位。
   （按 270° 舵机算，这次改完才是"命令角=物理角"，改之前那批角度其实是被
   360° 的换算压小了，所以这次是往"对"的方向修。但姿态是照旧映射调出来的，
   仍然要重新对。） */
#define SERVO_Z_TIM       TIM1
#define SERVO_Z_CH        TIM_CHANNEL_1
#define SERVO_Z_PORT      GPIOE
#define SERVO_Z_PIN       GPIO_PIN_9
#define SERVO_Z_AF        GPIO_AF1_TIM1
#define SERVO_Z_PRESCALER 167      /* 168MHz/168 = 1MHz(见上方说明) */
#define SERVO_Z_MIN_ANG   0.0f
#define SERVO_Z_MAX_ANG   270.0f
#define SERVO_Z_MIN_PULSE 500
#define SERVO_Z_MAX_PULSE 2500
#define SERVO_Z_START_ANG 0.0f    /* 上电初始角(°)：0°=量程起点 */

/* 夹爪舵机: 量程 0°~180°, 对应脉宽 500~2500us
   引脚 PA6 = TIM3_CH1（2026-10-14 那次 Z轴↔转盘 对调**没碰它**）
   2026-10-14：**换舵机**，量程 270° → **180°**（跟 Z轴的 360→270 同一天改的，
   但那两件事互不相干，别混）。脉宽还是 500~2500us 这 2000us 跨度，于是每度
   对应的脉宽从 2000/270 = 7.41us/° 变成 2000/180 = 11.11us/° ——
   **同一个"命令角"现在物理转 1.5 倍远**。
   ⚠ 后果：pick.h 的 PICK_OBSERVE/PICK_GRAB/PICK_RELEASE **这次故意没跟着缩**
   （留着原值当参照），所以它们指向的爪口位置全都偏了，**烧之前必须重对**
   （2026-10-14 当天已经在改了）。等比缩的起点是原值 × 180/270 = 0.667。
   对着看用模式 9 的 Pick_Test_Gripper()（观察/抓紧/放开 走 3 轮）或
   模式 3 的 Test_Pick_Scale()（开合死循环）。
   好消息：三个数都远小于 180，不会触发 Servo_SetAngle 里 `> maxAngle` 那句
   钳位，不存在被静默削角的坑。
   ⚠ 另一件必须上台架确认的事：**新舵机的脉宽标定**。这里沿用了 500~2500us
   对应 0~180° 的假设（跟原来 270° 那款一致），但 180° 舵机里有不少是
   1000~2000us 对应 0~180° 的（MG996R 一类）。要是新舵机属于后者，现在这套
   参数会把它往 0° 以下 / 180° 以上推 → 行程两端顶死、持续堵转嗡嗡响、发烫。
   验法：单独给 0.0f 和 180.0f 两个角，看舵机是不是**刚好停在机械两端**、
   有没有顶住的堵转声。不对就把 SERVO_G_MIN/MAX_PULSE 一起改（比如改成
   1000/2000），改完三个角度又要重对一遍。 */
#define SERVO_G_TIM       TIM3
#define SERVO_G_CH        TIM_CHANNEL_1
#define SERVO_G_PORT      GPIOA
#define SERVO_G_PIN       GPIO_PIN_6
#define SERVO_G_AF        GPIO_AF2_TIM3
#define SERVO_G_PRESCALER 83       /* 84MHz/84 = 1MHz */
#define SERVO_G_MIN_ANG   0.0f
#define SERVO_G_MAX_ANG   180.0f   /* 2026-10-14：270→180(换舵机)，见上方说明 */
#define SERVO_G_MIN_PULSE 500
#define SERVO_G_MAX_PULSE 2500
#define SERVO_G_START_ANG 0.0f    /* 上电初始角(°)：0°=量程起点 */

/* 车上转盘舵机: 量程 0°~270°, 30°/150°/270° = A/B/C 槽
   引脚 PA5 = TIM2_CH1（2026-10-14 由 PE9/TIM1 换过来，与 Z轴对调） */
#define SERVO_T_TIM       TIM2
#define SERVO_T_CH        TIM_CHANNEL_1
#define SERVO_T_PORT      GPIOA
#define SERVO_T_PIN       GPIO_PIN_5
#define SERVO_T_AF        GPIO_AF1_TIM2
#define SERVO_T_PRESCALER 83       /* 84MHz/84 = 1MHz */
#define SERVO_T_MIN_ANG   0.0f
#define SERVO_T_MAX_ANG   270.0f
#define SERVO_T_MIN_PULSE 500
#define SERVO_T_MAX_PULSE 2500
#define SERVO_T_START_ANG 0.0f     /* 上电停在 0° = A 槽 */

/* 雷达收放舵机: 量程 0°~360°, 对应脉宽 500~2500us
   引脚 PD12 = TIM4_CH1（CubeMX 里 tim.c 的 HAL_TIM_MspPostInit 早配好了
   GPIO_AF2_TIM4，之前一直没用上；这里把它接进舵机表）
   **0° = 雷达伸出、340° = 雷达收入车内**（2026-10-14 收起角 360°→180°(保险)→
   300°→**340°**(用户确认)；量程仍 0~360°）；上电初值 340°（收起）= 2389us
   ⚠ 340° 对应 2389us，离量程端点 2500us 只剩 111us(≈20°)，机械限位/堵转风险高于 300° */
#define SERVO_R_TIM       TIM4
#define SERVO_R_CH        TIM_CHANNEL_1
#define SERVO_R_PORT      GPIOD
#define SERVO_R_PIN       GPIO_PIN_12
#define SERVO_R_AF        GPIO_AF2_TIM4
#define SERVO_R_PRESCALER 83       /* 84MHz/84 = 1MHz(APB1 定时器时钟) */
#define SERVO_R_MIN_ANG   0.0f
#define SERVO_R_MAX_ANG   360.0f
#define SERVO_R_MIN_PULSE 500
#define SERVO_R_MAX_PULSE 2500
#define SERVO_R_START_ANG 340.0f   /* 上电初始角(°)：340° = 收起，别一上电就弹出去 */

/* 方向：0 = 正常，1 = **反向**（角度 → 脉宽 的映射翻过来，500us↔2500us 对调）。
   判断方法：让舵机去 0°，如果它往相反方向跑（或去 340° 反而跑到另一侧），置 1 试试。
   ⚠ 注意：反向只是把角度↔脉宽映射翻过来，**不会改变"哪个角度是收起位"** ——
     如果"上电停在伸出侧"，那不是方向问题，是角度值不对
     （改 radar_config.h 的 RADAR_SERVO_IN_DEG，或把 SERVO_R_MAX_ANG 改成舵机真实行程）。 */
#define SERVO_R_INVERT    0

/* 上电"接近角"（2026-10-14 默认**关闭**）：
   作用：上电时若舵机不在收起位，先起在 收起角−Δ 再平滑到收起角，让"上电这一跳"的方向确定
        （否则上电那一跳的方向可能由舵机自选，往伸出侧甩）。
   为什么默认关：现场常态是"上电前雷达就收着"（每次跑完都会收回），此时起在 收起角−Δ 会让它
        **外摆 Δ 度** 再回来 —— 用户 2026-10-14 反馈"上电往外伸了一点点"就是这个。
       关掉后上电直接给 340° 对应的 2389us：**已在收起位就完全不动**。
   何时打开：如果某次上电前雷达不在收起位（比如手动摆过/异常断电），又担心方向，把它置 1。 */
#define SERVO_R_APPROACH_ENABLE  0
#define SERVO_R_APPROACH_ANG     (SERVO_R_APPROACH_ENABLE ? (SERVO_R_START_ANG - 5.0f) : 0.0f)

/* 舵机运动参数(每路的 RAMP / STEP / SETTLE)已上移到 servo.h */
/* ===================================================== */

typedef struct {
    TIM_TypeDef *tim;
    uint32_t    channel;
    GPIO_TypeDef *port;
    uint16_t    pin;
    uint8_t     af;
    uint16_t    prescaler;   /* 使计数时钟 = 1MHz(每路可能不同，见配置区说明) */
    float       minAngle;
    float       maxAngle;
    uint16_t    minPulse;
    uint16_t    maxPulse;
    float       startAngle;
    uint8_t     rampSteps;   /* 加减速段步数，每路独立 */
    float       rampStart;   /* 起步/收尾首步比例，每路独立 */
    uint8_t     invert;      /* 1 = 角度→脉宽反向（舵机/摇臂装反时用），见 SERVO_R_INVERT */
    float       approachAngle;/* >0 时：起 PWM 先到这个角，再平滑转到 startAngle（让上电方向确定） */
} ServoCfg;

static const ServoCfg servoCfg[4] = {
    /* SERVO_Z     */ { SERVO_Z_TIM, SERVO_Z_CH, SERVO_Z_PORT, SERVO_Z_PIN, SERVO_Z_AF, SERVO_Z_PRESCALER,
                        SERVO_Z_MIN_ANG, SERVO_Z_MAX_ANG, SERVO_Z_MIN_PULSE, SERVO_Z_MAX_PULSE,
                        SERVO_Z_START_ANG, SERVO_Z_RAMP_STEPS, SERVO_Z_RAMP_START, 0U, 0.0f },
    /* SERVO_GRIP  */ { SERVO_G_TIM, SERVO_G_CH, SERVO_G_PORT, SERVO_G_PIN, SERVO_G_AF, SERVO_G_PRESCALER,
                        SERVO_G_MIN_ANG, SERVO_G_MAX_ANG, SERVO_G_MIN_PULSE, SERVO_G_MAX_PULSE,
                        SERVO_G_START_ANG, SERVO_G_RAMP_STEPS, SERVO_G_RAMP_START, 0U, 0.0f },
    /* SERVO_TURN  */ { SERVO_T_TIM, SERVO_T_CH, SERVO_T_PORT, SERVO_T_PIN, SERVO_T_AF, SERVO_T_PRESCALER,
                        SERVO_T_MIN_ANG, SERVO_T_MAX_ANG, SERVO_T_MIN_PULSE, SERVO_T_MAX_PULSE,
                        SERVO_T_START_ANG, SERVO_T_RAMP_STEPS, SERVO_T_RAMP_START, 0U, 0.0f },
    /* SERVO_RADAR */ { SERVO_R_TIM, SERVO_R_CH, SERVO_R_PORT, SERVO_R_PIN, SERVO_R_AF, SERVO_R_PRESCALER,
                        SERVO_R_MIN_ANG, SERVO_R_MAX_ANG, SERVO_R_MIN_PULSE, SERVO_R_MAX_PULSE,
                        SERVO_R_START_ANG, SERVO_R_RAMP_STEPS, SERVO_R_RAMP_START, SERVO_R_INVERT, SERVO_R_APPROACH_ANG },
};

static TIM_HandleTypeDef hServo[4] = {0};

/* 该路舵机的基础修正值(°)：只有 Z轴、夹爪两路有，转盘/雷达恒为 0。
   数值在 pick.h 的 SERVO_Z_BASE_OFFSET / SERVO_GRIP_BASE_OFFSET，
   和 POS_* / PICK_* 那批姿态角放一起，重装舵机后只改那一个数。 */
static float Servo_BaseOffset(ServoId id)
{
    if(id == SERVO_Z)    return SERVO_Z_BASE_OFFSET;
    if(id == SERVO_GRIP) return SERVO_GRIP_BASE_OFFSET;
    return 0.0f;
}

/* 角度 <-> 脉宽 换算（按每路的 invert 处理方向）。
   正常：角度 minAngle 对应 minPulse；
   反向：角度 minAngle 对应 maxPulse（500us↔2500us 对调）。
   入参/出参都是**姿态角**（宏里写的那个数的语义），基础修正在函数内部加减：
   出脉宽时 +offset，读回角度时 −offset。这样上层(Servo_GetAngle/Smooth/调用方)
   看到的永远是宏里的角度，重装舵机只改 offset。
   ⚠ 不做越界保护：offset 把角度推到量程外时，脉宽会跟着超出去。 */
static uint32_t Servo_AngleToPulse(ServoId id, const ServoCfg *c, float angle)
{
    float ang  = angle + Servo_BaseOffset(id);      /* 命令角 = 姿态角 + 修正 */
    float span = c->invert ? (c->maxAngle - ang) : (ang - c->minAngle);
    return (uint32_t)(c->minPulse
                      + span * (float)(c->maxPulse - c->minPulse)
                        / (c->maxAngle - c->minAngle) + 0.5f);
}

static float Servo_PulseToAngle(ServoId id, const ServoCfg *c, uint32_t pulse)
{
    float ang = c->minAngle
              + (float)(pulse - c->minPulse) * (c->maxAngle - c->minAngle)
                / (float)(c->maxPulse - c->minPulse);
    if(c->invert) ang = c->maxAngle + c->minAngle - ang;
    return ang - Servo_BaseOffset(id);              /* 还原成姿态角 */
}

/**
 * @brief 初始化一路舵机：使能时钟 -> 引脚复用 -> PWM
 */
static void Servo_TimInit(uint8_t id)
{
    const ServoCfg *c = &servoCfg[id];

    if(c->tim == TIM1)       __HAL_RCC_TIM1_CLK_ENABLE();
    else if(c->tim == TIM2)  __HAL_RCC_TIM2_CLK_ENABLE();
    else if(c->tim == TIM3)  __HAL_RCC_TIM3_CLK_ENABLE();
    else if(c->tim == TIM4)  __HAL_RCC_TIM4_CLK_ENABLE();
    if(c->port == GPIOA)     __HAL_RCC_GPIOA_CLK_ENABLE();
    else if(c->port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if(c->port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if(c->port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin = c->pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = c->af;
    HAL_GPIO_Init(c->port, &g);

    /* TIMxCLK/(prescaler+1) = 1MHz, ARR=19999 -> 20ms -> 50Hz */
    TIM_HandleTypeDef *h = &hServo[id];
    h->Instance = c->tim;
    h->Init.Prescaler = c->prescaler;
    h->Init.Period = 19999;
    h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(h);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    /* 初值按上电初始角换算，避免写死中位1500导致上电冲到量程中位 */
    /* 上电方向确定化：有"接近角"时先起在接近角，再平滑转到上电初值（见 SERVO_R_APPROACH_ANG）。
       没有接近角（另外三路）就是原来的行为。 */
    float bootAngle = c->startAngle;
    if(c->approachAngle > c->minAngle && c->approachAngle < c->maxAngle)
        bootAngle = c->approachAngle;
    oc.Pulse = Servo_AngleToPulse((ServoId)id, c, bootAngle);   /* 含 invert + 基础修正 */
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(h, &oc, c->channel);
    HAL_TIM_PWM_Start(h, c->channel);

    if(bootAngle != c->startAngle)
        Servo_SetAngleSmooth((ServoId)id, c->startAngle, SERVO_R_STEP_DEG, SERVO_R_STEP_MS);
}

/* 该路舵机是否已初始化（PWM 是否已起）。给"首次使用时才初始化"的调用方用 */
bool Servo_IsReady(ServoId id)
{
    if(id > SERVO_RADAR) return false;
    return hServo[id].Instance != NULL;
}

void Servo_InitId(ServoId id)
{
    if(id > SERVO_RADAR) return;
    Servo_TimInit((uint8_t)id);   /* 重复调用安全：重新 init 并回到该路上电初值 */
}

void Servo_Init(void)
{
    Servo_TimInit(SERVO_Z);
    Servo_TimInit(SERVO_GRIP);
    Servo_TimInit(SERVO_TURN);
    Servo_TimInit(SERVO_RADAR);   /* 雷达收放：上电复位到 340°(收起) */
}

void Servo_SetAngle(ServoId id, float angle)
{
    if(id > SERVO_RADAR) return;
    const ServoCfg *c = &servoCfg[id];
    if(angle > c->maxAngle) angle = c->maxAngle;
    if(angle < c->minAngle) angle = c->minAngle;
    __HAL_TIM_SET_COMPARE(&hServo[id], c->channel, Servo_AngleToPulse(id, c, angle));
}

/**
 * @brief 读取舵机当前命令角(°)，由当前比较寄存器反推
 * @note  返回的是"已下达的指令角"，不是物理回读角（位置舵机无反馈）
 */
float Servo_GetAngle(ServoId id)
{
    if(id > SERVO_RADAR) return 0.0f;
    const ServoCfg *c = &servoCfg[id];
    return Servo_PulseToAngle(id, c, __HAL_TIM_GET_COMPARE(&hServo[id], c->channel));
}

/**
 * @brief 平滑转动到目标角度：梯形速度曲线，中间按设定速度走，开头/结尾加减速
 * @note  位置舵机转速由内部电机决定，软件只能靠分步+延时"模拟慢转"。
 *        开头/结尾各 SERVO_{Z,G}_RAMP_STEPS 步从 SERVO_{Z,G}_RAMP_START 比例缓慢起步/收尾，
 *        避免起步/停车时的惯性冲击造成材料磨损；设 RAMP_STEPS=0 即退化为匀速。
 *        加减速参数每路独立（见 ServoCfg.rampSteps/rampStart）。
 * @param id          舵机编号
 * @param angle       目标角(°)
 * @param stepDeg     每步转过的角度(°)，中间巡航段用（如 1.0f）
 * @param stepDelayMs 每步间隔(ms)，全程固定（如 15）
 */
void Servo_SetAngleSmooth(ServoId id, float angle, float stepDeg, uint16_t stepDelayMs)
{
    if(id > SERVO_RADAR) return;
    if(stepDeg <= 0.0f) { Servo_SetAngle(id, angle); return; }
    const ServoCfg *c = &servoCfg[id];

    if(angle > c->maxAngle) angle = c->maxAngle;
    if(angle < c->minAngle) angle = c->minAngle;

    float start = Servo_GetAngle(id);      /* 当前命令角 */
    float total = fabsf(angle - start);    /* 总行程(°) */
    if(total < 1e-3f) return;              /* 已在目标 */

    float dir = (angle >= start) ? 1.0f : -1.0f;

    /* 加减速段配置（取本路舵机自己的参数） */
    uint8_t rampSteps = c->rampSteps;
    float   startRatio = c->rampStart;
    if(rampSteps == 0) startRatio = 1.0f;  /* 关闭加减速 = 原匀速 */

    float rampLen = (float)rampSteps * stepDeg;   /* 期望的加速/减速段距离(°) */
    float accelLen, decelLen;
    if(total >= 2.0f * rampLen) { accelLen = rampLen; decelLen = rampLen; }
    else                        { accelLen = total * 0.5f; decelLen = total * 0.5f; } /* 小行程变三角形 */

    float cur = start;
    uint32_t guard = 0;
    uint32_t guardMax = (uint32_t)(total / (stepDeg * startRatio)) + 20u;

    while(fabsf(cur - start) < total - 1e-3f)
    {
        float moved  = fabsf(cur - start);
        float remain = total - moved;
        if(remain <= 0.0f) break;

        float step;
        if(moved < accelLen)                 /* 开头：加速段，步幅从小到大 */
        {
            step = stepDeg * (startRatio + (1.0f - startRatio) * (moved / accelLen));
        }
        else if(remain <= decelLen)          /* 结尾：减速段，步幅从大到小 */
        {
            step = stepDeg * (startRatio + (1.0f - startRatio) * (remain / decelLen));
        }
        else                                 /* 中间：设定速度 */
        {
            step = stepDeg;
        }

        if(step > remain) step = remain;     /* 最后一步不越过目标 */
        cur += dir * step;
        Servo_SetAngle(id, cur);
        HAL_Delay(stepDelayMs);
        if(++guard > guardMax) break;        /* 防死循环 */
    }
}
