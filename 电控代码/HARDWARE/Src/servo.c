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
} ServoCfg;

static const ServoCfg servoCfg[3] = {
    /* SERVO_Z    */ { SERVO_Z_TIM, SERVO_Z_CH, SERVO_Z_PORT, SERVO_Z_PIN, SERVO_Z_AF, SERVO_Z_PRESCALER,
                       SERVO_Z_MIN_ANG, SERVO_Z_MAX_ANG, SERVO_Z_MIN_PULSE, SERVO_Z_MAX_PULSE,
                       SERVO_Z_START_ANG, SERVO_Z_RAMP_STEPS, SERVO_Z_RAMP_START },
    /* SERVO_GRIP */ { SERVO_G_TIM, SERVO_G_CH, SERVO_G_PORT, SERVO_G_PIN, SERVO_G_AF, SERVO_G_PRESCALER,
                       SERVO_G_MIN_ANG, SERVO_G_MAX_ANG, SERVO_G_MIN_PULSE, SERVO_G_MAX_PULSE,
                       SERVO_G_START_ANG, SERVO_G_RAMP_STEPS, SERVO_G_RAMP_START },
    /* SERVO_TURN */ { SERVO_T_TIM, SERVO_T_CH, SERVO_T_PORT, SERVO_T_PIN, SERVO_T_AF, SERVO_T_PRESCALER,
                       SERVO_T_MIN_ANG, SERVO_T_MAX_ANG, SERVO_T_MIN_PULSE, SERVO_T_MAX_PULSE,
                       SERVO_T_START_ANG, SERVO_T_RAMP_STEPS, SERVO_T_RAMP_START },
};

static TIM_HandleTypeDef hServo[3] = {0};

/**
 * @brief 初始化一路舵机：使能时钟 -> 引脚复用 -> PWM
 */
static void Servo_TimInit(uint8_t id)
{
    const ServoCfg *c = &servoCfg[id];

    if(c->tim == TIM1)       __HAL_RCC_TIM1_CLK_ENABLE();
    else if(c->tim == TIM2)  __HAL_RCC_TIM2_CLK_ENABLE();
    else if(c->tim == TIM3)  __HAL_RCC_TIM3_CLK_ENABLE();
    if(c->port == GPIOA)     __HAL_RCC_GPIOA_CLK_ENABLE();
    else if(c->port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
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
    oc.Pulse = (uint32_t)(c->minPulse
               + (c->startAngle - c->minAngle) * (float)(c->maxPulse - c->minPulse)
                 / (c->maxAngle - c->minAngle) + 0.5f);
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(h, &oc, c->channel);
    HAL_TIM_PWM_Start(h, c->channel);
}

void Servo_Init(void)
{
    Servo_TimInit(SERVO_Z);
    Servo_TimInit(SERVO_GRIP);
    Servo_TimInit(SERVO_TURN);
}

void Servo_SetAngle(ServoId id, float angle)
{
    if(id > SERVO_TURN) return;
    const ServoCfg *c = &servoCfg[id];
    if(angle > c->maxAngle) angle = c->maxAngle;
    if(angle < c->minAngle) angle = c->minAngle;
    float pulse = c->minPulse
                + (angle - c->minAngle) * (float)(c->maxPulse - c->minPulse)
                  / (c->maxAngle - c->minAngle);
    __HAL_TIM_SET_COMPARE(&hServo[id], c->channel, (uint32_t)(pulse + 0.5f));
}

/**
 * @brief 读取舵机当前命令角(°)，由当前比较寄存器反推
 * @note  返回的是"已下达的指令角"，不是物理回读角（位置舵机无反馈）
 */
float Servo_GetAngle(ServoId id)
{
    if(id > SERVO_TURN) return 0.0f;
    const ServoCfg *c = &servoCfg[id];
    uint32_t curPulse = __HAL_TIM_GET_COMPARE(&hServo[id], c->channel);
    return c->minAngle
         + (float)(curPulse - c->minPulse) * (c->maxAngle - c->minAngle)
           / (float)(c->maxPulse - c->minPulse);
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
    if(id > SERVO_TURN) return;
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
