/**
 * @file servo.c
 * @brief PWM 舵机驱动，手写 TIM 初始化，不依赖 CubeMX 重新生成。
 *        50Hz（20ms），脉宽换算由每路舵机的角度/脉宽配置决定。
 * @note  Z轴舵机: TIM2_CH1=PA5, 量程0°~360° (500~2500us)
 *        夹爪舵机: TIM3_CH1=PA6, 量程0°~270° (500~2500us)
 *        改接线/换舵机：只改本文件顶部配置区。
 */
#include "servo.h"
#include "stm32f4xx_hal.h"

/* ==================== 可移植配置区 ==================== */
/* Z轴旋转舵机: 量程 0°~360°, 对应脉宽 500~2500us */
#define SERVO_Z_TIM       TIM2
#define SERVO_Z_CH        TIM_CHANNEL_1
#define SERVO_Z_PORT      GPIOA
#define SERVO_Z_PIN       GPIO_PIN_5
#define SERVO_Z_AF        GPIO_AF1_TIM2
#define SERVO_Z_MIN_ANG   0.0f
#define SERVO_Z_MAX_ANG   360.0f
#define SERVO_Z_MIN_PULSE 500
#define SERVO_Z_MAX_PULSE 2500
#define SERVO_Z_START_ANG 5.0f    /* 上电初始角(°)：0°=量程起点 */

/* 夹爪舵机: 量程 0°~270°, 对应脉宽 500~2500us */
#define SERVO_G_TIM       TIM3
#define SERVO_G_CH        TIM_CHANNEL_1
#define SERVO_G_PORT      GPIOA
#define SERVO_G_PIN       GPIO_PIN_6
#define SERVO_G_AF        GPIO_AF2_TIM3
#define SERVO_G_MIN_ANG   0.0f
#define SERVO_G_MAX_ANG   270.0f
#define SERVO_G_MIN_PULSE 500
#define SERVO_G_MAX_PULSE 2500
#define SERVO_G_START_ANG 85.0f    /* 上电初始角(°)：0°=量程起点 */
/* ===================================================== */

typedef struct {
    TIM_TypeDef *tim;
    uint32_t    channel;
    GPIO_TypeDef *port;
    uint16_t    pin;
    uint8_t     af;
    float       minAngle;
    float       maxAngle;
    uint16_t    minPulse;
    uint16_t    maxPulse;
    float       startAngle;
} ServoCfg;

static const ServoCfg servoCfg[2] = {
    /* SERVO_Z    */ { SERVO_Z_TIM, SERVO_Z_CH, SERVO_Z_PORT, SERVO_Z_PIN, SERVO_Z_AF,
                       SERVO_Z_MIN_ANG, SERVO_Z_MAX_ANG, SERVO_Z_MIN_PULSE, SERVO_Z_MAX_PULSE,
                       SERVO_Z_START_ANG },
    /* SERVO_GRIP */ { SERVO_G_TIM, SERVO_G_CH, SERVO_G_PORT, SERVO_G_PIN, SERVO_G_AF,
                       SERVO_G_MIN_ANG, SERVO_G_MAX_ANG, SERVO_G_MIN_PULSE, SERVO_G_MAX_PULSE,
                       SERVO_G_START_ANG },
};

static TIM_HandleTypeDef hServo[2] = {0};

/**
 * @brief 初始化一路舵机：使能时钟 -> 引脚复用 -> PWM
 */
static void Servo_TimInit(uint8_t id)
{
    const ServoCfg *c = &servoCfg[id];

    if(c->tim == TIM2)       __HAL_RCC_TIM2_CLK_ENABLE();
    else if(c->tim == TIM3)  __HAL_RCC_TIM3_CLK_ENABLE();
    if(c->port == GPIOA)     __HAL_RCC_GPIOA_CLK_ENABLE();
    else if(c->port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin = c->pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = c->af;
    HAL_GPIO_Init(c->port, &g);

    /* 84MHz/84 = 1MHz, ARR=19999 -> 20ms -> 50Hz */
    TIM_HandleTypeDef *h = &hServo[id];
    h->Instance = c->tim;
    h->Init.Prescaler = 83;
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
}

void Servo_SetAngle(ServoId id, float angle)
{
    if(id > SERVO_GRIP) return;
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
    if(id > SERVO_GRIP) return 0.0f;
    const ServoCfg *c = &servoCfg[id];
    uint32_t curPulse = __HAL_TIM_GET_COMPARE(&hServo[id], c->channel);
    return c->minAngle
         + (float)(curPulse - c->minPulse) * (c->maxAngle - c->minAngle)
           / (float)(c->maxPulse - c->minPulse);
}

/**
 * @brief 平滑转动到目标角度：以小步长分步逼近，避免舵机全速猛转
 * @note  位置舵机转速由内部电机决定，软件只能靠分步+延时"模拟慢转"。
 *        步长越小/间隔越长 → 越慢。
 * @param id          舵机编号
 * @param angle       目标角(°)
 * @param stepDeg     每步转过的角度(°)（如 1.0f）
 * @param stepDelayMs 每步间隔(ms)（如 15）
 */
void Servo_SetAngleSmooth(ServoId id, float angle, float stepDeg, uint16_t stepDelayMs)
{
    if(id > SERVO_GRIP) return;
    if(stepDeg <= 0.0f) { Servo_SetAngle(id, angle); return; }
    const ServoCfg *c = &servoCfg[id];

    if(angle > c->maxAngle) angle = c->maxAngle;
    if(angle < c->minAngle) angle = c->minAngle;

    /* 由当前脉宽反推当前角度 */
    float cur = Servo_GetAngle(id);

    /* 分步逼近目标角 */
    float dir = (angle >= cur) ? stepDeg : -stepDeg;
    uint16_t guard = 0;
    while(dir > 0.0f ? (cur < angle) : (cur > angle))
    {
        cur += dir;
        if(dir > 0.0f && cur > angle) cur = angle;
        if(dir < 0.0f && cur < angle) cur = angle;
        Servo_SetAngle(id, cur);
        HAL_Delay(stepDelayMs);
        if(++guard > 500) break;   /* 防死循环 */
    }
}
