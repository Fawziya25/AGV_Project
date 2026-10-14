/**
 * @file menu.c
 * @brief 启动菜单模块：阻塞等待 A8 启动，期间 C6 切换路线
 * @note  在 AGV_Init() 末尾调用 menu_Init()。
 *        按键按下=低电平(GPIO_PIN_RESET)，上拉输入。
 *        扫描周期 15ms，连续 3 次相同判定为稳定(≈45ms消抖)，按下沿触发一次。
 *
 *        【2026-10-14 按键分工重排】**A8 只管启动、C6 只管换路线、C8 是运行时的
 *        暂停键**（暂停在 test.c 的 HAL_TIM_PeriodElapsedCallback 里扫，不在这儿）。
 *        这次把 C8 从菜单里摘掉了：以前 C8 和 C6 一样都能换路线，而 C8 现在又
 *        兼着暂停 —— 菜单里按一下 C8，路线翻了不说，还会把调试服务的 s_paused
 *        一起翻掉，等 Menu_Init 返回、Dbg_Resume() 之后车会**一启动就停在暂停**。
 *        分开之后三个键各管各的，上电启动时按 A8 不会再跟暂停打架。 */

#include "menu.h"
#include "task.h"          /* extern uint8_t Route_id */
#include "camera.h"        /* extern volatile uint8_t cam_start */
#include "test.h"          /* Dbg_Suspend/Dbg_Resume：菜单期间别让调试服务刷屏 */
#include "oled.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* 按键引脚：按下=低电平 */
#define KEY_START_GPIO  GPIOA
#define KEY_START_PIN   GPIO_PIN_8      /* A8 启动 —— 只此一个用途 */
#define KEY_ROUTE_GPIO  GPIOC
#define KEY_ROUTE_PIN   GPIO_PIN_6      /* C6 切换路线 —— 只此一个用途 */
/* C8 原来是这里的"预留"键、和 C6 一样能换路线，2026-10-14 改成运行时的暂停键，
   从菜单里摘掉了（见文件头）。要再加回菜单记得先解决 s_paused 被带翻的问题。 */

#define KEY_SCAN_PERIOD_MS  15          /* 扫描周期(ms) */
#define KEY_DEBOUNCE_CNT    3           /* 连续相同次数：3 × 15ms ≈ 45ms 消抖 */

/* 单键状态机 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t stable;     /* 消抖后状态: 1=按下 */
    uint8_t prevRaw;    /* 上次原始电平 */
    uint8_t cnt;        /* 连续相同计数 */
} Key_State_TypeDef;

static Key_State_TypeDef key_start = { KEY_START_GPIO, KEY_START_PIN, 0, 0, 0 };
static Key_State_TypeDef key_route = { KEY_ROUTE_GPIO, KEY_ROUTE_PIN, 0, 0, 0 };

/**
 * @brief 单键扫描：消抖+按下沿检测
 * @retval 1=检测到一次短按(刚按下)；0=无
 */
static uint8_t Key_Scan(Key_State_TypeDef *k)
{
    uint8_t raw = (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t press = 0;

    if(raw == k->prevRaw)
    {
        if(k->cnt < KEY_DEBOUNCE_CNT) k->cnt++;
        if(k->cnt >= KEY_DEBOUNCE_CNT && k->stable != raw)
        {
            k->stable = raw;
            if(raw == 1) press = 1;     /* 稳定为按下 → 短按一次 */
        }
    }
    else
    {
        k->prevRaw = raw;
        k->cnt = 0;
    }
    return press;
}

/**
 * @brief OLED 菜单显示：当前路线 + OpenMV 状态
 * @note  Route_id 或 cam_start 变化时调用；菜单首次进入也调用一次
 *        第1行 Route: 0/1，第2行 camera OK/error（cam_start==1）
 */
static void Menu_Display(void)
{
    char buf[16];
    OLED_Clear();
    sprintf(buf, "Route: %d", Route_id);
    OLED_ShowString(8, 16, (uint8_t *)buf, 16, 1);
    OLED_ShowString(8, 32, (uint8_t *)(cam_start ? "camera OK" : "camera error"), 16, 1);
    OLED_Refresh();
}

/**
 * @brief 阻塞等待启动：A8 短按返回；C6 短按切换 Route_id
 * @note  OLED 依赖 AGV_Init() 中 AGV_OLED_ShowReady() 已初始化
 *        进入时 Dbg_Suspend()：AGV_Init() 里 Dbg_Init() 早于本函数，TIM7 已经
 *        在跑了，而本函数的 HAL_Delay 会被 test.c 盖掉、内部调 Dbg_Service()
 *        —— 不挂起的话 200ms 一到就把菜单刷成相机数据画面，且不再自动恢复。
 */
void Menu_Init(void)
{
    Dbg_Suspend();                       /* 菜单期间屏幕归菜单，调试服务别插手 */

    Menu_Display();                      /* 首次显示菜单 */
    while(1)
    {
        if(Key_Scan(&key_start))         /* A8 短按 → 启动 */
        {
            OLED_Clear();                /* 清空OLED屏幕 */
            Dbg_Resume();                /* 交还屏幕，并补一次重画 */
            return;                      /* 退出菜单，进入主程序 */
        }
        if(Key_Scan(&key_route))         /* C6 短按 → 路线取反 */
        {
            Route_id ^= 1;
            Menu_Display();
        }
        HAL_Delay(KEY_SCAN_PERIOD_MS);
    }
}
