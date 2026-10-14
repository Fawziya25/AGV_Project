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

/* 启动菜单第二阶段：选雷达启停区（2026-10-14 新增，配合 Radar 模块）
   1 = A8 启动后再显示一页"启停区"，C6 改、A8 确认；0 = 完全恢复旧行为（A8 直接启动）。
   启停区就是雷达的工作区域 zone 1/2：两区的位姿独立保存，选定后 Radar_SetZone() 会
   同时把该区的雷达位姿与出发点方格一起装好（见 radar_config.h）。 */
#ifndef RADAR_MENU_PICK_ZONE
#define RADAR_MENU_PICK_ZONE  1
#endif

#if RADAR_MENU_PICK_ZONE
#include "radar_task.h"
#endif

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

#if RADAR_MENU_PICK_ZONE
/**
 * @brief OLED 雷达启停区选择页（阶段2）
 * @note  第1行标题 / 第2行当前启停区 / 第3行操作提示；C6 改、A8 确认
 */
static void Menu_DisplayZone(void)
{
    char buf[16];
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"RADAR zone", 16, 1);
    sprintf(buf, "zone: %u", (unsigned)Radar_GetZone());
    OLED_ShowString(0, 24, (uint8_t *)buf, 16, 1);
    OLED_ShowString(0, 48, (uint8_t *)"C6:chg  A8:ok", 16, 1);
    OLED_Refresh();
}
#endif

/**
 * @brief 阻塞等待启动：A8 短按进入下一阶段；C6 短按切换 Route_id
 * @note  OLED 依赖 AGV_Init() 中 AGV_OLED_ShowReady() 已初始化
 *        进入时 Dbg_Suspend()：AGV_Init() 里 Dbg_Init() 早于本函数，TIM7 已经
 *        在跑了，而本函数的 HAL_Delay 会被 test.c 盖掉、内部调 Dbg_Service()
 *        —— 不挂起的话 200ms 一到就把菜单刷成相机数据画面，且不再自动恢复。
 *
 *        RADAR_MENU_PICK_ZONE=1 时本函数分两阶段（都返回后 Radar_RunOnce() 才开跑）：
 *          阶段1 C6 选整车路线 / A8 进入阶段2；
 *          阶段2 C6 选雷达启停区(1↔2) / A8 确认启动。
 *        两阶段都靠 Key_Scan 消抖，A8 在阶段1 只负责"进入下一阶段"，
 *        与"启动"的语义差别只多按一次，不会误触发（每次都是独立的按下沿）。
 */
void Menu_Init(void)
{
    Dbg_Suspend();                       /* 菜单期间屏幕归菜单，调试服务别插手 */

    Menu_Display();                      /* 首次显示菜单 */
    while(1)
    {
        if(Key_Scan(&key_start))         /* A8 短按 → 进入下一阶段 */
        {
            break;
        }
        if(Key_Scan(&key_route))         /* C6 短按 → 路线取反 */
        {
            Route_id ^= 1;
            Menu_Display();
        }
        HAL_Delay(KEY_SCAN_PERIOD_MS);
    }

#if RADAR_MENU_PICK_ZONE
    /* 阶段2：选雷达启停区。选定后 Radar_SetZone() 会把该区的雷达位姿与出发点方格一起装好。 */
    Menu_DisplayZone();
    while(1)
    {
        if(Key_Scan(&key_start))         /* A8 短按 → 确认并启动 */
        {
            break;
        }
        if(Key_Scan(&key_route))         /* C6 短按 → 启停区 1↔2 */
        {
            uint8_t zone = (Radar_GetZone() == 1U) ? 2U : 1U;
            (void)Radar_SetZone(zone);   /* 规划开始后会被拒；此时必然成功 */
            Menu_DisplayZone();
        }
        HAL_Delay(KEY_SCAN_PERIOD_MS);
    }
#endif

    OLED_Clear();                        /* 清空OLED屏幕 */
    Dbg_Resume();                        /* 交还屏幕，并补一次重画 */
}
