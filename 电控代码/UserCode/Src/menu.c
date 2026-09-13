/**
 * @file menu.c
 * @brief 启动菜单模块：阻塞等待 A8 启动，期间 C6/C8 切换路线
 * @note  在 AGV_Init() 末尾调用 menu_Init()。
 *        按键按下=低电平(GPIO_PIN_RESET)，上拉输入。
 *        扫描周期固定 20ms，连续 2 次相同判定为稳定(40ms消抖)，按下沿触发一次。
 */
#include "menu.h"
#include "task.h"          /* extern uint8_t Route_id */
#include "openmv.h"        /* extern volatile uint8_t openmv_start */
#include "oled.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* 按键引脚：按下=低电平 */
#define KEY_START_GPIO  GPIOA
#define KEY_START_PIN   GPIO_PIN_8      /* A8 启动 */
#define KEY_ROUTE_GPIO  GPIOC
#define KEY_ROUTE_PIN   GPIO_PIN_6      /* C6 切换路线 */
#define KEY_RES_GPIO    GPIOC
#define KEY_RES_PIN     GPIO_PIN_8      /* C8 预留 */

#define KEY_SCAN_PERIOD_MS  15          /* 扫描周期 */
#define KEY_DEBOUNCE_CNT    3           /* 连续相同次数=40ms消抖 */

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
static Key_State_TypeDef key_res   = { KEY_RES_GPIO,   KEY_RES_PIN,   0, 0, 0 };

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
 * @note  Route_id 或 openmv_start 变化时调用；菜单首次进入也调用一次
 *        第1行 Route: 0/1，第2行 openmv OK/error（openmv_start==1）
 */
static void Menu_Display(void)
{
    char buf[16];
    OLED_Clear();
    sprintf(buf, "Route: %d", Route_id);
    OLED_ShowString(8, 16, (uint8_t *)buf, 16, 1);
    OLED_ShowString(8, 32, (uint8_t *)(openmv_start ? "openmv OK" : "openmv error"), 16, 1);
    OLED_Refresh();
}

/**
 * @brief 阻塞等待启动：A8短按返回；C6/C8短按切换 Route_id
 * @note  OLED 依赖 AGV_Init() 中 AGV_OLED_ShowReady() 已初始化
 */
void Menu_Init(void)
{
    Menu_Display();                      /* 首次显示菜单 */
    while(1)
    {
        if(Key_Scan(&key_start))         /* A8 短按 → 启动 */
        {
            OLED_Clear();                /* 清空OLED屏幕 */
            return;                      /* 退出菜单，进入主程序 */
        }
        if(Key_Scan(&key_route) || Key_Scan(&key_res))  /* C6/C8 短按 → 路线取反 */
        {
            Route_id ^= 1;
            Menu_Display();
        }
        HAL_Delay(KEY_SCAN_PERIOD_MS);
    }
}
