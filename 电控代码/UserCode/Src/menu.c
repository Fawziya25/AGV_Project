/**
 * @file menu.c
 * @brief 启动菜单：**单页**选雷达启停区（C6 切换 / A8 确认启动）
 * @note  在 AGV_Init() 末尾调用 Menu_Init()。
 *        按键按下=低电平(GPIO_PIN_RESET)，上拉输入。
 *        扫描周期 15ms，连续 3 次相同判定为稳定(≈45ms消抖)，按下沿触发一次。
 *
 *        【2026-11-01：两页合成一页，只选启停区】
 *        原来先一页"路线 Route_id"再一页"启停区"。但 Route_id 在全工程没有任何
 *        消费者（Depart/GoBack 早就改成按雷达规划的 Cruise(leg) 走），那一页按了
 *        等于没按，整页删掉；相机状态挪到同一页上显示。
 *
 *        选定启停区 = 一次调用两件事，两个都成功才重画：
 *          ① Radar_SetZone(zone)          雷达选区 + 该区雷达位姿 + 出发点方格
 *          ② AGV_Position_ApplyZone(zone) 小车自己的初始坐标与车头朝向（两区各一套）
 *        于是"屏幕显示哪个区"== "雷达用哪个区"== "车按哪个起点算坐标"，不会各说各话。
 *
 *        【按键分工】**A8 只管启动、C6 只管换启停区**、C8 是运行时的暂停键
 *        （暂停在 test.c 的 HAL_TIM_PeriodElapsedCallback 里扫，不在这儿）。
 */

#include "menu.h"
#include "camera.h"        /* extern volatile uint8_t cam_start */
#include "test.h"          /* Dbg_Suspend/Dbg_Resume：菜单期间别让调试服务刷屏 */
#include "chassis.h"       /* AGV_Position_ApplyZone + AGV_Position_X/Y */
#include "radar_task.h"    /* Radar_SetZone / Radar_GetZone */
#include "oled.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* 按键引脚：按下=低电平 */
#define KEY_START_GPIO  GPIOA
#define KEY_START_PIN   GPIO_PIN_8      /* A8 启动 —— 只此一个用途 */
#define KEY_ZONE_GPIO   GPIOC
#define KEY_ZONE_PIN    GPIO_PIN_6      /* C6 切换启停区 —— 只此一个用途 */
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
static Key_State_TypeDef key_zone  = { KEY_ZONE_GPIO,  KEY_ZONE_PIN,  0, 0, 0 };

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
 * @brief OLED 菜单显示：当前启停区 + 相机状态 + 本区的小车初始坐标
 * @note  第1行 标题 / 第2行 启停区与相机 / 第3行 小车坐标 / 第4行 按键提示。
 *        坐标直接读 AGV_Position_X/Y —— 它就是 AGV_Position_ApplyZone() 刚设进去的值，
 *        所以"屏上写的起点"和"车认为自己在哪"必然一致。
 */
static void Menu_Display(void)
{
    char buf[20];

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"RADAR zone", 16, 1);
    sprintf(buf, "zone: %u  cam %s", (unsigned)Radar_GetZone(), cam_start ? "OK" : "ERR");
    OLED_ShowString(0, 16, (uint8_t *)buf, 16, 1);
    sprintf(buf, "X%u Y%u", (unsigned)AGV_Position_X, (unsigned)AGV_Position_Y);
    OLED_ShowString(0, 32, (uint8_t *)buf, 16, 1);
    OLED_ShowString(0, 48, (uint8_t *)"C6:chg  A8:ok", 16, 1);
    OLED_Refresh();
}

/**
 * @brief 切到另一个启停区并立即生效（正式菜单与测试入口页共用）
 * @retval 切换后（或切换失败时保持的）区号 1/2
 * @note  一次装两样东西，都收在这里：
 *          ① Radar_SetZone()          雷达选区请求 + 该区雷达位姿 + 出发点方格
 *          ② AGV_Position_ApplyZone() 小车自己的初始坐标与车头朝向
 *        ① 失败就不动 ②，避免出现"雷达还在区A、车却按区B算坐标"。
 */
uint8_t Menu_SwitchZone(void)
{
    uint8_t zone = (Radar_GetZone() == 1U) ? 2U : 1U;

    if(Radar_SetZone(zone))
    {
        AGV_Position_ApplyZone(zone);
    }
    else
    {
        zone = Radar_GetZone();      /* 没切成，屏幕/返回值都还是原来的区 */
    }
    return zone;
}

/**
 * @brief 阻塞等待启动：C6 短按切换启停区，A8 短按确认并返回
 * @note  OLED 依赖 AGV_Init() 中 AGV_OLED_ShowReady() 已初始化
 *        进入时 Dbg_Suspend()：AGV_Init() 里 Dbg_Init() 早于本函数，TIM7 已经
 *        在跑了，而本函数的 HAL_Delay 会被 test.c 盖掉、内部调 Dbg_Service()
 *        —— 不挂起的话 200ms 一到就把菜单刷成相机数据画面，且不再自动恢复。
 *
 *        本函数返回后调用方还要跑 Radar_RunOnce()，真正的"出发"在 TaskCode=Depart。
 */
void Menu_Init(void)
{
    Dbg_Suspend();                       /* 菜单期间屏幕归菜单，调试服务别插手 */

    Menu_Display();                      /* 首次显示：当前停在哪个启停区 */
    while(1)
    {
        if(Key_Scan(&key_start))         /* A8 短按 → 确认启动 */
        {
            break;
        }
        if(Key_Scan(&key_zone))          /* C6 短按 → 启停区 1 ↔ 2 */
        {
            (void)Menu_SwitchZone();     /* 雷达 + 小车位姿一起换（失败则保持原区） */
            Menu_Display();
        }
        HAL_Delay(KEY_SCAN_PERIOD_MS);
    }

    OLED_Clear();                        /* 清空OLED屏幕 */
    Dbg_Resume();                        /* 交还屏幕，并补一次重画 */
}
