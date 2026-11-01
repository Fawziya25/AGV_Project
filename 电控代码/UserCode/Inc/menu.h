#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>

/* 启动菜单：**单页**选雷达启停区 —— C6 切换区 1/2，A8 确认启动。
   在 AGV_Init() 末尾调用，阻塞到按 A8 为止。
   选定区时会同时装好：雷达选区/位姿/出发点方格（Radar_SetZone）+
   小车自己的初始坐标与车头朝向（AGV_Position_ApplyZone）。
   内部 15ms 周期扫描 + 45ms 消抖，实时显示到 OLED。 */
void Menu_Init(void);

/* 切到另一个启停区并立即生效（雷达选区 + 小车初始坐标/朝向），返回最终区号 1/2。
   正式流程的启动菜单和测试流程的「按 A8 启动」页都调它 —— 两边选的区走同一套逻辑。
   Radar_SetZone() 失败时不改小车坐标（避免“雷达在区A、车按区B算”）。 */
uint8_t Menu_SwitchZone(void);

#endif
