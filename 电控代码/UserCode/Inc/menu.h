#ifndef __MENU_H
#define __MENU_H

/* 启动菜单：阻塞等待 A8 短按进入主流程；期间 C6/C8 短按切换 Route_id。
   在 AGV_Init() 末尾调用。内部按 20ms 周期扫描+消抖，并实时显示到 OLED。 */
void Menu_Init(void);

#endif
