#ifndef __TEST_H
#define __TEST_H

#include <stdint.h>              /* Dbg_ShowScreen 的 uint8_t 参数 */

#ifndef TEST_MODE
#define TEST_MODE   1 //具体模式在Test_Switch里面调节
#endif

#ifndef DBG_ENABLE
#define DBG_ENABLE  1
#endif


void Test_Switch(void);

#if DBG_ENABLE
/* 启动 TIM7 中断（MX_TIM7_Init 已经把 10ms 配置好了，这里只负责 Start_IT）。
   必须在 OLED_Init() 之后调 —— 中断里要驱动软件 I2C 刷屏。 */
void Dbg_Init(void);
/* 调试服务：暂停判断 + 到点刷屏。在中断上下文里调用会直接返回 */
void Dbg_Service(void);
/* 挂起 / 恢复刷屏：屏幕被别的模块占用期间（目前只有 Menu_Init 的启动菜单）
   调 Suspend，用完调 Resume。挂起期间 Dbg_Service 整个让路 —— 不刷屏也不
   响应 C8 暂停（菜单里 A8 才是启动键）。Resume 会立刻补一次重画。 */
void Dbg_Suspend(void);
void Dbg_Resume(void);
#else
/* 关掉时调用点原地变成空语句，别的模块（menu.c 等）不用改 */
#define Dbg_Init()     ((void)0)
#define Dbg_Service()  ((void)0)
#define Dbg_Suspend()  ((void)0)
#define Dbg_Resume()   ((void)0)
#endif

/* 整屏显示：相机 mode/flag/dx/dy + 任务码 12 位 + C8 提示。
   **不随 DBG_ENABLE 走** —— Cam_Init() 里上电诊断也要用它（原 Cam_OLED_Show()
   已删除，由它取代）。paused=1 显示暂停画面。 */
void Dbg_ShowScreen(uint8_t paused);

#endif /* __TEST_H */
