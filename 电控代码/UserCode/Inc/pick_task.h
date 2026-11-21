/**
 * @file pick_task.h
 * @brief 物料取放任务阶段：视觉闭环修正 + 各阶段取放动作
 * @note  颜色数组定义在 QRcode.c；RerunFlag 定义在 task.c
 */

#ifndef __PICK_TASK_H
#define __PICK_TASK_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "pick.h"
#include "camera.h"

/* ========================== 辅助宏定义 ========================== */

/* 视觉交互统一由 camera.h 的 Cam_SwitchTo(mode) 承担 —— 一条命令一个模式号，
   没有出参，结果读全局量 dx/dy、box_color[]、cam_flag。不再用宏。
   mode 直接写命令码的八进制字面量：01 任务码 / 02 最近物块 / 03 三框颜色 /
   04 圆环 / 05 指定颜色。 */

/* 下面这些宏一律用 Pick_JumpTo（两轴绝对位置）。前提：上电前高度/半径
   两轴人工放在 0 位，Pick_Axis_Init() 会把该位置写成驱动器绝对零点。
   pick.c 里已没有相对位置版本，所有动作都依赖这个零点。 */

/** 移动到普通圆环位置 */
#define MOVE_TO_RING(ring) \
    do { \
        if ((ring) == 1) Pick_JumpTo POS_1; \
        else if ((ring) == 2) Pick_JumpTo POS_2; \
        else if ((ring) == 3) Pick_JumpTo POS_3; \
    } while(0)

/** 移动到高位圆环位置 */
#define MOVE_TO_RING_HIGH(ring) \
    do { \
        if ((ring) == 1) Pick_JumpTo POS_1_HIGE; \
        else if ((ring) == 2) Pick_JumpTo POS_2_HIGE; \
        else if ((ring) == 3) Pick_JumpTo POS_3_HIGE; \
    } while(0)

/* 槽位约定：车上是「同一个臂位姿 POS_CAR + 转盘转角」，不是三个臂位姿。
   slot 0/1/2 = A/B/C 槽 = 转盘 30°/150°/270°，由 Pick_TurnTable_Set(slot) 选。
   Set 是**非阻塞**的：发完转动命令立刻返回，转盘在后台转，流程不等它。
   下面的宏也都不等待；靠随后的 Pick_JumpTo POS_CAR（Π 形抬升，耗时数秒）
   覆盖 120° 约 1s 的换槽行程。真要强制同步时再单独调 Pick_TurnTable_Wait()。 */

/** 从车上 slot 槽抓取并放置到圆环 */
#define GRAB_AND_PLACE_TO_RING_SLOT(slot, ring) \
    do { \
        Pick_TurnTable_Set(slot); \
        Pick_JumpTo POS_CAR_DOWN; \
        Pick_Gripper_Grab(); \
        MOVE_TO_RING(ring); \
        Pick_Gripper_Release(); \
    } while(0)

/** 从车上 slot 槽抓取并放置到圆环（支持高位） */
#define GRAB_AND_PLACE_TO_RING_SLOT_EX(slot, ring, is_high) \
    do { \
        Pick_TurnTable_Set(slot); \
        Pick_JumpTo POS_CAR_DOWN; \
        Pick_Gripper_Grab(); \
        if (is_high) { MOVE_TO_RING_HIGH(ring); } \
        else { MOVE_TO_RING(ring); } \
        Pick_Gripper_Release(); \
    } while(0)

/** 从圆环抓取并放置到车上 slot 槽
    先保持 release 状态摆到高位(POS_x_HIGE)，到高位后再张开(Observe)，
    然后下降到低位(POS_x)夹取 —— 下降路径上有料块，张开才不打料。 */
#define GRAB_FROM_RING_AND_PLACE_SLOT(ring, slot) \
    do { \
        MOVE_TO_RING_HIGH(ring);    /* 先保持 release 到高位 */ \
        Pick_Gripper_Observe();     /* 到高位后再张开，准备下降 */ \
        MOVE_TO_RING(ring);         /* 垂直下降到抓取位 */ \
        Pick_Gripper_Grab(); \
        Pick_TurnTable_Set(slot); \
        Pick_JumpTo POS_CAR_ON; \
        Pick_Gripper_Release(); \
    } while(0)

/* ========================== 函数声明 ========================== */

/* 槽位约定(slot)：0=A 槽, 1=B 槽, 2=C 槽，与 Place_Color[]/Pick_Color[] 下标一致
   圆环约定(ring)：1/2/3                                             */

/* 暂存区三个位置的颜色识别 + 中间位对正（任务1，2026-11-21）：
     摆 POS_2（低位） → 依次对任务码前三位颜色发 0x05（每个最多 2s）
       → |dx| 最小的那一色记为位置2 → 再请求这一色 + AGV_Correct(5s)
       → 摆 POS_1（低位）依次问三色，先识别到的记为位置1
       → 摆 POS_3（低位）同理得到位置3。
   三个位置一律用低位 POS_1/2/3，两轮都一样。
   结果写 Pick_ColorAtPos[0/1/2]（0 = 没识别到）。 */
void Pick_Correct(void);

/* 暂存区三个位置的颜色，下标 0/1/2 = 位置 1/2/3；0 = 没识别到。
   Pick_Correct() 写，Pick_TEMP() 读。 */
extern uint8_t Pick_ColorAtPos[3];

/* 原料区取料（当前路线）：摆 POS_RAW_B、开爪，反复发 0x02 直到相机报回来的颜色号
   等于任务码当前要抓的那一色（flag 即颜色号 1~6），才下手抓到车上对应槽。
   严格按任务码顺序 i=0/1/2，不匹配就一直等（无超时）。 */
void Pick_RAW(void);

/* 原料区三个抓取位各自的完整取料动作：抓一块 → 放到车上第 slot 槽。
   A/C 位比 B 位远，函数内部会先沿车头方向挪底盘 3cm 再抓、放完再挪回来；
   B 位底盘不动。拆成三个是因为三处以后可能各自加不同动作（现在 C 是 A 的镜像）。 */
void Pick_RAW_A(uint8_t slot);
void Pick_RAW_B(uint8_t slot);
void Pick_RAW_C(uint8_t slot);

/* 原料转盘的一个周期，拆成静止和运动两段（毫秒）：
     t1 静止 6000ms —— 可以抓；t2 运动 1200ms —— 不能抓。
   Fast 路线在"到达悬挂高度"时按这个相位判断（见 pick_task.c 的
   Raw_PickAtFollow），普通路线仍按两段之和作为两次抓取的起始间隔。 */
#define PICK_RAW_PLUS_STATIC_MS   4000U     /* t1 静止窗口 */
#define PICK_RAW_PLUS_MOVING_MS   3000U     /* t2 运动窗口 */

/* 一个完整周期 = t1 + t2 = 7200ms。
   六种抓取顺序中，相邻两次 Raw_PickAt 的目标起始间隔（毫秒）。 */
#define PICK_RAW_PLUS_GRAB_INTERVAL_MS (PICK_RAW_PLUS_STATIC_MS + PICK_RAW_PLUS_MOVING_MS)

/* Fast 路线第 2 块前的等待(ms)：它紧接第 1 块、没有间隔可等，但从 POS_CAR 到
   原料框是大幅 Z 轴旋转，臂会晃 —— 先摆到目标上方 30mm 的等待位停这么久，
   等晃动衰减后再下探抓取（见 pick_task.c 的 Raw_PickAtFollow）。 */
#define PICK_RAW_APPROACH_SETTLE_MS 200U

/* 原料区取料（普通路线）：摆 POS_OBSERVE_RAW_CENTER 读一次三框颜色(0x03)，
   记下任务码第 0/1/2 色开局各落在哪个框，再按这个排列查表分派给下面六个普通函数。
   六种路线均使用 PICK_RAW_PLUS_GRAB_INTERVAL_MS；修改上面的宏即可统一调整。 */
void Pick_RAW_PLUS(void);

/* 原料区取料（快速路线）：观测流程与 Pick_RAW_PLUS 完全相同（共用同一份 0x03
   观测），分派给下面六个 Fast 函数。原来的 PICK_RAW_PLUS_USE_FAST 编译开关已删，
   要切路线就在上层把 Pick_RAW_PLUS() 换成 Pick_RAW_PLUS_FAST()。 */
void Pick_RAW_PLUS_FAST(void);

/* 六种固定抓取路线。⚠ **名称 = 开局观测到的初始排列**（任务码第 0/1/2 色分别
   落在 A/B/C 哪一格），不是三次抓取实际去的框；三个槽固定按 0→1→2 依次放。
   抓取过程中**底盘完全不动**，每条只调三次臂 + 转盘动作。
   实际抓哪个框由转盘每抓一块挪一格（索引 −1）推出来：
     观测 ABC → 抓 A A A      观测 ACB → 抓 A B C      观测 BAC → 抓 B C A
     观测 BCA → 抓 B B B      观测 CAB → 抓 C C C      观测 CBA → 抓 C A B
   相邻两次 Raw_PickAt 的开始时间按宏值控制；**等待期间臂会先摆到下一块上方
   30mm 的等待位（pick.h 的 POS_RAW_x_APPROACH）再补足剩余时间**，到点后从
   等待位垂直下降到抓取位。若前次动作耗时已超过该值，则不摆等待位，下一次
   在前次完成后立即开始。一般由 Pick_RAW_PLUS 分派，不用直接调。 */
void Pick_RAW_ABC(void);
void Pick_RAW_ACB(void);
void Pick_RAW_BAC(void);
void Pick_RAW_BCA(void);
void Pick_RAW_CAB(void);
void Pick_RAW_CBA(void);

/* 六种快速抓取路线（Fast 版）。**名称同样 = 开局观测到的初始排列**，与上面六个
   一一对应。转盘每过一个周期按索引 −1 挪一格（A→C、C→B、B→A）。
   origin_tick = Raw_ObserveThreeBoxes() 返回的"颜色读到的时刻"，三块都按它计时：
     第 1 块：抓该颜色初始所在框，**不看时间**；
     第 2 块：紧接第 1 块出发，目标该颜色初始所在框；
             **到达悬挂高度(上方 30mm)时判断**：
               elapsed < t1(PICK_RAW_PLUS_STATIC_MS=6000ms)
                   → 还在静止窗口，位置没变，直接抓；
               t1 < elapsed < t1+t2(PICK_RAW_PLUS_MOVING_MS=1200ms)
                   → 转盘正在转，**先摆到下一格上方悬挂，等到 t1+t2 再抓**；
               elapsed > t1+t2
                   → 已经转过格，**直接摆到下一格上方悬挂抓取**；
     第 3 块：同样按相位比较，但**基准是 origin_tick + 一个整周期**
             (PICK_RAW_PLUS_GRAB_INTERVAL_MS)，在"静止时间对应的位置"抓
             （已过几个周期就往索引 −1 方向走几格，与第 2 块用同一套判断）。
   每块判断完都保证在悬挂位停够 PICK_RAW_APPROACH_SETTLE_MS 等晃动衰减。
   最快情况（三块都落在第一个静止窗口）的抓取顺序：
     观测 ABC → 抓 A B B      观测 ACB → 抓 A C A      观测 BAC → 抓 B A B
     观测 BCA → 抓 B C C      观测 CAB → 抓 C A A      观测 CBA → 抓 C B C
   由 Pick_RAW_PLUS_FAST 分派。 */
void Pick_RAW_ABC_Fast(uint32_t origin_tick);
void Pick_RAW_ACB_Fast(uint32_t origin_tick);
void Pick_RAW_BAC_Fast(uint32_t origin_tick);
void Pick_RAW_BCA_Fast(uint32_t origin_tick);
void Pick_RAW_CAB_Fast(uint32_t origin_tick);
void Pick_RAW_CBA_Fast(uint32_t origin_tick);

/* 粗加工区放置前的视觉对正，拆成三步，固定顺序 High → Angle → Low → Place_ROUGH()：

   Place_Correct_High()  摆高位观察位 POS_OBSERVE_RING_HIGH → 开爪 → 0x04 圆环2
                         粗修正(1s)。**不收爪**，留给后面的角度步继续用同一个位姿。
   Place_Correct_Angle() 沿车头方向挪 7cm（三个圆环在视野里排得更开，角度更准）
                         → 0x06 角度闭环（反复读数 + SetYaw，进死区为止）
                         → 记基准航向 a1 → 退回原位 → 航向复位 a1。
   Place_Correct_Low()   摆低位观察位 POS_OBSERVE_RING_LOW → 开爪 → 0x04 细修正(30s)
                         → 收爪。

   拿不到视觉结果就跳过对应那一段（免得用旧 dx/dy/cam_angle 把车带偏）。
   Place_ROUGH 自己不做这一步。Place_Correct_Low 也可以单独调（第二轮暂存区那条路）。
   角度符号(`PLACE_CORRECT_ANGLE_SGN`)与相机约定有关，实车越修越歪就取反。 */
void Place_Correct_High(void);
void Place_Correct_Angle(void);
void Place_Correct_Low(void);

/* 粗加工区放置（不含视觉修正，那一步在 Place_Correct_* 里）：把车上三个槽
   依次放到二维码给的圆环上。 */
void Place_ROUGH(void);
void Pick_ROUGH(void);

/* 暂存区取料（任务2，2026-11-21）：按任务码前三位的颜色顺序，去 Pick_ColorAtPos[]
   给出的位置(1/2/3)抓，依次放进车上 0/1/2 槽。位置用**低位** POS_x（与识别一致）。
   某色没识别到/位置重复时，用剩下的空位置按 1→2→3 补；中间那块的位置优先保住。 */
void Pick_TEMP(void);

/* 任务1+2 融合版（2026-11-21）：位置2 识别+PID 后，若是下一个要抓的立即抓走；
   再到位置1/3，识别到的正好是下一个要抓的也就地抓走；最后补抓剩下的。
   比"先 Pick_Correct 全部识别、再 Pick_TEMP 从头抓"少两趟往返。 */
void Pick_Correct_TEMP(void);

/* 车上三槽依次放回原料区（任务3，2026-11-21）：
   slot 0 → POS_RAW_B，slot 1 → POS_RAW_C，slot 2 → POS_RAW_C（按字面，两块都去 C）。 */
void Place_RAW(void);

/* 把车上三槽的物块依次抓到 POS_THROW 放掉（pick.h 的扔料位）。 */
void Pick_THROW(void);

/* 直接把地上（本轮 Place_Color[] 指定的环）的物块抓起来丢到 POS_THROW，
   不经过车上槽。抓取动作同 Pick_ROUGH：高位 → 张开 → 下降 → 夹取。 */
void Pick_ROUGH_THROW(void);

void Place_TEMP(void);
/* 暂存区放置前的视觉对正【第二轮】：摆 POS_OBSERVE_RING_MIDDLE → 开爪 → 0x05 指定颜色 → AGV_Correct(5000) → 收爪。
   第一轮别调它 —— 第一轮的观察位和粗加工区同一处，复用 Place_Correct_Low()。
   task.c 的 case Place2 按 RerunFlag 二选一。 */
void Place_TEMP_Correct(void);

#endif /* __PICK_TASK_H */
