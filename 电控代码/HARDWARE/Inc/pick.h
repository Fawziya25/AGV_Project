#ifndef __PICK_H
#define __PICK_H
/* 机械臂模块（原 UserCode/Inc/pick_ctrl.h，2026-10-04 改名成 pick 并移到 HARDWARE/）*/
/* 预设位置宏(括号宏)：task.c 里用不带括号方式调用 Pick_JumpTo POS_XXX;
   例如 Pick_JumpTo POS_CAR;  展开为 Pick_JumpTo(30.0f, 45.0f, 5.0f)
   (轮到 Pick_MoveTo 时把函数名换掉即可，两版签名一致)
   —— 括号里的数随下面宏走，别在这儿写死；改值只改下面。 */
/* ⚠ 第一列(角度z)的单位是 SERVO_Z 的角度，量程 2026-10-14 从 360° 改成 270°。
   换舵机后命令角到脉宽的换算变了（5.56 → 7.41us/°），**下面这批角度值全部要重对**：
   数值照旧是"命令角"，但同一个数现在物理转得更多。改之前看着差不多的姿态，
   改完都会明显偏过去。对位就是把值往小了调（大致按 270/360 = 0.75 的比例算起点）。 */
//======================角度z    高度z   半径r=====================
#define POS_DEFAULT  (0.0f,   0.0f,    0.0f)//断电和上电前的复位
#define POS_START    (90.0f,  10.0f,   0.0f)//车运行前的位置
#define POS_RUN      (90.0f,  10.0f,   10.0f)//车移动时候位置
/* ↑ 这行 2026-10-14 被误删过（改上面几个宏时带掉的），全工程 8 处
   `Pick_JumpTo POS_RUN;` 一起报 "expected ';' before 'POS_RUN'"。
   值是按原样补回来的(90,10,10)，**没跟着这次调姿重新对过** —— 如果 POS_START
   那些数也要动，这条一起看。 */
#define POS_OBSERVE_RAW_PLUS   (180.0f, 3.0f,  2.0f)//观察原料上的物料块位置

#define POS_OBSERVE_TEMP       (180.0f, 80.0f,  18.0f)//观察暂存区上物料块位置
#define POS_OBSERVE_RAW   (180.0f, 8.0f,  2.0f )//原料区抓取位置下

#define POS_RAW_A       (210.0f, 50.0f,  79.0f )//原料区抓取位置左前
#define POS_RAW_B       (180.0f, 50.0f,  2.0f )//原料区抓取位置下
#define POS_RAW_C       (150.0f, 50.0f,  79.0f )//原料区抓取位置右前

#define POS_CAR        (55.0f,   22.0f,  18.0f)//车上放置位置(Pick_RAW 三次抓取都放这一点)

/* 车上三个槽位不是一个臂位姿三套坐标，而是「同一个 POS_CAR + 转盘舵机转角」：
   30°=A 槽, 150°=B 槽, 270°=C 槽(上电初值 0°=A)。用 Pick_TurnTable_Set(slot) 选。
   【2026-10-14 由 0/120/240 改成 30/150/270 —— 转盘舵机是 270° 量程，想让三个槽
   均分整圈。⚠ C 槽的 270.0f 正好压在量程上限，Servo_SetAngle/Smooth 的钳位是
   `> maxAngle` 才钳，所以 270 本身合法、能到；但没有余量了，以后要是把
   SERVO_T_MAX_ANG 调小一点，C 槽会被静默削掉、跑到别处去。】 */
#define PICK_TURN_ANG_A   30.0f
#define PICK_TURN_ANG_B   150.0f
#define PICK_TURN_ANG_C   270.0f

#define POS_OBSERVE_RING (180.0f, 140.0f,  18.0f)//观察圆环位置
#define POS_1        (218.0f, 141.0f,  83.0f)//圆环左边
#define POS_2        (180.0f, 141.0f,  18.0f)//圆环中间
#define POS_3        (138.0f, 141.0f,  83.0f)//圆环右边
#define POS_1_HIGE   (220.0f, 81.0f,  80.0f)//圆环物块左边
#define POS_2_HIGE   (180.0f, 81.0f,  18.0f)//圆环物块中间
#define POS_3_HIGE   (140.0f, 81.0f,  80.0f)//圆环物块右边

/* 夹爪舵机角度参数(°)：调整角度只需改这里，pick.c 夹爪函数引用
   ⚠ 2026-10-14 夹爪舵机**换成了 180° 量程**的（原 270°）。脉宽还是 500~2500us，
     所以每度脉宽从 7.41 → 11.11us/°，**同一个命令角现在物理转 1.5 倍远**：
     下面这三个数原来的值(15/129/75)换舵机后指向的位置全偏了，正在用
     `Pick_Test_Gripper()`(模式 9) / `Test_Pick_Scale()`(模式 3) 重新对。
     旧的 15/129/75 等比缩(× 180/270 = 0.667)的起点是 10 / 86 / 50，可以当
     另一组参照。
     三个数都远小于 180，不会撞上 Servo_SetAngle 里 `> maxAngle` 那句钳位，
     不存在被静默削角的问题。详见 servo.c 夹爪配置区那段。 */
#define PICK_OBSERVE    20.0f   /* 张开观察 */
#define PICK_GRAB      100.0f   /* 抓住物料 */
#define PICK_RELEASE   65.0f   /* 放开物料 */

#include <stdint.h>
#include "positionPIDcontroller.h"
#include "camera.h"
#include "QRcode.h"
#include "chassis.h"
#include "usart.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>



/**********************************************************
 * 物料抓取模块（pick）
 * 当前版本：控制 3 舵机(Z轴旋转 / 夹爪 / 车上转盘) + 2 步进电机完成取放动作。
 * 依赖：servo（PWM 舵机）、Emm_V5（CAN 步进电机，与四轮同协议）
 **********************************************************/

/* 模块初始化（在 AGV_Init 中调用一次） */
void Pick_Init(void);

/* 转盘选槽：把车上转盘转到第 slot 个槽位，阻塞到停稳。
   slot 0=A(30°) / 1=B(150°) / 2=C(270°)，越界按 A 处理。
   内部走**平滑**(Servo_SetAngleSmooth) + SERVO_T_SETTLE_MS(300ms) 物理停稳裕量,
   参数全在 servo.h 的 SERVO_T_* 那组里。**转盘带料没停稳臂就下去会撞料**,
   所以那句 Delay 不能删。
   调用时序：**先本函数选好槽，再把臂摆到 POS_CAR** —— 转盘转动时臂不在车体上方。 */
void Pick_TurnTable_Set(uint8_t slot);

/* 基本动作（两版，按"路上有没有障碍"选）：

   Pick_JumpTo —— 门字形(Π形)：先升到 PICK_SAFE_HEIGHT_MM 再转向+走半径，最后降高度。
                  慢，但臂是抬起来跨过去的，路上有料台/已放物料也能安全通过。默认用这个。

   Pick_MoveTo —— 直接到位：不抬升，贴着当前高度直接转向+走半径，最后改高度。
                  快，但中间有东西就会蹭到。只用在起终点之间确实空的情况。

   servoZ: Z舵机 0~270°(2026-10-14 由 360° 改，换舵机了); motorZ_mm/motorR_mm:
           高度/半径(mm)，越界自动钳位。
   ⚠ 量程一改，**下面 POS_* 那批宏的第一个值(角度z)全都指向了新的物理位置**：
     数值没变，但同样的命令角现在物理转得更远（5.56 → 7.41us/°）。正式跑之前
     每个姿态都要重新对一遍位。
   两版都不控制夹爪，由调用方按需开合。 */
void Pick_JumpTo(float servoZ, float motorZ_mm, float motorR_mm);
void Pick_MoveTo(float servoZ, float motorZ_mm, float motorR_mm);



void Pick_Test_Transfer(void);



/* 夹爪控制 —— 每种动作两版，按需选用：
   Pick_Gripper_*       : 平滑分步转动（慢，起步/收尾加减速，冲击小）
   Pick_Gripper_Quick_* : 直接设角一步到位（快，起步即全速）
   两版都不额外等待；需要停顿由调用方自己加。 */
void Pick_Gripper_Grab(void);
void Pick_Gripper_Release(void);
void Pick_Gripper_Observe(void);

void Pick_Gripper_Quick_Grab(void);
void Pick_Gripper_Quick_Release(void);
void Pick_Gripper_Quick_Observe(void);

/* 逐个模块测试（桌面调试，均为死循环） */
void Pick_Test_ZServo(void);    /* Z舵机: 0°/100°/50° 循环，每处停 3s（纯舵机，不走 CAN）*/
void Pick_Test_Gripper(void);   /* 夹爪: 观察/抓紧/放开 走 3 轮，停在抓紧（纯舵机，不走 CAN；
                                   三个角度见上面 PICK_OBSERVE/GRAB/RELEASE，别抄死数）*/
void Pick_Test_MotorZ(void);    /* 高度电机: 0 <-> 30mm 往返，每处停 3s（测方向和比例用）*/
void Pick_Test_MotorR(void);    /* 半径电机: 0 <-> 30mm 往返，每处停 3s（同 MotorZ，测方向和比例）*/
void Pick_Test_TurnTable(void); /* 转盘舵机: A(30°)/B(150°)/C(270°) 循环，每处停 3s（纯舵机，不走 CAN）*/

/* 老的桌面调试入口：一段写死的取放动作(POS_RAW_A → POS_1 → POS_3)，
   **全工程没人调用**，--gc-sections 会连函数带符号一起收掉。
   （原来的注释写"当前指向 Pick_Test_ZServo"，其实它没调任何测试函数，是错的。）
   留着的唯一用处是当写死的动作序列参考；真要桌面调参请用上面那批 Pick_Test_*。 */
void Pick_DebugTest(void);

#endif
