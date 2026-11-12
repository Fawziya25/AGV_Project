#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Emm_V5.h"
#include "main.h"
#include "delay.h"
#include "yawPIDcontroller.h"
#include "radar_task.h"

#define a 100
#define b 115

/* ---------------- 场地坐标(mm) ----------------
   下面这些宏是"x, y"两个数打包成一个宏(带逗号)，用法：
       AGV_Position_Go(Raw, 0);   →  AGV_Position_Go(1200, 300, 0);
   所以只能整体当实参用，不能出现在赋值或运算里。 */
#define Origin_X 165                 // 初始X坐标
#define Origin_Y 165                 // 初始Y坐标

#define Vel 350                    // 默认行驶速度(RPM)

#define Run_With_Radar 0 // 是否避障运行，1 = 避障，0 = 不避障

/* ---------------- 行驶加速度(0~255，对应 Emm_V5 的 acc 字段) ----------------
   数值越大，起步/停车的加减速越猛；0 = 不插加减速曲线，直接给目标转速。
   两种模式分开：位置模式(直行/平移)和速度模式(原地转)的负载不一样，各调各的。 */
#define AGV_ACC_POS   150           // 位置模式加速度(AGV_Position_GoStraight / Translate)
#define AGV_ACC_VEL   100            // 速度模式加速度(AGV_Position_Rotate)

#define QRCode 360, 1110           // 二维码区坐标
#define Raw 1200, 300              // 原料区坐标
#define Raw_Depart 1200, 400       // 原料区出发点坐标
#define Rough 1200, 2150           // 粗加工区坐标
#define Temp 2090, 1230            // 暂存区坐标
#define Waypoint_LB 360, 300       // 左下角目标点坐标
#define Waypoint_RB 2090, 300      // 右下角目标点坐标
#define Waypoint_LF 400, 2150      // 左上角目标点坐标
#define Waypoint_RF 2090, 2150     // 右上角目标点坐标
#define Destination 165, 2285        // 目标点坐标

/* ---------------- 启停区 → 小车在位姿（2026-11-01） ----------------
   启动菜单只选启停区；选定后 AGV_Position_ApplyZone(zone) 一次装好三件事：
   "车停在哪"(AGV_Position_X/Y)、"车头朝哪"(AGV_Direction)、"出发先开到哪个格"(Depart 首点)。
   zone 编号与雷达 radar_config.h 的 RADAR_ZONEn_* 一致（1/2）。

   ORIGIN_DIR  车头朝向 0:X+ 1:Y+ 2:X- 3:Y-
   DEPART_GRID 停车位对应的拐点下标（Points_X/Points_Y 的索引），即 Depart 先对齐到哪一格；
               ⚠ 必须与 radar_config.h 的 RADAR_ZONEn_START_GRID 保持一致（当前 1 区=4、2 区=0），
                 将来改出发点方格时两边一起改。 */
#define AGV_ZONE1_ORIGIN_X       150U     /* 区1：左下角停车位 */
#define AGV_ZONE1_ORIGIN_Y       150U
#define AGV_ZONE1_ORIGIN_DIR     3U       /* 朝 Y- */
#define AGV_ZONE1_DEPART_GRID    4U       /* Points[4] = (310, 310) 左下角 */
#define AGV_ZONE2_ORIGIN_X       150U     /* 区2：左上角停车位（上电默认，见 RADAR_ZONE_DEFAULT） */
#define AGV_ZONE2_ORIGIN_Y       2250U
#define AGV_ZONE2_ORIGIN_DIR     3U       /* 朝 Y- */
#define AGV_ZONE2_DEPART_GRID    0U       /* Points[0] = (310, 2090) 左上角 */

/* 小车当前坐标(mm)。menu.c 要在启动菜单上显示，故在此声明 */
extern uint16_t AGV_Position_X;
extern uint16_t AGV_Position_Y;

/* 按启停区 1/2 设定小车初始坐标与车头朝向（其它值按区2处理）。
   只动坐标/朝向/当前区号，不碰雷达；雷达那边由 Radar_SetZone() 管。 */
void AGV_Position_ApplyZone(uint8_t zone);

void AGV_Position_GoStraight(uint32_t distance, uint16_t vel, uint8_t dir);
void AGV_Position_Translate(uint32_t distance, uint16_t vel, uint8_t dir);
void AGV_Position_Rotate(uint8_t dir);
/* 坐标导航：Set 重新锚定小车当前坐标，Go 按 AGV_Direction 自动拆成直行/平移走到(x,y) */
void AGV_Position_Init();
void AGV_Position_Set(uint16_t x, uint16_t y);
void AGV_Position_Go(uint16_t x, uint16_t y, uint8_t priDir);
void AGV_Position_Depart();
void AGV_Position_Qcode_to_Raw();
void AGV_Position_Raw_to_Rough();
void AGV_Position_Rough_to_Temp();
void AGV_Position_Temp_to_Raw();
void AGV_Position_GoBack();

#endif