#include "hwt101.h"
#include "usart.h"
#include "yawPIDcontroller.h"

/**********************************************************
*** HWT101 单轴航向角 IMU（维特智能）驱动程序
*** 通信接口：UART5（115200bps, 8N1）
*** 数据输出：0x55 + TYPE + 8字节数据 + SUM（共11字节）
***   0x52 角速度帧：0x55 0x52 0x00 0x00 RWzL RWzH WzL WzH 0x00 0x00 SUM
***   0x53 角度帧  ：0x55 0x53 0x00 0x00 0x00 0x00 YawL YawH VL VH SUM
*** 寄存器写：0xFF 0xAA ADDR DATAL DATAH（共5字节）
*** 协议文档：HWT101协议.md
**********************************************************/

HWT101_TypeDef hwt101;

/* 单字节中断接收缓冲（仅在中断上下文访问） */
static uint8_t hwt101_rxByte = 0U;

/**
 * @brief 初始化HWT101，绑定串口并启动中断接收
 * @param huart 串口句柄（应为&huart5）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_Init(UART_HandleTypeDef *huart)
{
    hwt101.huart = huart;
    hwt101.frameIndex = 0U;
    hwt101.yaw = 0.0f;
    hwt101.wz = 0.0f;
    hwt101.rawWz = 0.0f;
    hwt101.version = 0U;
    hwt101.dataReady = false;

    ZeroBias_Reset();   /* 清零零偏估计状态;随后 ManualCal() 会采集一次静止窗口 */

    /* 使能接收中断，逐字节接收并组帧 */
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    HAL_NVIC_SetPriority(UART5_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
    HAL_UART_Receive_IT(huart, &hwt101_rxByte, 1U);

    HWT101_SetYawZero();

    HWT101_SetRate(100);  /* 设置输出频率100Hz */

    HWT101_ManualCal();
}

/**
 * @brief UART5中断服务函数（覆盖startup弱定义，勿在其他文件重复定义）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart5);
}

/**
 * @brief UART接收完成回调：字节送入组帧解析并重启接收
 * @param huart 串口句柄
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
        HWT101_FeedByte(hwt101_rxByte);
        HAL_UART_Receive_IT(huart, &hwt101_rxByte, 1U);
    }
}

/**
 * @brief 解析一帧完整数据：校验SUM并换算物理量
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
static void HWT101_ParseFrame(void)
{
    uint8_t i;
    uint8_t sum = 0U;
    int16_t value;

    /* 校验和：0x55+TYPE+8字节数据 的低8位 */
    for(i = 0U; i < (HWT101_FRAME_LEN - 1U); i++)
    {
        sum += hwt101.frame[i];
    }
    if(sum != hwt101.frame[HWT101_FRAME_LEN - 1U])
    {
        return; /* 校验失败，丢弃本帧 */
    }

    switch(hwt101.frame[1])
    {
        case HWT101_FRAME_GYRO:
            /* 帧布局：[0]55 [1]52 [2]00 [3]00 [4]RWzL [5]RWzH [6]WzL [7]WzH [8]00 [9]00 [10]SUM */
            value = (int16_t)((uint16_t)hwt101.frame[5] << 8 | (uint16_t)hwt101.frame[4]);
            hwt101.rawWz = (float)value * HWT101_GYRO_SCALE;
            value = (int16_t)((uint16_t)hwt101.frame[7] << 8 | (uint16_t)hwt101.frame[6]);
            hwt101.wz = (float)value * HWT101_GYRO_SCALE;
            break;

        case HWT101_FRAME_ANGLE:
            /* 帧布局：[0]55 [1]53 [2]00 [3]00 [4]00 [5]00 [6]YawL [7]YawH [8]VL [9]VH [10]SUM */
            value = (int16_t)((uint16_t)hwt101.frame[7] << 8 | (uint16_t)hwt101.frame[6]);
            hwt101.yaw = (float)value * HWT101_ANGLE_SCALE;
            hwt101.version = (uint16_t)((uint16_t)hwt101.frame[9] << 8 | (uint16_t)hwt101.frame[8]);
            hwt101.dataReady = true;
            ZeroBias_Feed();    /* 每个有效角度帧喂一次零偏估计器(替代原TIM7节拍) */
            break;

        default:
            break;
    }
}

/**
 * @brief 接收字节送入组帧状态机，凑满11字节后解析
 * @param byte 收到的单字节数据
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_FeedByte(uint8_t byte)
{
    switch(hwt101.frameIndex)
    {
        case 0U:
            if(byte == HWT101_FRAME_HEAD)
            {
                hwt101.frame[0] = byte;
                hwt101.frameIndex = 1U;
            }
            break;

        case 1U:
            if(byte == HWT101_FRAME_GYRO || byte == HWT101_FRAME_ANGLE)
            {
                hwt101.frame[1] = byte;
                hwt101.frameIndex = 2U;
            }
            else
            {
                hwt101.frameIndex = 0U; /* 未知帧类型，重新找帧头 */
            }
            break;

        default:
            hwt101.frame[hwt101.frameIndex] = byte;
            hwt101.frameIndex++;
            if(hwt101.frameIndex >= HWT101_FRAME_LEN)
            {
                hwt101.frameIndex = 0U;
                HWT101_ParseFrame();
            }
            break;
    }
}

/**
 * @brief 写寄存器：0xFF 0xAA ADDR DATAL DATAH
 * @param addr 寄存器地址
 * @param data 16位数据（小端发送）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_WriteReg(uint8_t addr, uint16_t data)
{
    uint8_t cmd[5];

    cmd[0] = 0xFFU;
    cmd[1] = 0xAAU;
    cmd[2] = addr;
    cmd[3] = (uint8_t)(data & 0x00FFU);
    cmd[4] = (uint8_t)((data >> 8U) & 0x00FFU);

    HAL_UART_Transmit(hwt101.huart, cmd, 5U, 100U);
}

/**
 * @brief 解锁（写KEY寄存器0xB588），所有写操作前必须先解锁
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_Unlock(void)
{
    HWT101_WriteReg(HWT101_REG_KEY, 0xB588U);
    HAL_Delay(200);
}

/**
 * @brief 保存配置（写SAVE寄存器0x0000），设置类操作后调用
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_Save(void)
{
    HWT101_WriteReg(HWT101_REG_SAVE, 0x0000U);
    HAL_Delay(200);
}

/**
 * @brief Z轴航向角归零（解锁->写CALIYAW=0->保存）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_SetYawZero(void)
{
    HWT101_Unlock();
    HWT101_WriteReg(HWT101_REG_CALIYAW, 0x0000U);
    HAL_Delay(500);
    HWT101_Save();
}

/**
 * @brief 设置当前Z轴航向角（解锁->写CALIYAW=角度换算值->保存）
 * @param yawDeg 目标航向角（°，-180~180，换算公式 deg*32768/180）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_SetYaw(float yawDeg)
{
    uint16_t raw;

    raw = (uint16_t)(yawDeg * 32768.0f / 180.0f);

    HWT101_Unlock();
    HWT101_WriteReg(HWT101_REG_CALIYAW, raw);
    HAL_Delay(500);
    HWT101_Save();
}

/**
 * @brief 设置数据输出速率（解锁->写RRATE->保存）
 * @param hz 输出频率，支持：1/2/5/10/20/50/100/200/500/1000，无效值按10Hz处理
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_SetRate(uint16_t hz)
{
    uint16_t rateCode;

    switch(hz)
    {
        case 1U:    rateCode = 0x03U; break;   /* 1Hz   */
        case 2U:    rateCode = 0x04U; break;   /* 2Hz   */
        case 5U:    rateCode = 0x05U; break;   /* 5Hz   */
        case 10U:   rateCode = 0x06U; break;   /* 10Hz  */
        case 20U:   rateCode = 0x07U; break;   /* 20Hz  */
        case 50U:   rateCode = 0x08U; break;   /* 50Hz  */
        case 100U:  rateCode = 0x09U; break;   /* 100Hz */
        case 200U:  rateCode = 0x0BU; break;   /* 200Hz */
        case 500U:  rateCode = 0x0CU; break;   /* 500Hz */
        case 1000U: rateCode = 0x0DU; break;   /* 1000Hz */
        default:    rateCode = 0x06U; break;   /* 默认10Hz */
    }

    HWT101_Unlock();
    HWT101_WriteReg(HWT101_REG_RRATE, rateCode);
    HAL_Delay(200);
    HWT101_Save();
}

/**
 * @brief 手动获取零偏-开始（写MANUALCALI=1，机器人必须完全静止后调用）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_ManualCalStart(void)
{
    HWT101_WriteReg(HWT101_REG_MANUALCALI, 0x0001U);
}

/**
 * @brief 手动获取零偏-结束（写MANUALCALI=4，机器人运动前调用）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_ManualCalStop(void)
{
    HWT101_WriteReg(HWT101_REG_MANUALCALI, 0x0004U);
}

/**
 * @brief 设置陀螺仪自动校准开关
 * @param enable true=打开自动校准；false=关闭（AGV推荐关闭，配合手动零偏使用）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_SetAutoCal(bool enable)
{
    /* NOAUTOCALI寄存器：0=打开自动校准，1=关闭自动校准 */
    HWT101_WriteReg(HWT101_REG_NOAUTOCALI, enable ? 0x0000U : 0x0001U);
}

/**
 * @brief 查询是否有新的航向角数据
 * @retval bool true=有未读数据
 * @author Sisyphus
 * @date 2026-08-18
 */
bool HWT101_IsDataReady(void)
{
    return (hwt101.dataReady == true);
}

/**
 * @brief 清除新数据标志（读取数据后调用）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_ClearDataReady(void)
{
    hwt101.dataReady = false;
}

/**
 * @brief 获取Z轴航向角
 * @retval float 航向角（°，-180~180）
 * @author Sisyphus
 * @date 2026-08-18
 */
float HWT101_GetYaw(void)
{
    return hwt101.yaw;
}

/**
 * @brief 获取校准后Z轴角速度
 * @retval float 角速度（°/s）
 * @author Sisyphus
 * @date 2026-08-18
 */
float HWT101_GetWz(void)
{
    return hwt101.wz;
}

/**
 * @brief 手动获取零偏校准（写MANUALCALI=1，机器人必须完全静止后调用）
 * @retval None
 * @author Sisyphus
 * @date 2026-08-18
 */
void HWT101_ManualCal()
{
    ZeroBias_Start();
    HWT101_ManualCalStart();
    HAL_Delay(20000);
    HWT101_ManualCalStop();
    ZeroBias_End();
}
