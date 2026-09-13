#include "delay.h"

// 全局变量，缓存每微秒的时钟周期数，避免重复除法
static uint32_t usTick = 0;

/**
 * @brief 初始化 DWT 周期计数器（用于微秒延时）
 * @note  必须在使用 delay_us() 前调用一次
 */
void DWT_Init(void) {
    // 使能 DWT 跟踪单元（注意宏名末尾有 A）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    // 复位计数器（可选）
    DWT->CYCCNT = 0;
    
    // 使能周期计数器（注意正确宏名）
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // 缓存每微秒的计数步长（仅计算一次）
    usTick = SystemCoreClock / 1000000;
}

/**
 * @brief 微秒级阻塞延时（需先调用 DWT_Init）
 * @param us 延时微秒数（最大约 25.5 秒，因为 32 位计数器）
 */
void delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * usTick;
    while ((DWT->CYCCNT - startTick) < delayTicks);
}