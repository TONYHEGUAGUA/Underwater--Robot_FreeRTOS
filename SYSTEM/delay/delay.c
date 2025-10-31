#include "delay.h"

// 使用TIM2实现delay函数，避免与FreeRTOS的SysTick冲突

/**
 * @brief 延时初始化函数
 * @param 无
 * @retval 无
 */
void delay_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    // 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 配置TIM2
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;      // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;       // 168MHz/84 = 2MHz，1us计数
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief 微秒延时函数
 * @param us: 延时微秒数
 * @retval 无
 */
void delay_us(uint32_t us)
{
    uint32_t temp;
    temp = TIM_GetCounter(TIM2) + us;  // 计算目标计数值
    while(TIM_GetCounter(TIM2) < temp); // 等待到达目标值
}

/**
 * @brief 毫秒延时函数
 * @param ms: 延时毫秒数
 * @retval 无
 */
void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);  // 调用微秒延时函数
    }
}

/**
 * @brief 秒延时函数
 * @param s: 延时秒数
 * @retval 无
 */
void delay_s(uint32_t s)
{
    while(s--)
    {
        delay_ms(1000);  // 调用毫秒延时函数
    }
}
