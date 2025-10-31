#include "delay.h"

// ʹ��TIM2ʵ��delay������������FreeRTOS��SysTick��ͻ

/**
 * @brief ��ʱ��ʼ������
 * @param ��
 * @retval ��
 */
void delay_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    // ʹ��TIM2ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // ����TIM2
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;      // �Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;       // 168MHz/84 = 2MHz��1us����
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // ����TIM2
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief ΢����ʱ����
 * @param us: ��ʱ΢����
 * @retval ��
 */
void delay_us(uint32_t us)
{
    uint32_t temp;
    temp = TIM_GetCounter(TIM2) + us;  // ����Ŀ�����ֵ
    while(TIM_GetCounter(TIM2) < temp); // �ȴ�����Ŀ��ֵ
}

/**
 * @brief ������ʱ����
 * @param ms: ��ʱ������
 * @retval ��
 */
void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);  // ����΢����ʱ����
    }
}

/**
 * @brief ����ʱ����
 * @param s: ��ʱ����
 * @retval ��
 */
void delay_s(uint32_t s)
{
    while(s--)
    {
        delay_ms(1000);  // ���ú�����ʱ����
    }
}
