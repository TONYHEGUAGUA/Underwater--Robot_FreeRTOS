#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "usart.h"
#include "string.h"
#include "pid.h"
#include "main.h"

#define BASIC_TIM           		TIM6
#define BASIC_TIM_CLK       		RCC_APB1Periph_TIM6

#define BASIC_TIM_IRQn					TIM6_DAC_IRQn
#define BASIC_TIM_IRQHandler    TIM6_DAC_IRQHandler

void TIMx_Configuration(void);
void TIM7_Configuration(void);
#endif
