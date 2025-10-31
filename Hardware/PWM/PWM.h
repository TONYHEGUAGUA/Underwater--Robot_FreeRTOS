#ifndef __PWM_H
#define __PWM_H


#include "stm32f4xx.h"

void TIM11_CH1_PWM_Init(uint16_t arr,uint8_t psc);
void TIM10_CH1_PWM_Init(uint16_t arr,uint8_t psc);
void TIM13_CH1_PWM_Init(uint16_t arr,uint8_t psc);

#endif /*__PWM_H*/
