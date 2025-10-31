#ifndef __ADC_H
#define __ADC_H



#include "stm32f4xx.h"

void Myadc_Init(void);
uint16_t Get_ADC_Value(void);

#endif /* __ADC_H */
