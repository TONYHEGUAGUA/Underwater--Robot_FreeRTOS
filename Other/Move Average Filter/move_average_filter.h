#ifndef __MOVE_AVERAGE_FILTER_H
#define __MOVE_AVERAGE_FILTER_H

#include "stm32f4xx.h"
#include "string.h"
#include "stdbool.h"



#define FILTER_WINDOW_SIZE 5

#define SONAR_MAX_VALID_DISTANCE 2000

#define SONAR_INVALID_DISTANCE 65533

#define SONAR_TURN_DISTANSE 1000



typedef struct
{
	volatile uint16_t raw_distance;             //最新一次从声纳中读出的距离值
	uint16_t filter_buffer[FILTER_WINDOW_SIZE]; //滑动平均滤波数组
	uint32_t filter_sum;
	uint16_t filter_distanse;                   //滤波后的距离值，单位mm
	volatile bool new_data_ready;               //是否有新数据
	uint8_t filter_index;
}Sonar_t;
	
extern Sonar_t Sonar_Front_uart6;
extern Sonar_t Sonar_Back_uart5 ;
extern Sonar_t Sonar_Front_uart4;
extern Sonar_t Sonar_Back_uart2 ;




void Sonar_Filter_Init(void);
bool Sonar_Is_Data_Valid(uint16_t distance);
void Sonar_Data_Filter(Sonar_t* sonar);
uint16_t Sonar_Get_Filter_Distanse(Sonar_t *sonar);
void Sonar_Update_distanse(Sonar_t *sonar,uint16_t data);
bool Sonar_Check_Newdata(Sonar_t * sonar);

#endif
