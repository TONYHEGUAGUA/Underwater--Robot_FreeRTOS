#include "move_average_filter.h"


Sonar_t Sonar_Front_uart6 = {0};
Sonar_t Sonar_Back_uart5 = {0};
Sonar_t Sonar_Front_uart4 = {0};
Sonar_t Sonar_Back_uart2 = {0};


void Sonar_Filter_Init(void)
{
	Sonar_t * sonars[] = {&Sonar_Front_uart6,&Sonar_Back_uart5,&Sonar_Front_uart4,&Sonar_Back_uart2};
	
	
	for(int i = 0 ; i < 4 ; i++)
	{
		memset(sonars[i],0,sizeof(Sonar_t));
		
		for(int t = 0 ; t < FILTER_WINDOW_SIZE ; t++)
			sonars[i]->filter_buffer[t] = SONAR_MAX_VALID_DISTANCE;
		sonars[i]->filter_sum = SONAR_MAX_VALID_DISTANCE * FILTER_WINDOW_SIZE;
		sonars[i]->filter_distanse = SONAR_MAX_VALID_DISTANCE;
		sonars[i]->raw_distance = SONAR_INVALID_DISTANCE;
		sonars[i]->new_data_ready = false;
		sonars[i]->filter_index = 0;
	}
	
}


bool Sonar_Is_Data_Valid(uint16_t distance)
{
	return ((distance != SONAR_INVALID_DISTANCE ) && (distance <=  SONAR_MAX_VALID_DISTANCE));
}


void Sonar_Data_Filter(Sonar_t* sonar)
{
	if(sonar->new_data_ready)
	{
		sonar->new_data_ready = false;
		if(Sonar_Is_Data_Valid(sonar->raw_distance))
		{
			sonar->filter_sum -= sonar->filter_buffer[sonar->filter_index];
			sonar->filter_buffer[sonar->filter_index] = sonar->raw_distance;
			sonar->filter_sum += sonar->raw_distance;
			
			sonar->filter_index++;
			if(sonar->filter_index >= FILTER_WINDOW_SIZE)
			{
				sonar->filter_index = 0;
			}
			
			sonar->filter_distanse = sonar->filter_sum / FILTER_WINDOW_SIZE;
		}
	}
}

uint16_t Sonar_Get_Filter_Distanse(Sonar_t *sonar)
{
	return sonar->filter_distanse;
}

void Sonar_Update_distanse(Sonar_t *sonar,uint16_t data)
{
	sonar->raw_distance = data;
}

bool Sonar_Check_Newdata(Sonar_t * sonar)
{
	return sonar->new_data_ready;
}

