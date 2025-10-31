#include "sum.h"
/**
 *@breaf  计算校验和
 *@param  要校验的第一个字节地址
 *@param  要校验的长度
 *@retval 校验和
 */
uint8_t check_sum(uint8_t * arr , uint8_t len)
{
	uint8_t sum = 0;
	for(int i = 0 ; i < len ; i++)
	{
		sum += arr[i];
	}
	return sum;
}
