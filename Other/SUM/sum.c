#include "sum.h"
/**
 *@breaf  ����У���
 *@param  ҪУ��ĵ�һ���ֽڵ�ַ
 *@param  ҪУ��ĳ���
 *@retval У���
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
