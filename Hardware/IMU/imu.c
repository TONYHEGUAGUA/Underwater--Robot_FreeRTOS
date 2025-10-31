#include "imu.h"

float Get_Yaw(void)
{

	float yaw_angle_temp = ((I2C_readreg(0x50,Yaw) /32768*180));
	return yaw_angle_temp;
}
