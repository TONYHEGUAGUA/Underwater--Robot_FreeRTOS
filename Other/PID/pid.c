#include "pid.h"

float Target , Actual , Out;
float Kp = 1 , Ki = 0.001 , Kd = 0.1 ;
float Error0 , Error1 , ErrorInt ;

int Yaw_PID_Control(void)
{
	Actual = Get_Yaw();
	Error1 = Error0;
	Error0 = Target - Actual;
	
	ErrorInt += Error0;
	
	Out = Kp * Error0 + Ki * ErrorInt + Kd * ( Error0 - Error1 );
	
	if(Out > 80)
		Out=80;
	else if (Out < -80)
		Out=-80;
	return Out;
}
