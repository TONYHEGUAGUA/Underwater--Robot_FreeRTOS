#ifndef __MOTOR_H
#define __MOTOR_H


#include "stm32f4xx.h"
#include "PWM.h"

typedef enum 
{
	FORWARD=1,
	REVERSE
}MotorDirection;

typedef enum
{
	Right=1,
	Left,
	Top
}MotrotID;


void Motor_Init(void);
void Set_Motor(MotrotID id , MotorDirection dir , uint8_t speed);
void Set_Motor_Stop(void);
void PID_Control_Motor(MotrotID id , MotorDirection dir , int speed);

void Forward(uint8_t Left_Speed , uint8_t Right_Speed);

void Backward(uint8_t Left_Speed , uint8_t Right_Speed);

void Turn_Left(uint8_t Left_Speed , uint8_t Right_Speed);

void Turn_Right(uint8_t Left_Speed , uint8_t Right_Speed);

void Open_Top(void);

void Close_Top(void);



#endif /*__MOTOR_H*/


