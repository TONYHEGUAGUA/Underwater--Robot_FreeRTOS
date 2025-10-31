#include "motor.h"

void motor1_control_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void motor2_control_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//初始化控制io口

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO	
}

void motor3_control_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOC时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO	
}

void Motor_Init(void)
{
	TIM11_CH1_PWM_Init(100-1,84-1);
	TIM10_CH1_PWM_Init(100-1,84-1);
	TIM13_CH1_PWM_Init(100-1,84-1);
	motor1_control_gpio();
	motor2_control_gpio();
	motor3_control_gpio();
	Set_Motor_Stop();
}

void Set_Motor(MotrotID id , MotorDirection dir , uint8_t speed)
{
	switch(id)
	{
		case Right:
			(dir == FORWARD) ? (GPIO_ResetBits(GPIOA, GPIO_Pin_7)) : (GPIO_SetBits(GPIOA, GPIO_Pin_7)) ;
			TIM_SetCompare1(TIM11, speed);
			break;
		case Left:
			(dir == FORWARD) ? (GPIO_ResetBits(GPIOC, GPIO_Pin_4)) : (GPIO_SetBits(GPIOC, GPIO_Pin_4)) ;
			TIM_SetCompare1(TIM10, speed);
			break;
		case Top:
			(dir == FORWARD) ? (GPIO_ResetBits(GPIOA, GPIO_Pin_4)) : (GPIO_SetBits(GPIOA, GPIO_Pin_4)) ;
			TIM_SetCompare1(TIM13, speed);
			break;
	}
}

void Set_Motor_Stop(void)
{
	TIM_SetCompare1(TIM11, 0);
	TIM_SetCompare1(TIM10, 0);
	TIM_SetCompare1(TIM13, 0);
}


void PID_Control_Motor(MotrotID id , MotorDirection dir , int speed)
{
	if(speed < 0)
	{
		speed = -speed;
	}
	Set_Motor(id , dir , speed);
}

void Forward(uint8_t Left_Speed , uint8_t Right_Speed)
{
	Set_Motor(Left,FORWARD,Left_Speed);
	Set_Motor(Right,FORWARD,Right_Speed);
}
void Backward(uint8_t Left_Speed , uint8_t Right_Speed)
{
	Set_Motor(Left,REVERSE,Left_Speed);
	Set_Motor(Right,REVERSE,Right_Speed);
}
void Turn_Left(uint8_t Left_Speed , uint8_t Right_Speed)
{
	Set_Motor(Left,FORWARD,Left_Speed);
	Set_Motor(Right,REVERSE,Right_Speed);
}
void Turn_Right(uint8_t Left_Speed , uint8_t Right_Speed)
{
	Set_Motor(Left,REVERSE,Left_Speed);
	Set_Motor(Right,FORWARD,Right_Speed);
}
void Open_Top(void)
{
	Set_Motor(Top,FORWARD,100);
//	Set_Motor(Top,REVERSE,100);
}
void Close_Top(void)
{
	Set_Motor(Top,FORWARD,0);
}

