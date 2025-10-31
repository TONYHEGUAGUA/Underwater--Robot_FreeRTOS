/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stdio.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "motor.h"
#include "myiic.h"
#include "REG.h"
#include "adc.h"
#include "rain_sensor.h"
#include "sum.h"
#include "move_average_filter.h"

#include "freertos.h"
#include "task.h"
/* Exported types ------------------------------------------------------------*/


#define MIN(a, b) ((a) < (b) ? (a) : (b))


/* Exported constants --------------------------------------------------------*/

extern int PID_PWM;
extern float Target;
/*******************************************************************************
//����1��������
*******************************************************************************/
extern u8 UART1_FrameBuff[255];                   /* ����֡���������� */
extern u16 UART1_RcvCnt;                          /* ���ռ��� */
extern u32 timerRcvCnt1;                          /* ���ڶ�ʱ������ */   
/*******************************************************************************
//����2��������
*******************************************************************************/
extern u8 UART2_FrameBuff[255];                  /* ����֡���������� */
extern u16 UART2_RcvCnt;                         /* ���ռ��� */
extern u32 timerRcvCnt2;                         /* ���ڶ�ʱ������ */
/*******************************************************************************
//����3��������
*******************************************************************************/
extern u8 UART3_FrameBuff[255];                 /* ����֡���������� */
extern u16 UART3_RcvCnt;                        /* ���ռ��� */
extern u32 timerRcvCnt3;                        /* ���ڶ�ʱ������ */
/*******************************************************************************
//����4��������
*******************************************************************************/
extern u8 UART4_FrameBuff[255];                 /* ����֡���������� */
extern u16 UART4_RcvCnt;                        /* ���ռ��� */
extern u32 timerRcvCnt4;                        /* ���ڶ�ʱ������ */
/*******************************************************************************
//����5��������
*******************************************************************************/
extern u8 UART5_FrameBuff[255];                 /* ����֡���������� */
extern u16 UART5_RcvCnt;                        /* ���ռ��� */
extern u32 timerRcvCnt5;                        /* ���ڶ�ʱ������ */
/*******************************************************************************
//����6��������
*******************************************************************************/
extern u8 UART6_FrameBuff[255];                 /* ����֡���������� */
extern u16 UART6_RcvCnt;                        /* ���ռ��� */
extern u32 timerRcvCnt6;                        /* ���ڶ�ʱ������ */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#endif /* __MAIN_H */

