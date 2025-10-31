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
//串口1变量定义
*******************************************************************************/
extern u8 UART1_FrameBuff[255];                   /* 接收帧缓冲区数组 */
extern u16 UART1_RcvCnt;                          /* 接收计数 */
extern u32 timerRcvCnt1;                          /* 串口定时器计数 */   
/*******************************************************************************
//串口2变量定义
*******************************************************************************/
extern u8 UART2_FrameBuff[255];                  /* 接收帧缓冲区数组 */
extern u16 UART2_RcvCnt;                         /* 接收计数 */
extern u32 timerRcvCnt2;                         /* 串口定时器计数 */
/*******************************************************************************
//串口3变量定义
*******************************************************************************/
extern u8 UART3_FrameBuff[255];                 /* 接收帧缓冲区数组 */
extern u16 UART3_RcvCnt;                        /* 接收计数 */
extern u32 timerRcvCnt3;                        /* 串口定时器计数 */
/*******************************************************************************
//串口4变量定义
*******************************************************************************/
extern u8 UART4_FrameBuff[255];                 /* 接收帧缓冲区数组 */
extern u16 UART4_RcvCnt;                        /* 接收计数 */
extern u32 timerRcvCnt4;                        /* 串口定时器计数 */
/*******************************************************************************
//串口5变量定义
*******************************************************************************/
extern u8 UART5_FrameBuff[255];                 /* 接收帧缓冲区数组 */
extern u16 UART5_RcvCnt;                        /* 接收计数 */
extern u32 timerRcvCnt5;                        /* 串口定时器计数 */
/*******************************************************************************
//串口6变量定义
*******************************************************************************/
extern u8 UART6_FrameBuff[255];                 /* 接收帧缓冲区数组 */
extern u16 UART6_RcvCnt;                        /* 接收计数 */
extern u32 timerRcvCnt6;                        /* 串口定时器计数 */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#endif /* __MAIN_H */

