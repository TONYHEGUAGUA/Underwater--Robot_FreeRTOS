#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"

/*******************************************************************************
串口1参数设置
*******************************************************************************/
#define EN_USART1_RX 1                            /*使能（1）/禁止（0）串口1接收*/
#define UART1_FRAMEBUF_SIZE  255                  /*接收帧长度*/
#define UART1_IDLELINE_TIME  10                   /*空闲检测超时时间*/
extern u8 UART1_FrameBuff[UART1_FRAMEBUF_SIZE];   /*接收帧缓冲区数组*/
extern u16 UART1_RcvCnt;                         	/*接收计数器*/
/*******************************************************************************
串口2参数设置
*******************************************************************************/
#define EN_USART2_RX 1                            /*使能（1）/禁止（0）串口2接收*/
#define UART2_FRAMEBUF_SIZE  255                  /*接收帧长度*/
#define UART2_IDLELINE_TIME  10                   /*空闲检测超时时间*/
extern u8 UART2_FrameBuff[UART2_FRAMEBUF_SIZE];   /*接收帧缓冲区数组*/
extern u16 UART2_RcvCnt;                          /*接收计数器*/
/*******************************************************************************
串口3参数设置
*******************************************************************************/
#define EN_USART3_RX 1                            /*使能（1）/禁止（0）串口3接收*/
#define UART3_FRAMEBUF_SIZE  255                /*接收帧长度*/
#define UART3_IDLELINE_TIME  10                   /*空闲检测超时时间*/
extern u8 UART3_FrameBuff[UART3_FRAMEBUF_SIZE];   /*接收帧缓冲区数组*/
extern u16 UART3_RcvCnt;                          /*接收计数器*/
/*******************************************************************************
串口4参数设置
*******************************************************************************/
#define EN_USART4_RX 1                            /*使能（1）/禁止（0）串口3接收*/
#define UART4_FRAMEBUF_SIZE  255                  /*接收帧长度*/
#define UART4_IDLELINE_TIME  10                   /*空闲检测超时时间*/
extern u8 UART4_FrameBuff[UART4_FRAMEBUF_SIZE];   /*接收帧缓冲区数组*/
extern u16 UART4_RcvCnt;   
/*******************************************************************************
串口5参数设置
*******************************************************************************/
#define EN_USART5_RX 1                            /*使能（1）/禁止（0）串口3接收*/
#define UART5_FRAMEBUF_SIZE  255                  /*接收帧长度*/
#define UART5_IDLELINE_TIME  10                   /*空闲检测超时时间*/
extern u8 UART5_FrameBuff[UART5_FRAMEBUF_SIZE];   /*接收帧缓冲区数组*/
extern u16 UART5_RcvCnt;   
/*******************************************************************************
串口6参数设置
*******************************************************************************/
#define EN_USART6_RX 1                            /*使能（1）/禁止（0）串口3接收*/
#define UART6_FRAMEBUF_SIZE  255                  /*接收帧长度*/
#define UART6_IDLELINE_TIME  10                   /*空闲检测超时时间*/
extern u8 UART6_FrameBuff[UART6_FRAMEBUF_SIZE];   /*接收帧缓冲区数组*/
extern u16 UART6_RcvCnt;   

/*******************************************************************************
//串口初始化
*******************************************************************************/
void Uart1_init(u32 bound);
void Uart2_init(u32 bound);
void Uart3_init(u32 bound);
void Uart4_init(u32 bound);
void Uart5_init(u32 bound);
void Uart6_init(u32 bound);

/*******************************************************************************
//初始化串口定时器
*******************************************************************************/
void Uart_TIM_init(void);
/*******************************************************************************
//16进制转ASCII
*******************************************************************************/
u8 Byte_HexDispaly(u8 dat);
/*******************************************************************************
//UART 回调函数
*******************************************************************************/
void UART1_rxCallback(u8 *packet, u16 size); 
void UART2_rxCallback(u8 *packet, u16 size); 
void UART3_rxCallback(u8 *packet, u16 size); 
void UART4_rxCallback(u8 *packet, u16 size); 
void UART5_rxCallback(u8 *packet, u16 size); 
void UART6_rxCallback(u8 *packet, u16 size); 
/*******************************************************************************
//串口发送一个字节
*******************************************************************************/
void UART1_SendByte(u8 data);
void UART2_SendByte(u8 data);
void UART3_SendByte(u8 data);
void UART4_SendByte(u8 data);
void UART5_SendByte(u8 data);
void USART6_SendByte(u8 data);
/*******************************************************************************
//串口发送多个字节
*******************************************************************************/
void UART1_printf(u8 *ptr,u16 len);
void UART2_printf(u8 *ptr,u16 len);
void UART3_printf(u8 *ptr,u16 len);
void UART4_printf(u8 *ptr,u16 len);
void UART5_printf(u8 *ptr,u16 len);
void UART6_printf(u8 *ptr,u16 len);
	
#endif
