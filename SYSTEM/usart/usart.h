#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"

/*******************************************************************************
����1��������
*******************************************************************************/
#define EN_USART1_RX 1                            /*ʹ�ܣ�1��/��ֹ��0������1����*/
#define UART1_FRAMEBUF_SIZE  255                  /*����֡����*/
#define UART1_IDLELINE_TIME  10                   /*���м�ⳬʱʱ��*/
extern u8 UART1_FrameBuff[UART1_FRAMEBUF_SIZE];   /*����֡����������*/
extern u16 UART1_RcvCnt;                         	/*���ռ�����*/
/*******************************************************************************
����2��������
*******************************************************************************/
#define EN_USART2_RX 1                            /*ʹ�ܣ�1��/��ֹ��0������2����*/
#define UART2_FRAMEBUF_SIZE  255                  /*����֡����*/
#define UART2_IDLELINE_TIME  10                   /*���м�ⳬʱʱ��*/
extern u8 UART2_FrameBuff[UART2_FRAMEBUF_SIZE];   /*����֡����������*/
extern u16 UART2_RcvCnt;                          /*���ռ�����*/
/*******************************************************************************
����3��������
*******************************************************************************/
#define EN_USART3_RX 1                            /*ʹ�ܣ�1��/��ֹ��0������3����*/
#define UART3_FRAMEBUF_SIZE  255                /*����֡����*/
#define UART3_IDLELINE_TIME  10                   /*���м�ⳬʱʱ��*/
extern u8 UART3_FrameBuff[UART3_FRAMEBUF_SIZE];   /*����֡����������*/
extern u16 UART3_RcvCnt;                          /*���ռ�����*/
/*******************************************************************************
����4��������
*******************************************************************************/
#define EN_USART4_RX 1                            /*ʹ�ܣ�1��/��ֹ��0������3����*/
#define UART4_FRAMEBUF_SIZE  255                  /*����֡����*/
#define UART4_IDLELINE_TIME  10                   /*���м�ⳬʱʱ��*/
extern u8 UART4_FrameBuff[UART4_FRAMEBUF_SIZE];   /*����֡����������*/
extern u16 UART4_RcvCnt;   
/*******************************************************************************
����5��������
*******************************************************************************/
#define EN_USART5_RX 1                            /*ʹ�ܣ�1��/��ֹ��0������3����*/
#define UART5_FRAMEBUF_SIZE  255                  /*����֡����*/
#define UART5_IDLELINE_TIME  10                   /*���м�ⳬʱʱ��*/
extern u8 UART5_FrameBuff[UART5_FRAMEBUF_SIZE];   /*����֡����������*/
extern u16 UART5_RcvCnt;   
/*******************************************************************************
����6��������
*******************************************************************************/
#define EN_USART6_RX 1                            /*ʹ�ܣ�1��/��ֹ��0������3����*/
#define UART6_FRAMEBUF_SIZE  255                  /*����֡����*/
#define UART6_IDLELINE_TIME  10                   /*���м�ⳬʱʱ��*/
extern u8 UART6_FrameBuff[UART6_FRAMEBUF_SIZE];   /*����֡����������*/
extern u16 UART6_RcvCnt;   

/*******************************************************************************
//���ڳ�ʼ��
*******************************************************************************/
void Uart1_init(u32 bound);
void Uart2_init(u32 bound);
void Uart3_init(u32 bound);
void Uart4_init(u32 bound);
void Uart5_init(u32 bound);
void Uart6_init(u32 bound);

/*******************************************************************************
//��ʼ�����ڶ�ʱ��
*******************************************************************************/
void Uart_TIM_init(void);
/*******************************************************************************
//16����תASCII
*******************************************************************************/
u8 Byte_HexDispaly(u8 dat);
/*******************************************************************************
//UART �ص�����
*******************************************************************************/
void UART1_rxCallback(u8 *packet, u16 size); 
void UART2_rxCallback(u8 *packet, u16 size); 
void UART3_rxCallback(u8 *packet, u16 size); 
void UART4_rxCallback(u8 *packet, u16 size); 
void UART5_rxCallback(u8 *packet, u16 size); 
void UART6_rxCallback(u8 *packet, u16 size); 
/*******************************************************************************
//���ڷ���һ���ֽ�
*******************************************************************************/
void UART1_SendByte(u8 data);
void UART2_SendByte(u8 data);
void UART3_SendByte(u8 data);
void UART4_SendByte(u8 data);
void UART5_SendByte(u8 data);
void USART6_SendByte(u8 data);
/*******************************************************************************
//���ڷ��Ͷ���ֽ�
*******************************************************************************/
void UART1_printf(u8 *ptr,u16 len);
void UART2_printf(u8 *ptr,u16 len);
void UART3_printf(u8 *ptr,u16 len);
void UART4_printf(u8 *ptr,u16 len);
void UART5_printf(u8 *ptr,u16 len);
void UART6_printf(u8 *ptr,u16 len);
	
#endif
