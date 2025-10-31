#include "usart.h"	  
#include "string.h"
#include "sys.h"
/* 本文件主要是配置和操作三个串口：USART1, USART2, USART3。
	包括初始化、中断处理、数据发送。
	每个串口都有独立的缓冲区和计数器来存储接收数据，并
	可通过中断处理将接收数据传递给回调函数。
	此外，还提供了发送单个字节和多个字节的函数。
*/

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 


#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)


/**
  * @brief  retargets the c library printf function to the usart.
  * @param  none
  * @retval none
  */
PUTCHAR_PROTOTYPE
{
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  USART_SendData(USART1, ch);
  return ch;
}
#endif

/*******************************************************************************
//串口1参数设置
*******************************************************************************/
u8 UART1_FrameBuff[UART1_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
u16 UART1_RcvCnt=0;                     /*接收计数器*/
u32 timerRcvCnt1=0;                     /*串口定时计数器*/    
/*******************************************************************************
//串口2参数设置
*******************************************************************************/
u8 UART2_FrameBuff[UART2_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
u16 UART2_RcvCnt=0;                     /*接收计数器*/
u32 timerRcvCnt2=0;                     /*串口定时计数器*/
/*******************************************************************************
//串口3参数设置
*******************************************************************************/
u8 UART3_FrameBuff[UART3_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
u16 UART3_RcvCnt;                     /*接收计数器*/
u32 timerRcvCnt3=0;                     /*串口定时计数器*/
/*******************************************************************************
//串口4参数设置
*******************************************************************************/
u8 UART4_FrameBuff[UART4_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
u16 UART4_RcvCnt=0;                     /*接收计数器*/
u32 timerRcvCnt4=0;                     /*串口定时计数器*/
/*******************************************************************************
//串口5参数设置
*******************************************************************************/
u8 UART5_FrameBuff[UART5_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
u16 UART5_RcvCnt=0;                     /*接收计数器*/
u32 timerRcvCnt5=0;                     /*串口定时计数器*/
/*******************************************************************************
//串口6参数设置
*******************************************************************************/
u8 UART6_FrameBuff[UART6_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
u16 UART6_RcvCnt=0;                     /*接收计数器*/
u32 timerRcvCnt6=0;                     /*串口定时计数器*/


/*******************************************************************************
//16进制转ASCII
*******************************************************************************/
u8 Byte_HexDispaly(u8 dat)
{
	if(dat>=10&&dat<16) dat+=0x07;	    //A-F +0x37
	dat+=0x30;						   						//0-9 +0x30
	return dat;
}

/*******************************************************************************
//串口1初始化
*******************************************************************************/
void Uart1_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 1. 开启GPIOA和USART1时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	/* 2. USART1 对应引脚复用映射 */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	/* 3. USART1 端口配置 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
		
    /* 4. USART1 初始化设置 */
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	/* 5. UART1 NVIC 配置 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);				//开启相关中断
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;	//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;			//子优先级7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);								//根据指定的参数初始化VIC寄存器
	
	 /* 5. 使能USART1 */
	USART_Cmd(USART1, ENABLE); 
}

/*******************************************************************************
//串口2初始化
*******************************************************************************/	
void Uart2_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. 使能GPIOD和USART2时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOD时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 使能USART2时钟

    /* 2. USART2 对应引脚复用映射 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // GPIOD5复用为USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // GPIOD6复用为USART2

    /* 3.USART2 端口配置 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // GPIOD5与GPIOD6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure); // 初始化GPIOD5，GPIOD6

    /* 4. USART2 初始化设置 */
    USART_InitStructure.USART_BaudRate = bound; // 波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(USART2, &USART_InitStructure); // 初始化串口2
	
	/* 5. UART2 NVIC 配置 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//开启相关中断
	
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//子优先级7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	/* 6. 使能UART2 */
    USART_Cmd(USART2, ENABLE); // 使能串口2
}

/*******************************************************************************
//串口3初始化
*******************************************************************************/
void Uart3_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 1. 使能GPIOC和USART3时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART3时钟
 
	/* 2. USART3 对应引脚复用映射 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10复用为USART3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11复用为USART3
	
	/* 3.USART3 端口配置 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA2，PA3

	/* 4. USART3 初始化设置 */
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3 

	/* 5. UART3 NVIC 配置 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			//开启相关中断
	
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;	//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//子优先级7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	/* 6. 使能UART3 */
	USART_Cmd(USART3, ENABLE);  //使能串口3
}

/*******************************************************************************
//串口4初始化
*******************************************************************************/
void Uart4_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 1. 使能GPIOC和UART4时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); // 使能UART4时钟
 
	/* 2. UART4 对应引脚复用映射 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART5); // GPIOD10复用为UART4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART5); // GPIOC11复用为UART4
	
	/* 3.UART4 端口配置 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	 
	/* 4. UART4 初始化设置 */
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART4, &USART_InitStructure); //初始化串口4
	
	/* 5. UART4 NVIC 配置 */
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);           //开启相关中断
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;        //串口5中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//子优先级7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化NVIC寄存器

	/* 6. 使能UART4 */
	USART_Cmd(UART4, ENABLE);
}
/*******************************************************************************
//串口5初始化
*******************************************************************************/
void Uart5_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 1. 使能GPIOC和UART5时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE); // 使能GPIOC时钟GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); // 使能UART5时钟
 
	/* 2. UART5 对应引脚复用映射 */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);  // GPIOD2复用为UART5
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); // GPIOC12复用为UART5
	
	/* 3.UART5 端口配置 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_12; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
	 
   /* 4. UART5 初始化设置 */
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART5, &USART_InitStructure); //初始化串口5
	
	/* 5. UART5 NVIC 配置 */
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);          //开启相关中断
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;        //串口5中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//子优先级7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化NVIC寄存器
	
	/* 6. 使能UART5 */
	USART_Cmd(UART5, ENABLE);
}

/*******************************************************************************
//串口6初始化
*******************************************************************************/
void Uart6_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. 使能GPIOC 和UASRT6时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 使能GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); // 使能USART6时钟

    /* 2. USART6 对应引脚复用映射 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); // GPIOC6复用为USART6_TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); // GPIOC7复用为USART6_RX

    /* 3. USART6 端口配置 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // GPIOC6与GPIOC7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure); // 初始化GPIOC6，GPIOC7

    /* 4. USART6 初始化设置 */
    USART_InitStructure.USART_BaudRate = bound; // 波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(USART6, &USART_InitStructure); // 初始化串口6
	
	/* 5. Usart6 NVIC 配置 */
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);          //开启相关中断
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;       //串口6中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//子优先级7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化NVIC寄存器
	
	/* 6. 使能USART6 */
    USART_Cmd(USART6, ENABLE); 
}


/*******************************************************************************
//串口1接收中断
//当接收到数据时，将数据存储到缓冲区。如果缓冲区满，则调用回调函数并清空缓冲区。
*******************************************************************************/
#if EN_USART1_RX   //如果使能了接收
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
	{
		if(UART1_RcvCnt>=UART1_FRAMEBUF_SIZE)
		{
			UART1_rxCallback(UART1_FrameBuff,UART1_RcvCnt);//向串口回调函数中赋值
			memset(UART1_FrameBuff,0,sizeof(UART1_FrameBuff));//清空数组
			UART1_RcvCnt=0;
		}
		UART1_FrameBuff[UART1_RcvCnt]=USART_ReceiveData(USART1);	//读取接收到的数据
		UART1_RcvCnt++;
		timerRcvCnt1=0;
  } 
} 
#endif

/*******************************************************************************
//串口2接收中断
//当接收到数据时，将数据存储到缓冲区。如果缓冲区满，则调用回调函数并清空缓冲区。
*******************************************************************************/
#if EN_USART2_RX   //如果使能了接收
void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		if(UART2_RcvCnt>=UART2_FRAMEBUF_SIZE)
		{
			UART2_rxCallback(UART2_FrameBuff,UART2_RcvCnt);//向串口回调函数中赋值
			memset(UART2_FrameBuff,NULL,sizeof(UART2_FrameBuff));//清空数组
			UART2_RcvCnt=0;
		}
		UART2_FrameBuff[UART2_RcvCnt]=USART_ReceiveData(USART2);	//读取接收到的数据		
		UART2_RcvCnt++;
		timerRcvCnt2=0;
  } 
} 
#endif 

/*******************************************************************************
//串口3接收中断
首先检查当前接收计数器 UART3_RcvCnt 是否已达到帧缓冲区大小 UART3_FRAMEBUF_SIZE 的限制。
如果是，则调用回调函数 UART3_rxCallback() 处理用户数据并清空缓冲区。
从USART3读取接收到的数据存储在 UART3_FrameBuff 缓冲区，并增加接收计数器。
同时，数据也被复制到两个额外的缓冲区 CAT_RxBuffer 和 WIFI_RxBuffer 中，这些缓冲区用于不同的模块目的（如CAT-1和Wi-Fi通信）。
在复制之前，每个缓冲区都会检查其计数器是否达到最大值（255），如果是，则重置对应的缓冲区。
*******************************************************************************/
#if EN_USART3_RX   //如果使能了接收
void USART3_IRQHandler(void)                	//串口3中断服务程序
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		if(UART3_RcvCnt>=UART3_FRAMEBUF_SIZE)
		{
			UART3_rxCallback(UART3_FrameBuff,UART3_RcvCnt);//向串口回调函数中赋值
			memset(UART3_FrameBuff,NULL,sizeof(UART3_FrameBuff));//清空数组
			UART3_RcvCnt=0;
		}
		UART3_FrameBuff[UART3_RcvCnt]=USART_ReceiveData(USART3);	//读取接收到的数据
		UART3_RcvCnt++;
		timerRcvCnt3=0;
  }
}
#endif

/*******************************************************************************
//串口4接收中断
//当接收到数据时，将数据存储到缓冲区。如果缓冲区满，则调用回调函数并清空缓冲区。
*******************************************************************************/
#if EN_USART4_RX   //如果使能了接收
void UART4_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断
	{
		if(UART4_RcvCnt>=UART4_FRAMEBUF_SIZE)
		{
			UART4_rxCallback(UART4_FrameBuff,UART4_RcvCnt);//向串口回调函数中赋值
			memset(UART4_FrameBuff,NULL,sizeof(UART4_FrameBuff));//清空数组
			UART4_RcvCnt=0;
		}
		UART4_FrameBuff[UART4_RcvCnt]=USART_ReceiveData(UART4);	//读取接收到的数据
		UART4_RcvCnt++;
		timerRcvCnt4=0;
  } 
}
#endif

/*******************************************************************************
//串口5接收中断
//当接收到数据时，将数据存储到缓冲区。如果缓冲区满，则调用回调函数并清空缓冲区。
*******************************************************************************/
#if EN_USART5_RX   //如果使能了接收
void UART5_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //接收中断
	{
		if(UART5_RcvCnt>=UART5_FRAMEBUF_SIZE)
		{
			UART5_rxCallback(UART5_FrameBuff,UART5_RcvCnt);//向串口回调函数中赋值
			memset(UART5_FrameBuff,NULL,sizeof(UART5_FrameBuff));//清空数组
			UART5_RcvCnt=0;
		}
		UART5_FrameBuff[UART5_RcvCnt]=USART_ReceiveData(UART5);	//读取接收到的数据
		UART5_RcvCnt++;
		timerRcvCnt5=0;
  } 
}
#endif
/*******************************************************************************
//串口5接收中断
//当接收到数据时，将数据存储到缓冲区。如果缓冲区满，则调用回调函数并清空缓冲区。
*******************************************************************************/
#if EN_USART6_RX   //如果使能了接收
void USART6_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断
	{
		if(UART6_RcvCnt>=UART6_FRAMEBUF_SIZE)
		{
			UART6_rxCallback(UART6_FrameBuff,UART6_RcvCnt);//向串口回调函数中赋值
			memset(UART6_FrameBuff,NULL,sizeof(UART6_FrameBuff));//清空数组
			UART6_RcvCnt=0;
		}
		UART6_FrameBuff[UART6_RcvCnt]=USART_ReceiveData(USART6);	//读取接收到的数据
		UART6_RcvCnt++;
		timerRcvCnt6=0;
  } 
}
#endif


/*******************************************************************************
//串口1发送一个字节
*******************************************************************************/
void UART1_SendByte(u8 data)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){};//等待发送结束
  USART_SendData(USART1, data);//向串口1发送数据
}

/*******************************************************************************
//串口2发送一个字节
*******************************************************************************/
void UART2_SendByte(u8 data)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){};//等待发送结束
  USART_SendData(USART2, data);//向串口2发送数据
}

/*******************************************************************************
//串口3发送一个字节
*******************************************************************************/
void UART3_SendByte(u8 data)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){};//等待发送结束
  USART_SendData(USART3, data);//向串口3发送数据
}

/*******************************************************************************
//串口4发送一个字节
*******************************************************************************/
void UART4_SendByte(u8 data)
{
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){};//等待发送结束
  USART_SendData(UART4, data);//向串口3发送数据
}
/*******************************************************************************
//串口5发送一个字节
*******************************************************************************/
void UART5_SendByte(u8 data)
{
	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET){};//等待发送结束
  USART_SendData(UART5, data);//向串口3发送数据
}
/*******************************************************************************
//串口6发送一个字节
*******************************************************************************/
void USART6_SendByte(u8 data)
{
	while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET){};//等待发送结束
  USART_SendData(USART6, data);//向串口3发送数据
}

/*******************************************************************************
//串口1发送多个字节
*******************************************************************************/
void UART1_printf(u8 *ptr,u16 len)
{
  int i=0;
  for(i=0;i<len;i++)
  {
    UART1_SendByte(*ptr);
    ptr++;
  }
  while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
}

/*******************************************************************************
//串口2发送多个字节
*******************************************************************************/
void UART2_printf(u8 *ptr,u16 len)
{
  int i=0;
  for(i=0;i<len;i++)
  {
    UART2_SendByte(*ptr);
    ptr++;
  }
  while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET);
}

/*******************************************************************************
//串口3发送多个字节
*******************************************************************************/
void UART3_printf(u8 *ptr,u16 len)
{
  int i=0;
  for(i=0;i<len;i++)
  {
    UART3_SendByte(*ptr);
    ptr++;
  }
  while(USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
}

/*******************************************************************************
//串口3发送多个字节
*******************************************************************************/
void UART4_printf(u8 *ptr,u16 len)
{
  int i=0;
  for(i=0;i<len;i++)
  {
    UART4_SendByte(*ptr);
    ptr++;
  }
  while(USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET);
}
