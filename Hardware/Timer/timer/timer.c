#include "timer.h"


/*******************************************************************************
//串口1变量定义
*******************************************************************************/
extern u8 UART1_FrameBuff[UART1_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
extern u16 UART1_RcvCnt;                     /*接收计数*/
extern u32 timerRcvCnt1;                     /*串口定时器计数*/    
/*******************************************************************************
//串口2变量定义
*******************************************************************************/
extern u8 UART2_FrameBuff[UART2_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
extern u16 UART2_RcvCnt;                     /*接收计数*/
extern u32 timerRcvCnt2;                     /*串口定时器计数*/
/*******************************************************************************
//串口3变量定义
*******************************************************************************/
extern u8 UART3_FrameBuff[UART3_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
extern u16 UART3_RcvCnt;                     /*接收计数*/
extern u32 timerRcvCnt3;                     /*串口定时器计数*/
/*******************************************************************************
//串口4变量定义
*******************************************************************************/
extern u8 UART4_FrameBuff[UART4_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
extern u16 UART4_RcvCnt;                     /*接收计数*/
extern u32 timerRcvCnt4;                     /*串口定时器计数*/
/*******************************************************************************
//串口5变量定义
*******************************************************************************/
extern u8 UART5_FrameBuff[UART5_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
extern u16 UART5_RcvCnt;                     /*接收计数*/
extern u32 timerRcvCnt5;                     /*串口定时器计数*/
/*******************************************************************************
//串口6变量定义
*******************************************************************************/
extern u8 UART6_FrameBuff[UART6_FRAMEBUF_SIZE];/*接收帧缓冲区数组*/
extern u16 UART6_RcvCnt;                     /*接收计数*/
extern u32 timerRcvCnt6;                     /*串口定时器计数*/



/**
  * @brief  基本定时器 TIMx,x[6,7]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 		
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQn; 	
		// 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 
	  // 设置子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
static void TIM7_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // 设置中断组为0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; 	
		// 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 
	  // 设置子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	
	

	// 开启TIMx_CLK,x[6,7] 
  RCC_APB1PeriphClockCmd(BASIC_TIM_CLK, ENABLE); 

  /* 累计 TIM_Period个后产生一个更新或者中断*/		
  //当定时器从0计数到10，即为10次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Period = 10-1;       
	
	//定时器时钟源TIMxCLK = 2 * PCLK1  
  //				PCLK1 = HCLK / 4 
  //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;	
	
	// 初始化定时器TIMx, x[2,3,4,5]
	TIM_TimeBaseInit(BASIC_TIM, &TIM_TimeBaseStructure);
	
	
	// 清除定时器更新中断标志位
	TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);
	
	// 开启定时器更新中断
	TIM_ITConfig(BASIC_TIM,TIM_IT_Update,ENABLE);
	
	// 使能定时器
	TIM_Cmd(BASIC_TIM, ENABLE);	
}


void TIM_Pid_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	// 开启TIMx_CLK,x[6,7] 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 

  /* 累计 TIM_Period个后产生一个更新或者中断*/		
  //当定时器从0计数到10，即为10次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Period = 500-1;       
	
	//定时器时钟源TIMxCLK = 2 * PCLK1  
  //				PCLK1 = HCLK / 4 
  //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;	
	
	// 初始化定时器TIMx, x[2,3,4,5]
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	
	// 清除定时器更新中断标志位
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	
	// 开启定时器更新中断
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	
	// 使能定时器
	TIM_Cmd(TIM7, ENABLE);	
}


/* TIM6中断处理函数 - 处理软件定时器标志位和串口超时 */
void TIM6_DAC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        /*****************************************************************************
		//UART1 超时处理
		*****************************************************************************/
		#if EN_USART1_RX   //如果使能了接收
		if(UART1_RcvCnt!=0)
		{
			if(timerRcvCnt1>=UART1_IDLELINE_TIME)
			{
				timerRcvCnt1=0;
				UART1_rxCallback(UART1_FrameBuff,UART1_RcvCnt);//向串口回调函数中赋值
				memset(UART1_FrameBuff,0,sizeof(UART1_FrameBuff));//清空数组
				UART1_RcvCnt=0;	
			}
			else
			{
				timerRcvCnt1++;
			}
		}
		#endif
		/*****************************************************************************
		//UART2 超时处理
		*****************************************************************************/
		#if EN_USART2_RX   //如果使能了接收
		if(UART2_RcvCnt!=0)
		{
			if(timerRcvCnt2>=UART2_IDLELINE_TIME)
			{
				UART2_rxCallback(UART2_FrameBuff,UART2_RcvCnt);//向串口回调函数中赋值
				memset(UART2_FrameBuff,NULL,sizeof(UART2_FrameBuff));//清空数组
				UART2_RcvCnt=0;
			}
			else
			{
				timerRcvCnt2++;
			}
		}
		#endif
		/*****************************************************************************
		//UART3 超时处理
		*****************************************************************************/
		#if EN_USART3_RX   //如果使能了接收
		if(UART3_RcvCnt!=0)
		{
			if(timerRcvCnt3>=UART3_IDLELINE_TIME)
			{
				UART3_rxCallback(UART3_FrameBuff,UART3_RcvCnt);//向串口回调函数中赋值
				memset(UART3_FrameBuff,NULL,sizeof(UART3_FrameBuff));//清空数组
				UART3_RcvCnt=0;
			}
			else
			{
				timerRcvCnt3++;
			}
		}
		#endif
		/*****************************************************************************
		//UART4 超时处理
		*****************************************************************************/
		#if EN_USART4_RX   //如果使能了接收
		if(UART4_RcvCnt!=0)
		{
			if(timerRcvCnt4>=UART4_IDLELINE_TIME)
			{
				UART4_rxCallback(UART4_FrameBuff,UART4_RcvCnt);//向串口回调函数中赋值
				memset(UART4_FrameBuff,NULL,sizeof(UART4_FrameBuff));//清空数组
				UART4_RcvCnt=0;
			}
			else
			{
				timerRcvCnt4++;
			}
		}
		#endif    
		/*****************************************************************************
		//UART5 超时处理
		*****************************************************************************/
		#if EN_USART5_RX   //如果使能了接收
		if(UART5_RcvCnt!=0)
		{
			if(timerRcvCnt5>=UART5_IDLELINE_TIME)
			{
				UART5_rxCallback(UART5_FrameBuff,UART5_RcvCnt);//向串口回调函数中赋值
				memset(UART5_FrameBuff,NULL,sizeof(UART5_FrameBuff));//清空数组
				UART5_RcvCnt=0;
			}
			else
			{
				timerRcvCnt5++;
			}
		}
		#endif
		/*****************************************************************************
		//UART6 超时处理
		*****************************************************************************/
		#if EN_USART6_RX   //如果使能了接收
		if(UART6_RcvCnt!=0)
		{
			if(timerRcvCnt6>=UART6_IDLELINE_TIME)
			{
				UART6_rxCallback(UART6_FrameBuff,UART6_RcvCnt);//向串口回调函数中赋值
				memset(UART6_FrameBuff,NULL,sizeof(UART6_FrameBuff));//清空数组
				UART6_RcvCnt=0;
			}
			else
			{
				timerRcvCnt6++;
			}
		}
		#endif
    }
}


void TIM7_IRQHandler(void)
{
	if(TIM_GetFlagStatus(TIM7, TIM_FLAG_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		PID_PWM = Yaw_PID_Control();
	}
}

/**
  * @brief  初始化基本定时器定时，1ms产生一次中断
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
	TIMx_NVIC_Configuration();	
  
  TIM_Mode_Config();
}

/**
  * @brief  初始化基本定时器7定时，50ms产生一次中断
  * @param  无
  * @retval 无
  */
void TIM7_Configuration(void)
{
	TIM7_NVIC_Configuration();	
  
  TIM_Pid_Config();
}



