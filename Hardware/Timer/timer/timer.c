#include "timer.h"


/*******************************************************************************
//����1��������
*******************************************************************************/
extern u8 UART1_FrameBuff[UART1_FRAMEBUF_SIZE];/*����֡����������*/
extern u16 UART1_RcvCnt;                     /*���ռ���*/
extern u32 timerRcvCnt1;                     /*���ڶ�ʱ������*/    
/*******************************************************************************
//����2��������
*******************************************************************************/
extern u8 UART2_FrameBuff[UART2_FRAMEBUF_SIZE];/*����֡����������*/
extern u16 UART2_RcvCnt;                     /*���ռ���*/
extern u32 timerRcvCnt2;                     /*���ڶ�ʱ������*/
/*******************************************************************************
//����3��������
*******************************************************************************/
extern u8 UART3_FrameBuff[UART3_FRAMEBUF_SIZE];/*����֡����������*/
extern u16 UART3_RcvCnt;                     /*���ռ���*/
extern u32 timerRcvCnt3;                     /*���ڶ�ʱ������*/
/*******************************************************************************
//����4��������
*******************************************************************************/
extern u8 UART4_FrameBuff[UART4_FRAMEBUF_SIZE];/*����֡����������*/
extern u16 UART4_RcvCnt;                     /*���ռ���*/
extern u32 timerRcvCnt4;                     /*���ڶ�ʱ������*/
/*******************************************************************************
//����5��������
*******************************************************************************/
extern u8 UART5_FrameBuff[UART5_FRAMEBUF_SIZE];/*����֡����������*/
extern u16 UART5_RcvCnt;                     /*���ռ���*/
extern u32 timerRcvCnt5;                     /*���ڶ�ʱ������*/
/*******************************************************************************
//����6��������
*******************************************************************************/
extern u8 UART6_FrameBuff[UART6_FRAMEBUF_SIZE];/*����֡����������*/
extern u16 UART6_RcvCnt;                     /*���ռ���*/
extern u32 timerRcvCnt6;                     /*���ڶ�ʱ������*/



/**
  * @brief  ������ʱ�� TIMx,x[6,7]�ж����ȼ�����
  * @param  ��
  * @retval ��
  */
static void TIMx_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 		
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQn; 	
		// ������ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 
	  // ���������ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
static void TIM7_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // �����ж���Ϊ0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; 	
		// ������ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 
	  // ���������ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         ����
 * TIM_CounterMode			 TIMx,x[6,7]û�У��������У�������ʱ����
 * TIM_Period            ����
 * TIM_ClockDivision     TIMx,x[6,7]û�У���������(������ʱ��)
 * TIM_RepetitionCounter TIMx,x[1,8]����(�߼���ʱ��)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	
	

	// ����TIMx_CLK,x[6,7] 
  RCC_APB1PeriphClockCmd(BASIC_TIM_CLK, ENABLE); 

  /* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������10����Ϊ10�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Period = 10-1;       
	
	//��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
  //				PCLK1 = HCLK / 4 
  //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=10000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;	
	
	// ��ʼ����ʱ��TIMx, x[2,3,4,5]
	TIM_TimeBaseInit(BASIC_TIM, &TIM_TimeBaseStructure);
	
	
	// �����ʱ�������жϱ�־λ
	TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);
	
	// ������ʱ�������ж�
	TIM_ITConfig(BASIC_TIM,TIM_IT_Update,ENABLE);
	
	// ʹ�ܶ�ʱ��
	TIM_Cmd(BASIC_TIM, ENABLE);	
}


void TIM_Pid_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	// ����TIMx_CLK,x[6,7] 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 

  /* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������10����Ϊ10�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Period = 500-1;       
	
	//��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
  //				PCLK1 = HCLK / 4 
  //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=10000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;	
	
	// ��ʼ����ʱ��TIMx, x[2,3,4,5]
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	
	// �����ʱ�������жϱ�־λ
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	
	// ������ʱ�������ж�
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	
	// ʹ�ܶ�ʱ��
	TIM_Cmd(TIM7, ENABLE);	
}


/* TIM6�жϴ����� - ���������ʱ����־λ�ʹ��ڳ�ʱ */
void TIM6_DAC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        /*****************************************************************************
		//UART1 ��ʱ����
		*****************************************************************************/
		#if EN_USART1_RX   //���ʹ���˽���
		if(UART1_RcvCnt!=0)
		{
			if(timerRcvCnt1>=UART1_IDLELINE_TIME)
			{
				timerRcvCnt1=0;
				UART1_rxCallback(UART1_FrameBuff,UART1_RcvCnt);//�򴮿ڻص������и�ֵ
				memset(UART1_FrameBuff,0,sizeof(UART1_FrameBuff));//�������
				UART1_RcvCnt=0;	
			}
			else
			{
				timerRcvCnt1++;
			}
		}
		#endif
		/*****************************************************************************
		//UART2 ��ʱ����
		*****************************************************************************/
		#if EN_USART2_RX   //���ʹ���˽���
		if(UART2_RcvCnt!=0)
		{
			if(timerRcvCnt2>=UART2_IDLELINE_TIME)
			{
				UART2_rxCallback(UART2_FrameBuff,UART2_RcvCnt);//�򴮿ڻص������и�ֵ
				memset(UART2_FrameBuff,NULL,sizeof(UART2_FrameBuff));//�������
				UART2_RcvCnt=0;
			}
			else
			{
				timerRcvCnt2++;
			}
		}
		#endif
		/*****************************************************************************
		//UART3 ��ʱ����
		*****************************************************************************/
		#if EN_USART3_RX   //���ʹ���˽���
		if(UART3_RcvCnt!=0)
		{
			if(timerRcvCnt3>=UART3_IDLELINE_TIME)
			{
				UART3_rxCallback(UART3_FrameBuff,UART3_RcvCnt);//�򴮿ڻص������и�ֵ
				memset(UART3_FrameBuff,NULL,sizeof(UART3_FrameBuff));//�������
				UART3_RcvCnt=0;
			}
			else
			{
				timerRcvCnt3++;
			}
		}
		#endif
		/*****************************************************************************
		//UART4 ��ʱ����
		*****************************************************************************/
		#if EN_USART4_RX   //���ʹ���˽���
		if(UART4_RcvCnt!=0)
		{
			if(timerRcvCnt4>=UART4_IDLELINE_TIME)
			{
				UART4_rxCallback(UART4_FrameBuff,UART4_RcvCnt);//�򴮿ڻص������и�ֵ
				memset(UART4_FrameBuff,NULL,sizeof(UART4_FrameBuff));//�������
				UART4_RcvCnt=0;
			}
			else
			{
				timerRcvCnt4++;
			}
		}
		#endif    
		/*****************************************************************************
		//UART5 ��ʱ����
		*****************************************************************************/
		#if EN_USART5_RX   //���ʹ���˽���
		if(UART5_RcvCnt!=0)
		{
			if(timerRcvCnt5>=UART5_IDLELINE_TIME)
			{
				UART5_rxCallback(UART5_FrameBuff,UART5_RcvCnt);//�򴮿ڻص������и�ֵ
				memset(UART5_FrameBuff,NULL,sizeof(UART5_FrameBuff));//�������
				UART5_RcvCnt=0;
			}
			else
			{
				timerRcvCnt5++;
			}
		}
		#endif
		/*****************************************************************************
		//UART6 ��ʱ����
		*****************************************************************************/
		#if EN_USART6_RX   //���ʹ���˽���
		if(UART6_RcvCnt!=0)
		{
			if(timerRcvCnt6>=UART6_IDLELINE_TIME)
			{
				UART6_rxCallback(UART6_FrameBuff,UART6_RcvCnt);//�򴮿ڻص������и�ֵ
				memset(UART6_FrameBuff,NULL,sizeof(UART6_FrameBuff));//�������
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
  * @brief  ��ʼ��������ʱ����ʱ��1ms����һ���ж�
  * @param  ��
  * @retval ��
  */
void TIMx_Configuration(void)
{
	TIMx_NVIC_Configuration();	
  
  TIM_Mode_Config();
}

/**
  * @brief  ��ʼ��������ʱ��7��ʱ��50ms����һ���ж�
  * @param  ��
  * @retval ��
  */
void TIM7_Configuration(void)
{
	TIM7_NVIC_Configuration();	
  
  TIM_Pid_Config();
}



