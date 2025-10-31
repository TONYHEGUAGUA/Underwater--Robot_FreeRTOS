#include "usart.h"	  
#include "string.h"
#include "sys.h"
/* ���ļ���Ҫ�����úͲ����������ڣ�USART1, USART2, USART3��
	������ʼ�����жϴ������ݷ��͡�
	ÿ�����ڶ��ж����Ļ������ͼ��������洢�������ݣ���
	��ͨ���жϴ����������ݴ��ݸ��ص�������
	���⣬���ṩ�˷��͵����ֽںͶ���ֽڵĺ�����
*/

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
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
//����1��������
*******************************************************************************/
u8 UART1_FrameBuff[UART1_FRAMEBUF_SIZE];/*����֡����������*/
u16 UART1_RcvCnt=0;                     /*���ռ�����*/
u32 timerRcvCnt1=0;                     /*���ڶ�ʱ������*/    
/*******************************************************************************
//����2��������
*******************************************************************************/
u8 UART2_FrameBuff[UART2_FRAMEBUF_SIZE];/*����֡����������*/
u16 UART2_RcvCnt=0;                     /*���ռ�����*/
u32 timerRcvCnt2=0;                     /*���ڶ�ʱ������*/
/*******************************************************************************
//����3��������
*******************************************************************************/
u8 UART3_FrameBuff[UART3_FRAMEBUF_SIZE];/*����֡����������*/
u16 UART3_RcvCnt;                     /*���ռ�����*/
u32 timerRcvCnt3=0;                     /*���ڶ�ʱ������*/
/*******************************************************************************
//����4��������
*******************************************************************************/
u8 UART4_FrameBuff[UART4_FRAMEBUF_SIZE];/*����֡����������*/
u16 UART4_RcvCnt=0;                     /*���ռ�����*/
u32 timerRcvCnt4=0;                     /*���ڶ�ʱ������*/
/*******************************************************************************
//����5��������
*******************************************************************************/
u8 UART5_FrameBuff[UART5_FRAMEBUF_SIZE];/*����֡����������*/
u16 UART5_RcvCnt=0;                     /*���ռ�����*/
u32 timerRcvCnt5=0;                     /*���ڶ�ʱ������*/
/*******************************************************************************
//����6��������
*******************************************************************************/
u8 UART6_FrameBuff[UART6_FRAMEBUF_SIZE];/*����֡����������*/
u16 UART6_RcvCnt=0;                     /*���ռ�����*/
u32 timerRcvCnt6=0;                     /*���ڶ�ʱ������*/


/*******************************************************************************
//16����תASCII
*******************************************************************************/
u8 Byte_HexDispaly(u8 dat)
{
	if(dat>=10&&dat<16) dat+=0x07;	    //A-F +0x37
	dat+=0x30;						   						//0-9 +0x30
	return dat;
}

/*******************************************************************************
//����1��ʼ��
*******************************************************************************/
void Uart1_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 1. ����GPIOA��USART1ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	/* 2. USART1 ��Ӧ���Ÿ���ӳ�� */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	/* 3. USART1 �˿����� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10
		
    /* 4. USART1 ��ʼ������ */
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	/* 5. UART1 NVIC ���� */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);				//��������ж�
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;	//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;			//�����ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);								//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	 /* 5. ʹ��USART1 */
	USART_Cmd(USART1, ENABLE); 
}

/*******************************************************************************
//����2��ʼ��
*******************************************************************************/	
void Uart2_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. ʹ��GPIOD��USART2ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ʹ��GPIODʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // ʹ��USART2ʱ��

    /* 2. USART2 ��Ӧ���Ÿ���ӳ�� */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // GPIOD5����ΪUSART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // GPIOD6����ΪUSART2

    /* 3.USART2 �˿����� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // GPIOD5��GPIOD6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // �ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // ���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
    GPIO_Init(GPIOA, &GPIO_InitStructure); // ��ʼ��GPIOD5��GPIOD6

    /* 4. USART2 ��ʼ������ */
    USART_InitStructure.USART_BaudRate = bound; // ����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No; // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(USART2, &USART_InitStructure); // ��ʼ������2
	
	/* 5. UART2 NVIC ���� */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//��������ж�
	
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//�����ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	/* 6. ʹ��UART2 */
    USART_Cmd(USART2, ENABLE); // ʹ�ܴ���2
}

/*******************************************************************************
//����3��ʼ��
*******************************************************************************/
void Uart3_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 1. ʹ��GPIOC��USART3ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // ʹ��USART3ʱ��
 
	/* 2. USART3 ��Ӧ���Ÿ���ӳ�� */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10����ΪUSART3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11����ΪUSART3
	
	/* 3.USART3 �˿����� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA2��PA3

	/* 4. USART3 ��ʼ������ */
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3 

	/* 5. UART3 NVIC ���� */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			//��������ж�
	
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;	//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//�����ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	/* 6. ʹ��UART3 */
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3
}

/*******************************************************************************
//����4��ʼ��
*******************************************************************************/
void Uart4_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 1. ʹ��GPIOC��UART4ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); // ʹ��UART4ʱ��
 
	/* 2. UART4 ��Ӧ���Ÿ���ӳ�� */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART5); // GPIOD10����ΪUART4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART5); // GPIOC11����ΪUART4
	
	/* 3.UART4 �˿����� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	 
	/* 4. UART4 ��ʼ������ */
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(UART4, &USART_InitStructure); //��ʼ������4
	
	/* 5. UART4 NVIC ���� */
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);           //��������ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;        //����5�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//�����ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��NVIC�Ĵ���

	/* 6. ʹ��UART4 */
	USART_Cmd(UART4, ENABLE);
}
/*******************************************************************************
//����5��ʼ��
*******************************************************************************/
void Uart5_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 1. ʹ��GPIOC��UART5ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE); // ʹ��GPIOCʱ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); // ʹ��UART5ʱ��
 
	/* 2. UART5 ��Ӧ���Ÿ���ӳ�� */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);  // GPIOD2����ΪUART5
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); // GPIOC12����ΪUART5
	
	/* 3.UART5 �˿����� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_12; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
	 
   /* 4. UART5 ��ʼ������ */
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(UART5, &USART_InitStructure); //��ʼ������5
	
	/* 5. UART5 NVIC ���� */
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);          //��������ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;        //����5�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//�����ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	
	/* 6. ʹ��UART5 */
	USART_Cmd(UART5, ENABLE);
}

/*******************************************************************************
//����6��ʼ��
*******************************************************************************/
void Uart6_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. ʹ��GPIOC ��UASRT6ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // ʹ��GPIOCʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); // ʹ��USART6ʱ��

    /* 2. USART6 ��Ӧ���Ÿ���ӳ�� */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); // GPIOC6����ΪUSART6_TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); // GPIOC7����ΪUSART6_RX

    /* 3. USART6 �˿����� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // GPIOC6��GPIOC7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // �ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // ���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
    GPIO_Init(GPIOC, &GPIO_InitStructure); // ��ʼ��GPIOC6��GPIOC7

    /* 4. USART6 ��ʼ������ */
    USART_InitStructure.USART_BaudRate = bound; // ����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No; // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(USART6, &USART_InitStructure); // ��ʼ������6
	
	/* 5. Usart6 NVIC ���� */
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);          //��������ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;       //����6�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;		//�����ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	
	/* 6. ʹ��USART6 */
    USART_Cmd(USART6, ENABLE); 
}


/*******************************************************************************
//����1�����ж�
//�����յ�����ʱ�������ݴ洢���������������������������ûص���������ջ�������
*******************************************************************************/
#if EN_USART1_RX   //���ʹ���˽���
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
		if(UART1_RcvCnt>=UART1_FRAMEBUF_SIZE)
		{
			UART1_rxCallback(UART1_FrameBuff,UART1_RcvCnt);//�򴮿ڻص������и�ֵ
			memset(UART1_FrameBuff,0,sizeof(UART1_FrameBuff));//�������
			UART1_RcvCnt=0;
		}
		UART1_FrameBuff[UART1_RcvCnt]=USART_ReceiveData(USART1);	//��ȡ���յ�������
		UART1_RcvCnt++;
		timerRcvCnt1=0;
  } 
} 
#endif

/*******************************************************************************
//����2�����ж�
//�����յ�����ʱ�������ݴ洢���������������������������ûص���������ջ�������
*******************************************************************************/
#if EN_USART2_RX   //���ʹ���˽���
void USART2_IRQHandler(void)                	//����2�жϷ������
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
	{
		if(UART2_RcvCnt>=UART2_FRAMEBUF_SIZE)
		{
			UART2_rxCallback(UART2_FrameBuff,UART2_RcvCnt);//�򴮿ڻص������и�ֵ
			memset(UART2_FrameBuff,NULL,sizeof(UART2_FrameBuff));//�������
			UART2_RcvCnt=0;
		}
		UART2_FrameBuff[UART2_RcvCnt]=USART_ReceiveData(USART2);	//��ȡ���յ�������		
		UART2_RcvCnt++;
		timerRcvCnt2=0;
  } 
} 
#endif 

/*******************************************************************************
//����3�����ж�
���ȼ�鵱ǰ���ռ����� UART3_RcvCnt �Ƿ��Ѵﵽ֡��������С UART3_FRAMEBUF_SIZE �����ơ�
����ǣ�����ûص����� UART3_rxCallback() �����û����ݲ���ջ�������
��USART3��ȡ���յ������ݴ洢�� UART3_FrameBuff �������������ӽ��ռ�������
ͬʱ������Ҳ�����Ƶ���������Ļ����� CAT_RxBuffer �� WIFI_RxBuffer �У���Щ���������ڲ�ͬ��ģ��Ŀ�ģ���CAT-1��Wi-Fiͨ�ţ���
�ڸ���֮ǰ��ÿ����������������������Ƿ�ﵽ���ֵ��255��������ǣ������ö�Ӧ�Ļ�������
*******************************************************************************/
#if EN_USART3_RX   //���ʹ���˽���
void USART3_IRQHandler(void)                	//����3�жϷ������
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
	{
		if(UART3_RcvCnt>=UART3_FRAMEBUF_SIZE)
		{
			UART3_rxCallback(UART3_FrameBuff,UART3_RcvCnt);//�򴮿ڻص������и�ֵ
			memset(UART3_FrameBuff,NULL,sizeof(UART3_FrameBuff));//�������
			UART3_RcvCnt=0;
		}
		UART3_FrameBuff[UART3_RcvCnt]=USART_ReceiveData(USART3);	//��ȡ���յ�������
		UART3_RcvCnt++;
		timerRcvCnt3=0;
  }
}
#endif

/*******************************************************************************
//����4�����ж�
//�����յ�����ʱ�������ݴ洢���������������������������ûص���������ջ�������
*******************************************************************************/
#if EN_USART4_RX   //���ʹ���˽���
void UART4_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //�����ж�
	{
		if(UART4_RcvCnt>=UART4_FRAMEBUF_SIZE)
		{
			UART4_rxCallback(UART4_FrameBuff,UART4_RcvCnt);//�򴮿ڻص������и�ֵ
			memset(UART4_FrameBuff,NULL,sizeof(UART4_FrameBuff));//�������
			UART4_RcvCnt=0;
		}
		UART4_FrameBuff[UART4_RcvCnt]=USART_ReceiveData(UART4);	//��ȡ���յ�������
		UART4_RcvCnt++;
		timerRcvCnt4=0;
  } 
}
#endif

/*******************************************************************************
//����5�����ж�
//�����յ�����ʱ�������ݴ洢���������������������������ûص���������ջ�������
*******************************************************************************/
#if EN_USART5_RX   //���ʹ���˽���
void UART5_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //�����ж�
	{
		if(UART5_RcvCnt>=UART5_FRAMEBUF_SIZE)
		{
			UART5_rxCallback(UART5_FrameBuff,UART5_RcvCnt);//�򴮿ڻص������и�ֵ
			memset(UART5_FrameBuff,NULL,sizeof(UART5_FrameBuff));//�������
			UART5_RcvCnt=0;
		}
		UART5_FrameBuff[UART5_RcvCnt]=USART_ReceiveData(UART5);	//��ȡ���յ�������
		UART5_RcvCnt++;
		timerRcvCnt5=0;
  } 
}
#endif
/*******************************************************************************
//����5�����ж�
//�����յ�����ʱ�������ݴ洢���������������������������ûص���������ջ�������
*******************************************************************************/
#if EN_USART6_RX   //���ʹ���˽���
void USART6_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //�����ж�
	{
		if(UART6_RcvCnt>=UART6_FRAMEBUF_SIZE)
		{
			UART6_rxCallback(UART6_FrameBuff,UART6_RcvCnt);//�򴮿ڻص������и�ֵ
			memset(UART6_FrameBuff,NULL,sizeof(UART6_FrameBuff));//�������
			UART6_RcvCnt=0;
		}
		UART6_FrameBuff[UART6_RcvCnt]=USART_ReceiveData(USART6);	//��ȡ���յ�������
		UART6_RcvCnt++;
		timerRcvCnt6=0;
  } 
}
#endif


/*******************************************************************************
//����1����һ���ֽ�
*******************************************************************************/
void UART1_SendByte(u8 data)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){};//�ȴ����ͽ���
  USART_SendData(USART1, data);//�򴮿�1��������
}

/*******************************************************************************
//����2����һ���ֽ�
*******************************************************************************/
void UART2_SendByte(u8 data)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){};//�ȴ����ͽ���
  USART_SendData(USART2, data);//�򴮿�2��������
}

/*******************************************************************************
//����3����һ���ֽ�
*******************************************************************************/
void UART3_SendByte(u8 data)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){};//�ȴ����ͽ���
  USART_SendData(USART3, data);//�򴮿�3��������
}

/*******************************************************************************
//����4����һ���ֽ�
*******************************************************************************/
void UART4_SendByte(u8 data)
{
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){};//�ȴ����ͽ���
  USART_SendData(UART4, data);//�򴮿�3��������
}
/*******************************************************************************
//����5����һ���ֽ�
*******************************************************************************/
void UART5_SendByte(u8 data)
{
	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET){};//�ȴ����ͽ���
  USART_SendData(UART5, data);//�򴮿�3��������
}
/*******************************************************************************
//����6����һ���ֽ�
*******************************************************************************/
void USART6_SendByte(u8 data)
{
	while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET){};//�ȴ����ͽ���
  USART_SendData(USART6, data);//�򴮿�3��������
}

/*******************************************************************************
//����1���Ͷ���ֽ�
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
//����2���Ͷ���ֽ�
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
//����3���Ͷ���ֽ�
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
//����3���Ͷ���ֽ�
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
