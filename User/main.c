#include "main.h"

int PID_PWM=0;

//TONYHEGUAGUA夺回控制权
/******************关于imu的变量************************/
float imu_yaw;
/*******************************************************/

/******************关于超声波的变量*********************/
uint16_t sonic_uart2;
uint16_t sonic_uart4;
uint16_t sonic_uart5;
uint16_t sonic_uart6;
/*******************************************************/


/****************关于电机速度的变量*******************************/
uint8_t Left_Motor_Speed_Global = 100;
uint8_t Right_Motor_Speed_Global = 100;
uint8_t Top_Motor_Speed_Global = 100;
/*******************************************************/

/**************** 蓝牙模块(usart3)有关的变量 *************/

uint8_t BLE_Update_Flag = 0;

typedef enum
{
	Forward_Flag = 1,
	Backward_Flag,
	Left_Flag,
	Right_Flag,
	Open_Top_Motor_Flag,
	Close_Top_Motor_Flag,
	Speed_Up_Flag,
	Slow_Down_Flag,
	Stop_Motor_Flag,
	Top_Stop_Flag
}BLE_Motor_State;
BLE_Motor_State BLE_Moter_Flag;
/*******************************************************/




// 初始化和创建任务
#define Start_Task_Priority 1
#define Start_Stack_Size 128
TaskHandle_t Start_Task_Handler;
void Start_Task(void * pvParameters );

// 获取进水传感器数据
#define Water_Task_Priority 1
#define Water_Stack_Size 128
TaskHandle_t Water_Task_Handler;
void Water_Task(void * pvParameters );

// 超声波传感器获取数据任务
#define Ultrasonic_Task_Priority 1
#define Ultrasonic_Stack_Size 128
TaskHandle_t Ultrasonic_Task_Handler;
void Ultrasonic_Task( void * pvParameters );

// 超声波数据处理任务
#define Ultrasonic_Process_Task_Priority 1
#define Ultrasonic_Process_Stack_Size 128
TaskHandle_t Ultrasonic_Process_Task_Handler;
void Ultrasonic_Process_Task( void * pvParameters );

//移动前后左右转弯控制任务
#define Move_Task_Priority 1
#define Move_Stack_Size 128
TaskHandle_t Move_Task_Handler;
void Move_Task( void * pvParameters );

//IMU任务
#define IMU_Task_Priority 3
#define IMU_Task_Stack_Size 512
TaskHandle_t IMU_Task_Handler;
void IMU_Task( void * pvParameters );

// IMU 20Hz UART3 transmit
#define IMU_Tx_Task_Priority 2
#define IMU_Tx_Task_Stack_Size 512  // 增加栈大小，避免sprintf栈溢出
TaskHandle_t IMU_Tx_Task_Handler;
void IMU_Tx_Task( void * pvParameters );

//移动平均滤波任务
#define MAF_Task_Priority 1
#define MAF_Task_Stack_Size 256
TaskHandle_t MAF_Task_Handler;
void MAF_Task( void * pvParamerters );


/**
 *@breaf  串口1回调函数
 *@param  packet:串口1的接收缓冲区
 *@param  size:串口1的接收缓冲区大小
 *@retval 无
 */
void UART1_rxCallback(u8 *packet, u16 size)
{
	BLE_Update_Flag = 1;
    if(packet[0] == 'A')
	{
		BLE_Moter_Flag = Forward_Flag ;
	}
	else if(packet[0] == 'B')
	{
		BLE_Moter_Flag = Backward_Flag ;
	}
	else if(packet[0] == 'C')
	{
		BLE_Moter_Flag = Right_Flag ;
	}
	else if(packet[0] == 'D')
	{
		BLE_Moter_Flag = Left_Flag ;
	}
	else if(packet[0] == '!')
	{
		BLE_Moter_Flag = Open_Top_Motor_Flag ;
	}
	else if(packet[0] == 'Q')
	{
		BLE_Moter_Flag = Close_Top_Motor_Flag ;
	}
	else if(packet[0] == '%')
	{
		BLE_Moter_Flag = Top_Stop_Flag;
	}
	else if(packet[0] == '3')
	{
		BLE_Moter_Flag = Speed_Up_Flag ;
	}
	else if(packet[0] == '6')
	{
		BLE_Moter_Flag = Slow_Down_Flag ;
	}
	else if(packet[0] == '0')
	{
		BLE_Moter_Flag = Stop_Motor_Flag;
	}
}
/**
 *@breaf  串口2回调函数
 *@param  packet:串口2的接收缓冲区
 *@param  size:串口2的接收缓冲区大小
 *@retval 无
 */
void UART2_rxCallback(u8 *packet, u16 size)
{
	if(size == 4 && packet[0] == 0xFF && packet[3] == check_sum(packet , 3))
	{
		Sonar_Back_uart2.new_data_ready = true;
		sonic_uart2 =  (((packet[1]) << 8) & 0xFFFF) + packet[2];
		Sonar_Update_distanse(&Sonar_Back_uart2,sonic_uart2);
	}
}
/**
 *@breaf  串口3回调函数
 *@param  packet:串口3的接收缓冲区
 *@param  size:串口3的接收缓冲区大小
 *@retval 无
 */
void UART3_rxCallback(u8 *packet, u16 size)
{
	BLE_Update_Flag = 1;
    if(packet[0] == 'A')
	{
		BLE_Moter_Flag = Forward_Flag ;
	}
	else if(packet[0] == 'B')
	{
		BLE_Moter_Flag = Backward_Flag ;
	}
	else if(packet[0] == 'C')
	{
		BLE_Moter_Flag = Right_Flag ;
	}
	else if(packet[0] == 'D')
	{
		BLE_Moter_Flag = Left_Flag ;
	}
	else if(packet[0] == '!')
	{
		BLE_Moter_Flag = Open_Top_Motor_Flag ;
	}
	else if(packet[0] == 'Q')
	{
		BLE_Moter_Flag = Close_Top_Motor_Flag ;
	}
	else if(packet[0] == '%')
	{
		BLE_Moter_Flag = Top_Stop_Flag;
	}
	else if(packet[0] == '3')
	{
		BLE_Moter_Flag = Speed_Up_Flag ;
	}
	else if(packet[0] == '6')
	{
		BLE_Moter_Flag = Slow_Down_Flag ;
	}
	else if(packet[0] == '0')
	{
		BLE_Moter_Flag = Stop_Motor_Flag;
	}
}
/**
 *@breaf  串口4回调函数
 *@param  packet:串口4的接收缓冲区
 *@param  size:串口4的接收缓冲区大小
 *@retval 无
 */
void UART4_rxCallback(u8 *packet, u16 size)
{
    if(size == 4 && packet[0] == 0xFF && packet[3] == check_sum(packet , 3))
	{
		Sonar_Front_uart4.new_data_ready = false;
		sonic_uart4 =  (((packet[1]) << 8) & 0xFFFF) + packet[2];
		Sonar_Update_distanse(&Sonar_Front_uart4,sonic_uart4);
	}
}

void UART5_rxCallback(u8 *packet, u16 size)
{
	if(size == 4 && packet[0] == 0xFF && packet[3] == check_sum(packet , 3))
	{
		Sonar_Back_uart5.new_data_ready = true;
		sonic_uart5 =  (((packet[1]) << 8) & 0xFFFF) + packet[2];
		Sonar_Update_distanse(&Sonar_Back_uart5,sonic_uart5);
	}
}
void UART6_rxCallback(u8 *packet, u16 size)
{
	if(size == 4 && packet[0] == 0xFF && packet[3] == check_sum(packet , 3))
	{
		Sonar_Front_uart6.new_data_ready = false;
		sonic_uart6 =  (((packet[1]) << 8) & 0xFFFF) + packet[2];
		Sonar_Update_distanse(&Sonar_Front_uart6,sonic_uart6);
	}
}

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init();
	Uart1_init(115200);
	Uart2_init(115200);
	Uart3_init(115200);
	Uart4_init(115200);
	Uart5_init(115200);
	Uart6_init(115200);
	TIMx_Configuration();
	Motor_Init();
	IIC_Init();
	Myadc_Init();
	Sonar_Filter_Init();
	
	
	if(pdPASS != xTaskCreate(Start_Task,"create other task" , Start_Stack_Size , NULL , Start_Task_Priority , NULL))
	{
		printf("start task creater error \r\n");
		return 1;
	}

	
	vTaskStartScheduler(); 
}


void testprint(void * parameters)
{
	while(1)
	{
		printf("hello\r\n");
		vTaskDelay(2000);
		printf("aa11\r\n");
		delay_ms(2000);
	}
}


void Start_Task(void * pvParameters )
{
	taskENTER_CRITICAL();
	
	if(pdPASS != xTaskCreate(Water_Task,"check for water entry", Water_Stack_Size , NULL , Water_Task_Priority,&Water_Task_Handler))
	{
		printf("water task create error \r\n");
	}
	if(pdPASS != xTaskCreate(Move_Task,"control motion",Move_Stack_Size , NULL , Move_Task_Priority , &Move_Task_Handler))
	{
		printf("move task create error \r\n");
	}
	if(pdPASS != xTaskCreate(Ultrasonic_Task,"get ultrasonic data", Ultrasonic_Stack_Size , NULL ,Ultrasonic_Task_Priority , &Ultrasonic_Task_Handler))
	{
		printf("ultrasonic task create error \r\n");
	}
	/* 停止Ultrasonic_Process_Task，因为它用printf发送距离数据，现在串口1用于IMU和指令 */
	/* if(pdPASS != xTaskCreate(Ultrasonic_Process_Task,"process ultrasonic data",Ultrasonic_Process_Stack_Size , NULL , Ultrasonic_Process_Task_Priority , &Ultrasonic_Task_Handler))
	{
		printf("process ultrasonic data task create error \r\n");
	} */
	
	
	
//	if(pdPASS != xTaskCreate(testprint,"test" , 512 , NULL , 1 , NULL))
//	{
//		printf("erros\r\n");
//	}
	
	
	if(pdPASS != xTaskCreate(IMU_Task,"read imu data task" , IMU_Task_Stack_Size , NULL , IMU_Task_Priority , &IMU_Task_Handler))
	{
		printf("imu task create error \r\n");
	}
	if(pdPASS != xTaskCreate(IMU_Tx_Task,"imu tx 20hz" , IMU_Tx_Task_Stack_Size , NULL , IMU_Tx_Task_Priority , &IMU_Tx_Task_Handler))
	{
		printf("imu tx task create error \r\n");
	}
	
	taskEXIT_CRITICAL();
	vTaskDelete(NULL);
}

void Water_Task(void * pvParameters )
{
	while(1)
	{
		/* 移除打印，避免干扰串口1的IMU数据 */
		/* printf("water : %d \r\n",Get_ADC_Value()); */
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void Ultrasonic_Task( void * pvParameters )
{
	while(1)
	{
		UART2_SendByte(0x01);
		Sonar_Data_Filter(&Sonar_Back_uart2);
		vTaskDelay(pdMS_TO_TICKS(20));
		UART4_SendByte(0x01);
		vTaskDelay(pdMS_TO_TICKS(20));
		UART5_SendByte(0x01);
		Sonar_Data_Filter(&Sonar_Back_uart5);
		vTaskDelay(pdMS_TO_TICKS(20));
		USART6_SendByte(0x01);
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void Ultrasonic_Process_Task( void * pvParameters )
{
	uint16_t Current_Distanse = 0;
	while(1)
	{
//		printf("sonic2 : %d , sonic 5 : %d\r\n" , Sonar_Get_Filter_Distanse(&Sonar_Back_uart2) , Sonar_Get_Filter_Distanse(&Sonar_Back_uart5));
		

		Current_Distanse = MIN(Sonar_Get_Filter_Distanse(&Sonar_Back_uart2),Sonar_Get_Filter_Distanse(&Sonar_Back_uart5));
		printf("Current Distanse = %d\r\n" , Current_Distanse);
		if(Current_Distanse < SONAR_TURN_DISTANSE)
		{
			Turn_Left(60,60);
			vTaskDelay(pdMS_TO_TICKS(3000));
			Backward(50 , 50);
			vTaskDelete(NULL);
		}
		
		
		
	}
}

void Move_Task( void * pvParameters )
{
	while(1)
	{
		if(BLE_Update_Flag)
		{
			BLE_Update_Flag = 0;
			switch(BLE_Moter_Flag)
			{
				case Forward_Flag:
					Forward(Left_Motor_Speed_Global,Right_Motor_Speed_Global);
					break;
				case Backward_Flag:
					Backward(Left_Motor_Speed_Global,Right_Motor_Speed_Global);
					break;
				case Left_Flag:
					Turn_Left(Left_Motor_Speed_Global,Right_Motor_Speed_Global);
					break;
				case Right_Flag:
					Turn_Right(Left_Motor_Speed_Global,Right_Motor_Speed_Global);
					break;
				case Open_Top_Motor_Flag:
					Open_Top();
					break;
				case Close_Top_Motor_Flag:
					Close_Top();
					break;
				case Speed_Up_Flag:
					Left_Motor_Speed_Global += 10;
					Right_Motor_Speed_Global += 10;
					if(Left_Motor_Speed_Global >= 100)
						Left_Motor_Speed_Global = 100;
					if(Right_Motor_Speed_Global >= 100)
						Right_Motor_Speed_Global = 100;
					break;
				case Slow_Down_Flag:
					Left_Motor_Speed_Global -= 10;
					Right_Motor_Speed_Global -= 10;
					if(Left_Motor_Speed_Global > 200)
						Left_Motor_Speed_Global = 0;
					if(Right_Motor_Speed_Global > 200)
						Right_Motor_Speed_Global = 0;
					break;
				case Stop_Motor_Flag:
					Set_Motor(Left,FORWARD,0);
					Set_Motor(Right,FORWARD,0);
					break;
			case Top_Stop_Flag:
				Close_Top();
				break;
			}
		}
	}
}

void IMU_Task( void * pvParameters )
{
	while(1)
	{
		/* 移除打印，避免干扰串口1的IMU数据（IMU数据由IMU_Tx_Task发送） */
		/* printf("Current Yaw = %d\r\n" , (I2C_readreg(0x50,Yaw) *180/32768)); */
		vTaskDelay(2000);
	}
}

static u8 imu_calc_checksum(const u8 *data, u16 len)
{
	u8 cs = 0;
	for(u16 i = 0; i < len; i++)
	{
		cs ^= data[i];
	}
	return cs;
}

static void uart1_send_frame(const char *payload)
{
	u8 cs = imu_calc_checksum((const u8*)payload, (u16)strlen(payload));
	char frame[96];
	int len = sprintf(frame, "$%s,%02X\r\n", payload, cs);
	if(len > 0 && len < (int)sizeof(frame))
	{
		extern int uart1_send_with_timeout(const u8 *buf, u16 len, u32 timeout_loop);
		if(uart1_send_with_timeout((const u8*)frame, (u16)len, 100000) != 0)
		{
			UART1_printf((u8*)frame, (u16)len);
		}
	}
}

static void imu_format_and_send_on_uart1(void)
{
	/* Read raw yaw, convert to centi-degrees to avoid float */
	int32_t raw = I2C_readreg(0x50, Yaw);
	
	/* 检查I2C读取是否失败（返回-1表示失败）*/
	if(raw == -1)
	{
		/* I2C读取失败，发送错误帧或跳过本次发送 */
		/* 可选：发送错误帧 "$IMU,ERR,XX\r\n" */
		return;  // 跳过本次发送，避免发送错误数据
	}
	
	/* 检查yaw值是否在合理范围内（-32768到32767对应-180到180度）*/
	if(raw < -32768 || raw > 32767)
	{
		return;  // 数据异常，跳过发送
	}
	
	int32_t yaw_centi = (int32_t)(raw * 18000 / 32768); /* -18000..18000 -> -180.00..180.00 */

	/* Build payload: IMU,<yaw> where <yaw> has two decimals */
	char payload[32];
	int sign = 0;
	if(yaw_centi < 0)
	{
		sign = 1;
		yaw_centi = -yaw_centi;
	}
	int32_t deg = yaw_centi / 100;
	int32_t frac = yaw_centi % 100;
	if(sign)
	{
		sprintf(payload, "IMU,-%ld.%02ld", (long)deg, (long)frac);
	}
	else
	{
		sprintf(payload, "IMU,%ld.%02ld", (long)deg, (long)frac);
	}

	/* Checksum over payload (without '$') */
	uart1_send_frame(payload);
}

static void sonar_format_and_send_on_uart1(void)
{
	uint16_t dist_uart2 = Sonar_Get_Filter_Distanse(&Sonar_Back_uart2);
	uint16_t dist_uart4 = Sonar_Get_Filter_Distanse(&Sonar_Front_uart4);
	uint16_t dist_uart5 = Sonar_Get_Filter_Distanse(&Sonar_Back_uart5);
	uint16_t dist_uart6 = Sonar_Get_Filter_Distanse(&Sonar_Front_uart6);

	char payload[64];
	sprintf(payload, "SONAR,U2:%u,U4:%u,U5:%u,U6:%u",
	        (unsigned int)dist_uart2,
	        (unsigned int)dist_uart4,
	        (unsigned int)dist_uart5,
	        (unsigned int)dist_uart6);

	uart1_send_frame(payload);
}

void IMU_Tx_Task( void * pvParameters )
{
	while(1)
	{
		imu_format_and_send_on_uart1();
		sonar_format_and_send_on_uart1();
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

/* Bounded-time UART1 sender to avoid permanent blocking */
int uart1_send_with_timeout(const u8 *buf, u16 len, u32 timeout_loop)
{
	for(u16 i = 0; i < len; i++)
	{
		u32 wait = timeout_loop;
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		{
			if(wait-- == 0)
			{
				return -1; /* TXE timeout */
			}
		}
		USART_SendData(USART1, buf[i]);
	}
	/* Wait for transmission complete */
	u32 wait_tc = timeout_loop;
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
		if(wait_tc-- == 0)
		{
			return -2; /* TC timeout */
		}
	}
	return 0;
}

//void Sonic_AutoMove_Task( void * pvParameters )
//{
//	sonic_uart6
//}



////移动平均滤波
//#define MAF_Task_Priority 1
//#define MAF_Task_Stack_Size 256
//TaskHandle_t MAF_Task_Handler;

//void MAF_Task( void * pvParamerters )
//{
//	while(1)
//	{
//		Sonar_Data_Filter(&Sonar_Back_uart2);
//		Sonar_Data_Filter(&Sonar_Back_uart5);
//	}
//}






