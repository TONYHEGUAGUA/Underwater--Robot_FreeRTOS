#include "main.h"

int PID_PWM=0;


/******************�й�imu�ı���************************/
float imu_yaw;
/*******************************************************/

/******************�йس������ı���*********************/
uint16_t sonic_uart2;
uint16_t sonic_uart4;
uint16_t sonic_uart5;
uint16_t sonic_uart6;
/*******************************************************/


/****************����ٶ�*******************************/
uint8_t Left_Motor_Speed_Global = 100;
uint8_t Right_Motor_Speed_Global = 100;
uint8_t Top_Motor_Speed_Global = 100;
/*******************************************************/

/**************** ������(usart3)�йصı��� *************/

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
	Stop_Motor_Flag
}BLE_Motor_State;
BLE_Motor_State BLE_Moter_Flag;
/*******************************************************/




// ��ʼ����������������
#define Start_Task_Priority 1
#define Start_Stack_Size 128
TaskHandle_t Start_Task_Handler;
void Start_Task(void * pvParameters );

// ��ȡ��ˮ����������
#define Water_Task_Priority 1
#define Water_Stack_Size 128
TaskHandle_t Water_Task_Handler;
void Water_Task(void * pvParameters );

// ����������������ȡ���ɣ�����
#define Ultrasonic_Task_Priority 1
#define Ultrasonic_Stack_Size 128
TaskHandle_t Ultrasonic_Task_Handler;
void Ultrasonic_Task( void * pvParameters );

// ����������������������������
#define Ultrasonic_Process_Task_Priority 1
#define Ultrasonic_Process_Stack_Size 128
TaskHandle_t Ultrasonic_Process_Task_Handler;
void Ultrasonic_Process_Task( void * pvParameters );

//�ƶ���ǰ�������ˡ���ת����ת������
#define Move_Task_Priority 1
#define Move_Stack_Size 128
TaskHandle_t Move_Task_Handler;
void Move_Task( void * pvParameters );

//IMU����
#define IMU_Task_Priority 3
#define IMU_Task_Stack_Size 512
TaskHandle_t IMU_Task_Handler;
void IMU_Task( void * pvParameters );

// IMU 20Hz UART3 transmit
#define IMU_Tx_Task_Priority 2
#define IMU_Tx_Task_Stack_Size 256
TaskHandle_t IMU_Tx_Task_Handler;
void IMU_Tx_Task( void * pvParameters );

//�ƶ�ƽ���˲�����
#define MAF_Task_Priority 1
#define MAF_Task_Stack_Size 256
TaskHandle_t MAF_Task_Handler;
void MAF_Task( void * pvParamerters );


/**
 *@breaf  ����1�ص�����
 *@param  packet:����1�Ľ��ܻ�����
 *@param  size:����1�Ľ��ջ�������С
 *@retval ��
 */
void UART1_rxCallback(u8 *packet, u16 size)
{
    
}
/**
 *@breaf  ����2�ص�����
 *@param  packet:����2�Ľ��ܻ�����
 *@param  size:����2�Ľ��ջ�������С
 *@retval ��
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
 *@breaf  ����3�ص�����
 *@param  packet:����3�Ľ��ܻ�����
 *@param  size:����3�Ľ��ջ�������С
 *@retval ��
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
 *@breaf  ����4�ص�����
 *@param  packet:����4�Ľ��ܻ�����
 *@param  size:����4�Ľ��ջ�������С
 *@retval ��
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
	if(pdPASS != xTaskCreate(Ultrasonic_Process_Task,"process ultrasonic data",Ultrasonic_Process_Stack_Size , NULL , Ultrasonic_Process_Task_Priority , &Ultrasonic_Task_Handler))
	{
		printf("process ultrasonic data task create error \r\n");
	}
	
	
	
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
		printf("water : %d \r\n",Get_ADC_Value());
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
					Set_Motor_Stop();
					break;
			}
		}
	}
}

void IMU_Task( void * pvParameters )
{
	while(1)
	{
		printf("Current Yaw = %d\r\n" , (I2C_readreg(0x50,Yaw) *180/32768));
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

static void imu_format_and_send_on_uart3(void)
{
	/* Read raw yaw, convert to centi-degrees to avoid float */
	int32_t raw = I2C_readreg(0x50, Yaw);
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
	u8 cs = imu_calc_checksum((const u8*)payload, (u16)strlen(payload));

	/* Build final frame: $<payload>,<CS>\r\n with CS as 2-digit uppercase hex */
	char frame[48];
	sprintf(frame, "$%s,%02X\r\n", payload, cs);
	UART3_printf((u8*)frame, (u16)strlen(frame));
}

void IMU_Tx_Task( void * pvParameters )
{
	while(1)
	{
		imu_format_and_send_on_uart3();
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

//void Sonic_AutoMove_Task( void * pvParameters )
//{
//	sonic_uart6
//}



////�ƶ�ƽ���˲�
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






