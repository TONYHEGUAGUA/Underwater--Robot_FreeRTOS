#include "adc.h"


void Myadc_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

    // ���� ADC1 �Ĳ���
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  // 12 λ�ֱ���
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // ����ת��ģʽ
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;  // ��ʹ��ɨ��ģʽ
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  // �����Ҷ���
    ADC_InitStructure.ADC_NbrOfConversion = 1;  // һ��ֻת��һ��ͨ��
    ADC_Init(ADC1, &ADC_InitStructure);

    // ���� ADC ͨ����C0 ����ͨ����Ӧ ADC ͨ�� 10
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);

    // ʹ�� ADC1
    ADC_Cmd(ADC1, ENABLE);
	
	

	
}


// ��ȡģ���ź�
uint16_t Get_ADC_Value(void) 
{
    // ����������� ADC ת��
    ADC_SoftwareStartConv(ADC1);
    // �ȴ�ת�����
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    // ��ȡת�����
    return ADC_GetConversionValue(ADC1);
}
