#include "adc.h"
#include "main.h"
#include "CCD.h"
#include "DJI_motor_contral.h"
#include "tim.h"
#include "usart.h"

uint16_t ADV[128]={0};              
uint8_t CCD_final_value,CCD_Yuzhi;   //����CCD��ر���

/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
�������ܣ�AD����
��ڲ�����ADC��ͨ��
����  ֵ��ADת�����
**************************************************************************/
uint16_t Get_Adc(void)   
{
	
	HAL_ADC_Start(&hadc1);  //�ȿ���ADC
	HAL_ADC_PollForConversion(&hadc1,1);//��ѯ��������ѯEOC��־λ��ÿ�β�����CUP�����ﶼҪ 
                                       //�ȴ�������ɲ��ܽ�����һ�������ʱ��CUPû�и����� 
                                       //�£����Խ�����CUPʹ����
 
	return HAL_ADC_GetValue(&hadc1);    //�õ�ADC��ֵ
}

/**************************************************************************
�������ܣ���ʱ
��ڲ�������
����  ֵ����
**************************************************************************/
void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim3, 0);  // ��TIMER��counter��Ϊ0
  while (__HAL_TIM_GET_COUNTER(&htim3) < us); //��ȡ��ʱ����counterֵ
 }

/**************************************************************************
�������ܣ�CCD���ݲɼ�
��ڲ�������
����  ֵ����
**************************************************************************/
 void RD_TSL(void) 
{
  uint8_t i=0,tslp=0;
  HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
	
 // delay_us(20);
      
  HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);
  //delay_us(20);
	
	
  HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
  //delay_us(20);
	
  for(i=0;i<128;i++)					//��ȡ128�����ص��ѹֵ
  {
    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);
    //delay_us(20);//�����ع�ʱ��
		delay_us(20);

    ADV[tslp]=(Get_Adc())>>4;
    ++tslp;
    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
    //delay_us(20);	
  }  
}
 
/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_final_value(void)
{ 
	 static uint16_t i,j,Left,Right,Last_CCD_Zhongzhi;
	 static uint16_t value1_max,value1_min;
	
	   value1_max=ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ
     for(i=5;i<123;i++)   //���߸�ȥ��5����
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //��Сֵ
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
		 
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //���������������ȡ����ֵ
		 
	 for(i = 5;i<118; i++)   //Ѱ�����������
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//Ѱ���ұ�������      
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_final_value=(Right+Left)/2;//��������λ��
	if(ABS(CCD_final_value-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫��
	CCD_final_value=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
	Last_CCD_Zhongzhi=CCD_final_value;  //������һ�ε�ƫ��
}
