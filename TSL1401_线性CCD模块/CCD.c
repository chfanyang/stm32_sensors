#include "adc.h"
#include "main.h"
#include "CCD.h"
#include "DJI_motor_contral.h"
#include "tim.h"
#include "usart.h"

uint16_t ADV[128]={0};              
uint8_t CCD_final_value,CCD_Yuzhi;   //线性CCD相关变量

/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
函数功能：AD采样
入口参数：ADC的通道
返回  值：AD转换结果
**************************************************************************/
uint16_t Get_Adc(void)   
{
	
	HAL_ADC_Start(&hadc1);  //先开启ADC
	HAL_ADC_PollForConversion(&hadc1,1);//查询函数，查询EOC标志位。每次采样，CUP在这里都要 
                                       //等待采样完成才能进行下一步，这段时间CUP没有干其他 
                                       //事，所以降低了CUP使用率
 
	return HAL_ADC_GetValue(&hadc1);    //得到ADC的值
}

/**************************************************************************
函数功能：延时
入口参数：无
返回  值：无
**************************************************************************/
void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim3, 0);  // 把TIMER的counter设为0
  while (__HAL_TIM_GET_COUNTER(&htim3) < us); //读取定时器的counter值
 }

/**************************************************************************
函数功能：CCD数据采集
入口参数：无
返回  值：无
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
	
  for(i=0;i<128;i++)					//读取128个像素点电压值
  {
    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);
    //delay_us(20);//调节曝光时间
		delay_us(20);

    ADV[tslp]=(Get_Adc())>>4;
    ++tslp;
    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
    //delay_us(20);	
  }  
}
 
/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_final_value(void)
{ 
	 static uint16_t i,j,Left,Right,Last_CCD_Zhongzhi;
	 static uint16_t value1_max,value1_min;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
		 
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
		 
	 for(i = 5;i<118; i++)   //寻找左边跳变沿
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//寻找右边跳变沿      
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_final_value=(Right+Left)/2;//计算中线位置
	if(ABS(CCD_final_value-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
	CCD_final_value=Last_CCD_Zhongzhi;    //则取上一次的值
	Last_CCD_Zhongzhi=CCD_final_value;  //保存上一次的偏差
}
