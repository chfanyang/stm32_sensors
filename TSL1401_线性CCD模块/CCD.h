#ifndef __CCD_H
#define __CCD_H	

#include "main.h"

extern uint16_t ADV[128];              
extern uint8_t CCD_final_value,CCD_Yuzhi;   //����CCD��ر���

uint16_t Get_Adc(void);
void Dly_us(void);
void RD_TSL(void); 
void  Find_CCD_final_value(void);
#endif 


