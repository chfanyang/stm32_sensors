#ifndef HWT_101_H
#define HWT_101_H

#include "main.h"

//#define delta_angle

#ifdef delta_angle
	extern float delta_yaw_angle;//ƫ������Գ�ʼֵ�Ĳ�ֵ
#endif
 
extern uint8_t receive_angle_buffer[11];//�����ǽǶȽ�������
extern float yaw_angle;//�����ƫ����
extern float yaw_w;//����Ľ��ٶ�

void begin_receive_angle(void);
void read_angle_data(void);

#endif
