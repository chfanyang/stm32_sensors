#ifndef HWT_101_H
#define HWT_101_H

#include "main.h"

//#define delta_angle

#ifdef delta_angle
	extern float delta_yaw_angle;//偏航角相对初始值的差值
#endif
 
extern uint8_t receive_angle_buffer[11];//陀螺仪角度接收数组
extern float yaw_angle;//解算的偏航角
extern float yaw_w;//解算的角速度

void begin_receive_angle(void);
void read_angle_data(void);

#endif
