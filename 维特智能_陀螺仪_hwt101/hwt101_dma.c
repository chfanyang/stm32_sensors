#include "hwt101_dma.h"
#include "main.h"
#include "dma.h"
#include "usart.h"

uint8_t receive_angle_buffer[11]={0};//陀螺仪角度接收数组

float yaw_angle=0;//解算的偏航角,单位是°
float yaw_w=0;//解算的角速度，单位是°/s



#ifdef delta_angle
	float delta_yaw_angle=0;//偏航角相对于上电时的位置的差值
	float first_angle=0;//偏航角的初始角度 
#endif

void begin_receive_angle(void){//启动接收
	HAL_UART_Receive_DMA(&huart8, receive_angle_buffer,1u);//单个接收，寻找帧头
}

void read_angle_data(void){
	#ifdef delta_angle
		static int cnt=0;
	#endif 
	static uint8_t recv_10byte_flag=1;//本次接收多个字节的标志位
	if(recv_10byte_flag && receive_angle_buffer[0]==0x55){
		HAL_UART_Receive_DMA(&huart8, receive_angle_buffer,10u);//一次接收10个
		recv_10byte_flag=0;
	}
	else{
		HAL_UART_Receive_DMA(&huart8, receive_angle_buffer,1u);//单个接收，寻找帧头
		if(recv_10byte_flag==0){//上一次开启了接收10个字节
			uint8_t sum_crc=0;//校验和
			switch(receive_angle_buffer[0]){
				case 0x52:{//角速度
					sum_crc=0x55+0x52+receive_angle_buffer[3]+receive_angle_buffer[4]+receive_angle_buffer[5]+receive_angle_buffer[6];
					if(sum_crc==receive_angle_buffer[9]){
						yaw_w=((receive_angle_buffer[6]<<8) | receive_angle_buffer[5])/32768.0*2000;
					}
					else{//error
						//USART_printf("ERR\n");
					}
					break;
				}
				case 0x53:{//角度
					sum_crc=0x55+0x53+receive_angle_buffer[5]+receive_angle_buffer[6]+receive_angle_buffer[7]+receive_angle_buffer[8];
					if(sum_crc==receive_angle_buffer[9]){
						yaw_angle=((receive_angle_buffer[6]<<8) | receive_angle_buffer[5])/32768.0*180;
						#ifdef delta_angle 
							cnt++;
							if(cnt==100){
								first_angle=yaw_angle;
							}
							delta_yaw_angle=yaw_angle-first_angle;
						#endif
					}
					else{//error
						//USART_printf("ERR\n");
					}
					break;
				}
				default:{//error
					//USART_printf("ERR\n");
				}
			}
		}
		recv_10byte_flag=1;
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
//{
//	if(huart->Instance == UART8){
//		read_angle_data();
//	}
//}
