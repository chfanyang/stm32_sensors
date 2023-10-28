#include <stdio.h>

typedef double ElemType;

#define delta_t 0.001//时间间隔

typedef struct {
	ElemType Kp;
	ElemType Ki;
	ElemType Kd;

	ElemType err;//误差值

	ElemType integral_limit_upper;//积分上限限幅
	ElemType integral_limit_lower;//积分下限限幅
	ElemType integral;//积分项的误差累积和
	
	ElemType err_last;//上次误差值

	ElemType out_limit_upper;//最大输出限幅
	ElemType out_limit_lower;//最小输出限幅
}pid_location_struct;

/*位置式pid初始化*/
void pid_location_struct_init(pid_location_struct* pid_struct, ElemType Kp, ElemType Ki, ElemType Kd,ElemType out_limit_upper, ElemType out_limit_lower) {
	pid_struct->Kp = Kp;
	pid_struct->Ki = Ki;
	pid_struct->Kd = Kd;

	pid_struct->err = 0;

	
	pid_struct->integral = 0;
	pid_struct->integral_limit_upper = 100;//可更改
	pid_struct->integral_limit_lower = 0;

	pid_struct->err_last = 0;
	pid_struct->out_limit_upper = out_limit_upper;
	pid_struct->out_limit_lower = out_limit_lower;
}

/*位置式pid计算*/
ElemType pid_location_cal(pid_location_struct* pid_struct, ElemType position, ElemType target) {   
	
	ElemType p_out = 0;
	ElemType i_out = 0;
	ElemType d_out = 0;
	ElemType pid_cal_result = 0;

	/*比例控制*/
	pid_struct->err = target - position;//误差更新
	p_out = (pid_struct->Kp) * (pid_struct->err);

	/*积分控制*/
	if (pid_struct->Ki != 0) {
		pid_struct->integral += pid_struct->err;//积分误差累积
		if (pid_struct->integral > pid_struct->integral_limit_upper) {//积分限幅
			pid_struct->integral = pid_struct->integral_limit_upper;
		}
		else if (pid_struct->integral < pid_struct->integral_limit_lower) {
			pid_struct->integral = pid_struct->integral_limit_lower;
		}
		i_out = (pid_struct->Ki) * (pid_struct->integral) * delta_t;
	}
	else {
		i_out = 0;
	}

	/*微分控制*/
	if (pid_struct->Kd != 0) {
		d_out = (pid_struct->Kd) * (pid_struct->err - pid_struct->err_last) / delta_t;
		pid_struct->err_last = pid_struct->err;//上次误差更新
	}
	else {
		d_out = 0;
	}

	/*pid结果输出*/
	pid_cal_result = p_out + i_out + d_out;
	if (pid_cal_result > pid_struct->out_limit_upper) {
		pid_cal_result = pid_struct->out_limit_upper;
	}
	else if(pid_cal_result< pid_struct->out_limit_lower){
		pid_cal_result = pid_struct->out_limit_lower;
	}
	return pid_cal_result;
}

typedef struct {
	ElemType Kp;
	ElemType Ki;
	ElemType Kd;

	ElemType last_out;//上次输出

	ElemType err;//本次误差
	ElemType err_last;//上次误差
	ElemType err_last_last;//上上次误差

	ElemType out_limit_upper;//最大输出限幅
	ElemType out_limit_lower;//最小输出限幅
}pid_incremental_struct;

/*增量式pid初始化*/
void pid_increment_struct_init(pid_incremental_struct* pid_struct, ElemType Kp, ElemType Ki, ElemType Kd, ElemType out_limit_upper, ElemType out_limit_lower) {
	pid_struct->Kp = Kp;
	pid_struct->Ki = Ki;
	pid_struct->Kd = Kd;

	pid_struct->last_out = 0;

	pid_struct->err = 0;
	pid_struct->err_last = 0;
	pid_struct->err_last_last = 0;

	pid_struct->out_limit_upper = out_limit_upper;
	pid_struct->out_limit_lower = out_limit_lower;
}

/*增量式pid计算*/
ElemType pid_incremental_cal(pid_incremental_struct* pid_struct, ElemType position, ElemType target) {

	ElemType p_out = 0;
	ElemType i_out = 0;
	ElemType d_out = 0;
	ElemType delta_pid_cal_result = 0;//输出量的增量
	ElemType pid_cal_result = pid_struct->last_out;//最终输出结果

	pid_struct->err = target - position;//更新本次误差

	/*计算输出量*/
	p_out = (pid_struct->Kp) * (pid_struct->err - pid_struct->err_last);
	i_out = (pid_struct->Ki) * (pid_struct->err);
	d_out = (pid_struct->Kd) * (pid_struct->err - 2 * (pid_struct->err_last) + pid_struct->err_last_last);

	/*更新上上次和上次误差*/
	pid_struct->err_last_last = pid_struct->err_last;
	pid_struct->err_last = pid_struct->err;

	/*得到最终输出结果*/
	delta_pid_cal_result = p_out + i_out + d_out;
	pid_cal_result += delta_pid_cal_result;

	/*输出限幅*/
	if (pid_cal_result > pid_struct->out_limit_upper) {
		pid_cal_result = pid_struct->out_limit_upper;
	}
	else if (pid_cal_result < pid_struct->out_limit_lower) {
		pid_cal_result = pid_struct->out_limit_lower;
	}

	return pid_cal_result;
}
