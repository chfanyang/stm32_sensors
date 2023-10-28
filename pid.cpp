#include <stdio.h>

typedef double ElemType;

#define delta_t 0.001//ʱ����

typedef struct {
	ElemType Kp;
	ElemType Ki;
	ElemType Kd;

	ElemType err;//���ֵ

	ElemType integral_limit_upper;//���������޷�
	ElemType integral_limit_lower;//���������޷�
	ElemType integral;//�����������ۻ���
	
	ElemType err_last;//�ϴ����ֵ

	ElemType out_limit_upper;//�������޷�
	ElemType out_limit_lower;//��С����޷�
}pid_location_struct;

/*λ��ʽpid��ʼ��*/
void pid_location_struct_init(pid_location_struct* pid_struct, ElemType Kp, ElemType Ki, ElemType Kd,ElemType out_limit_upper, ElemType out_limit_lower) {
	pid_struct->Kp = Kp;
	pid_struct->Ki = Ki;
	pid_struct->Kd = Kd;

	pid_struct->err = 0;

	
	pid_struct->integral = 0;
	pid_struct->integral_limit_upper = 100;//�ɸ���
	pid_struct->integral_limit_lower = 0;

	pid_struct->err_last = 0;
	pid_struct->out_limit_upper = out_limit_upper;
	pid_struct->out_limit_lower = out_limit_lower;
}

/*λ��ʽpid����*/
ElemType pid_location_cal(pid_location_struct* pid_struct, ElemType position, ElemType target) {   
	
	ElemType p_out = 0;
	ElemType i_out = 0;
	ElemType d_out = 0;
	ElemType pid_cal_result = 0;

	/*��������*/
	pid_struct->err = target - position;//������
	p_out = (pid_struct->Kp) * (pid_struct->err);

	/*���ֿ���*/
	if (pid_struct->Ki != 0) {
		pid_struct->integral += pid_struct->err;//��������ۻ�
		if (pid_struct->integral > pid_struct->integral_limit_upper) {//�����޷�
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

	/*΢�ֿ���*/
	if (pid_struct->Kd != 0) {
		d_out = (pid_struct->Kd) * (pid_struct->err - pid_struct->err_last) / delta_t;
		pid_struct->err_last = pid_struct->err;//�ϴ�������
	}
	else {
		d_out = 0;
	}

	/*pid������*/
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

	ElemType last_out;//�ϴ����

	ElemType err;//�������
	ElemType err_last;//�ϴ����
	ElemType err_last_last;//���ϴ����

	ElemType out_limit_upper;//�������޷�
	ElemType out_limit_lower;//��С����޷�
}pid_incremental_struct;

/*����ʽpid��ʼ��*/
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

/*����ʽpid����*/
ElemType pid_incremental_cal(pid_incremental_struct* pid_struct, ElemType position, ElemType target) {

	ElemType p_out = 0;
	ElemType i_out = 0;
	ElemType d_out = 0;
	ElemType delta_pid_cal_result = 0;//�����������
	ElemType pid_cal_result = pid_struct->last_out;//����������

	pid_struct->err = target - position;//���±������

	/*���������*/
	p_out = (pid_struct->Kp) * (pid_struct->err - pid_struct->err_last);
	i_out = (pid_struct->Ki) * (pid_struct->err);
	d_out = (pid_struct->Kd) * (pid_struct->err - 2 * (pid_struct->err_last) + pid_struct->err_last_last);

	/*�������ϴκ��ϴ����*/
	pid_struct->err_last_last = pid_struct->err_last;
	pid_struct->err_last = pid_struct->err;

	/*�õ�����������*/
	delta_pid_cal_result = p_out + i_out + d_out;
	pid_cal_result += delta_pid_cal_result;

	/*����޷�*/
	if (pid_cal_result > pid_struct->out_limit_upper) {
		pid_cal_result = pid_struct->out_limit_upper;
	}
	else if (pid_cal_result < pid_struct->out_limit_lower) {
		pid_cal_result = pid_struct->out_limit_lower;
	}

	return pid_cal_result;
}
