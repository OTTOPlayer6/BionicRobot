/*******************************************************************************
  * @file	 	  	pid.c
  * @author  		金鼎承
  * @version		V1.0.8
  * @date     	    2022/2/6
  * @brief   		PID的一系列函数
  ******************************************************************************
  * @attention 	    据说使用二阶差分会更加稳定，但我比较懒QAQ
*******************************************************************************/

#include "pid.h"

#define    ABS(x)    ((x>0)? x: -x)

void abs_limit(float *input_num, float ABS_MAX) {
    if (*input_num > ABS_MAX)
        *input_num = ABS_MAX;
    if (*input_num < -ABS_MAX)
        *input_num = -ABS_MAX;
}

//PID参数初始赋值
static void pid_param_init(PID_TypeDef *pid, PID_ENUM mode, uint16_t maxout, uint16_t intergral_limit,
                           float deadband, int16_t max_err, float kp, float ki, float kd) {
    pid->mode = mode;
    pid->deadband = deadband;
    pid->integral_limit = intergral_limit;
    pid->max_output = maxout;
    pid->max_err = max_err;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

//改变PID各项系数
static void pid_reset(PID_TypeDef *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

//PID结果计算
float pid_calculate(PID_TypeDef *pid, float get, float set) {

    pid->get[NOW] = get;                                                //本次观测值
    pid->set[NOW] = set;                                                //本次期望值
    pid->err[NOW] = set - get;                                          //本次误差值

    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)         //大于最大误差，系统失调
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)       //小于死区，系统趋于稳定
        return 0;

    if (pid->mode == Position_PID)                                      //位置式PID
    {
        pid->pout = pid->kp * pid->err[NOW];
        pid->iout += pid->ki * pid->err[NOW];
        pid->dout = pid->kd * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&(pid->iout), pid->integral_limit);
        pid->position_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->position_out), pid->max_output);
        pid->last_position_out = pid->position_out;                     //更新上一次的位置
    } else if (pid->mode == Delta_PID) {                                //增量式PID
        pid->pout = pid->kp * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->ki * pid->err[NOW];
        pid->dout = pid->kd * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->integral_limit);
        pid->delta = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta;
        abs_limit(&(pid->delta_out), pid->max_output);
        pid->last_delta_out = pid->delta_out;                           //更新上一次的增量
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    return pid->mode == Position_PID ? pid->position_out : pid->delta_out;
}

//PID结构体初始化，只需调用一次
void pid_struct_init(PID_TypeDef *pid, PID_ENUM mode, uint16_t maxout, uint16_t intergral_limit,
                     float deadband, int16_t max_err, float kp, float ki, float kd) {
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
    pid->f_pid_calculate = pid_calculate;

    pid->f_param_init(pid, mode, maxout, intergral_limit, deadband, max_err, kp, ki, kd);
}
