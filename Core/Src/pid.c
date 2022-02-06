/*******************************************************************************
  * @file	 	  	pid.c
  * @author  		金鼎承
  * @version		V1.0.0
  * @date     	    2022/1/21
  * @brief   		
  ******************************************************************************
  * @attention 	使用二阶差分会更加稳定
*******************************************************************************/

#include "pid.h"

#define 	ABS(x)	((x>0)? x: -x) 

PID_TypeDef pid_pitch,pid_pithch_speed,pid_roll,pid_roll_speed,pid_yaw_speed;

//PID参数初始赋值
static void pid_param_init(PID_TypeDef *pid, PID_ID id, uint16_t maxout, uint16_t intergral_limit,
	float deadband, uint16_t period, int16_t max_err, int16_t target, float kp, float ki, float kd)
{
	pid ->id = id;
	pid ->ControlPeriod = period;
	pid ->DeadBand = deadband;
	pid ->IntegralLimit = intergral_limit;
	pid ->MaxOutput = maxout;
	pid ->Max_Err = max_err;
	pid ->target = target;
	
	pid ->kp = kp;
	pid ->ki = ki;
	pid ->kd = kd;
	
	pid ->output = 0;
}

//改变PID各项系数
static void pid_reset(PID_TypeDef *pid, float kp, float ki, float kd)
{
	pid ->kp = kp;
	pid ->ki = ki;
	pid ->kd = kd;
}

//PID结果计算
static float pid_calculate(PID_TypeDef *pid, float measure)
{
	//本次观测值赋值
	pid ->measure = measure;
	
	//上次误差值转移
	pid ->last_err = pid ->err;
	
	//本次误差值计算
	pid ->err = pid ->target - pid ->measure;
	
	//上次输出值转移
	pid ->last_output = pid ->output;
	
	//如果进入死区就直接返回PID输出
	if((ABS(pid ->err) > pid ->DeadBand))
	{
        //分别计算各项的值
        pid->pout = pid->kp * pid->err;
        pid->iout += (pid->ki * pid->err);
        pid->dout = pid->kd * (pid->err - pid->last_err);

        //限制积分项输出大小
        if (pid->iout > pid->IntegralLimit) {
            pid->iout = pid->IntegralLimit;
        }
        if (pid->iout < -(pid->IntegralLimit)) {
            pid->iout = -(pid->IntegralLimit);
        }

        //计算PID输出
        pid->output = pid->pout + pid->iout + pid->dout;

        //限制PID输出大小
        if (pid->output > pid->MaxOutput) {
            pid->output = pid->MaxOutput;
        }
        if (pid->output < -(pid->MaxOutput)) {
            pid->output = -(pid->MaxOutput);
        }
    }
	return pid ->output;
}

//PID结构体初始化，只需调用一次
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}
