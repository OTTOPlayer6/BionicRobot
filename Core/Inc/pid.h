/*******************************************************************************
  * @file	 	  	pid.h
  * @author  		金鼎承
  * @version		V1.0.0
  * @date     	    2022/1/21
  * @brief   		define PID strut
  ******************************************************************************
  * @attention 	    none
*******************************************************************************/

#ifndef _PID_H
#define _PID_H

#include "stdint.h"

//PID三环控制类型枚举
typedef enum
{
	PID_Position,							//位置PID
	PID_Speed,								//速度PID
	PID_Current								//电流PID	
}PID_ID;

//PID结构体
typedef struct $PID_TypeDef {
    PID_ID id;                                //三环控制ID

    float target;                            //算法目标值
    float kp;                                    //比例系数
    float ki;                                    //积分系数
    float kd;                                    //微分系数

    float measure;                        //测量值
    float err;                                //误差值
    float last_err;                //上次误差值

    float pout;                                //比例输出项
    float iout;                                //积分输出项
    float dout;                                //微分输出项

    float output;                            //总输出
    float last_output;                //上次输出

    float MaxOutput;                    //输出限幅
    float IntegralLimit;            //积分限幅
    float DeadBand;                    //死区绝对值
    float Max_Err;                        //最大误差范围
    float ControlPeriod;            //控制周期

    //PID参数初始赋值函数指针
    void (*f_param_init)(struct $PID_TypeDef *pid, PID_ID id, uint16_t maxout, uint16_t intergral_limit,
                         float deadband, uint16_t period, int16_t max_err, int16_t target, float kp, float ki,
                         float kd);

    //PID参数调节函数指针
    void (*f_pid_reset)(struct $PID_TypeDef *pid, float kp, float ki, float kd);

    //PID结果计算函数指针
    float (*f_cal_pid)(struct $PID_TypeDef *pid, float measure);
}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
#endif

//引用定义在main.c中的motor_pid
extern PID_TypeDef motor_pid[6];
