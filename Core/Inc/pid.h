/*******************************************************************************
  * @file	 	  	pid.h
  * @author  		金鼎承
  * @version		V1.0.8
  * @date     	    2022/2/6
  * @brief   		define PID strut
  ******************************************************************************
  * @attention 	    none
*******************************************************************************/

#ifndef _PID_H
#define _PID_H

#include "stdint.h"

//PID类型枚举
typedef enum {
    //记录的三次数值
    LLAST = 0,                                  //上上次
    LAST = 1,                                   //上次
    NOW = 2,                                    //本次

    Position_PID,                               //位置PID
    Delta_PID,                                  //增量PID
} PID_ENUM;

//PID结构体
typedef struct $PID_TypeDef {
    PID_ENUM mode;                              //PID控制类型

    float kp;                                   //比例系数
    float ki;                                   //积分系数
    float kd;                                   //微分系数

    float set[3];                               //目标值
    float get[3];                               //测量值
    float err[3];                               //误差值

    float pout;                                 //比例输出项
    float iout;                                 //积分输出项
    float dout;                                 //微分输出项

    float position_out;                         //本次位置PID输出
    float last_position_out;                    //上次位置PID输出
    float delta;                                //本次增量PID增量
    float delta_out;                            //本次增量PID输出
    float last_delta_out;                       //上次增量PID输出

    float max_output;                           //输出限幅
    float integral_limit;                       //积分限幅
    float deadband;                             //死区绝对值
    float max_err;                              //最大误差绝对值

    //PID参数赋值函数指针
    void (*f_param_init)(struct $PID_TypeDef *pid, PID_ENUM mode, uint16_t maxout, uint16_t intergral_limit,
                         float deadband, int16_t max_err, float kp, float ki, float kd);

    //PID参数调节函数指针
    void (*f_pid_reset)(struct $PID_TypeDef *pid, float kp, float ki, float kd);

    //PID参数计算函数指针
    float (*f_pid_calculate)(struct $PID_TypeDef *pid, float get, float set);
} PID_TypeDef;

void pid_struct_init(PID_TypeDef *pid, PID_ENUM mode, uint16_t maxout, uint16_t intergral_limit,
                     float deadband, int16_t max_err, float kp, float ki, float kd);

float pid_calculate(PID_TypeDef *pid, float get, float set);

//引用定义在main.c中的motor_pid
extern PID_TypeDef motor_pid[6];

#endif
