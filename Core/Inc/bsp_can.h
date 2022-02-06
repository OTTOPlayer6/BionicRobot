/*******************************************************************************
  * @file	 	  	bsp_can.h
  * @author  		金鼎承
  * @version		V1.0.3
  * @date     	    2022/2/6
  * @brief   		定义CAN接收结构体
  ******************************************************************************
  * @attention  	NONE
*******************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4

#include "stm32f4xx_hal.h"

#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif

#include "can.h"

//CAN接受的电调ID
typedef enum {
    CAN_MOTOR_ID_ALL __attribute__((unused)) = 0x200,
    CAN_MOTOR_ID_1 = 0x201,
    CAN_MOTOR_ID_2 = 0x202,
    CAN_MOTOR_ID_3 = 0x203,
    CAN_MOTOR_ID_4 = 0x204,
    CAN_MOTOR_ID_5 = 0x205,
    CAN_MOTOR_ID_6 = 0x206
} CAN_MOTOR_ID;

//CAN接受到的电调的数据与电机PID参数结构体
typedef struct {
    uint16_t offset_angle;                              //转子的初始机械角度
    uint16_t angle;                                     //转子机械角度绝对值:[0,8191]对应[0:360]
    uint16_t last_angle;                                //上次转子的机械角度绝对值

    int16_t speed_rpm;                                  //转子转速，单位:speed_rpm
    int16_t real_current;                               //转子实际输出电流测量值
    int16_t given_current;                              //转子的目标输出电流

    int32_t round_cnt;                                  //电机旋转圈数
    int32_t total_angle;                                //电机旋转的总角度
} moto_measure;

//引用电机观察值数组
extern moto_measure moto_chassis[];
extern moto_measure moto_info;

//定义函数，详见bsp_can.c
HAL_StatusTypeDef CANFilterCfg_Scale16_IdMask_ReceiveALL(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef set_motor_current(CAN_HandleTypeDef *hcanx, uint16_t stdid,
                                    int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void get_motor_measure(moto_measure *motor, const uint8_t RxData[]);
#endif
