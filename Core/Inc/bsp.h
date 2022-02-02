/*******************************************************************************
  * @file	 	  	bsp.h
  * @author  		金鼎承
  * @version		V1.0.0
  * @date     	2022/1/21
  * @brief   		define BSP strut
  ******************************************************************************
  * @attention 	none
*******************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"

//CAN接受的电调ID
typedef enum
{
	CAN_MOTOR_ID_ALL = 0x200,
	CAN_MOTOR_ID_1   = 0x201,
	CAN_MOTOR_ID_2   = 0x202,
	CAN_MOTOR_ID_3   = 0x203,
	CAN_MOTOR_ID_4   = 0x204,
	CAN_MOTOR_ID_5   = 0x205,
	CAN_MOTOR_ID_6   = 0x206
}CAN_MOTOR_ID;

//CAN接受到的电调的数据与电机PID参数结构体
typedef struct {
	uint16_t	offset_angle; 	//转子的初始机械角度
	
	uint16_t angle; 					//转子机械角度绝对值:[0,8191]对应[0:360]
	uint16_t last_angle;   	  //上次转子的机械角度绝对值
	int16_t rpm;							//转子转速，单位:rpm]
	float real_current;				//转子实际输出转矩电流
	int32_t round_cnt;    	  //电机旋转圈数
	int32_t	total_angle;			//电机旋转的总角度
	
}moto_measure;	

//引用电机观察值数组
extern moto_measure  moto_chassis[];

//定义函数，详见bsp.c
HAL_StatusTypeDef CANFilterConfig_Scale16_IdMask(CAN_HandleTypeDef *hcanx);
HAL_StatusTypeDef CanTransmit(CAN_HandleTypeDef *hcanx,uint16_t stdid, 
	int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CanGetMotorMessure(moto_measure *motor, uint8_t RxData[]);

#endif
