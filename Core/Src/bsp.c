/*******************************************************************************
  * @file	 	  	bsp.c
  * @author  		金鼎承
  * @version		V1.0.0
  * @date     	2022/1/21
  * @brief   		定义CAN滤波器，并进行PID三环控制
  ******************************************************************************
  * @attention 	none
*******************************************************************************/

#include "can.h"
#include "bsp.h"

moto_measure moto_chassis[6] = {0};																			//一共六个M2006CAN电机

void CanMotoOffset(moto_measure *motor, CAN_HandleTypeDef *hcan);
void CanGetTotalAngle(moto_measure *motor);

//初始化配置CAN滤波器，该滤波器没有配置过滤即ID全部通过，启动CAN通讯，开启CAN接受中断
HAL_StatusTypeDef CANFilterConfig_Scale16_IdMask(CAN_HandleTypeDef *hcanx)
{
	CAN_FilterTypeDef sFilterConfig;																			//CAN滤波器初始化配置结构体
	
	sFilterConfig.FilterBank = 0;																					//起始过滤组
	sFilterConfig.SlaveStartFilterBank = 14;															//中间过滤组，can1(0-13)和can2(14-27)分别得到一半的filter
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;											//过滤为掩码模式
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;										//寄存器位32位
	sFilterConfig.FilterIdHigh = 0x0000;																	//验证码高8位
	sFilterConfig.FilterIdLow = 0x0000;																		//验证码低8位
	sFilterConfig.FilterMaskIdHigh = 0x0000;															//掩码高8位
	sFilterConfig.FilterMaskIdLow = 0x0000;																//掩码低8位
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;										//过滤所得数据的存放到FIFO0
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;										//使能该过滤器
	
	if(HAL_CAN_ConfigFilter(hcanx, &sFilterConfig) != HAL_OK)							//配置CAN滤波器，如果发生错误则报警
	{
		Error_Handler();
		return HAL_ERROR;
	}
	
	HAL_CAN_Start(hcanx);																									//启动CAN通讯
	HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);			//使能FIFO0接收中断
	
	return HAL_OK;																												//回调状态
}

//发送CAN讯息，控制电调电流
HAL_StatusTypeDef CanTransmit(CAN_HandleTypeDef *hcanx,uint16_t stdid, 
	int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	static uint8_t TxData[8];																							//要发送的8位数据
	CAN_TxHeaderTypeDef TxMessage;																				//CAN数据发送结构体
	
	TxMessage.StdId = stdid;																							//ID赋值
	TxMessage.DLC = 0x08;																									//数据长度&位数
	TxMessage.IDE = CAN_ID_STD;																						//标准ID
	TxMessage.RTR = CAN_RTR_DATA;																					//数据帧
	TxMessage.TransmitGlobalTime = DISABLE;																//是否要发送时间戳
	
	//将传入的电调电流填充进入将要发送的TxData中，通过移位操作将uint16_t转换为uint8_t
	TxData[0] = (uint8_t)(cm1_iq >> 8);
	TxData[1] = (uint8_t)cm1_iq;
	TxData[2] = (uint8_t)(cm2_iq >> 8);
	TxData[3] = (uint8_t)cm2_iq;
	TxData[4] = (uint8_t)(cm3_iq >> 8); 
	TxData[5] = (uint8_t)cm3_iq;
	TxData[6] = (uint8_t)(cm4_iq >> 8);
	TxData[7] = (uint8_t)cm4_iq;
	
	//发送数据
	HAL_CAN_AddTxMessage(hcanx, &TxMessage, TxData, (uint32_t*)CAN_TX_MAILBOX0/*发送所使用的邮箱*/);
	
	return HAL_OK;																												//回调状态
}

//重写FIFO0接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	static uint8_t RxData[8];																							//接受数据存放
	CAN_RxHeaderTypeDef RxHeader;																					//接收数据头
		
	if(hcan ->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);				//接收数据
	}
	if(hcan ->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);				//接收数据
	}
	
	switch (RxHeader.StdId)																								//只解析电调发送的报文
	{
		case CAN_MOTOR_ID_1:
		case CAN_MOTOR_ID_2:
		case CAN_MOTOR_ID_3:
		case CAN_MOTOR_ID_4:
		case CAN_MOTOR_ID_5:	
		case CAN_MOTOR_ID_6:
		{
			static uint8_t n;
			n	= RxHeader.StdId - CAN_MOTOR_ID_1;
			CanGetMotorMessure(&moto_chassis[n], RxData); 												//调用电机数据解析 
		}break;
	}
	
}

//电机数据解析,送往进行PID调参
void CanGetMotorMessure(moto_measure *motor, uint8_t RxData[])
{
	uint16_t angle = RxData[0] << 8 | RxData[1];													//转子的机械角度
	int16_t rpm = RxData[2] << 8 | RxData[3];															//转子转速
	float current = (RxData[4] << 8 | RxData[5])*5.f/16384.f;							//实际输出转矩电流

	//将得到的数据填入
	motor ->last_angle = motor ->angle;
	motor ->angle = angle;
	motor ->rpm = rpm;
	motor ->real_current = current;
	
	if(motor ->angle - motor ->last_angle > 4096)
		motor ->round_cnt--;
	else if(motor ->angle - motor ->last_angle < -4096)
		motor ->round_cnt++;
	
	motor ->total_angle = motor ->round_cnt * 8192 + motor ->angle - motor ->offset_angle;
}

//电机数据接收初始化，须在CAN通讯开始时调用
void CanMotoOffset(moto_measure *motor, CAN_HandleTypeDef *hcan)
{
	static uint8_t RxData[8];																							//接受数据存放
	CAN_RxHeaderTypeDef RxHeader;																					//接收数据头
		
	if(hcan ->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);				//接收数据
	}
	if(hcan ->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);				//接收数据
	}
	
	motor ->angle = (uint16_t)(RxData[0] << 8 | RxData[1]);								//获取电机的初始机械角度
	motor ->offset_angle = motor ->angle;																	//将初始机械角度赋值
}
	
#define 	ABS(x)	((x>0)? x: -x) 

//得到电机总共转过的角度
void CanGetTotalAngle(moto_measure *motor)
{
	static uint32_t res1, res2, delta;																		//定义三个中间变量
	if(motor ->angle < motor ->last_angle)																//如果这次的角度比上一次小
	{																																			
		res1 = motor ->angle + 8192 - motor ->last_angle;										//正转
		res2 = motor ->angle - motor ->last_angle;													//反转
	}
	else
	{
		res1 = motor ->angle - 8192 - motor ->last_angle;										//反转
		res2 = motor ->angle - motor ->last_angle;													//正转
	}
	
	if(ABS(res1)<ABS(res2))																								//不管正反转，肯定是转的角度小的那个是真的	
		delta = res1;
	else
		delta = res2;
	
	motor ->total_angle += delta;																					//得到总共转的角度
	motor ->last_angle = motor ->angle;																		//赋值电机上次的机械角度
}
