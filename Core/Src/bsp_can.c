/*******************************************************************************
  * @file	 	  	bsp_can.c
  * @author  		金鼎承
  * @version		V1.0.3
  * @date       	2022/2/6
  * @brief   		定义CAN滤波器，并进行PID三环控制
  ******************************************************************************
  * @attention 	    NONE
*******************************************************************************/

#include "can.h"
#include "bsp_can.h"

moto_measure moto_chassis[6] = {0};                                        //一共六个M2006CAN底盘电机
moto_measure moto_info;

void get_moto_offset(moto_measure *motor, const uint8_t RxData[]);

void get_total_angle(moto_measure *motor);

//初始化配置CAN滤波器，该滤波器没有配置过滤即ID全部通过，启动CAN通讯，开启CAN接受中断
HAL_StatusTypeDef CANFilterCfg_Scale16_IdMask_ReceiveALL(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef CAN_FilterConfigStructure;                          //CAN滤波器初始化配置结构体

    CAN_FilterConfigStructure.FilterBank = 0;                             //起始过滤组
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;                  //中间过滤组，can1(0-13)和can2(14-27)分别得到一半的filter
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;         //过滤为掩码模式
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;        //寄存器位32位
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;                      //验证码高8位
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;                       //验证码低8位
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;                  //掩码高8位
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;                   //掩码低8位
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_RX_FIFO0;        //过滤所得数据的存放到FIFO0
    CAN_FilterConfigStructure.FilterActivation = CAN_FILTER_ENABLE;       //使能该过滤器

    //配置CAN滤波器，如果发生错误则报警
    if (HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfigStructure) != HAL_OK) {
        Error_Handler();
        return HAL_ERROR;
    }

    HAL_CAN_Start(hcan);                                                          //启动CAN通讯
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);     //使能FIFO0接收中断

    return HAL_OK;                                                                //回调状态
}

//重写FIFO0接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    static uint8_t RxData[8];                                                                //接受数据存放
    CAN_RxHeaderTypeDef RxHeader;                                                            //接收数据头

    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);    //接收数据
    }
    if (hcan->Instance == CAN2) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);    //接收数据
    }

    switch (RxHeader.StdId)                                                                  //只解析电调发送的报文
    {
        case CAN_MOTOR_ID_1:
        case CAN_MOTOR_ID_2:
        case CAN_MOTOR_ID_3:
        case CAN_MOTOR_ID_4:
        case CAN_MOTOR_ID_5:
        case CAN_MOTOR_ID_6: {
            static uint8_t n;
            n = RxHeader.StdId - CAN_MOTOR_ID_1;
            moto_chassis[n].round_cnt++ <= 50 ? get_moto_offset(&moto_chassis[n], RxData) : get_motor_measure(
                    &moto_chassis[n], RxData);
//            get_motor_measure(&moto_chassis[n], RxData);                             //调用电机数据解析
        }
            break;
    }

    //据说加入这两行可以解决只收到一个电机报文问题,当然也可能没啥用
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_FilterFIFO0);
    __HAL_CAN_ENABLE_IT(&hcan2, CAN_FilterFIFO0);
}

//电机数据解析,送往进行PID调参
void get_motor_measure(moto_measure *motor, const uint8_t RxData[]) {

    motor->last_angle = motor->angle;
    motor->angle = (uint16_t) (RxData[0] << 8 | RxData[1]);                      //转子的机械角度
    motor->real_current = (int16_t) (RxData[2] << 8 | RxData[3]);                //转子转速
    motor->speed_rpm = motor->real_current;                                      //这里是因为两种电调对应位不一样的信息
    motor->given_current = (int16_t) ((RxData[4] << 8 | RxData[5]) / -5);        //实际输出转矩电流

    if (motor->angle - motor->last_angle > 4096)
        motor->round_cnt--;
    else if (motor->angle - motor->last_angle < -4096)
        motor->round_cnt++;

    motor->total_angle = motor->round_cnt * 8192 + motor->angle - motor->offset_angle;
}

//电机数据接收初始化，须在CAN通讯开始时调用
void get_moto_offset(moto_measure *motor, const uint8_t RxData[]) {

    motor->angle = (uint16_t) (RxData[0] << 8 | RxData[1]);                      //获取电机的初始机械角度
    motor->offset_angle = motor->angle;                                          //将初始机械角度赋值
}

#define    ABS(x)    (((x)>0)? (x): -(x))

void get_total_angle(moto_measure *motor) {                                      //得到电机总共转过的角度

    static int32_t res1, res2, delta = 0;                                        //定义三个中间变量
    if (motor->angle < motor->last_angle)                                        //如果这次的角度比上一次小
    {
        res1 = motor->angle + 8192 - motor->last_angle;                          //正转
        res2 = motor->angle - motor->last_angle;                                 //反转
    } else {
        res1 = motor->angle - 8192 - motor->last_angle;                          //反转
        res2 = motor->angle - motor->last_angle;                                 //正转
    }

    if (ABS(res1) < ABS(res2))                                                   //不管正反转，肯定是转的角度小的那个是真的
        delta = res1;
    else
        delta = res2;

    motor->total_angle += delta;                                                 //得到总共转的角度
    motor->last_angle = motor->angle;                                            //赋值电机上次的机械角度
}

//发送CAN讯息，控制电调电流
HAL_StatusTypeDef set_motor_current(CAN_HandleTypeDef *hcanx, uint16_t stdid, int16_t cm1_iq,
                                    int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq) {

    static uint8_t TxData[8];                                                    //要发送的8位数据
    CAN_TxHeaderTypeDef TxMessage;                                               //CAN数据发送结构体

    TxMessage.StdId = stdid;                                                     //ID赋值
    TxMessage.DLC = 0x08;                                                        //数据长度&位数
    TxMessage.IDE = CAN_ID_STD;                                                  //标准ID
    TxMessage.RTR = CAN_RTR_DATA;                                                //数据帧
    TxMessage.TransmitGlobalTime = DISABLE;                                      //是否要发送时间戳

    //将传入的电调电流填充进入将要发送的TxData中，通过移位操作将uint16_t转换为uint8_t
    TxData[0] = (uint8_t) (cm1_iq >> 8);
    TxData[1] = (uint8_t) cm1_iq;
    TxData[2] = (uint8_t) (cm2_iq >> 8);
    TxData[3] = (uint8_t) cm2_iq;
    TxData[4] = (uint8_t) (cm3_iq >> 8);
    TxData[5] = (uint8_t) cm3_iq;
    TxData[6] = (uint8_t) (cm4_iq >> 8);
    TxData[7] = (uint8_t) cm4_iq;

    HAL_CAN_AddTxMessage(hcanx, &TxMessage, TxData,          //发送数据
                         (uint32_t *) CAN_TX_MAILBOX0/*发送所使用的邮箱*/);

    return HAL_OK;                                                               //回调状态
}
