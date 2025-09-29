/*****************************************************************************************************************************
 * @file: bsp_can.h
 * @author: Shiki
 * @date: 2025.9.23
 * @brief:	哨兵2025赛季CAN通讯支持包，此文件适配云台C板
 *****************************************************************************************************************************/

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "pid.h"
#include "can.h"

#define BIG_YAW_DM6006_TransID 0x01 // CAN2

typedef enum
{
	CAN_RC_TO_CHASSIS_FIRST_CMD,
	CAN_RC_TO_CHASSIS_SECOND_CMD,
	CAN_BIG_YAW_CMD,
	CAN_SMALL_YAW_AND_PITCH_CMD,
	CAN_FRIC_CMD,
	CAN_DIAL_CMD
} CAN_CMD_ID; // CAN发送命令类型,用于把不同的can消息送入对应消息队列统一发送

typedef struct
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t data[8];
} CanTxMsgTypeDef;   // CAN报文结构体，用于can发送队列中


void Can_Filter_Init(void);
void Can_Buffer_Init(void);
void Create_Can_Send_Queues(void);
void Allocate_Can_Buffer(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id); 
void CAN_TX_TimerIRQHandler(void);
void CAN_Resend_Timer_IRQHandler(void);
void Ctrl_DM_Motor(float _pos, float _vel, float _KP, float _KD, float _torq);
void enable_DM(uint8_t id, uint8_t ctrl_mode);
void disable_DM(uint8_t id, uint8_t ctrl_mode);


#endif
