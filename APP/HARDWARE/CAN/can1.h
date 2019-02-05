#ifndef __CAN1_H
#define __CAN1_H	 
#include "common.h"	 

//////////////////////////////////////////////////////////////////////////////////	 


	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
										 							 				    
u8 CAN1_Mode_Init(u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据

u8 CAN1_StartEngine(void);
u8 CAN1_StopEngine(void);

u8 CAN1_OpenDoor(void);
u8 CAN1_CloseDoor(void);

u8 CAN1_JumpLamp(u8 times);
u8 CAN1_RingAlarm(u8 times);

u8 CAN1_OneKeyStart(void);

#endif

















