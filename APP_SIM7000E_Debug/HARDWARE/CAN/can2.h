#ifndef __CAN2_H
#define __CAN2_H	 
#include "common.h"	 

//////////////////////////////////////////////////////////////////////////////////	 


	
//CAN2����RX0�ж�ʹ��
#define CAN2_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN2_Mode_Init(u8 mode);//CAN��ʼ��
 
u8 CAN2_Send_Msg(u8* msg,u8 len);						//��������

u8 CAN2_Receive_Msg(u8 *buf);							//��������
u8 CAN2_JumpLamp(u8 times);
#endif
















