#ifndef __USART3_H
#define __USART3_H 
#include "sys.h"
#include "stdio.h"	  

#define U3_RECV_BUF_CNT		4
#define U3_RECV_LEN_ONE		2560
#define U3_DATA_LEN_ONE		2048
#define USART3_MAX_RECV_LEN		(U3_RECV_LEN_ONE * U3_RECV_BUF_CNT)
#define USART3_MAX_SEND_LEN		400
#define USART3_RX_EN 			1

//extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN];
extern u8*  USART3_RX_BUF;
extern u8  USART3_TX_BUF[USART3_MAX_SEND_LEN];
extern vu16 USART3_RX_STA[4];
extern u8 U3_RX_ID;// 0~3

void usart3_init(u32 bound);
void u3_printf(char* fmt,...);
#endif	   
















