#ifndef __USART3_H
#define __USART3_H 
#include "sys.h"
#include "stdio.h"	  

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	

void usart3_init(u32 bound);

void UART3_SendData(u8 *str, u16 strlen);
#endif	   















