#ifndef __UART6_H
#define __UART6_H 
#include "sys.h"
#include "stdio.h"	  
//////////////////////////////////////////////////////////////////////////////////	   
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����3��ʼ������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/5/14
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
////////////////////////////////////////////////////////////////////////////////// 	
#define UART6_REC_NUM  			200  	//�����������ֽ��� 200

#define UART6_MAX_RECV_LEN		400					//�����ջ����ֽ���
#define UART6_MAX_SEND_LEN		400					//����ͻ����ֽ���
#define UART6_RX_EN 			1					//0,������;1,����.

extern u8  UART6_RX_BUF[UART6_MAX_RECV_LEN]; 		//���ջ���,���UART6_MAX_RECV_LEN�ֽ�
extern u8  UART6_TX_BUF[UART6_MAX_SEND_LEN]; 		//���ͻ���,���UART6_MAX_SEND_LEN�ֽ�
extern vu16 UART6_RX_STA;   						//��������״̬

void usart6_init(u32 bound);
void UART6_SendData(u8 *str, u16 strlen);

#endif	   
















