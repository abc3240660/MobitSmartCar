#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.csom
//�޸�����:2011/6/14
//�汾��V1.4
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
////////////////////////////////////////////////////////////////////////////////// 	
#define U1_RECV_LEN_ONE		2560
#define U1_DATA_LEN_ONE		2048
#define USART1_MAX_SEND_LEN		400
#define USART1_RX_EN 			1

#define U1_RX_BUF_CNT		5
#define U1_RX_LEN_ONE		256

extern u8* USART1_RX_BUF;
extern u8  USART1_TX_BUF[USART1_MAX_SEND_LEN];
extern vu16 USART1_RX_STA;

extern u8* DW_RX_BUF;
extern u8* AT_RX_BUF;
extern u8* MOBIT_RX_BUF;

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void u1_printf(char* fmt,...);
#endif


