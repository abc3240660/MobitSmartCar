#ifndef __LED_H
#define __LED_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED�˿ڶ���
#define SIM7000E_PWR PEout(2)
#define SIM7000E_RST PEout(3)
#define LED0 PEout(3)	 
#define LED1 PEout(4)	 
#define LED2 PGout(9)	  
#define LEDX PFout(11)

void LED_Init(void);//��ʼ��		 				    
#endif

















