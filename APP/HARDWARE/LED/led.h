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
#define HC08_BT_RST  PBout(7)
#define SIM7000E_PWR PEout(2)
#define SIM7000E_RST PEout(3)

#define LED_G PDout(7)
#define LED_Y PDout(6)
#define LED_R PDout(5)

#define LED_M PDout(3)
#define LED_N PDout(4)

void LED_Init(void);//��ʼ��		 				    
#endif

















