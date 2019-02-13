#ifndef __LED_H
#define __LED_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED端口定义
#define HC08_BT_RST  PBout(7)
#define SIM7000E_PWR PEout(2)
#define SIM7000E_RST PEout(3)

#define LED_G PDout(7)
#define LED_Y PDout(6)
#define LED_R PDout(5)

#define LED_M PDout(3)
#define LED_N PDout(4)

void LED_Init(void);//初始化		 				    
#endif

















