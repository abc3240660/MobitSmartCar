#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//按键输入驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


#define KEY0 		PFin(9)   
#define KEY1 		PFin(8)		
#define KEY2 		PFin(7)		
#define WK_UP		PFin(6)		
#define KEY_HAND_BRAKE	PDin(14)		

#define KEY0_PRES 	1
#define KEY1_PRES		2
#define KEY2_PRES		3
#define WKUP_PRES   4

void KEY_Init(void);	//IO初始化
u8 KEY_Scan(u8);  		//按键扫描函数
					    
#endif
