#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.csom
//修改日期:2011/6/14
//版本：V1.4
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
////////////////////////////////////////////////////////////////////////////////// 	
#define U1_RECV_BUF_CNT		4
#define U1_RECV_LEN_ONE		2560
#define U1_DATA_LEN_ONE		2048
#define USART1_MAX_RECV_LEN		(U1_RECV_LEN_ONE * U1_RECV_BUF_CNT)
#define USART1_MAX_SEND_LEN		400
#define USART1_RX_EN 			1

//extern u8  USART1_RX_BUF[USART1_MAX_RECV_LEN];
extern u8*  USART1_RX_BUF;
extern u8  USART1_TX_BUF[USART1_MAX_SEND_LEN];
extern vu16 USART1_RX_STA[4];
extern u8 U1_RX_ID;// 0~3

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void u1_printf(char* fmt,...);
#endif


