#include "sys.h"
#include "usart6.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"
#include "ucos_ii.h"
#include "blue.h"

__align(8) u8 UART6_TX_BUF[UART6_MAX_SEND_LEN]; 	//发送缓冲,最大UART6_MAX_SEND_LEN字节
u8 UART6_RX_BUF[UART6_MAX_RECV_LEN]; 				//接收缓冲,最大UART6_MAX_RECV_LEN个字节.

void usart6_init(u32 bound)
{   
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOB时钟 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE); //使能USART3时钟 
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  //GPIOB10复用为USART6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOB11复用为USART6
	//USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);          //初始化PB10，PB11
	//USART6 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART6, &USART_InitStructure); //初始化串口1	
	USART_Cmd(USART6, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);         //开启相关中断
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;      //串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		   //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			   //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	  //根据指定的参数初始化VIC寄存器、

	//RESET pin
	/******************************* 蓝牙IO口初始化******************************/
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;    //GPIOA7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;             //普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;          
	GPIO_Init(GPIOD,&GPIO_InitStructure);          //初始化 
	
	PDout(0) = 0;
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	PDout(0) = 1;

	TIM3_Int_Init(500-1,8400-1);	//10ms中断一次
	
  TIM_Cmd(TIM3, DISABLE); //关闭定时器7
	
	UART6_RX_STA=0;				//清零 
}

void UART6_SendChar(u8 ch)
{      
	while ((USART6->SR&0x40)==0);
    USART6->DR = (u8) ch;
}

void UART6_SendData(u8 *str, u16 strlen)
{ 
	u16 k = 0;

	do {
		UART6_SendChar(*(str + k)); k++;
	} while (k < strlen);
} 

vu16 UART6_RX_STA = 0;
void USART6_IRQHandler(void)
{
	u8 res = 0;

	OSIntEnter();    

	if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) != RESET) {
		res = USART_ReceiveData(USART6);
		if (0 == (UART6_RX_STA&(1<<15))) {
			if (UART6_RX_STA < UART6_MAX_RECV_LEN) {
				TIM_SetCounter(TIM3, 0);
				if (0 == UART6_RX_STA) {
					TIM_Cmd(TIM3, ENABLE);
				}

				UART6_RX_BUF[UART6_RX_STA++] = res;
			} else {
				UART6_RX_STA |= 1<<15;
			} 
		}
	}  											 

	OSIntExit();  											 
}
