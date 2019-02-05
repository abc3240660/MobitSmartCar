#include "sys.h"
#include "usart.h"
#include "usart3.h"
#include "ucos_ii.h"
#include "malloc.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
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
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  


__align(8) u8 USART1_TX_BUF[USART1_MAX_SEND_LEN];

//u8 USART1_RX_BUF[USART1_MAX_RECV_LEN];
u8* USART1_RX_BUF = NULL;
u8 U1_RX_ID = 0;// 0~3
vu16 USART1_RX_STA[4] = {0};

// Get free buf id for next use
// >>> U1_RX_ID <<<
// Not Busy:    0->1->0->1->0->1->... (always switch between 0 and 1)
// little Busy: 0->1->2->0->1->0->1->... (may use till 2)
// very Busy:   0->1->2->3->0->1->0->1->... (may use till 3)
void USART1_Get_Free_Buf(void)
{
	u8 i = 0;
	//U1_RX_ID = 0;
	for (i=0; i<U1_RECV_BUF_CNT; i++) {
		if ((USART1_RX_STA[i]&(1<<15)) == 0) {// first free Buf
			U1_RX_ID = i;
			memset(USART1_RX_BUF+U1_RECV_LEN_ONE*i, 0, U1_RECV_LEN_ONE);
			//printf("switch to buf id = %d\n", i);
			break;
		}
	}
}

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断  
		
	USART_Cmd(USART1, ENABLE);                    //使能串口

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	TIM7_Int_Init(1000-1,8400-1);	//10ms中断一次
	
  TIM_Cmd(TIM7, DISABLE); //关闭定时器7
	
	USART1_RX_BUF = mymalloc(SRAMIN, USART1_MAX_RECV_LEN);
	
	USART1_RX_STA[0] = 0;
	USART1_RX_STA[1] = 0;
	USART1_RX_STA[2] = 0;
	USART1_RX_STA[3] = 0;
}

void USART1_IRQHandler(void)
{
	u8 res = 0;

	OSIntEnter();

	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
		res = USART_ReceiveData(USART1);
		if ((USART1_RX_STA[U1_RX_ID]&(1<<15)) == 0) {// frame not complete
			if (USART1_RX_STA[U1_RX_ID] < USART1_MAX_RECV_LEN) {
				TIM_SetCounter(TIM7, 0);
				if (USART1_RX_STA[U1_RX_ID] == 0) {
					TIM_Cmd(TIM7, ENABLE);
				}
				USART1_RX_BUF[U1_RECV_LEN_ONE*U1_RX_ID + USART1_RX_STA[U1_RX_ID]++] = res;
			} else {
				USART1_RX_STA[U1_RX_ID] |= 1<<15;
			}
		}
	}

	OSIntExit();
}

//串口1,printf 函数
//确保一次发送数据不超过USART1_MAX_SEND_LEN字节
void u1_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART1_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART1_TX_BUF);//此次发送数据的长度
	for(j=0;j<i;j++)//循环发送数据
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//循环发送,直到发送完毕   
		USART_SendData(USART1,(uint8_t)USART1_TX_BUF[j]);   
	}
}
