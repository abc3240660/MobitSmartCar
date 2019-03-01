#include "sys.h"
#include "usart3.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"
#include "ucos_ii.h"
#include "malloc.h"
#include "can1.h"
#include "delay.h"

#define SET_GPS_GAP "SET-GPS-GAP="
#define SET_HBEAT_GAP "SET-HBEAT-GAP="
#define TRIG_DOOR_OPENED "TRIG-DOOR-OPENED"
#define TRIG_DOOR_CLOSED "TRIG-DOOR-CLOSED"
#define TRIG_INVALID_MOVE "TRIG-INVALID-MOVE"
#define TRIg_bms_charged_timesRTED "TRIG-CHARGE-STARTED"
#define TRIG_CHARGE_STOPED "TRIG-CHARGE-STOPED"
#define PLAY_MP3_MUSIC "PLAY-MP3="
#define SET_PEPS "SET-PEPS="

extern int g_hbeat_gap;
extern u8 g_drlock_sta_chged;
extern u8 g_invaid_move;
extern u8 g_bms_charge_sta_chged;
extern u8 g_mp3_play;
extern u8 g_mp3_play_name[LEN_FILE_NAME+1];
extern int g_gps_trace_gap;

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	USART3->DR = (u8) ch;      
	return ch;
}
#endif

void debug_process(void)
{
    u8 test_mode = 0;

	if (0 == strncmp((const char*)USART_RX_BUF, SET_GPS_GAP, strlen(SET_GPS_GAP))) {
		g_gps_trace_gap = atoi((const char*)(USART_RX_BUF+strlen(SET_GPS_GAP)));
		printf("g_gps_trace_gap = %d\n", g_gps_trace_gap);
	} else if (0 == strncmp((const char*)USART_RX_BUF, SET_HBEAT_GAP, strlen(SET_HBEAT_GAP))) {
		g_hbeat_gap = atoi((const char*)(USART_RX_BUF+strlen(SET_HBEAT_GAP)));
		printf("g_hbeat_gap = %d\n", g_hbeat_gap);
	} else if (0 == strncmp((const char*)USART_RX_BUF, TRIG_DOOR_OPENED, strlen(TRIG_DOOR_OPENED))) {
		g_drlock_sta_chged = 1;
		g_drlock_sta_chged |= 0x80; 
	} else if (0 == strncmp((const char*)USART_RX_BUF, TRIG_DOOR_CLOSED, strlen(TRIG_DOOR_CLOSED))) {
		g_drlock_sta_chged = 0;
		g_drlock_sta_chged |= 0x80; 
	} else if (0 == strncmp((const char*)USART_RX_BUF, TRIG_INVALID_MOVE, strlen(TRIG_INVALID_MOVE))) {
		g_invaid_move = 1;
	} else if (0 == strncmp((const char*)USART_RX_BUF, TRIg_bms_charged_timesRTED, strlen(TRIg_bms_charged_timesRTED))) {
		g_bms_charge_sta_chged = 1;
		g_bms_charge_sta_chged |= 0x80; 
	} else if (0 == strncmp((const char*)USART_RX_BUF, TRIG_CHARGE_STOPED, strlen(TRIG_CHARGE_STOPED))) {
		g_bms_charge_sta_chged = 0;
		g_bms_charge_sta_chged |= 0x80; 
	} else if (0 == strncmp((const char*)USART_RX_BUF, SET_PEPS, strlen(SET_PEPS))) {
        test_mode = atoi((const char*)(USART_RX_BUF+strlen(SET_PEPS)));
        if (0 == test_mode) {
            CAN1_JumpLamp(5);
        } else if (1 == test_mode) {
            CAN1_RingAlarm(5);
        } else if (2 == test_mode) {
            CAN1_StartEngine();
            CAN1_StartEngine();
            CAN1_StartEngine();
            CAN1_StartEngine();
        } else if (3 == test_mode) {
            CAN1_StopEngine();
            CAN1_StopEngine();
            CAN1_StopEngine();
            CAN1_StopEngine();
        } else if (4 == test_mode) {
            CAN1_OpenDoor();
            CAN1_OpenDoor();
            CAN1_OpenDoor();
            CAN1_OpenDoor();
        } else if (5 == test_mode) {
            CAN1_CloseDoor();
            CAN1_CloseDoor();
            CAN1_CloseDoor();
            CAN1_CloseDoor();
        } else if (6 == test_mode) {
        } else if (7 == test_mode) {
            CAN1_JumpLamp(5);
            CAN1_JumpLamp(5);
            CAN1_JumpLamp(5);
            CAN1_JumpLamp(5);
        } else if (8 == test_mode) {
            CAN1_RingAlarm(5);
            CAN1_RingAlarm(5);
            CAN1_RingAlarm(5);
            CAN1_RingAlarm(5);
        } else {
        }
	} else if (0 == strncmp((const char*)USART_RX_BUF, PLAY_MP3_MUSIC, strlen(PLAY_MP3_MUSIC))) {
		memset(g_mp3_play_name, 0, LEN_FILE_NAME);
		strncpy((char*)g_mp3_play_name, (const char*)(USART_RX_BUF+strlen(PLAY_MP3_MUSIC)), LEN_FILE_NAME);
		printf("g_mp3_play_name = %s\n", g_mp3_play_name);

		g_mp3_play = 1;
	}

	USART_RX_STA = 0;
}

void USART3_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else {
					USART_RX_STA|=0x8000;	//接收完成了 
					debug_process();
					USART_RX_STA = 0;
				}
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 

void usart3_init(u32 bound)
{  	
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	USART_DeInit(USART3);  //复位串口3
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化GPIOB11，和GPIOB10
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3	  
	
	USART_InitStructure.USART_BaudRate = bound;//波特率一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断  
		
	USART_Cmd(USART3, ENABLE);                    //使能串口 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
