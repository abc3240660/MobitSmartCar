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

//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
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

void USART3_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else {
					USART_RX_STA|=0x8000;	//��������� 
					debug_process();
					USART_RX_STA = 0;
				}
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 

void usart3_init(u32 bound)
{  	
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	USART_DeInit(USART3);  //��λ����3
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��GPIOB11����GPIOB10
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3	  
	
	USART_InitStructure.USART_BaudRate = bound;//������һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�  
		
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ��� 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}
