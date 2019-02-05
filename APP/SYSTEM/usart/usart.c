#include "sys.h"
#include "usart.h"
#include "usart3.h"
#include "ucos_ii.h"
#include "malloc.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4̽���߿�����
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/6/10
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
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
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
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

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�  
		
	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
	TIM7_Int_Init(1000-1,8400-1);	//10ms�ж�һ��
	
  TIM_Cmd(TIM7, DISABLE); //�رն�ʱ��7
	
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

//����1,printf ����
//ȷ��һ�η������ݲ�����USART1_MAX_SEND_LEN�ֽ�
void u1_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART1_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART1_TX_BUF);//�˴η������ݵĳ���
	for(j=0;j<i;j++)//ѭ����������
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//ѭ������,ֱ���������   
		USART_SendData(USART1,(uint8_t)USART1_TX_BUF[j]);   
	}
}
