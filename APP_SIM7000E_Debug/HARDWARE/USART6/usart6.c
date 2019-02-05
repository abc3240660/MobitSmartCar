#include "sys.h"
#include "usart6.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"
#include "ucos_ii.h"
#include "blue.h"

__align(8) u8 UART6_TX_BUF[UART6_MAX_SEND_LEN]; 	//���ͻ���,���UART6_MAX_SEND_LEN�ֽ�
u8 UART6_RX_BUF[UART6_MAX_RECV_LEN]; 				//���ջ���,���UART6_MAX_RECV_LEN���ֽ�.

void usart6_init(u32 bound)
{   
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOBʱ�� 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE); //ʹ��USART3ʱ�� 
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  //GPIOB10����ΪUSART6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOB11����ΪUSART6
	//USART6�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOB10��GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);          //��ʼ��PB10��PB11
	//USART6 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART6, &USART_InitStructure); //��ʼ������1	
	USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);         //��������ж�
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;      //����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		   //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			   //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	  //����ָ���Ĳ�����ʼ��VIC�Ĵ�����

	//RESET pin
	/******************************* ����IO�ڳ�ʼ��******************************/
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;    //GPIOA7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;             //��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;          
	GPIO_Init(GPIOD,&GPIO_InitStructure);          //��ʼ�� 
	
	PDout(0) = 0;
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	PDout(0) = 1;

	TIM3_Int_Init(500-1,8400-1);	//10ms�ж�һ��
	
  TIM_Cmd(TIM3, DISABLE); //�رն�ʱ��7
	
	UART6_RX_STA=0;				//���� 
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
