#include "sys.h"
#include "usart5.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"
#include "ucos_ii.h"

__align(8) u8 UART5_TX_BUF[UART5_MAX_SEND_LEN]; 	//���ͻ���,���UART5_MAX_SEND_LEN�ֽ�
//���ڽ��ջ����� 	
u8 UART5_RX_BUF[UART5_MAX_RECV_LEN]; 				//���ջ���,���UART5_MAX_RECV_LEN���ֽ�.
u8 UART5_RX_BUF_BAK[UART5_MAX_RECV_LEN];

void uart5SendChar(u8 ch)
{      
		while((USART2->SR&0x40)==0);  
    USART2->DR = (u8) ch;      
}

void UART5_SendData(u8 *str, u16 strlen)
{ 
	u16 k = 0;

	do {
		uart5SendChar(*(str + k)); k++;
	} while (k < strlen);
} 

//ͨ���жϽ�������2���ַ�֮���ʱ������10ms�������ǲ���һ������������.
//���2���ַ����ռ������10ms,����Ϊ����1����������.Ҳ���ǳ���10msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
vu16 UART5_RX_STA=0;   	 
vu16 UART5_RX_STA_BAK=0;
void USART2_IRQHandler(void)
{
	u8 res;	    
	OSIntEnter();    
	if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)//���յ�����
	{	 
		res=USART_ReceiveData(USART2);
		if((UART5_RX_STA&(1<<15))==0)//�������һ������,��û�б�����,���ٽ�����������
		{ 
			if(UART5_RX_STA<UART5_MAX_RECV_LEN)	//�����Խ�������
			{
				TIM_SetCounter(TIM6,0);       				//���������
				if(UART5_RX_STA==0) 				//ʹ�ܶ�ʱ��7���ж� 
				{
					TIM_Cmd(TIM6, ENABLE); 	    			//ʹ�ܶ�ʱ��7
				}
				UART5_RX_BUF[UART5_RX_STA++]=res;	//��¼���յ���ֵ	 
			}else 
			{
				UART5_RX_STA|=1<<15;				//ǿ�Ʊ�ǽ������
			} 
		} else {
			if (UART5_RX_STA_BAK >= UART5_MAX_RECV_LEN) {
				UART5_RX_STA_BAK = 0;
			}
			
			UART5_RX_BUF_BAK[UART5_RX_STA_BAK++]=res;
		}
	}  											 
	OSIntExit();  											 
}   
//��ʼ��IO ����3
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������ 
void usart5_init(u32 bound)
{  	
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	USART_DeInit(USART2);  //��λ����3
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��UART5ʱ��
	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��GPIOB11����GPIOB10
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA,&GPIO_InitStructure);   
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOB11����ΪUART5
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOB10����ΪUART5	  
	
	USART_InitStructure.USART_BaudRate = bound;//������һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //��ʼ������3
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�  
		
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 
	
 
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	TIM6_Int_Init(500-1,8400-1);	//10ms�ж�һ��
	
  TIM_Cmd(TIM6, DISABLE); //�رն�ʱ��7
	
	UART5_RX_STA=0;				//���� 
}

//����3,printf ����
//ȷ��һ�η������ݲ�����UART5_MAX_SEND_LEN�ֽ�
void u5_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)UART5_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART5_TX_BUF);//�˴η������ݵĳ���
	for(j=0;j<i;j++)//ѭ����������
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//ѭ������,ֱ���������   
		USART_SendData(USART2,(uint8_t)UART5_TX_BUF[j]);   
	}
}



































