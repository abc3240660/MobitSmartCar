#include "can1.h"
#include "led.h"
#include "sim900a.h"

extern u16 g_car_sta;
extern u8 g_door_state;
extern u8 g_power_state;
extern u32 g_trip_meters;
extern u8 g_drlock_sta_chged;
/****************************************************************************
* ��    ��: u8 CAN1_Mode_Init(u8 mode)
* ��    �ܣ�CAN��ʼ��
* ��ڲ�����mode:CAN����ģʽ;0,��ͨģʽ;1,����ģʽ
* ���ز�����0,�ɹ�;
           	����,ʧ��;
* ˵    ����       
****************************************************************************/	
u8 CAN1_Mode_Init(u8 mode)
{
  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ����
	 
/***************************************************************************************/		
    //����can����ģʽ	
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq; //ʱ���1��ʱ�䵥Ԫ.  Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq; //ʱ���2��ʱ�䵥Ԫ.  Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=12;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
/***************************************************************************************/			
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
    CAN_Receive(CAN1, 0, &RxMessage);
	
		printf("CAN1 Recved Msg 0x%.8X\n", RxMessage.ExtId);
}
#endif

/****************************************************************************
* ��    ��: u8 CAN1_Send_Msg(u8* msg,u8 len)
* ��    �ܣ�can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)
* ��ڲ�����len:���ݳ���(���Ϊ8)				     
            msg:����ָ��,���Ϊ8���ֽ�.
* ���ز�����0,�ɹ�;
           	����,ʧ��;
* ˵    ����       
****************************************************************************/		
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x78563412;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x1004C899;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=CAN_ID_EXT;		   // ʹ����չ��ʶ��
  TxMessage.RTR=0;		   // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;	   // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		
}

/****************************************************************************
* ��    ��: u8 CAN1_Receive_Msg(u8 *buf)
* ��    �ܣ�can�ڽ������ݲ�ѯ
* ��ڲ�����buf:���ݻ�����;	 			     
* ���ز�����0,�����ݱ��յ�;
    		    ����,���յ����ݳ���;
* ˵    ����       
****************************************************************************/	
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  

	// Byte1:
	// BIT0-door: 0-closed / 1-opened
	// BIT1-frontdoor: 0-unlocked / 1-locked
	// BIT2-backdoor: 0-unlocked / 1-locked
	// BIT3-ShaCheTaBan: 0-Down / 1-Release
	// BIT4-ACC12: 0-LowLevel / 1-HighLevel
	// BIT5-ON12: 0-LowLevel / 1-HighLevel
	// BIT6&7-PEPS DangWei: 00-OFF / 01-ACC / 10-ON
	// Byte2:
	// BIT0-lamp: 0-OFF / 1-ON
	// BIT1-ring: 0-OFF / 1-ON
	if (0x100850C8 == RxMessage.ExtId) {// PEPS
		if (0x02 == (RxMessage.Data[0]&0x2)) {// locked
			if (1 == g_drlock_sta_chged) {
				g_drlock_sta_chged = 0;
			}
		} else {// unlocked
			if (0 == g_drlock_sta_chged) {
				g_drlock_sta_chged = 1;
			}
		}

		if (0x01 == (RxMessage.Data[0]&0x1)) {// Opened
			g_door_state = 1;
			g_car_sta |= (1<<BIT_LEFT_DOOR);
			g_car_sta |= (1<<BIT_RIGHT_DOOR);
		} else {// Closed
			g_door_state = 0;
			g_car_sta &= ~(1<<BIT_LEFT_DOOR);
			g_car_sta &= ~(1<<BIT_RIGHT_DOOR);
		}

		if (0x20 == (RxMessage.Data[0]&0x20)) {// Power ON
			g_power_state = 1;
		} else {// Power OFF
			g_power_state = 0;
		}

		g_drlock_sta_chged |= 0x80;
	// Byte1:
	// BIT0~1: 00-N, 01-D, 10-R
	// Byte2:Speed LB(rpm/bit)
	// Byte3:Speed HB(rpm/bit)
	// Byte6:TripMeter LB(0.1km/bit)
	// Byte7:TripMeter HB(0.1km/bit)
	// TBD: TotalMeter need to calc
	} else if (0x10F8109A == RxMessage.ExtId) {// MC3624
		// Get N/D/R
		g_car_sta &= 0xFF;
		g_car_sta |= (((RxMessage.Data[0])&0x03)<<8);

		g_trip_meters = (RxMessage.Data[7]<<8) + RxMessage.Data[6];
	}

	return RxMessage.DLC;	
}

u8 CAN1_StartEngine(void)
{
	u8 can1_sendbuf[8]={0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_StopEngine(void)
{
	u8 can1_sendbuf[8]={0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_OpenDoor(void)
{
	u8 can1_sendbuf[8]={0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_CloseDoor(void)
{
	u8 can1_sendbuf[8]={0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_JumpLamp(u8 times)
{
	u8 can1_sendbuf[8]={0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_RingAlarm(u8 times)
{
	u8 can1_sendbuf[8]={0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_StartAll(void)
{
	u8 can1_sendbuf[8]={0x3F, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_StartHint(void)
{
	u8 can1_sendbuf[8]={0x0C, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}
