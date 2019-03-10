#include "can1.h"
#include "led.h"
#include "sim900a.h"

char log_msg[64] = {0};
u16 test_cnt_mc3264 = 0;

u32 can1_rx_cnt = 0;


#define BIT_PEPS_LOCK					(1 << 0)
#define BIT_PEPS_ENGINE				(1 << 1)
#define BIT_PEPS_LAMP   			(1 << 2)
#define BIT_PEPS_ALARM				(1 << 3)
#define BIT_PEPS_UNLOCK				(1 << 4)
#define BIT_PEPS_ENGINE_STOP	(1 << 5)

// BIT0-lock status
// BIT1-engine status
// BIT2-lamp status
// BIT3-alarm status
u8 g_peps_sta = 0;

// BIT0-lock request
// BIT1-engine request
// BIT2-lamp request
// BIT3-alarm request
u8 g_peps_req = 0;

u8 g_relamp_cnt = 0;
u8 g_realarm_cnt = 0;

u8 g_relock_cnt = 0;
u8 g_reunlock_cnt = 0;

u8 g_restart_engine_cnt = 0;
u8 g_restop_engine_cnt = 0;

extern u16 g_car_sta;
extern u8 g_door_state;
extern u8 g_power_state;
extern u32 g_trip_meters;
extern u8 g_drlock_sta_chged;
extern u8 g_dropen_sta_chged;

void write_logs(char *module, char *log, u16 size, u8 mode);
u8 CAN1_JumpLamp_no_delay(u8 times);
u8 CAN1_RingAlarm_no_delay(u8 times);
u8 CAN1_OpenDoor_no_delay(void);
u8 CAN1_CloseDoor_no_delay(void);
u8 CAN1_StartEngine_no_delay(void);
u8 CAN1_StopEngine_no_delay(void);
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
	
		can1_rx_cnt++;

    // CAN_Receive(CAN1, 0, &RxMessage);
		CAN1_Receive_Msg(NULL);
	
		// printf("CAN1 Recved Msg 0x%.8X\n", RxMessage.ExtId);
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
	u32 can1_rx_cnt_old = can1_rx_cnt;
	
  TxMessage.StdId=0x78563412;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x1004C899;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=CAN_ID_EXT;		   // ʹ����չ��ʶ��
  TxMessage.RTR=0;		   // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;	   // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
	
	i = 0;
	while(1) {
		// just RX 1ms ago
		if (can1_rx_cnt_old != can1_rx_cnt) {
			break;
		}
		// No RX for 1s
		if (i >= 1000) {
			break;
		}
		
		i++;
		delay_ms(1);
	}
	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  // while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)) {
		i++;
		delay_ms(1);
	}
	
  if(i>=0XFFF)return 1;
  return 0;		
}

u8 CAN1_Send_Msg_no_delay(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
	u32 can1_rx_cnt_old = can1_rx_cnt;
	
  TxMessage.StdId=0x78563412;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x1004C899;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=CAN_ID_EXT;		   // ʹ����չ��ʶ��
  TxMessage.RTR=0;		   // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;	   // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  // while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)) {
		i++;
		delay_ms(1);
	}
	
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
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
//    for(i=0;i<RxMessage.DLC;i++)
//    buf[i]=RxMessage.Data[i];  

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
			g_peps_sta |= BIT_PEPS_LOCK;// locked

			if (1 == (g_drlock_sta_chged&0x7F)) {
				g_drlock_sta_chged = 0;
                g_drlock_sta_chged |= 0x80;
                printf("peps door locked\n");
			}
		} else {// unlocked
			g_peps_sta &= ~BIT_PEPS_LOCK;// unlocked

			if (0 == (g_drlock_sta_chged&0x7F)) {
				g_drlock_sta_chged = 1;
                g_drlock_sta_chged |= 0x80;
                printf("peps door unlocked\n");
			}
		}

		if (0x01 == (RxMessage.Data[0]&0x1)) {// Opened
			g_door_state = 1;
			g_car_sta |= BIT_LEFT_DOOR;
			g_car_sta |= BIT_RIGHT_DOOR;

			if (0 == (g_dropen_sta_chged&0x7F)) {
				g_dropen_sta_chged = 1;
                g_dropen_sta_chged |= 0x80;
                printf("peps door opened\n");
			}
		} else {// Closed
			g_door_state = 0;
			g_car_sta &= ~BIT_LEFT_DOOR;
			g_car_sta &= ~BIT_RIGHT_DOOR;

			if (1 == (g_dropen_sta_chged&0x7F)) {
				g_dropen_sta_chged = 0;
                g_dropen_sta_chged |= 0x80;
                printf("peps door closed\n");
			}
		}

		if (0x20 == (RxMessage.Data[0]&0x20)) {// Power
			g_power_state = 1;
			g_peps_sta |= BIT_PEPS_ENGINE;// ON
		} else {
			g_power_state = 0;
			g_peps_sta &= ~BIT_PEPS_ENGINE;// OFF
		}
		
		if (0x01 == (RxMessage.Data[1]&0x01)) {// Lamp
			g_peps_sta |= BIT_PEPS_LAMP;// ON
		} else {
			g_peps_sta &= ~BIT_PEPS_LAMP;// OFF
		}
		
		if (0x02 == (RxMessage.Data[1]&0x2)) {// Alarm
			g_peps_sta |= BIT_PEPS_ALARM;// ON
		} else {
			g_peps_sta &= ~BIT_PEPS_ALARM;// OFF
		}

		if (g_peps_req&BIT_PEPS_LAMP) {
			if (g_peps_sta&BIT_PEPS_LAMP) {// Lamping
				g_peps_req &= ~BIT_PEPS_LAMP;// Clear Request
			} else {
				g_relamp_cnt++;
				if (g_relamp_cnt >= 2) {// 1s
					g_relamp_cnt = 0;
					CAN1_JumpLamp_no_delay(5);
				}
			}
		} else {
			g_relamp_cnt = 0;
		}

		if (g_peps_req&BIT_PEPS_ALARM) {
			if (g_peps_sta&BIT_PEPS_ALARM) {// Alarming
				g_peps_req &= ~BIT_PEPS_ALARM;// Clear Request
			} else {
				g_realarm_cnt++;
				if (g_realarm_cnt >= 2) {// 1s
					g_realarm_cnt = 0;
					CAN1_RingAlarm_no_delay(5);
				}
			}
		} else {
			g_realarm_cnt = 0;
		}

		if (g_peps_req&BIT_PEPS_LOCK) {
			if (g_peps_sta&BIT_PEPS_LOCK) {// Locking
				g_peps_req &= ~BIT_PEPS_LOCK;// Clear Request
			} else {
				g_relock_cnt++;
				if (g_relock_cnt >= 2) {// 1s
					g_relock_cnt = 0;
					CAN1_CloseDoor_no_delay();
				}
			}
		} else {
			g_relock_cnt = 0;
		}

		if (g_peps_req&BIT_PEPS_UNLOCK) {
			if (!(g_peps_sta&BIT_PEPS_LOCK)) {// Unlocking
				g_peps_req &= ~BIT_PEPS_UNLOCK;// Clear Request
			} else {
				g_reunlock_cnt++;
				if (g_reunlock_cnt >= 2) {// 1s
					g_reunlock_cnt = 0;
					CAN1_OpenDoor_no_delay();
				}
			}
		} else {
			g_reunlock_cnt = 0;
		}

		if (g_peps_req&BIT_PEPS_ENGINE) {
			if (g_peps_sta&BIT_PEPS_ENGINE) {// Starting
				g_peps_req &= ~BIT_PEPS_ENGINE;// Clear Request
			} else {
				g_restart_engine_cnt++;
				if (g_restart_engine_cnt >= 2) {// 1s
					g_restart_engine_cnt = 0;
					CAN1_StartEngine_no_delay();
				}
			}
		} else {
			g_restart_engine_cnt = 0;
		}

		if (g_peps_req&BIT_PEPS_ENGINE_STOP) {
			if (!(g_peps_sta&BIT_PEPS_ENGINE)) {// Stoping
				g_peps_req &= ~BIT_PEPS_ENGINE_STOP;// Clear Request
			} else {
				g_restop_engine_cnt++;
				if (g_restop_engine_cnt >= 2) {// 1s
					g_restop_engine_cnt = 0;
					CAN1_StopEngine_no_delay();
				}
			}
		} else {
			g_restop_engine_cnt = 0;
		}

	// Byte1:
	// BIT0~1: 00-N, 01-D, 10-R
	// Byte2:Speed LB(rpm/bit)
	// Byte3:Speed HB(rpm/bit)
	// Byte6:TripMeter LB(0.1km/bit)
	// Byte7:TripMeter HB(0.1km/bit)
	} else if (0x10F8109A == RxMessage.ExtId) {// MC3624
		// Get N/D/R
		g_car_sta &= 0xFF;
		g_car_sta |= (((RxMessage.Data[0])&0x03)<<8);

		g_trip_meters = (RxMessage.Data[6]<<8) + RxMessage.Data[5];

        test_cnt_mc3264++;

        if (test_cnt_mc3264 > 10) {
						u32 speed = (RxMessage.Data[2]<<8) + RxMessage.Data[1];;
					
						// (speed*314*465*3600)/(100*12*1000*1000);
						speed = (speed*314*93*3)/(1*1*200*1000*60);
						printf("speed = %d KM/H\n", speed);

            memset(log_msg, 0, 64);
            sprintf(log_msg, "RECV: %.8X - %.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X", RxMessage.ExtId, RxMessage.Data[0], RxMessage.Data[1], RxMessage.Data[2], RxMessage.Data[3], RxMessage.Data[4], RxMessage.Data[5], RxMessage.Data[6], RxMessage.Data[7]);

            //write_logs("CAN1", (char*)log_msg, strlen((char*)log_msg), 2);
            test_cnt_mc3264 = 0;
        }
	}

	return RxMessage.DLC;	
}

u8 CAN1_Wakeup(void)
{
	u8 can1_sendbuf[8]={0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_StartEngine(void)
{
	u8 can1_sendbuf[8]={0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Wakeup();
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 

	g_peps_req |= BIT_PEPS_ENGINE;
	
	printf("%02d%02d%02d:PEPS StartEngine...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);

	return 0;
}

u8 CAN1_StartEngine_no_delay(void)
{
	u8 can1_sendbuf[8]={0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Send_Msg_no_delay(can1_sendbuf,8);//����8���ֽ� 

	printf("%02d%02d%02d:Retry PEPS StartEngine...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);

	return 0;
}

u8 CAN1_StopEngine(void)
{
	u8 can1_sendbuf[8]={0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Wakeup();
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 

	g_peps_req |= BIT_PEPS_ENGINE_STOP;

	printf("%02d%02d%02d:PEPS StopEngine...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	
	return 0;
}

u8 CAN1_StopEngine_no_delay(void)
{
	u8 can1_sendbuf[8]={0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Send_Msg_no_delay(can1_sendbuf,8);//����8���ֽ� 

	printf("%02d%02d%02d:Retry PEPS StopEngine...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	
	return 0;
}

u8 CAN1_OpenDoor(void)
{
	u8 can1_sendbuf[8]={0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Wakeup();
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 

	g_peps_req |= BIT_PEPS_UNLOCK;
	
	printf("%02d%02d%02d:PEPS OpenDoor...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	
	return 0;
}

u8 CAN1_OpenDoor_no_delay(void)
{
	u8 can1_sendbuf[8]={0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Send_Msg_no_delay(can1_sendbuf,8);//����8���ֽ� 

	printf("%02d%02d%02d:Retry PEPS OpenDoor...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	
	return 0;
}

u8 CAN1_CloseDoor(void)
{
	u8 can1_sendbuf[8]={0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Wakeup();
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 

	g_peps_req |= BIT_PEPS_LOCK;
	
	printf("%02d%02d%02d:PEPS CloseDoor...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);

	return 0;
}

u8 CAN1_CloseDoor_no_delay(void)
{
	u8 can1_sendbuf[8]={0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Send_Msg_no_delay(can1_sendbuf,8);//����8���ֽ� 

	printf("%02d%02d%02d:Retry PEPS CloseDoor...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);

	return 0;
}

u8 CAN1_JumpLamp(u8 times)
{
	u8 can1_sendbuf[8]={0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);
	
	CAN1_Wakeup();
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 

	g_peps_req |= BIT_PEPS_LAMP;

	printf("%02d%02d%02d:PEPS JumpLamp...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	
	return 0;
}

u8 CAN1_JumpLamp_no_delay(u8 times)
{
	u8 can1_sendbuf[8]={0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);
	
	CAN1_Send_Msg_no_delay(can1_sendbuf,8);//����8���ֽ� 

	printf("%02d%02d%02d:Retry PEPS JumpLamp...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	
	return 0;
}

u8 CAN1_RingAlarm(u8 times)
{
	u8 can1_sendbuf[8]={0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Wakeup();
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	g_peps_req |= BIT_PEPS_ALARM;

	printf("%02d%02d%02d:PEPS RingAlarm...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	
	return 0;
}

u8 CAN1_RingAlarm_no_delay(u8 times)
{
	u8 can1_sendbuf[8]={0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

	CAN1_Send_Msg_no_delay(can1_sendbuf,8);//����8���ֽ� 

	printf("%02d%02d%02d:Retry PEPS RingAlarm...\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	
	return 0;
}

u8 CAN1_StartAll(void)
{
	u8 can1_sendbuf[8]={0x3F, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Wakeup();
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}

u8 CAN1_StartHint(void)
{
	u8 can1_sendbuf[8]={0x0C, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Wakeup();
	CAN1_Send_Msg(can1_sendbuf,8);//����8���ֽ� 
	
	return 0;
}
