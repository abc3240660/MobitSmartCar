#include "can2.h"
#include "led.h"
#include "common.h"

extern u16 g_bms_vot;
extern u16 g_bms_percent;
extern u16 g_bms_temp_max;
extern u16 g_bms_temp_min;
extern u16 g_bms_charged_times;// Save into ExFlash

// BIT7: 0-idle, 1-changed
// BIT0: 0-Start, 1-Stop
extern u8 g_bms_charge_sta_chged;

char log_msg_bms[64] = {0};
u16 test_cnt_bms = 0;

/****************************************************************************
* ��    ��: u8 CAN2_Mode_Init(u8 mode)
* ��    �ܣ�CAN��ʼ��
* ��ڲ�����mode:CAN����ģʽ;0,��ͨģʽ;1,����ģʽ
* ���ز�����0,�ɹ�;
           	����,ʧ��;
* ˵    ����       
****************************************************************************/	
u8 CAN2_Mode_Init(u8 mode)
{
  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2, ENABLE); //ʹ��CAN2��ʱ��ҲҪʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOA12����ΪCAN1
	  
//	CAN_DeInit(CAN2);
//	CAN_StructInit(&CAN_InitStructure);
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
		
/***************************************************************************************/		
    //����can����ģʽ	
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq; //ʱ���1��ʱ�䵥Ԫ.  Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq; //ʱ���2��ʱ�䵥Ԫ.  Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=6;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
/***************************************************************************************/			
  	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN2 
    
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=14;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN2_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN2_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN2_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	
    // CAN_Receive(CAN2, 0, &RxMessage);
		CAN2_Receive_Msg(NULL);
	
		// printf("CAN2 Recved Msg 0x%.8X\n", RxMessage.ExtId);
}
#endif

/****************************************************************************
* ��    ��: u8 CAN2_Send_Msg(u8* msg,u8 len)
* ��    �ܣ�can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)
* ��ڲ�����len:���ݳ���(���Ϊ8)				     
            msg:����ָ��,���Ϊ8���ֽ�.
* ���ز�����0,�ɹ�;
           	����,ʧ��;
* ˵    ����       
****************************************************************************/	
u8 CAN2_Send_Msg(u8* msg,u8 len)
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
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		
}

/****************************************************************************
* ��    ��: u8 CAN2_Receive_Msg(u8 *buf)
* ��    �ܣ�can�ڽ������ݲ�ѯ
* ��ڲ�����buf:���ݻ�����;	 			     
* ���ز�����0,�����ݱ��յ�;
    		    ����,���յ����ݳ���;
* ˵    ����       
****************************************************************************/
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];
	
		if (0x18FF28F4 == RxMessage.ExtId) {
			if (0x02 == (RxMessage.Data[0]&0x2)) {// battery charging
				if (0 == g_bms_charge_sta_chged) {
					g_bms_charge_sta_chged = 1;
					printf("bms charge sta = 1\n");
					g_bms_charged_times++;
					sys_env_init();

                    g_bms_charge_sta_chged |= 0x80;
				}
			} else {
				if (1 == g_bms_charge_sta_chged) {
					g_bms_charge_sta_chged = 0;
					printf("bms charge sta = 0\n");
                    g_bms_charge_sta_chged |= 0x80;
				}
			}
			
			g_bms_percent = RxMessage.Data[1];
            g_bms_vot = ((RxMessage.Data[5]<<8) + RxMessage.Data[4]) / 10;

            test_cnt_bms++;

						//printf("bms xx percent = %d, vot = %d(L%d)\n", g_bms_percent, g_bms_vot, RxMessage.Data[4]);
			
            if ((test_cnt_bms>50)/* || (g_bms_charge_sta_chged&0x80)*/) {
                test_cnt_bms = 0;

                printf("bms percent = %d, vot = %d(L%d)\n", g_bms_percent, g_bms_vot, RxMessage.Data[4]);

                memset(log_msg_bms, 0, 64);
                sprintf(log_msg_bms, "RECV: %.8X - %.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X", RxMessage.ExtId, RxMessage.Data[0], RxMessage.Data[1], RxMessage.Data[2], RxMessage.Data[3], RxMessage.Data[4], RxMessage.Data[5], RxMessage.Data[6], RxMessage.Data[7]);

                write_logs("CAN2", (char*)log_msg_bms, strlen((char*)log_msg_bms), 2);
			}
		} else if (0x18FE28F4 == RxMessage.ExtId) {
			g_bms_temp_max = RxMessage.Data[4];
			g_bms_temp_min = RxMessage.Data[5];
		} else {
			if (0x18B00000 == (RxMessage.ExtId&0xFFF)) {// All Temp
				u8 index = ((RxMessage.ExtId&0xFFFF)>>15) - 0x18B4;
			} else if (0x18C00000 == (RxMessage.ExtId&0xFFF)) {// All Vot
				u8 index = ((RxMessage.ExtId&0xFFFF)>>15) - 0x18C8;
			} 
		}

	return RxMessage.DLC;
}

