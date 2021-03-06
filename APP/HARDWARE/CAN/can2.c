#include "can2.h"
#include "led.h"
#include "common.h"

extern u8 g_bms_temp_max;// 30
extern u8 g_bms_temp_min;// 30
extern u8 g_bms_battery_vol;// 50%
extern u16 g_bms_charged_times;// Save into ExFlash

// BIT7: 0-idle, 1-changed
// BIT0: 0-Start, 1-Stop
extern u8 g_bms_charge_sta_chged;

char log_msg_bms[64] = {0};
u16 test_cnt_bms = 0;

/****************************************************************************
* 名    称: u8 CAN2_Mode_Init(u8 mode)
* 功    能：CAN初始化
* 入口参数：mode:CAN工作模式;0,普通模式;1,环回模式
* 返回参数：0,成功;
           	其他,失败;
* 说    明：       
****************************************************************************/	
u8 CAN2_Mode_Init(u8 mode)
{
  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2, ENABLE); //使用CAN2的时候也要使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOA12复用为CAN1
	  
//	CAN_DeInit(CAN2);
//	CAN_StructInit(&CAN_InitStructure);
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
		
/***************************************************************************************/		
    //配置can工作模式	
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq; //时间段1的时间单元.  Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq; //时间段2的时间单元.  Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=6;  //分频系数(Fdiv)为brp+1	
/***************************************************************************************/			
  	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2 
    
		//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN2_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN2_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN2_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	
    CAN_Receive(CAN2, 0, &RxMessage);
		// CAN2_Receive_Msg(NULL);
	
		// printf("CAN2 Recved Msg 0x%.8X\n", RxMessage.ExtId);
}
#endif

/****************************************************************************
* 名    称: u8 CAN2_Send_Msg(u8* msg,u8 len)
* 功    能：can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
* 入口参数：len:数据长度(最大为8)				     
            msg:数据指针,最大为8个字节.
* 返回参数：0,成功;
           	其他,失败;
* 说    明：       
****************************************************************************/	
u8 CAN2_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x78563412;	 // 标准标识符为0
  TxMessage.ExtId=0x1004C899;	 // 设置扩展标示符（29位）
  TxMessage.IDE=CAN_ID_EXT;		   // 使用扩展标识符
  TxMessage.RTR=0;		   // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;	   // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
}

/****************************************************************************
* 名    称: u8 CAN2_Receive_Msg(u8 *buf)
* 功    能：can口接收数据查询
* 入口参数：buf:数据缓存区;	 			     
* 返回参数：0,无数据被收到;
    		    其他,接收的数据长度;
* 说    明：       
****************************************************************************/
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];

		if (0x18FF28F4 == RxMessage.ExtId) {
			if (0x02 == (RxMessage.Data[0]&0x2)) {// battery charging
				if (0 == g_bms_charge_sta_chged) {
					g_bms_charge_sta_chged = 1;
					printf("bms = 0\n");
					g_bms_charged_times++;
					sys_env_init();
				}
			} else {
				if (1 == g_bms_charge_sta_chged) {
					g_bms_charge_sta_chged = 0;
					printf("bms = 1\n");
				}
			}
			
			g_bms_charge_sta_chged |= 0x80;
			g_bms_battery_vol = RxMessage.Data[1];
			printf("g_bms_battery_vol = %d\n", g_bms_battery_vol);
			
			test_cnt_bms++;

			if (test_cnt_bms > 25) {
					memset(log_msg_bms, 0, 64);
					sprintf(log_msg_bms, "RECV: %.8X - %.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X", RxMessage.ExtId, RxMessage.Data[0], RxMessage.Data[1], RxMessage.Data[2], RxMessage.Data[3], RxMessage.Data[4], RxMessage.Data[5], RxMessage.Data[6], RxMessage.Data[7]);
					write_logs("CAN2", (char*)log_msg_bms, strlen((char*)log_msg_bms), 2);
					test_cnt_bms = 0;
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

		if (0 == (test_cnt_bms%20)) {
			LED_N = !LED_N;
		}
	return RxMessage.DLC;	
}






