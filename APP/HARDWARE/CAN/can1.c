#include "can1.h"
#include "led.h"
#include "sim900a.h"

extern u16 g_car_sta;
extern u8 g_door_state;
extern u8 g_power_state;
extern u32 g_trip_meters;
extern u8 g_drlock_sta_chged;
/****************************************************************************
* 名    称: u8 CAN1_Mode_Init(u8 mode)
* 功    能：CAN初始化
* 入口参数：mode:CAN工作模式;0,普通模式;1,环回模式
* 返回参数：0,成功;
           	其他,失败;
* 说    明：       
****************************************************************************/	
u8 CAN1_Mode_Init(u8 mode)
{
  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决
	 
/***************************************************************************************/		
    //配置can工作模式	
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq; //时间段1的时间单元.  Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq; //时间段2的时间单元.  Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=12;  //分频系数(Fdiv)为brp+1	
/***************************************************************************************/			
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
    CAN_Receive(CAN1, 0, &RxMessage);
	
		printf("CAN1 Recved Msg 0x%.8X\n", RxMessage.ExtId);
}
#endif

/****************************************************************************
* 名    称: u8 CAN1_Send_Msg(u8* msg,u8 len)
* 功    能：can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
* 入口参数：len:数据长度(最大为8)				     
            msg:数据指针,最大为8个字节.
* 返回参数：0,成功;
           	其他,失败;
* 说    明：       
****************************************************************************/		
u8 CAN1_Send_Msg(u8* msg,u8 len)
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
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
}

/****************************************************************************
* 名    称: u8 CAN1_Receive_Msg(u8 *buf)
* 功    能：can口接收数据查询
* 入口参数：buf:数据缓存区;	 			     
* 返回参数：0,无数据被收到;
    		    其他,接收的数据长度;
* 说    明：       
****************************************************************************/	
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
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
	
	CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节 
	
	return 0;
}

u8 CAN1_StopEngine(void)
{
	u8 can1_sendbuf[8]={0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节 
	
	return 0;
}

u8 CAN1_OpenDoor(void)
{
	u8 can1_sendbuf[8]={0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节 
	
	return 0;
}

u8 CAN1_CloseDoor(void)
{
	u8 can1_sendbuf[8]={0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节 
	
	return 0;
}

u8 CAN1_JumpLamp(u8 times)
{
	u8 can1_sendbuf[8]={0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节 
	
	return 0;
}

u8 CAN1_RingAlarm(u8 times)
{
	u8 can1_sendbuf[8]={0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节 
	
	return 0;
}

u8 CAN1_StartAll(void)
{
	u8 can1_sendbuf[8]={0x3F, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节 
	
	return 0;
}

u8 CAN1_StartHint(void)
{
	u8 can1_sendbuf[8]={0x0C, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节 
	
	return 0;
}
