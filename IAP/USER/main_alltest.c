#include "sys.h"
#include "delay.h"  
#include "usart.h"   
#include "led.h"
#include "lcd.h"
#include "key.h"   
#include "stmflash.h" 
#include "iap.h"   
#include "sdio_sdcard.h"    
#include "malloc.h" 
#include "w25qxx.h"    
#include "ff.h"  
#include "exfuns.h" 

typedef struct {
	u32 need_iap_flag;
	u32 need_bak_flag;
	u32 iap_ok_flag;
	u32 try_run_cnt;
} IAP_ENV;

//flag定义
//31~16 保留
//15~8  try         --- 范围为0~10启动次数，每次启动都会++，在APP中应该讲其清0，否认会认为该次启动失败，失败10次后，跳转到以前的app
//7~5   保留   
//4     update flag ---1:更新 0 : 不更新。该标志在app中接受完固件以后，设置此标志为1，下次重启的时候会更新固件，
//					     更新成功后清除此标志位0，并切换jump，启动新更新的app
//						 如果更新失败，不会清除次标记，也不会切换jump，依然启动旧的app，在app中检测次标记，如果标记为1，则表明更新失败。
//3~1   保留
//0	    jump ---       0：启动app1   1:启动app2

//要写入到W25Q16的字符串数组
const u8 TEXT_Buffer[]={"Explorer STM32F4 SPI TEST"};
#define SIZE sizeof(TEXT_Buffer)	 
u8 g_flash_buf[W25Q_SECTOR_SIZE] = {0};

int main(void)
{ 
	u8 t;
	u8 key;
	u16 oldcount=0;	//老的串口接收数据值
	u32 applenth=30*1024;	//接收到的app代码长度
	u8 clearflag=0; 
	u32 app_offset=0;
	u32 CHECK_FLAG=0;
	u32 flag=0;
	u32 update_addr=0;
	u32 run_addr=0;
	u32 tmp[2]={0};
	u32 try_cnt=10;
	
	u8 datatemp[SIZE];
	u32 FLASH_SIZE;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200

	W25QXX_Init();			//W25QXX初始化

#if 0
	while(W25QXX_ReadID()!=W25Q128)								//检测不到W25Q128
	{
		delay_ms(500);
		LED0=!LED0;		//DS0闪烁
	}

	FLASH_SIZE=16*1024*1024;	//FLASH 大小为16字节
	
	W25QXX_Write((u8*)TEXT_Buffer,FLASH_SIZE-100,SIZE);		//从倒数第100个地址处开始,写入SIZE长度的数据
	delay_ms(10);
	W25QXX_Read(datatemp,FLASH_SIZE-100,SIZE);					//从倒数第100个地址处开始,读出SIZE个字节
#endif

	while ((SD_OK != SD_Init()) && (try_cnt--)>0) {
		delay_us(10);
	}
	
	if (try_cnt > 0) {
		exfuns_init();
		if (f_mount(fs[0],"0:",1) != FR_OK) {
			printf("\r\n FAT Error !\r\n");
		} else {
			printf("\r\n FAT OK !\r\n");
		}
	}
	
	IAP_ENV iap_env;
	
	W25QXX_Read((u8*)&iap_env, ENV_SECTOR_INDEX_IAP*W25Q_SECTOR_SIZE, sizeof(IAP_ENV));
	
	printf("try_run_cnt = %d\n", iap_env.try_run_cnt);
	
	if (iap_env.try_run_cnt > 10) {
		iap_env.try_run_cnt = 0;
	}
	
	iap_env.try_run_cnt++;
	W25QXX_Write((u8*)&iap_env, ENV_SECTOR_INDEX_IAP*W25Q_SECTOR_SIZE, sizeof(IAP_ENV));
	
	if (3 == iap_env.try_run_cnt) {
#if 0
		do_upddate_firm(FLASH_RUN_ADDR);
#else
		do_upddate_firm_spi();
#endif
		printf("update run success\n");
	} else if (5 == iap_env.try_run_cnt) {
		printf("backup run start\n");
		//do_backup_run();
		printf("backup run success\n");
	} else if (7 == iap_env.try_run_cnt) {
		printf("restore run start\n");
		do_restore_run();
		printf("restore run success\n");
	}
	
	try_cnt = 0;
	while(1) {
		if(((*(vu32*)(FLASH_RUN_ADDR+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
		{	 
				iap_load_app(FLASH_RUN_ADDR);//执行FLASH APP代码
		}
		
		printf("jump app failed\n");
		
		if (10 == try_cnt++) {
			do_restore_run();
		}
		
		delay_ms(1000);
	}
}

