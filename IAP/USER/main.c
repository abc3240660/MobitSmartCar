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
#include "common.h"

u8 g_periph_sta = 0;

// 要写入到W25Q16的字符串数组
u8 g_flash_buf[W25Q_SECTOR_SIZE] = {0};

int main(void)
{ 
	IAP_ENV iap_env;
	u32 try_cnt = 10;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置系统中断优先级分组2
	delay_init(168);// 初始化延时函数
	usart3_init(115200);// 初始化串口波特率为115200

	W25QXX_Init(); // W25QXX初始化

	while(W25QXX_ReadID()!=W25Q128 && (try_cnt--)>0) {
		delay_ms(100);
	}
	
	if (try_cnt > 0) {
		g_periph_sta |= (1<<BIT_W25Q_STA);
		printf("\r\nW25Q Detect OK\r\n");
	} else {
		printf("\r\nW25Q Detect Failed!\r\n");
	}

	try_cnt = 10;
	while ((SD_OK != SD_Init()) && (try_cnt--)>0) {
		delay_ms(100);
	}
	
	if (try_cnt > 0) {
		exfuns_init();
		if (f_mount(fs[0],"0:",1) != FR_OK) {
			printf("\r\nSD FATFS Setup Failed!\r\n");
		} else {
			g_periph_sta |= (1<<BIT_SDTF_STA);
			printf("\r\nSD FATFS Setup OK\r\n");
		}
	}
	
	memset(&iap_env, 0, sizeof(iap_env));
	
	if (g_periph_sta&(1<<BIT_W25Q_STA)) {
		W25QXX_Read((u8*)&iap_env, ENV_SECTOR_INDEX_IAP*W25Q_SECTOR_SIZE, sizeof(IAP_ENV));
		
		printf("try_run_cnt = %d\n", iap_env.try_run_cnt);
		
		// first boot, do initialize into zero
		if (iap_env.try_run_cnt > 100) {
			iap_env.try_run_cnt = 0;
		}

		iap_env.try_run_cnt++;
		if (iap_env.try_run_cnt > 10) {
			iap_env.try_run_cnt = 0;
			do_restore_run();
		}
		
		// recved new BIN in SD/TF, do upgrade
		if (0x1A1A2B2B == iap_env.need_iap_flag) {
			if (g_periph_sta&(1<<BIT_SDTF_STA)) {
				printf("update bin start\n");
				if (0 == do_upddate_firm(FLASH_RUN_ADDR)) {
					iap_env.iap_sta_flag = 0;
				} else {
					iap_env.iap_sta_flag = 0x52816695;// NG
				}
				printf("update bin end\n");
			} else {
				iap_env.iap_sta_flag = 0x52816695;// NG
			}
			
			iap_env.need_iap_flag = 0x3C3C4D4D;
		}
		
		// recved new BIN in SPI Flash, do upgrade
		if (0x5A5A6B6B == iap_env.need_iap_flag) {
			if (g_periph_sta&(1<<BIT_W25Q_STA)) {
				printf("update bin start\n");
				do_upddate_firm_spi();
				printf("update bin end\n");
				
				iap_env.iap_sta_flag = 0;
			} else {
				iap_env.iap_sta_flag = 0x52816695;// NG
			}
			
			iap_env.need_iap_flag = 0x3C3C4D4D;
		}
		
		// backup RUN sector into BAKOK sector
		if (0x51516821 == iap_env.need_bak_flag) {
			printf("backup run start\n");
			do_backup_run();
			printf("backup run end\n");
			
			iap_env.bak_sta_flag = 0x61828155;
			iap_env.need_bak_flag = 0;
		}
		
		// after first APP run, APP will set this flag
		// next boot, IAP will backup the first OK APP into BAKOK sector
		if (0x12345678 == iap_env.bak_sta_flag) {
			do_backup_run();
			iap_env.bak_sta_flag = 0x61828155;
		}
		
		W25QXX_Write((u8*)&iap_env, ENV_SECTOR_INDEX_IAP*W25Q_SECTOR_SIZE, sizeof(IAP_ENV));
	}
	
	try_cnt = 0;
	while(1) {
		if(((*(vu32*)(FLASH_RUN_ADDR+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
		{	 
				iap_load_app(FLASH_RUN_ADDR);//执行FLASH APP代码
		}
		
		printf("jump app failed\n");
		
		// if try jump app failed more than 10 times
		// restore BAKOK into RUN
		if (10 == try_cnt++) {
			do_restore_run();
		}
		
		delay_ms(1000);
	}
}

