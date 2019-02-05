#ifndef __IAP_H__
#define __IAP_H__
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//IAP 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/7/21
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////    
typedef  void (*iapfun)(void);				//定义一个函数类型的参数.   

/*
Sector Size:
1-4:  16KB
5:    64KB
6-12: 128KB
*/

#define FLASH_BASE_ADDR		0x08000000// offset:0K
#define FLASH_IAP_SIZE		(64*1024)// size:16K*4 = 64K

#define FLASH_FLAG_ADDR		0x08010000// offset:64K
#define FLASH_FLAG_SIZE		(64*1024)// size:64KB = 0x10000

// For Normal Run
#define FLASH_RUN_ADDR		0x08020000// offset:128K
#define FLASH_RUN_SIZE		(256*1024)// size:128K*2 = 256KB = 0x40000
//#define FLASH_RUN_SIZE		(64*1024)// size:128K*2 = 256KB = 0x40000

// For Origin Backup
#define FLASH_OKBAK_ADDR		0x08060000// offset:384KB
#define FLASH_OKBAK_SIZE		(256*1024)// size:128K*2 = 256KB = 0x40000
//#define FLASH_OKBAK_SIZE		(64*1024)// size:128K*2 = 256KB = 0x40000

// Reserved for later use
#define FLASH_APP2_ADDR		0x080A0000// offset:640KB
#define FLASH_APP2_SIZE		(256*1024)// size:128K*2 = 256KB = 0x40000

void iap_load_app(u32 appxaddr);			//跳转到APP程序执行
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 applen);	//在指定地址开始,写入bin
void write_flag(u32 flag);
u32 read_flag(u32 offset);
void UP_success(void);
int do_upddate_firm(u32 update_addr);
int do_backup_run(void);
int do_restore_run(void);
int do_upddate_firm_spi(void);

#define BOOT_TRY_MAX_TIMES	10

#endif