#include "common.h"
#include "malloc.h"

static u32 *g_flash_sector =  NULL;

extern u16 g_bms_charged_times;

extern u8 g_svr_ip[32];
extern u8 g_svr_port[8];
extern u8 g_svr_apn[32];

extern u32 g_total_meters;

void sys_env_init(void)
{
	g_flash_sector = mymalloc(SRAMIN, 4096);	//申请4K字节内存  
	if (NULL == g_flash_sector) {
		myfree(SRAMIN, g_flash_sector);
	}
}

void sys_env_save(void)
{
	u16 j = 0;
	SYS_ENV sys_env;
	u32 *buf = g_flash_sector;

	sys_env.active_flag = 0x6821;
	strcpy(sys_env.svr_ip, "47.105.112.41");
	strcpy(sys_env.svr_apn, "CMNET");
	strcpy(sys_env.svr_port, "88");
	g_bms_charged_times = 123;
	sys_env.charge_times = g_bms_charged_times;
	sys_env.total_meters = g_total_meters;

	W25QXX_Read((u8*)buf, ENV_SECTOR_INDEX*4096, 4096);//读出整个扇区的内容
	for (j=0; j<1024; j++) {//校验数据
		if(buf[j]!=0XFFFFFFFF)break;//需要擦除  	  
	}
	
	if (j != 1024) {
		W25QXX_Erase_Sector(ENV_SECTOR_INDEX);	//需要擦除的扇区
	}

	memset((u8*)buf, 0, 6096);
	memcpy((u8*)buf, (u8*)&sys_env, sizeof(SYS_ENV));
	W25QXX_Write((u8*)buf, ENV_SECTOR_INDEX*4096, 4096);
	
//	myfree(SRAMIN, buf);
}

void sys_env_dump(void)
{
	SYS_ENV sys_env;

	W25QXX_Read((u8*)&sys_env, ENV_SECTOR_INDEX*4096, sizeof(SYS_ENV));//读出整个扇区的内容
	
	if (0x6821 == sys_env.active_flag) {
		g_bms_charged_times = sys_env.charge_times;
		printf("charge_times = %d\n", sys_env.charge_times);

		g_total_meters = sys_env.total_meters;
		printf("total_meters = %d\n", sys_env.total_meters);

		memcpy(g_svr_ip, sys_env.svr_ip, 32);
		memcpy(g_svr_port, sys_env.svr_port, 8);
		memcpy(g_svr_apn, sys_env.svr_apn, 32);
		printf("svr_ip = %s\n", sys_env.svr_ip);
		printf("svr_port = %s\n", sys_env.svr_port);
		printf("svr_apn = %s\n", sys_env.svr_apn);
	} else {
		printf("haven't saved env params before\n");
	}
}
