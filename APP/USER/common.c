#include "common.h"
#include "malloc.h"

extern u16 g_bms_charged_times;

extern u8 g_svr_ip[32];
extern u8 g_svr_port[8];
extern u8 g_svr_apn[32];

extern u8 mp3_list[128];

extern u32 g_total_meters;

extern int g_sd_existing;

extern u8 g_logname[LEN_FILE_NAME+1];
extern u8 g_logmsg[LEN_LOG_MSG];

extern OS_EVENT* sem_beep;

void SoftReset(void);

void sys_env_init(void)
{
	SYS_ENV sys_env;
	
	memset(&sys_env, 0, sizeof(sys_env));
	W25QXX_Read((u8*)&sys_env, ENV_SECTOR_INDEX_ECAR*W25Q_SECTOR_SIZE, sizeof(SYS_ENV));
	
	if (sys_env.active_flag != 0x6789) {
		sys_env.active_flag = 0x6789;
		
		sys_env.charge_times = 0;
		sys_env.total_meters = 0;
		
		strcpy((char*)sys_env.svr_ip, "47.105.112.41");
		strcpy((char*)sys_env.svr_apn, "CMNET");
		strcpy((char*)sys_env.svr_port, "88");

		W25QXX_Write((u8*)&sys_env, ENV_SECTOR_INDEX_ECAR*W25Q_SECTOR_SIZE, sizeof(SYS_ENV));
	}
}

void sys_env_dump(void)
{
	SYS_ENV sys_env;

	memset(&sys_env, 0, sizeof(sys_env));
	W25QXX_Read((u8*)&sys_env, ENV_SECTOR_INDEX_ECAR*W25Q_SECTOR_SIZE, sizeof(SYS_ENV));//读出整个扇区的内容
	
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
}

void sys_env_update_meter(u32 value)
{
	SYS_ENV sys_env;
	
	memset(&sys_env, 0, sizeof(sys_env));
	W25QXX_Read((u8*)&sys_env, ENV_SECTOR_INDEX_ECAR*W25Q_SECTOR_SIZE, sizeof(SYS_ENV));

	sys_env.total_meters = value;
	
	W25QXX_Write((u8*)&sys_env, ENV_SECTOR_INDEX_ECAR*W25Q_SECTOR_SIZE, sizeof(SYS_ENV));
}

// value:
//	0x1A1A2B2B - trigger iap from sd
//	0x5A5A6B6B - trigger iap from spi flash
//	0x00000000 - reset to idle
void env_update_iap_req(u32 value)
{
	IAP_ENV iap_env;
		
	memset(&iap_env, 0, sizeof(iap_env));
	W25QXX_Read((u8*)&iap_env, ENV_SECTOR_INDEX_IAP*W25Q_SECTOR_SIZE, sizeof(IAP_ENV));

	iap_env.need_iap_flag = value;// Trigger IAP from SD
	W25QXX_Write((u8*)&iap_env, ENV_SECTOR_INDEX_IAP*W25Q_SECTOR_SIZE, sizeof(IAP_ENV));
	
	if ((0x1A1A2B2B == value) || (0x5A5A6B6B == value)) {
		SoftReset();
	}
}

FRESULT scan_files(char *path)
{
	u16 i = 0;
	
	DIR dir;
	FRESULT res;
	static FILINFO fno;
	
	res = f_opendir(&dir, path);
	if (res == FR_OK) {
		for(;;) {
			res = f_readdir(&dir, &fno);
			if (res != FR_OK || fno.fname[0] == 0) {
				break;
			}
			
			if (fno.fattrib & AM_DIR) {
				i = strlen(path);
				sprintf(&path[i],"/%s", fno.fname);
				printf("this is directory %s\n", path);
				res = scan_files(path);
				if (res != FR_OK) {
					break;
				}
				path[i] = 0; 
			} else {
				u8 j = 0;
				u8 len = strlen((const char*)mp3_list);
				for (j=len; j<128; j++) {
					if ('.' == fno.fname[j-len]) {
						mp3_list[j] = ':';
						break;
					} else {
						mp3_list[j] = fno.fname[j-len];
					}
				}
				strcpy((char*)mp3_list+j+1, "01234567890123456789012345678901|");
				printf("this is file %s/%s\n", path, fno.fname);
			}
		}
		
		f_closedir(&dir);
	} else {
		printf("ERROR - %s\n", &res);
	}
	
	return res;
}

void create_directories(void)
{
	DIR dir;
	FRESULT res = FR_OK;
	
	res = f_opendir(&dir, "0:/MUSIC");
	if (FR_NO_PATH == res) {
		f_mkdir("0:/MUSIC");
	}
	
	res = FR_OK;
	res = f_opendir(&dir, "0:/LOG");
	if (FR_NO_PATH == res) {
		f_mkdir("0:/LOG");
	}
	
	res = FR_OK;
	res = f_opendir(&dir, "0:/IAP");
	if (FR_NO_PATH == res) {
		f_mkdir("0:/IAP");
	}
	
	// f_unlink("0:/MUSIC");
	// f_unlink("0:/LOG");
	// f_unlink("0:/IAP");
}

void create_logfile(void)
{
	if (0 == g_sd_existing) {
		return;
	} else {
		u8 res;
		FIL f_txt;
		RTC_TimeTypeDef RTC_TimeStruct;
		RTC_DateTypeDef RTC_DateStruct;
		
		RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
		
		sprintf((char*)g_logname,"0:/LOG/2Y0%02d%02d%02d_%02d%02d%02d.log",RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date,RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
		
		res = f_open(&f_txt,(const TCHAR*)g_logname,FA_READ|FA_WRITE|FA_CREATE_ALWAYS);
		if (0 == res) {
			f_close(&f_txt);
		}
	}
}

void write_logs(char *module, char *log, u16 size, u8 mode)
{
	if (0 == g_sd_existing) {
		return;
	} else {
		u8 err;
		u8 res;
		u32 br;
		FIL f_txt;
		RTC_TimeTypeDef RTC_TimeStruct;

		RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

		memset(g_logmsg, 0, 256);
		
		if (0 == mode) {
			sprintf((char*)g_logmsg,"%02d%02d%02d:RECV Data(%s) %s\n",RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds,module,log);
		} else if (1 == mode) {
			sprintf((char*)g_logmsg,"%02d%02d%02d:SEND Data(%s) %s\n",RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds,module,log);
		} else if (2 == mode) {
			sprintf((char*)g_logmsg,"%02d%02d%02d:IMPT Data(%s) %s\n",RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds,module,log);
		} else {
			sprintf((char*)g_logmsg,"%02d%02d%02d:OTHR Data(%s) %s\n",RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds,module,log);
		}

		printf("%s", g_logmsg);
		
		OSSemPend(sem_beep,0,&err);
		res = f_open(&f_txt,(const TCHAR*)g_logname,FA_READ|FA_WRITE);
		if (0 == res) {
			f_lseek(&f_txt, f_txt.fsize);
			f_write(&f_txt,g_logmsg, strlen((const char*)g_logmsg), (UINT*)&br);
			f_close(&f_txt);
		}
		OSSemPost(sem_beep);
	}
}
