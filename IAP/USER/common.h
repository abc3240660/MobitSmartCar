#ifndef __COMMON_H
#define __COMMON_H 	
#include "sys.h"

#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif 

#define LEN_FILE_NAME   64
#define LEN_DW_MD5      64
#define LEN_DW_URL      128
#define LEN_LOG_MSG     256

#define BIT_W25Q_STA 0
#define BIT_SDTF_STA 1

#define	app_pi	3.1415926535897932384626433832795 
/////////////////////////////////////////////////////////////////////////

typedef struct {
	u16 active_flag;
	u16 charge_times;
	u32 total_meters;
	u8 svr_ip[32];
	u8 svr_port[8];
	u8 svr_apn[32];
} SYS_ENV;

typedef struct {
	// (APP SET)0x1A1A2B2B - need do update from SD Card
	// (APP SET)0x5A5A6B6B - need do update from SPI Flash
	// (IAP SET)0x3C3C4D4D - update finished
	// (APP CLR)0x00000000 - idle(after app detect 0x3C3C4D4D)
	u32 need_iap_flag;
	
	// (APP SET)0x51516821 - need backup hex data from RUN sector into BAKOK sector
	// (IAP CLR)0x00000000 - idle
	u32 need_bak_flag;
	
	// (APP SET)0x12345678 - APP is running
	// (IAP SET)0x61828155 - already done backup hex data from RUN sector into BAKOK sector
	u32 bak_sta_flag;
	
	// (IAP SET)0x52816695 - iap update NG
	// (APP CLR)0x00000000 - idle
	u32 iap_sta_flag;
	
	// (IAP SET)0 -> 10 - if equal to 10, need do restore hex data from BAKOK sector into RUN sector
	// (APP CLR)0 - jump to app ok
	u32 try_run_cnt;
} IAP_ENV;

void sys_env_init(void);
void sys_env_save(void);
void sys_env_dump(void);
	
#endif
