#ifndef __COMMON_H
#define __COMMON_H 	
#include "sys.h"
#include "includes.h"

// For VS1003 to Suport Double HW Board
#define QMXX_STM32 1

// Ex SPI Flash
#define ENV_SECTOR_INDEX	0

#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif 


// #define HC08_ENABLE 1
// #define MPU_ENABLE 1

#define LEN_FILE_NAME   64
#define LEN_DW_MD5      64
#define LEN_DW_URL      128
#define LEN_LOG_MSG     256

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

void sys_env_init(void);
void sys_env_save(void);
void sys_env_dump(void);
	
#endif




























































