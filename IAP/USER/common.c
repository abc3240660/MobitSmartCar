#include "common.h"
#include "malloc.h"

static u32 *g_flash_sector =  NULL;

void sys_env_init(void)
{
	g_flash_sector = mymalloc(SRAMIN, 4096);	//ÉêÇë4K×Ö½ÚÄÚ´æ  
	if (NULL == g_flash_sector) {
		myfree(SRAMIN, g_flash_sector);
	}
}
