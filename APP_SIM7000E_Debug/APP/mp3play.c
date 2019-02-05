#include "vs10XX.h"
#include "ff.h"
#include "key.h"
#include "MP3PLAY.H"
#include "delay.h" 	
#include "sdio_sdcard.H"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

unsigned char data[2048];
unsigned char sd_buffer[32];
unsigned char buffer_count; 
unsigned char stop = 0;
unsigned short yl=0x2020;
unsigned char mode,keyResult,keyCount;

//extern FATFS fs;
extern FRESULT res;
extern UINT br;

unsigned char mp3Play(const char* mp3name)
{
	u8 res;
	FIL file;
	
	res = f_open(&file, mp3name, FA_READ);

	VS_HD_Reset();
	VS_Soft_Reset();
	//Vs1003_CMD_Write(0x0b,yl);
	for(buffer_count=0; buffer_count<32; buffer_count++)
		sd_buffer[buffer_count] = 0;
	if(!res)
	{
		for(;;)
		{
			res = f_read(&file, sd_buffer, 32, &br);
			while(VS_DQ==0);
			VS_Send_MusicData(sd_buffer);
			//for(i=0;i<32;i++)Vs1003_DATA_Write(sd_buffer[i]);
			if(br<32||stop == 1)
			{
				stop = 0;
				break;
			}
		}
	}
	f_close(&file);
 	return 0;
}

void mp3_play(const char* mp3name)
{
	FIL file;
	u8 res;
	
	res = f_open(&file, mp3name, FA_READ);
	if(!res)
	{	  
		memset(sd_buffer, 0,32);
	  VS_Restart_Play();
		VS_Set_All();
		VS_Reset_DecodeTime();

		for(;;)
		{
			res = f_read(&file, sd_buffer, 32, &br);
			while(VS_DQ==0);
			VS_Send_MusicData(sd_buffer);
			if(br<32) {
				break;
			}
		}
	}	  	 		  	    
}

void music_play(const char* mp3name)
{
	VS_HD_Reset();
	VS_Soft_Reset();

	// 0~254
	vsset.mvol=254;
	mp3_play(mp3name);
}

