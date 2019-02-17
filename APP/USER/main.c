#include "includes.h"
#include "malloc.h"
#include "common.h"
#include "usart3.h"
#include "usart5.h"
#include "usart6.h"
#include "blue.h"
#include "sim900a.h"
#include "can1.h"
#include "can2.h"
#include "rfid.h"
#include "vs10xx.h"
#include "mp3play.h"
#include "rtc.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/////////////////////////UCOSII TASK///////////////////////////////////

// Lowest Priority
#define START_TASK_PRIO                 10
#define START_STK_SIZE                  64
__align(8) static OS_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *pdata);

#define LOWER_TASK_PRIO                 8
#define LOWER_STK_SIZE                  1024
__align(8) static OS_STK LOWER_TASK_STK[LOWER_STK_SIZE];
void lower_task(void *pdata);

#define USART_TASK_PRIO                 7
#define USART_STK_SIZE                  1024
__align(8) static OS_STK USART_TASK_STK[USART_STK_SIZE];
void usart_task(void *pdata);

#define MAIN_TASK_PRIO                  6
#define MAIN_STK_SIZE                   1024
__align(8) static OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
void main_task(void *pdata);

#define HIGHER_TASK_PRIO                3
#define HIGHER_STK_SIZE                 1024
__align(8) static OS_STK HIGHER_TASK_STK[HIGHER_STK_SIZE];
void higher_task(void *pdata);

//////////////////////////////////////////////////////////////////////////////

int total_ms = 0;
int g_sd_existing = 0;

OS_EVENT* sem_beep;

u8 g_logname[LEN_FILE_NAME+1] = "";
u8 g_logmsg[LEN_LOG_MSG] = "";

u8 g_mp3_play = 0;
u8 g_mp3_play_name[LEN_FILE_NAME+1] = "";

u8 g_mp3_list[512] = "";
u32 g_trip_meters = 0;
u32 g_trip_meters_old = 0;
u32 g_total_meters = 0;
u32 g_total_meters_old = 0;

extern int pluse_num_new;
extern u8 g_mp3_update_name[LEN_FILE_NAME+1];
extern u8 g_mp3_update;
extern u8 g_iap_update;
extern u8 g_dw_write_enable;
extern vu16 g_data_pos;
extern vu16 g_data_size;
extern u8 USART1_RX_BUF_BAK[U1_RECV_LEN_ONE];

int g_mpu_sta = 0;

extern void sim7500e_mobit_process(u8 index);
void create_logfile(void);
void write_logs(char *module, char *log, u16 size, u8 mode);

//////////////////////////////////////////////////////////////////////////////

void do_sd_check()
{
	u8 res = 0;
	u16 temp = 0;
	u32 dtsize = 0;
	u32 dfsize = 0;

	do {
		temp++;
		res = exf_getfree("0:", &dtsize, &dfsize);
		delay_ms(200);
	} while (res && (temp<5));

	if (0 == res) {
		g_sd_existing = 1;
		printf("Read SD OK!\r\n");
	} else {
		printf("Read SD Failed!\r\n");
	}
}

void do_sd_init()
{
	u8 sw_ver[64] = "";
	
	exfuns_init();// alloc for fats

	// This func will call SD_Init internally
  f_mount(fs[0],"0:",1);

	do_sd_check();

	create_logfile();
	
	sprintf((char*)sw_ver, "SW_VER = %s", SW_VERSION);
	
	write_logs("SIM7000E", (char*)sw_ver, strlen((char*)sw_ver), 2);
}

void mpu6050_init()
{
	MPU_Init();
	
	delay_ms(200);
	while (mpu_dmp_init()) {
		delay_ms(200);
	}
}

void spiflash_init()
{
	u32 try_cnt = 10;
	
	W25QXX_Init();

	while(W25QXX_ReadID()!=W25Q128 && (try_cnt--)>0) {
		delay_ms(100);
	}
	
	sys_env_init();

	// Get ENV Params
	sys_env_dump();

	g_trip_meters_old = g_trip_meters;
}

void do_vs_test(void)
{
	u16 vs_id = 0;
	
	delay_ms(1000);
	vs_id = VS_Ram_Test();
	
	if (0x807F != vs_id) {
		return;
	}
	
	delay_ms(1000);
	music_play("0:/MUSIC/sax.mp3");
	
	while(1) {
		delay_ms(1000);
		VS_Sine_Test();
		delay_ms(1000);
	}
}

void system_init(void)
{
	u8 CAN1_mode = 0;
	u8 CAN2_mode = 0;
	
	delay_init(168);
	uart_init(115200);
	usart3_init(115200);
	usart5_init(115200);
	usart6_init(9600);
 	LED_Init();
 	KEY_Init();
	
	printf("SmartMotor Starting...\n");
	// printf("SmartMotor Starting VerSD-TF-Card...\n");
	// printf("SmartMotor Starting VerSPI...\n");

	SIM7000E_RST = 1;
	delay_ms(1000);
	
	hc08_init();

	My_RTC_Init();
	RTC_Set_WakeUp(RTC_WakeUpClock_CK_SPRE_16bits,0);

	CAN1_Mode_Init(CAN1_mode);// 250Kbps
	CAN2_Mode_Init(CAN2_mode);// 500Kbps

	my_mem_init(SRAMIN);
	my_mem_init(SRAMCCM);

	TIM2_Init(9999,8399);
	TIM4_Init(9999,8399);                                                                

	VS_Init();

	mpu6050_init();

	do_sd_init();
	
	spiflash_init();

	create_directories();
	
	// CAN1_JumpLamp(5);
	// CAN1_RingAlarm(5);
	
	delay_ms(1000);
	SIM7000E_RST = 0;
	
	// do_vs_test();
}

void SoftReset(void)
{
#if 0
	while (1) {
		delay_ms(1000);
	}
#endif
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

int main(void)
{
    // SCB->VTOR = *((u32 *)0x0800FFF8);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	system_init();
	OSInit();
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO);
	OSStart();
}

void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr = 0;
	pdata = pdata;
	sem_beep = OSSemCreate(1);

	OSStatInit();

	OS_ENTER_CRITICAL();
	//OSTaskCreate(main_task,(void *)0,(OS_STK*)&MAIN_TASK_STK[MAIN_STK_SIZE-1],MAIN_TASK_PRIO);
	OSTaskCreate(usart_task,(void *)0,(OS_STK*)&USART_TASK_STK[USART_STK_SIZE-1],USART_TASK_PRIO);
	//OSTaskCreate(higher_task,(void *)0,(OS_STK*)&HIGHER_TASK_STK[HIGHER_STK_SIZE-1],HIGHER_TASK_PRIO);
	//OSTaskCreate(lower_task,(void *)0,(OS_STK*)&LOWER_TASK_STK[LOWER_STK_SIZE-1],LOWER_TASK_PRIO);
	OSTaskSuspend(START_TASK_PRIO);

	OS_EXIT_CRITICAL();
}

void MPU6050_Risk_Check()
{
	u8 ret = 0;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����

	ret = mpu_dmp_get_data(&pitch, &roll, &yaw);
	if (ret == 0) {
		MPU_Get_Accelerometer(&aacx, &aacy, &aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);	//�õ�����������

		// printf(" (int)(roll*100)=%d ,(int)(pitch*100)=%d ,(int)(yaw*10)=%d \r\n",(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
		if((roll*100>4500)||(roll*100<-4500)||(pitch*100>4500)||(pitch*100<-4500)||(yaw*10>450)||(yaw*10<-450))
		{
			g_mpu_sta = 1;
			printf("FCL \r\n");
		} else {
			g_mpu_sta = 0;
			// printf("MFC \r\n");
		}
	} else {
		// printf("Get mpu data failed! \r\n");
	}
}

// Play MP3 task
void lower_task(void *pdata)
{
	while(1) {
		if (g_mp3_play) {
			char filename[64] = "";

			if (strlen((const char*)g_mp3_play_name) < 40) {
				sprintf(filename, "0:/MUSIC/%s.mp3", g_mp3_play_name);
				music_play((const char*)filename);
			}

			g_mp3_play = 0;
			memset(g_mp3_play_name, 0, LEN_FILE_NAME);
		}

		MPU6050_Risk_Check();

		if (0 == KEY_HAND_BRAKE) {// La
			g_mp3_play = 0;
		} else {// Fang
			g_mp3_play = 0;
		}
		
    OSTimeDlyHMSM(0,0,0,500);// 500ms
	}
}

// MPU6050 Check and BT Data Parse
void main_task(void *pdata)
{
	u8 i = 0;
	u8 loop_cnt = 0;

	while(1) {
		for (i=0; i<U1_RECV_BUF_CNT; i++) {
			if (USART1_RX_STA[i]&0X8000) {
				if (strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*i), PROTOCOL_HEAD)) {
					sim7500e_mobit_process(i);
				}
			}
		}

		if ((g_trip_meters - g_trip_meters_old) > 10) {// Unit: 0.1KM/BIT
			g_total_meters += (g_trip_meters - g_trip_meters_old) / 10;
			g_trip_meters_old = g_trip_meters;
		}

		// Update total_meters into flash
		if (50 == loop_cnt++) {
			loop_cnt = 0;

			if (g_total_meters != g_total_meters_old) {
				g_total_meters_old = g_total_meters;
				sys_env_update_meter(g_total_meters);
			}

			printf("main_task test\n");
		}

		OSTimeDlyHMSM(0,0,0,100);// 500ms
	}
}

void usart_task(void *pdata)
{
	u8 loop_cnt = 0;

	while (1) {
		if ((UART5_RX_STA&(1<<15)) != 0) {
			cpr74_read_calypso();
			UART5_RX_STA = 0;
    }

    if (loop_cnt++ == 10) {
			loop_cnt = 0;
			CAN1_JumpLamp(5);
			//delay_ms(10);
			//CAN1_JumpLamp(5);
			//delay_ms(10);
			//CAN1_JumpLamp(5);
			//delay_ms(10);
			//CAN1_JumpLamp(5);
			//delay_ms(10);
			//CAN1_JumpLamp(5);
			// CAN1_RingAlarm(5);
      printf("pluse_num_new = %d\n", pluse_num_new);
    }

		if (1 == g_dw_write_enable) {
			if (1 == g_sd_existing) {
				u32 br = 0;
				u8 res = 0;
				FIL f_txt;

        if (g_mp3_update != 0) {
					u8 mp3_file[LEN_FILE_NAME+1] = "";

          if (strlen((const char*)g_mp3_update_name) > 40) {
						g_mp3_update = 0;
            printf("file name is too long\n");
            continue;
          }

					sprintf((char*)mp3_file, "0:/MUSIC/%s.mp3", g_mp3_update_name);
					res = f_open(&f_txt,(const TCHAR*)mp3_file,FA_READ|FA_WRITE);
				} else if (g_iap_update != 0) {
						res = f_open(&f_txt,(const TCHAR*)"0:/IAP/APP.BIN",FA_READ|FA_WRITE);
        }

				if (0 == res) {
					f_lseek(&f_txt, f_txt.fsize);
					f_write(&f_txt, USART1_RX_BUF_BAK+g_data_pos, g_data_size, (UINT*)&br);
					f_close(&f_txt);
				}
			}

			g_dw_write_enable = 0;
		}

		OSTimeDlyHMSM(0,0,0,500);// 500ms
		OSTimeDlyHMSM(0,0,0,500);// 500ms
	}
}

void higher_task(void *pdata)
{
	SIM7000E_PWR = 0;
	delay_ms(4000);
	SIM7000E_PWR = 1;
	delay_ms(4000);
	SIM7000E_PWR = 0;
	
	while (1) {
		sim7500e_communication_loop(0,NULL,NULL);
    OSTimeDlyHMSM(0,0,0,500);// 500ms
	}
}

void HardFault_Handler(void)
{
	u32 i;
	u8 t=0;
	u32 temp;
	temp=SCB->CFSR;
	printf("CFSR:%8X\r\n",temp);
	temp=SCB->HFSR;
	printf("HFSR:%8X\r\n",temp);
	temp=SCB->DFSR;
	printf("DFSR:%8X\r\n",temp);
	temp=SCB->AFSR;
	printf("AFSR:%8X\r\n",temp);

	write_logs("SIM7000E", (char*)"HardFault_Handler -> Reboot\n", strlen((char*)"HardFault_Handler Enter -> Reboot\n"), 3);
	SoftReset();

	while(t<5)
	{
		t++;
		LED_R = !LED_R;
		for(i=0;i<0X1FFFFF;i++);
	}
}
