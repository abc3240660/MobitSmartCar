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
#include "led.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/////////////////////////UCOSII TASK///////////////////////////////////

// Lowest Priority
#define START_TASK_PRIO                 10
#define START_STK_SIZE                  64
__align(8) static OS_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *pdata);

#define TIMER_TASK_PRIO                  9
#define TIMER_STK_SIZE                   1024
__align(8) static OS_STK TIMER_TASK_STK[TIMER_STK_SIZE];
void timer_task(void *pdata);

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
OS_EVENT* sem_atsend;

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
extern u8 USART1_RX_BUF_BAK[U1_RX_LEN_ONE];
extern u32 os_jiffies;
int g_mpu_sta = 0;

u8 g_sim_retry_cnt = 0;
u8 hbrake_bound_cnt = 0;

extern u8 g_gps_active;
extern u16 g_gps_trace_gap;
extern u32 g_time_start_gps;

extern u8 g_hbeat_active;
extern u8 g_hbeat_gap;// default 6s
extern u32 g_time_start_hbeat;

extern u8 g_hbrake_sta_chged;

extern __sim7500dev sim7500dev;

extern void sim7500e_mobit_process(u8 index);
void create_logfile(void);
void write_logs(char *module, char *log, u16 size, u8 mode);

//////////////////////////////////////////////////////////////////////////////

u8 is_jiffies_timeout(u32 time_start, u16 delayms)
{
    if (0 == delayms) {
        return 0;
    }

    if (delayms < 100) {
        delayms = 100;
    }

    if (os_jiffies >= time_start) {
        if ((os_jiffies-time_start) > (delayms/100)) {
            return 1;
        }
    } else {
        if ((10000-time_start+os_jiffies) > (delayms/100)) {
            return 1;
        }
    }

    return 0;
}

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

    write_logs("SDTF", (char*)sw_ver, strlen((char*)sw_ver), 2);
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
    music_play("0:/MUSIC/OPENLOCK.mp3");

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

    // 100ms
    TIM5_Init(999,8399);

    VS_Init();

    mpu6050_init();

    do_sd_init();

    printf("before spiflash\n");
    spiflash_init();
    printf("after spiflash\n");

    create_directories();

    delay_ms(1000);
    SIM7000E_RST = 0;
//    do_vs_test();
//    printf("test mp3 ok\n");
}

void SoftReset(void)
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

int main(void)
{
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
    sem_atsend = OSSemCreate(1);

    OSStatInit();

    OS_ENTER_CRITICAL();
    OSTaskCreate(main_task,(void *)0,(OS_STK*)&MAIN_TASK_STK[MAIN_STK_SIZE-1],MAIN_TASK_PRIO);
    OSTaskCreate(usart_task,(void *)0,(OS_STK*)&USART_TASK_STK[USART_STK_SIZE-1],USART_TASK_PRIO);
    OSTaskCreate(timer_task,(void *)0,(OS_STK*)&TIMER_TASK_STK[TIMER_STK_SIZE-1],TIMER_TASK_PRIO);
    OSTaskCreate(higher_task,(void *)0,(OS_STK*)&HIGHER_TASK_STK[HIGHER_STK_SIZE-1],HIGHER_TASK_PRIO);
    OSTaskCreate(lower_task,(void *)0,(OS_STK*)&LOWER_TASK_STK[LOWER_STK_SIZE-1],LOWER_TASK_PRIO);
    OSTaskSuspend(START_TASK_PRIO);

    OS_EXIT_CRITICAL();
}

void MPU6050_Risk_Check()
{
    u8 ret = 0;
    float pitch,roll,yaw;         //欧拉角
    short aacx,aacy,aacz;        //加速度传感器原始数据
    short gyrox,gyroy,gyroz;    //陀螺仪原始数据

    ret = mpu_dmp_get_data(&pitch, &roll, &yaw);
    if (ret == 0) {
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);    //得到加速度传感器数据
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);    //得到陀螺仪数据

        // printf(" (int)(roll*100)=%d ,(int)(pitch*100)=%d ,(int)(yaw*10)=%d \r\n",(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
        if((roll*100>4500)||(roll*100<-4500)||(pitch*100>4500)||(pitch*100<-4500)||(yaw*10>450)||(yaw*10<-450))
        {
            g_mpu_sta = 1;
            // printf("FCL \r\n");
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
#if 0
    // for the 1st time report
    if (0 == KEY_HAND_BRAKE) {// Locked
        g_hbrake_sta_chged = 0;
    } else {
        g_hbrake_sta_chged = 1;
    }

    g_hbrake_sta_chged |= 0x80;
#endif

    g_hbrake_sta_chged = 8;

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

        if (0 == KEY_HAND_BRAKE) {// Locked
            //printf("g_hbrake_sta_chged LA = 0x%.2x\n", g_hbrake_sta_chged);
            if ((g_hbrake_sta_chged&0x7F) != 0) {
                if (hbrake_bound_cnt >= 3) {
                    hbrake_bound_cnt = 0;
                    g_hbrake_sta_chged = 0;
                    g_hbrake_sta_chged |= 0x80;
                }

                hbrake_bound_cnt++;
            } else {
                hbrake_bound_cnt = 0;
            }
        } else {// Unlocked
            //printf("g_hbrake_sta_chged FA = 0x%.2x\n", g_hbrake_sta_chged);
            if ((g_hbrake_sta_chged&0x7F) != 1) {
                if (hbrake_bound_cnt >= 3) {
                    hbrake_bound_cnt = 0;
                    g_hbrake_sta_chged = 1;
                    g_hbrake_sta_chged |= 0x80;
                }

                hbrake_bound_cnt++;
            } else {
                hbrake_bound_cnt = 0;
            }
        }

        OSTimeDlyHMSM(0,0,0,500);// 500ms
    }
}

extern vu16 MOBIT_RX_STA[U1_RX_BUF_CNT];

extern vu16 DW_RX_STA;

extern u8 U1_MOBIT_RX_PRO_ID;

// MPU6050 Check and BT Data Parse
void main_task(void *pdata)
{
    u8 i = 0;
    u8 loop_cnt = 0;

    while(1) {
        for (i=0; i<U1_RX_BUF_CNT; i++) {
            if (MOBIT_RX_STA[U1_MOBIT_RX_PRO_ID]&0X8000) {
//                if (strstr((const char*)(MOBIT_RX_BUF+U1_RX_LEN_ONE*U1_MOBIT_RX_PRO_ID), PROTOCOL_HEAD)) {
                    sim7500e_mobit_process(U1_MOBIT_RX_PRO_ID);
//                }

                MOBIT_RX_STA[U1_MOBIT_RX_PRO_ID] = 0;
                // next index to process
                U1_MOBIT_RX_PRO_ID = (U1_MOBIT_RX_PRO_ID+1)%U1_RX_BUF_CNT;
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

void timer_task(void *pdata)
{
    u8 loop_cnt = 0;

    while (1) {
        if (g_gps_trace_gap) {
            if (is_jiffies_timeout(g_time_start_gps, (g_gps_trace_gap*1000))) {
                g_gps_active = 1;
                g_time_start_gps = os_jiffies;
            }
        }

        if (is_jiffies_timeout(g_time_start_hbeat, (g_hbeat_gap*1000))) {
            g_hbeat_active = 1;
            g_time_start_hbeat = os_jiffies;
        }

        // LEDs Running for 10s
        if (loop_cnt <= 100) {
            if (100 == loop_cnt) {
                // NET NG
                LED_R = 1;
                LED_G = 0;
                LED_Y = 0;
            } else {
                switch (loop_cnt%3)
                {
                    case 0:
                        LED_R = 1;
                        LED_G = 0;
                        LED_Y = 0;
                        break;
                    case 1:
                        LED_R = 0;
                        LED_G = 1;
                        LED_Y = 0;
                        break;
                    case 2:
                        LED_R = 0;
                        LED_G = 0;
                        LED_Y = 1;
                        break;
                    default:
                    break;
                }
            }

            loop_cnt++;
        }

        OSTimeDlyHMSM(0,0,0,100);// 50ms
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

                    sprintf((char*)mp3_file, "0:/MUSIC/%s_tmp.mp3", g_mp3_update_name);
                    res = f_open(&f_txt,(const TCHAR*)mp3_file,FA_READ|FA_WRITE);
                } else if (g_iap_update != 0) {
                        res = f_open(&f_txt,(const TCHAR*)"0:/IAP/APP.BIN",FA_READ|FA_WRITE);
                }

                if (0 == res) {
                    f_lseek(&f_txt, f_txt.fsize);
                    f_write(&f_txt, DW_RX_BUF+g_data_pos, g_data_size, (UINT*)&br);
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
        if (g_sim_retry_cnt++ > 10) {
            SoftReset();
        }
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

    write_logs("HW ERROR", (char*)"HardFault_Handler -> Reboot\n", strlen((char*)"HardFault_Handler Enter -> Reboot\n"), 3);
    SoftReset();

    LED_G = 0;

    while(t<5)
    {
        t++;
        LED_Y = !LED_Y;
        LED_R = !LED_R;
        for(i=0;i<0X1FFFFF;i++);
    }
}
