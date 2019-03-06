#include "sim900a.h"
#include "delay.h"
#include "led.h"
#include "w25qxx.h"
#include "malloc.h"
#include "string.h"
#include "usart3.h"
#include "ff.h"
#include "ucos_ii.h"
#include "can1.h"
#include "rfid.h"
#include "MP3PLAY.H"
#include "rtc.h"
#include "stdarg.h"

// Door LOCK Sta:
// BIT7: 0-idle, 1-changed
// BIT0: 0-locked, 1-unlocked
u8 g_drlock_sta_chged = 0;

// Door Open/Close Sta:
// BIT7: 0-idle, 1-changed
// BIT0: 0-closed, 1-opened
u8 g_dropen_sta_chged = 0;

// HandBrake LOCK Sta:
// BIT7: 0-idle, 1-changed
// BIT0: 0-locked, 1-unlocked
u8 g_hbrake_sta_chged = 0;

// DOOR Sta:
u8 g_door_state = 0;

// ECAR ON12V:
u8 g_power_state = 0;

u8 g_rssi_empty_cnt = 0;

u8 g_auto_evt_dr_locked = 0;
u8 g_dr_unlocked_report = 0;

u8 g_iap_update = 0;
u8 g_iap_update_md5[LEN_DW_MD5+1]   = "";
u8 g_iap_update_url[LEN_DW_URL+1] = "";

u8 g_mp3_update = 0;
u8 g_mp3_update_md5[LEN_DW_MD5+1]   = "";
u8 g_mp3_update_url[LEN_DW_URL+1]   = "";
u8 g_mp3_update_name[LEN_FILE_NAME+1] = "test";

u32 g_dw_recved_sum = 0;
u32 g_dw_size_total = 0;

// BIT7: 0-idle, 1-changed
// BIT0: 0-Start, 1-Stop
u8 g_bms_charge_sta_chged = 0;

u8 g_invaid_move = 0;

u8 g_calypso_active = 0;

u8 g_mac_addr[32]  = "112233445566";
u8 g_rssi_sim[32]  = "";
u8 g_iccid_sim[32] = "";

u16 g_bms_vot = 8888;
u16 g_bms_percent = 8888;
u16 g_bms_temp_max = 8888;
u16 g_bms_temp_min = 8888;

u8 g_bms_vot_str[8] = "F";
u8 g_bms_percent_str[8] = "F";
u8 g_bms_temp_max_str[8] = "F";
u8 g_bms_temp_min_str[8] = "F";

u16 g_bms_charged_times = 0;// Save into ExFlash

u8 gps_temp_dat1[32] = "";
u8 gps_temp_dat2[32] = "";
u8 gps_temp_dat3[32] = "";
u8 gps_temp_dat4[32] = "";
u8 gps_temp_dat5[32] = "";
u8 gps_temp_dat6[32] = "";
u8 gps_temp_dat7[32] = "";
u8 gps_temp_dat8[32] = "";

u8 g_hbeaterrcnt = 0;

u16 g_car_sta = 0;

u8 g_svr_ip[32]  = "47.254.143.11";
u8 g_svr_port[8] = "10100";
u8 g_svr_apn[32] = "CMNET";

u32 g_need_iap_flag = 0;
u32 g_iap_sta_flag = 0;


u8 g_cgatt_sta = 0;
u8 g_http_action_sta = 0;

u8 USART1_RX_BUF_BAK[U1_RX_LEN_ONE];

u32 os_jiffies = 0;// 100ms per
u32 g_time_start_gps = 0;
u32 g_time_start_hbeat = 0;

void SoftReset(void);
void write_logs(char *module, char *log, u16 size, u8 mode);

extern u8 g_mp3_list[512];
extern u8 g_mp3_play;
extern u8 g_mp3_play_name[LEN_FILE_NAME+1];
extern u8 g_calypso_card_id[CARD_ID_SIZE+1];
extern u8 g_calypso_serial_num[SERIAL_NUM_SIZE+1];

extern OS_EVENT* sem_atsend;

__sim7500dev sim7500dev;

vu16 AT_RX_STA[U1_RX_BUF_CNT];


u8 U1_AT_RX_PRO_ID = 0;// 0~3
u8 U1_MOBIT_RX_PRO_ID = 0;// 0~3

vu16 MOBIT_RX_STA[U1_RX_BUF_CNT];

vu16 DW_RX_STA = 0;

u32 g_need_ack = 0;

const char* cmd_list[] = {
	CMD_DEV_REGISTER,
	CMD_HEART_BEAT,
	CMD_QUERY_PARAMS,
	CMD_RING_ALARM,
	CMD_UNLOCK_DOOR,
	CMD_DOOR_LOCKED,
	CMD_DOOR_UNLOCKED,
	CMD_JUMP_LAMP,
	CMD_CALYPSO_UPLOAD,
	CMD_ENGINE_START,
	CMD_INVALID_MOVE,
	CMD_REPORT_GPS,
	CMD_IAP_SUCCESS,
	CMD_CHARGE_STARTED,
	CMD_CHARGE_STOPED,
	CMD_DEV_SHUTDOWN,
	CMD_QUERY_GPS,
	CMD_IAP_UPGRADE,
	CMD_MP3_UPDATE,
	CMD_MP3_PLAY,
	CMD_START_TRACE,
	CMD_STOP_TRACE,
	CMD_QUERY_BMS,
	CMD_QUERY_MP3,
	CMD_QUERY_CAR,
	CMD_LOCK_DOOR,
	CMD_DOOR_OPENED,
	CMD_DOOR_CLOSED,
	CMD_BRAKE_LOCKED,
	CMD_BRAKE_UNLOCKED,
	NULL
};

// used in sim7500e_communication_loop path
char send_buf[LEN_MAX_SEND] = "";

// used in sim7500e_mobit_process path
char send_buf_main[LEN_MAX_SEND] = "";

u8 g_server_time[LEN_SYS_TIME+1] = "";

static u8 g_ring_times = 0;
static u8 g_lamp_times = 0;

u8 g_hbeat_gap = 6;// default 6s
u8 g_gps_active = 0;
u8 g_hbeat_active = 0;
u16 g_gps_trace_gap = 0;

u8 g_longitude[32] = "";
u8 g_latitude[32] = "";
u8 g_gps_speed[32] = "";
u8 g_gps_degree[32] = "";

u8 g_imei_str[32] = "";

u8 g_dw_write_enable = 0;
vu16 g_data_pos = 0;
vu16 g_data_size = 0;
void sim7500e_mobit_process(u8 index);

u8 is_jiffies_timeout(u32 time_start, u16 delayms);

u8 sim7500e_get_cmd_count()
{
	u8 cnt = 0;
	while(1) {
		if (NULL == cmd_list[cnt]) {
			break;
		}
		cnt++;
	}

	return cnt;
}

u8 sim7500e_is_supported_cmd(u8 cnt, char* str)
{
	u8 i = 0;

	for (i=0; i<cnt; i++) {
		if (0 == strncmp(str, cmd_list[i], strlen(cmd_list[i]))) {
			break;
		}
	}

	if (i != UNKNOWN_CMD) {
		printf("Recved CMD/ACK %s\n", str);
	}

	return i;
}

char* sim7500e_connect_check(u8 index)
{
	char *strx=0;
	strx=strstr((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*index),(const char*)"CONNECT OK");
	if (NULL == strx) {
		strx=strstr((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*index),(const char*)"ALREADY CONNECT");
		if (NULL == strx) {
			strx=strstr((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*index),(const char*)"CONNECT FAIL");
			if (NULL == strx) {
				strx=strstr((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*index),(const char*)"ERROR");
				if (strx != NULL) {
					sim7500dev.tcp_status = 2;// Connect Failed/Error
				} else {
					// Unknown Recved MSG
				}
			} else {
				sim7500dev.tcp_status = 2;// Connect Failed/Error
			}
		} else {
			sim7500dev.tcp_status = 1;// Connect OK
		}
	} else {
		sim7500dev.tcp_status = 2;// Connect Failed/Error
	}
	
	return strx;
}

u8 sim7500e_file_dw_check(void)
{
    u16 i = 0;
    u16 try_cnt = 0;
    u16 size_pos = 0;
    u16 data_pos = 0;
    u8 size_str[8] = "";

    for (i=0; i<U1_RECV_LEN_ONE; i++) {
        if ((':' == DW_RX_BUF[i]) && (' ' == DW_RX_BUF[i+1])) {
            size_pos = i + 2;
        }

        if ((i >= size_pos) && (size_pos != 0)) {
            if ((0x0D == DW_RX_BUF[i]) && (0x0A == DW_RX_BUF[i+1])) {
                data_pos = i + 2;
                break;
            }

            if ((i-size_pos) < 8) {
                size_str[i-size_pos] = DW_RX_BUF[i];
            } else {
                printf("DW pkt_size is too large = %s!\n", size_str);
                return -1;
            }
        }
    }

    g_data_size = atoi((const char*)size_str);

    g_data_pos = data_pos;
    g_dw_write_enable = 1;

    while(1) {
        // wait too long for write into SD
        if (try_cnt >= 200) {// 10s
            return -1;
        }

        if (0 == g_dw_write_enable) {
            break;
        }

        try_cnt++;
        delay_ms(50);
    }

    g_dw_recved_sum += g_data_size;

    return 0;
}

u8 sim7500e_iccid_check(void)
{
	u8 i = 0;
	
	memset(g_iccid_sim, 0, 32);
	while(1) {
		if ((USART1_RX_BUF_BAK[2+i] == '\r') && (USART1_RX_BUF_BAK[2+i+1] == '\n')) {
				break;
		} else {
			g_iccid_sim[i] = USART1_RX_BUF_BAK[2+i];
		}
		i++;
	}
	
	printf("ICCID = %s\n", g_iccid_sim);
	
	if (strlen((const char*)g_iccid_sim) >= 10) {
		return 1;// OK
	} else {
		return 0;// NG
	}
}

u8 sim7500e_imei_check(void)
{
	u8 i = 0;
	
	memset(g_imei_str, 0, 16);
	while(1) {
		if ((USART1_RX_BUF_BAK[2+i]>='0') && (USART1_RX_BUF_BAK[2+i]<='9')) {
			g_imei_str[i] = USART1_RX_BUF_BAK[2+i];
		} else {
			if ((USART1_RX_BUF_BAK[2+i]!='\r') && (USART1_RX_BUF_BAK[2+i]!='\n')) {
				break;
			}
		}
		i++;
	}
	
	printf("IMEI = %s\n", g_imei_str);
	
	if (strlen((const char*)g_imei_str) >= 10) {
		return 1;// OK
	} else {
		return 0;// NG
	}
}

// +CSQ: 29,99
u8 sim7500e_rssi_check(void)
{
	u8 i = 0;
	
	memset(g_rssi_sim, 0, 32);
	while(1) {
		if (USART1_RX_BUF_BAK[8+i] == ',') {
				break;
		} else {
			if ((USART1_RX_BUF_BAK[8+i]>='0') && (USART1_RX_BUF_BAK[8+i]<='9')) {
				g_rssi_sim[i] = USART1_RX_BUF_BAK[8+i];
			} else {
				break;
			}
		}
		i++;
	}
	
	printf("RSSI = %s\n", g_rssi_sim);
	
	if (strlen((const char*)g_rssi_sim) >= 10) {
		return 1;// OK
	} else {
		return 0;// NG
	}
}

u8 sim7500e_query_rssi(void)
{
		u8 err = 0;
		OSSemPend(sem_atsend,0,&err);

    // Get RSSI
    if(sim7500e_send_cmd("AT+CSQ","OK",100)) {
        sim7500e_send_cmd("AT+CSQ","OK",100);
    }

    sim7500e_rssi_check();
		
		OSSemPost(sem_atsend);
}

u8 sim7500e_gps_check(void)
{
	memset(gps_temp_dat4, 0, 32);
	memset(gps_temp_dat5, 0, 32);
	sscanf((const char*)USART1_RX_BUF_BAK, "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]", gps_temp_dat1, gps_temp_dat2, gps_temp_dat3, gps_temp_dat4, gps_temp_dat5, gps_temp_dat6, gps_temp_dat7, gps_temp_dat8);
	
	// 如果中途SIM7000E断电重启，那么是有回显的
	// 判断GPS PWR是否因为异常断电导致被关闭了
	// Exception Process
	
	memset(g_latitude, 0, 32);
	memset(g_longitude, 0, 32);
	memset(g_gps_speed, 0, 32);
	memset(g_gps_degree, 0, 32);
	if (strlen((const char*)gps_temp_dat4) > 5) {
		strcpy((char*)g_latitude, (const char*)gps_temp_dat4);
		strcpy((char*)g_longitude, (const char*)gps_temp_dat5);
		strcpy((char*)g_gps_speed, (const char*)gps_temp_dat7);
		strcpy((char*)g_gps_degree, (const char*)gps_temp_dat8);
		printf("GPS Latitude(%s), Longitude(%s), Speed(%s), Degree(%s)\n", gps_temp_dat4, gps_temp_dat5, gps_temp_dat7, gps_temp_dat8);
	} else {
		printf("Get GPS Nothing...\n");
	}
	
	
	return 1;// OK
}

// TBD: To Check GPS or MPU6050
u8 sim7500e_invalid_move_check(void)
{
	if (1) {
		g_invaid_move = 0;
	} else {
		g_invaid_move = 1;
	}
	
	return 0;// OK
}

u8* sim7500e_check_cmd(u8 *str, u8 index)
{
	char *strx = 0;
	RTC_TimeTypeDef RTC_TimeStruct;

	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);

#if 0
	u8 i =0 ;
	printf("ACK+++");
	
	for (i=0; i<10; i++) {
		if (0 == AT_RX_BUF[U1_RX_LEN_ONE*index+i]) {
			break;
		}
		printf("%c", AT_RX_BUF[U1_RX_LEN_ONE*index+i]);
	}
	printf("---ACK\n");
#endif

	printf("%02d%02d%02d:CHECK AT_ACK(%d) = %s\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds, index, AT_RX_BUF+U1_RX_LEN_ONE*index);

	// CONNECT's actual result ACK maybe recved 150sec later after "OK"
	// very very long time wait, so must wait till recved CONNECT or ERROR
	
	if (strstr((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*index), "CLOSED")) {
		sim7500dev.tcp_status = 2;
	}

	if (0 == strcmp((const char*)str, "CONNECT")) {
		strx = sim7500e_connect_check(index);
	} else if (0 == strcmp((const char*)str, "XXX")) {
		strx = strstr((const char*)str, "ERROR");
		if (NULL == strx) {
			strx = strstr((const char*)str, "OK");
		}
	} else {
		strx = strstr((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*index), (const char*)str);
	}
	
	if (strx != NULL) {
        if (0 == strncmp((const char*)strx, "SEND ", strlen("SEND "))) {
            g_hbeaterrcnt = 0;
        }

		if (0 == strncmp((const char*)strx, "+CGATT: ", strlen("+CGATT: "))) {
			if ('1' == *(strx+sizeof("+CGATT: ")-1)) {
				g_cgatt_sta = 1;
			} else {
				g_cgatt_sta = 0;
			}
			
			printf("g_cgatt_sta=%d\n", g_cgatt_sta);
		}

		if (0 == strncmp((const char*)strx, "+HTTPACTION: ", strlen("+HTTPACTION: "))) {
			if (strstr((const char*)strx, "+HTTPACTION: 0,200")) {
				if (0 == g_http_action_sta) {
					g_http_action_sta = 1;
				}
				printf("g_http_action_sta=%d\n", g_http_action_sta);
			}
		}
	}
	
	// for further parse info from acks
	memcpy(USART1_RX_BUF_BAK, AT_RX_BUF+U1_RX_LEN_ONE*index, U1_RX_LEN_ONE);
	
	return (u8*)strx;
}

void sim7500e_clear_recved_buf()
{
	u8 i = 0;
	RTC_TimeTypeDef RTC_TimeStruct;

	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);
	
	// Clear All Previous Recved AT-ACK from SIM7000E Except MOBIT MSGs
	for (i=0; i<U1_RX_BUF_CNT; i++) {
		if (AT_RX_STA[U1_AT_RX_PRO_ID]&0X8000) {
				printf("%02d%02d%02d:CLEAR AT_ACK(%d) = %s\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds, U1_AT_RX_PRO_ID, AT_RX_BUF+U1_RX_LEN_ONE*U1_AT_RX_PRO_ID);
			
				if (strstr((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*U1_AT_RX_PRO_ID), "CLOSED")) {
					sim7500dev.tcp_status = 2;
				}

				if (strstr((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*U1_AT_RX_PRO_ID), "+HTTPACTION: 0,200")) {
					if (0 == g_http_action_sta) {
						g_http_action_sta = 1;
					}
					printf("g_http_action_sta=%d\n", g_http_action_sta);
					
					if (0 == g_dw_size_total) {
						g_dw_size_total = atoi((const char*)(AT_RX_BUF+U1_RX_LEN_ONE*U1_AT_RX_PRO_ID+21));
						printf("g_dw_size_total = %d\n", g_dw_size_total);
					}
				}

				AT_RX_STA[U1_AT_RX_PRO_ID] = 0;
				// next index to process
				U1_AT_RX_PRO_ID = (U1_AT_RX_PRO_ID+1)%U1_RX_BUF_CNT;
		}
	}
}

// this function will delay 10ms or waittime*50ms internal
u8 sim7500e_send_cmd(u8 *cmd, u8 *ack, u16 waittime)
{
	u8 i = 0;
	u8 ret = 0;
	u8 res = 0;
	RTC_TimeTypeDef RTC_TimeStruct;

	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);
	
	if (strstr((const char*)(cmd), PROTOCOL_HEAD)) {
		write_logs("SIM7000E", (char*)cmd, strlen((char*)cmd), 1);
	} else {
		if ((u32)cmd <= 0XFF) {
			printf("%02d%02d%02d:SIM7000E Send Data 0x1A/1B\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
		} else {
			printf("%02d%02d%02d:SIM7000E Send Data %s\n", RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds,cmd);
		}
	}
	
	sim7500e_clear_recved_buf();

	if (1 == g_http_action_sta) {
		if (0 == strcmp((const char*)ack, "+HTTPACTION: ")) {
			return 0;
		}
	}

	// Setup TCP Connect
	if (0 == strcmp((const char*)cmd, "AT+CIPSTART")) {
		sim7500dev.tcp_status = 0;// Reset to IDLE
	}
	
	if ((u32)cmd <= 0XFF) {
		while ((USART1->SR&0X40) == 0);
		USART1->DR = (u32)cmd;
	} else {
		u1_printf("%s\r\n", cmd);
	}

	if (ack&&waittime) {
		while (--waittime) {
			// quick-ack only wait 10ms
			delay_ms(10);
			for (i=0; i<U1_RX_BUF_CNT; i++) {
				if (AT_RX_STA[U1_AT_RX_PRO_ID]&0X8000) {
					if (sim7500e_check_cmd(ack, U1_AT_RX_PRO_ID)) {
						res = 0;
						ret = 1;
					} else {
						res = 1;
					}

					AT_RX_STA[U1_AT_RX_PRO_ID] = 0;
					// next index to process
					U1_AT_RX_PRO_ID = (U1_AT_RX_PRO_ID+1)%U1_RX_BUF_CNT;
					
					if (0 == res) {
						break;
					}
				}
			}

			if (1 == ret) {
				break;
			}

			// slow-ack. need more wait
			delay_ms(40);
		}

		if (waittime == 0) {
			res = 2;
		}
	}

//	printf("Leave SEND\n");
	return res;
}

void sim7500e_tcp_send(char* send)
{
		u8 err = 0;
		u8 try_cnt = 2;
		OSSemPend(sem_atsend,0,&err);
	
		while (try_cnt--) {
			if (sim7500e_send_cmd("AT+CIPSEND",">",40)==0) {
					sim7500e_send_cmd((u8*)send,0,500);
					delay_ms(20);
					if (sim7500e_send_cmd((u8*)0X1A,"SEND ",500)) {// SEND OK
							printf("[TCP] cannot receive SEND OK!\n");
					} else {
						break;
					}
			} else {
					printf("[TCP] cannot receive > TAG!\n");
					sim7500e_send_cmd((u8*)0X1B,"OK",500);
			}
		}
		
		OSSemPost(sem_atsend);
}

// DEV ACK
void sim7500e_do_engine_start_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_ENGINE_START, g_power_state);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_lock_door_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_LOCK_DOOR, g_power_state);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_unlock_door_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_UNLOCK_DOOR, g_door_state);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_jump_lamp_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_JUMP_LAMP);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_ring_alarm_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_RING_ALARM);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_query_params_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_PARAMS, g_mac_addr, g_iccid_sim);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_query_car_ack()
{
	u8 car_status[12] = "";

	if (0 == KEY_HAND_BRAKE) {
		g_car_sta &= ~(1<<BIT_HAND_BRAKE);// Locked
	} else {
		g_car_sta |= (1<<BIT_HAND_BRAKE);// Unlocked
	}

	car_status[0] = (g_car_sta&BIT_HAND_BRAKE)?'1':'0';
	car_status[1] = '0';//(g_car_sta&BIT_FAR_LED)?'1':'0';
	car_status[2] = '0';//(g_car_sta&BIT_NEAR_LED)?'1':'0';
	car_status[3] = '0';//(g_car_sta&BIT_FOG_LED)?'1':'0';
	car_status[4] = '0';//(g_car_sta&BIT_CLEAR_LED)?'1':'0';
	car_status[5] = (g_car_sta&BIT_LEFT_DOOR)?'1':'0';
	car_status[6] = (g_car_sta&BIT_RIGHT_DOOR)?'1':'0';
	car_status[7] = (g_car_sta >> 8) + '0';
	car_status[8] = (g_car_sta&BIT_CHARGE_STA)?'1':'0';

	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_CAR, car_status);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_start_trace_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_START_TRACE);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_stop_trace_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_STOP_TRACE);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_dev_shutdown_ack()
{
	// do something power saving
	// let sim7000e goto sleep

#if 0
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_DEV_SHUTDOWN);

	sim7500e_tcp_send(send_buf_main);
#endif
}

// DEV ACK
void sim7500e_do_query_gps_ack()
{
	u8 err = 0;
	OSSemPend(sem_atsend,0,&err);
	
	memset(send_buf_main, 0, LEN_MAX_SEND);

	sim7500e_send_cmd("AT+CGNSINF","OK",40);
	sim7500e_gps_check();

	OSSemPost(sem_atsend);
	
	if (strlen((const char*)g_longitude) > 5) {
        LED_Y = 0;
		sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s,%s,%s,%s,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_GPS, g_longitude, g_latitude, g_gps_speed, g_gps_degree);
	} else {
        LED_Y = 1;
		sprintf(send_buf_main, "%s,%s,%s,%s,%s,F,F,F,F,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_GPS);
	}

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_iap_upgrade_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_IAP_UPGRADE);

	sim7500e_tcp_send(send_buf_main);

	// Must stop other send/recv first
	// Do http get and Flash
	// Do SoftReset
}

// DEV ACK
void sim7500e_do_mp3_play_ack()
{
	FIL f_txt;
	char filename[64] = "";

	memset(send_buf_main, 0, LEN_MAX_SEND);
	if (strlen((const char*)g_mp3_play_name) < 40) {
        sprintf(filename, "0:/MUSIC/%s.mp3", g_mp3_play_name);
    }

	if (0 == f_open(&f_txt,(const TCHAR*)filename, FA_READ)) {// existing
		g_mp3_play = 1;
		sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s,1$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_MP3_PLAY, g_mp3_play_name);
		// Play mp3 in other task
	} else {// file non-existing
		g_mp3_play = 0;
		sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_MP3_PLAY, g_mp3_play_name);
	}

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_query_bms_ack()
{
	if (8888 != g_bms_temp_max) {
		sprintf((char*)g_bms_temp_max_str, "%d", g_bms_temp_max);
	} else {
		sprintf((char*)g_bms_temp_max_str, "%s", "F");
	}
	g_bms_temp_max = 8888;

	if (8888 != g_bms_percent) {
		sprintf((char*)g_bms_percent_str, "%d", g_bms_percent);
	} else {
		sprintf((char*)g_bms_percent_str, "%s", "F");
	}
	g_bms_percent = 8888;

	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s,%d,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_BMS, g_bms_percent_str, g_bms_charged_times, g_bms_temp_max_str);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_query_mp3_ack()
{
	memset(g_mp3_list, 0, 512);
	memset(send_buf_main, 0, LEN_MAX_SEND);
	scan_files("0:/MUSIC");
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_MP3, g_mp3_list);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_mp3_dw_success_ack()
{
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_MP3_UPDATE, g_mp3_update_name);

	sim7500e_tcp_send(send_buf_main);
}

// DEV Auto Send
u8 sim7500e_do_dev_register_auto()
{
	if (8888 != g_bms_percent) {
		sprintf((char*)g_bms_percent_str, "%d", g_bms_percent);
	} else {
		sprintf((char*)g_bms_percent_str, "%s", "F");
	}
	g_bms_percent = 8888;

	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_REGISTER, HW_VERSION, SW_VERSION, g_bms_percent_str);

	sim7500e_tcp_send(send_buf);
	
	return 0;
}

// DEV Auto Send
void sim7500e_do_heart_beat_auto()
{
	u8 car_status[12] = "";
	char dev_time[LEN_SYS_TIME] = "";

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
		
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

	sprintf((char*)dev_time,"20%02d%02d%02d%02d%02d%02d",RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date,RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);


	if (0 == KEY_HAND_BRAKE) {
		g_car_sta &= ~(1<<BIT_HAND_BRAKE);// Locked
	} else {
		g_car_sta |= (1<<BIT_HAND_BRAKE);// Unlocked
	}

	car_status[0] = (g_car_sta&BIT_HAND_BRAKE)?'1':'0';
	car_status[1] = '0';//(g_car_sta&BIT_FAR_LED)?'1':'0';
	car_status[2] = '0';//(g_car_sta&BIT_NEAR_LED)?'1':'0';
	car_status[3] = '0';//(g_car_sta&BIT_FOG_LED)?'1':'0';
	car_status[4] = '0';//(g_car_sta&BIT_CLEAR_LED)?'1':'0';
	car_status[5] = (g_car_sta&BIT_LEFT_DOOR)?'1':'0';
	car_status[6] = (g_car_sta&BIT_RIGHT_DOOR)?'1':'0';
	car_status[7] = (g_car_sta >> 8) + '0';
	car_status[8] = (g_car_sta&BIT_CHARGE_STA)?'1':'0';

	if (8888 != g_bms_vot) {
		sprintf((char*)g_bms_vot_str, "%d", g_bms_vot);
	} else {
		sprintf((char*)g_bms_vot_str, "%s", "F");
	}
	g_bms_vot = 8888;

	if (8888 != g_bms_percent) {
		sprintf((char*)g_bms_percent_str, "%d", g_bms_percent);
	} else {
		sprintf((char*)g_bms_percent_str, "%s", "F");
	}
	g_bms_percent = 8888;

	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%s,%d,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_HEART_BEAT, dev_time, ((~g_drlock_sta_chged)&0x01), g_rssi_sim, g_bms_vot_str, g_bms_percent_str, car_status);
	
	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_door_locked_report()
{
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DOOR_LOCKED);

    sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_door_unlocked_report()
{
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DOOR_UNLOCKED);

    sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_door_opened_report()
{
    memset(send_buf, 0, LEN_MAX_SEND);
    sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DOOR_OPENED);

    sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_door_closed_report()
{
    memset(send_buf, 0, LEN_MAX_SEND);
    sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DOOR_CLOSED);

    sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_brake_locked_report()
{
    memset(send_buf, 0, LEN_MAX_SEND);
    sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_BRAKE_LOCKED);

    sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_brake_unlocked_report()
{
    memset(send_buf, 0, LEN_MAX_SEND);
    sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_BRAKE_UNLOCKED);

    sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_invalid_moving_report()
{
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_INVALID_MOVE);

	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_gps_location_report()
{
	u8 err = 0;
	OSSemPend(sem_atsend,0,&err);

	memset(send_buf, 0, LEN_MAX_SEND);

	sim7500e_send_cmd("AT+CGNSINF","OK",40);
	sim7500e_gps_check();

	OSSemPost(sem_atsend);

	if (strlen((const char*)g_longitude) > 5) {
        LED_Y = 0;
		sprintf(send_buf, "%s,%s,%s,%s,%s,%s,%s,%s,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_REPORT_GPS, g_longitude, g_latitude, g_gps_speed, g_gps_degree);
	} else {
        LED_Y = 1;
        // continue report till located
        g_gps_active = 1;
		sprintf(send_buf, "%s,%s,%s,%s,F,F,F,F,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_REPORT_GPS);
	}

	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_iap_success_report()
{
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_IAP_SUCCESS, SW_VERSION);

	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_charge_start_report()
{
	if (8888 != g_bms_percent) {
		sprintf((char*)g_bms_percent_str, "%d", g_bms_percent);
	} else {
		sprintf((char*)g_bms_percent_str, "%s", "F");
	}
	g_bms_percent = 8888;

	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_CHARGE_STARTED, g_bms_percent_str, g_bms_charged_times);

	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_charge_stop_report()
{
	if (8888 != g_bms_percent) {
		sprintf((char*)g_bms_percent_str, "%d", g_bms_percent);
	} else {
		sprintf((char*)g_bms_percent_str, "%s", "F");
	}
	g_bms_percent = 8888;

	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_CHARGE_STOPED, g_bms_percent_str, g_bms_charged_times);

	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_calypso_report()
{
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_CALYPSO_UPLOAD, g_calypso_card_id);

	sim7500e_tcp_send(send_buf);
}

void sim7500e_parse_msg(char* msg)
{
	int index = 0;
	int data_pos = 0;
	char delims[] = ",";
	char* split_str = NULL;

	enum CMD_TYPE cmd_type = UNKNOWN_CMD;

	int cmd_count = sim7500e_get_cmd_count();

#ifdef DEBUG_USE
	//printf("Support %d CMDs\n", cmd_count);
#endif

	split_str = strtok(msg, delims);
	while(split_str != NULL) {
#ifdef DEBUG_USE
		//printf("split_str = %s\n", split_str);
#endif
		// index = 3: SVR CMD
		// index = 4: SVR ACK
		if ((3 == index) || (4 == index)) {
			if (UNKNOWN_CMD == cmd_type) {
				cmd_type = (enum CMD_TYPE)sim7500e_is_supported_cmd(cmd_count, split_str);

				if (strstr((const char*)msg, (const char*)"B5")) {
					g_door_state = 0;
				}
				
				if (cmd_type != UNKNOWN_CMD) {
					if (0 == data_pos) {
						data_pos = index;
						// printf("data_pos = %d, cmd_type = %d\n", data_pos, cmd_type);
					}

					// No need Parse extra params
          if (UNLOCK_DOOR == cmd_type) {
						CAN1_OpenDoor();
            g_need_ack |= (1<<cmd_type);
          } else if (ENGINE_START == cmd_type) {
						CAN1_StartEngine();
            g_need_ack |= (1<<cmd_type);
          } else if (LOCK_DOOR == cmd_type) {
						CAN1_CloseDoor();
						CAN1_StopEngine();
            g_need_ack |= (1<<cmd_type);
          } else if (QUERY_PARAMS == cmd_type) {
                g_need_ack |= (1<<cmd_type);
          } else if (QUERY_CAR == cmd_type) {
                g_need_ack |= (1<<cmd_type);
          } else if (DEV_SHUTDOWN == cmd_type) {
                g_need_ack |= (1<<cmd_type);
          } else if (QUERY_GPS == cmd_type) {
                g_need_ack |= (1<<cmd_type);
          } else if (QUERY_BMS == cmd_type) {
                g_need_ack |= (1<<cmd_type);
          } else if (QUERY_MP3 == cmd_type) {
                g_need_ack |= (1<<cmd_type);
          } else if (STOP_TRACE == cmd_type) {
								g_gps_trace_gap = 0;
								printf("g_gps_trace_gap = %d\n", g_gps_trace_gap);
						
								g_need_ack |= (1<<cmd_type);
					} else if (HEART_BEAT == cmd_type) {
					// Do nothing
					} else if (DOOR_LOCKED == cmd_type) {
                g_drlock_sta_chged &= 0x7F;
                //printf("recved DOOR_LOCKED ACK from Server\n");
					} else if (DOOR_UNLOCKED == cmd_type) {
                g_drlock_sta_chged &= 0x7F;
                //printf("recved DOOR_UNLOCKED ACK from Server\n");
					} else if (DOOR_OPENED == cmd_type) {
                g_dropen_sta_chged &= 0x7F;
                //printf("recved DOOR_OPENED ACK from Server\n");
					} else if (DOOR_CLOSED == cmd_type) {
                g_dropen_sta_chged &= 0x7F;
                //printf("recved DOOR_CLOSED ACK from Server\n");
					} else if (BRAKE_LOCKED == cmd_type) {
                g_hbrake_sta_chged &= 0x7F;
                //printf("recved BRAKE_LOCKED ACK from Server\n");
					} else if (BRAKE_UNLOCKED == cmd_type) {
                g_hbrake_sta_chged &= 0x7F;
                //printf("recved BRAKE_UNLOCKED ACK from Server\n");
					}
				} else {// Maybe Re, So Can Not break
					// break;
				}
			}
		}

		// Parse CMD or ACK
		// Need to Parse extra params
		if (index > data_pos) {
			if (DEV_REGISTER == cmd_type) {
				if (5 == index) {
					strncpy((char*)g_server_time, split_str, LEN_SYS_TIME);
					g_server_time[LEN_SYS_TIME] = '\0';
					printf("g_server_time = %s\n", g_server_time);
					RTC_Sync_time(g_server_time);
				} else if (6 == index) {
                    g_time_start_hbeat = os_jiffies;
					g_hbeat_gap = atoi(split_str);
					printf("g_hbeat_gap = %d\n", g_hbeat_gap);
                    if (g_hbeaterrcnt < 5) {
                        g_hbeaterrcnt = 5;
                        printf("change g_hbeat_gap = %d\n", g_hbeat_gap);
                    }
				}
			} else if (IAP_UPGRADE == cmd_type) {
				g_iap_update = 1;
				memset(g_iap_update_url, 0, LEN_DW_URL);
				strncpy((char*)g_iap_update_url, split_str, LEN_DW_URL);
				printf("g_iap_update_url = %s\n", g_iap_update_url);
				
				g_need_ack |= (1<<cmd_type);
			} else if (MP3_PLAY == cmd_type) {
				memset(g_mp3_play_name, 0, LEN_FILE_NAME);
				strncpy((char*)g_mp3_play_name, split_str, LEN_FILE_NAME);
				printf("g_mp3_play_name = %s\n", g_mp3_play_name);
				
				g_need_ack |= (1<<cmd_type);
			} else if (MP3_UPDATE == cmd_type) {
				if (4 == index) {
					memset(g_mp3_update_name, 0, LEN_FILE_NAME);
					strncpy((char*)g_mp3_update_name, split_str, LEN_FILE_NAME);
					printf("g_mp3_update_name = %s\n", g_mp3_update_name);
				} else if (5 == index) {
					memset(g_mp3_update_url, 0, LEN_DW_URL);
					if (strlen(split_str) < 10) {// if url NULL -> delete file
						u8 mp3_file[LEN_FILE_NAME+1] = "";
						sprintf((char*)mp3_file, "0:/MUSIC/%s.mp3", g_mp3_update_name);
						f_unlink((const char*)mp3_file);
					} else {
						g_mp3_update = 1;
						strncpy((char*)g_mp3_update_url, split_str, LEN_DW_URL);
					}
					printf("g_mp3_update_url = %s\n", g_mp3_update_url);
				} else if (6 == index) {
					memset(g_mp3_update_md5, 0, LEN_DW_MD5);
					strncpy((char*)g_mp3_update_md5, split_str, LEN_DW_MD5);
					printf("g_mp3_update_md5 = %s\n", g_mp3_update_md5);
				}
			} else if (START_TRACE == cmd_type) {
                g_time_start_gps = os_jiffies;
				g_gps_trace_gap = atoi(split_str);
				printf("g_gps_trace_gap = %d\n", g_gps_trace_gap);
                if (g_gps_trace_gap < 5) {
                    g_gps_trace_gap = 5;
                    printf("change g_gps_trace_gap = %d\n", g_gps_trace_gap);
                }

				g_need_ack |= (1<<cmd_type);
			} else if (RING_ALARM == cmd_type) {
				g_ring_times = atoi(split_str);
				printf("g_ring_times = %d\n", g_ring_times);

				CAN1_RingAlarm(g_ring_times);
				g_need_ack |= (1<<cmd_type);
			} else if (JUMP_LAMP == cmd_type) {
				g_lamp_times = atoi(split_str);
				printf("g_lamp_times = %d\n", g_lamp_times);

				CAN1_JumpLamp(g_lamp_times);
				g_need_ack |= (1<<cmd_type);
			}
		}
		split_str = strtok(NULL, delims);
		index++;
	}
}

u8 sim7500e_setup_connect(void)
{
	u8 i = 0;
	u8 err = 0;

  // NET NG
  LED_R = 1;
  LED_G = 0;

	sim7500dev.tcp_status=0;// IDLE
	
	if ((strlen((const char*)g_svr_ip)>0) && (strlen((const char*)g_svr_port)>0)) {
		memset(send_buf, 0, LEN_MAX_SEND);
		sprintf(send_buf, "AT+CIPSTART=\"TCP\",\"%s\",%s", g_svr_ip, g_svr_port);
	} else {
		return 1;
	}

	i = 0;
	OSSemPend(sem_atsend,0,&err);

	while (1) {
		// sim7500e_send_cmd("AT+CIPSHUT","SHUT OK",200);
		// if (sim7500e_send_cmd("AT+CIPSTART=\"TCP\",\"47.105.112.41\",88", "CONNECT",600)) {// Max 600*50ms = 30s		
		if (sim7500e_send_cmd((u8*)send_buf, "CONNECT",600)) {// Max 600*50ms = 30s		
			if(sim7500e_send_cmd("AT+CIPSHUT","SHUT OK",200)) {
				delay_ms(1000);
				sim7500e_send_cmd("AT+CIPSHUT","SHUT OK",200);
			}
		}

		if (1 == sim7500dev.tcp_status) {// Connected OK
			write_logs("SIM7000E", (char*)"TCP CONNECTED", strlen((char*)"TCP CONNECTED"), 2);
			break;
		} else {
			// write_logs("SIM7000E", (char*)"Cannot Setup TCP Connect, just soft restart...\n", strlen((char*)"Cannot Setup TCP Connect, just soft restart...\n"), 3);
			// printf("Cannot Setup TCP Connect, just soft restart...\n");
			// Re-Close->Open Try
			delay_ms(100);
		}
		
		if (5 == i++) {
			break;
		}
	}

	OSSemPost(sem_atsend);

	if (1 == sim7500dev.tcp_status) {// Connected OK
		delay_ms(10);
		if (sim7500e_do_dev_register_auto()) return 1;
		delay_ms(10);

		if (0x3C3C4D4D == g_need_iap_flag) {
			if (0 == g_iap_sta_flag) {
				sim7500e_do_iap_success_report();
			}
		}

		return 0;
	} else {
		return 1;
	}
}

u8 sim7500e_setup_initial(void)
{
	u8 i = 0;
	u8 ret = 0;
	u8 err = 0;

	OSSemPend(sem_atsend,0,&err);

  // NET NG
  LED_R = 1;
  LED_G = 0;

	g_cgatt_sta = 0;

	do {
		for (i=0; i<5; i++) {
			if (0 == sim7500e_send_cmd("AT","OK",20))break;
			if (4 == i) {
				ret = 1;
				break;
			}
			delay_ms(50);
		}
		
		if (1 == ret) {
			break;
		}
		
		// Close Echo Display
		if(sim7500e_send_cmd("ATE0","OK",40)) {
			if(sim7500e_send_cmd("ATE0","OK",40)) {
				ret = 1;
				break;
			}
		}

		// Get IMEI
		if(sim7500e_send_cmd("AT+GSN","OK",40)) {
			if(sim7500e_send_cmd("AT+GSN","OK",40)) {
				ret = 1;
				break;
			}
		}
		sim7500e_imei_check();

		// Get ICCID
		if(sim7500e_send_cmd("AT+CCID","OK",40)) {
			if(sim7500e_send_cmd("AT+CCID","OK",40)) {
				ret = 1;
				break;
			}
		}
		sim7500e_iccid_check();
		
		// Open GPS
		if(sim7500e_send_cmd("AT+CGNSPWR=1","OK",40)) {
			if(sim7500e_send_cmd("AT+CGNSPWR=1","OK",40)) {
				ret = 1;
				break;
			}
		}
		
		if(sim7500e_send_cmd("AT+CNMP=13","OK",40)) {
			if(sim7500e_send_cmd("AT+CNMP=13","OK",40)) {
				ret = 1;
				break;
			}
		}
		
		if(sim7500e_send_cmd("AT+CMNB=2","OK",40)) {
			if(sim7500e_send_cmd("AT+CMNB=2","OK",40)) {
				ret = 1;
				break;
			}
		}
		
		g_cgatt_sta = 0;

		for (i=0; i<5; i++) {
			sim7500e_send_cmd("AT+CGATT?","+CGATT: ",40);
			if (1 == g_cgatt_sta) {
				break;
			}

			if (4 == i) {
				ret = 1;
				break;
			}

			delay_ms(1000);
		}
		
		if (1 == ret) {
			break;
		}
		
		if(sim7500e_send_cmd("AT+CIPSHUT","SHUT OK",200)) {
			if(sim7500e_send_cmd("AT+CIPSHUT","SHUT OK",200)) {
				ret = 1;
				break;
			}
		}
		
		if (0 == strlen((const char*)g_svr_apn)) {
				ret = 1;
				break;
		}

		memset(send_buf, 0, LEN_MAX_SEND);
		sprintf(send_buf, "AT+CSTT=\"%s\"", g_svr_apn);
		if(sim7500e_send_cmd((u8*)send_buf,"OK",40)) {
			if(sim7500e_send_cmd((u8*)send_buf,"OK",40)) {
				ret = 1;
				break;
			}
		}
		
		if(sim7500e_send_cmd("AT+CIICR","OK",200)) {
			if(sim7500e_send_cmd("AT+CIICR","OK",200)) {
				ret = 1;
				break;
			}
		}
		
		if (sim7500e_send_cmd("AT+CIFSR",0,200)) {
				ret = 1;
				break;
		}
	} while(0);

	OSSemPost(sem_atsend);
	
	if (1 == ret) {
		return 1;
	}

	delay_ms(500);

	return sim7500e_setup_connect();
}

u8 sim7500e_setup_http(void)
{
    u8 i = 0;
		u8 ret = 0;
    u32 time_start = 0;
		u8 err = 0;
	
		OSSemPend(sem_atsend,0,&err);

		do {
			sim7500e_send_cmd("AT+HTTPTERM","XXX",100);
			delay_ms(1000);
			sim7500e_send_cmd("AT+SAPBR=0,1","XXX",100);
			delay_ms(1000);

			if (0 == strlen((const char*)g_svr_apn)) {
				ret = 1;
				break;
			}
		
			memset(send_buf, 0, LEN_MAX_SEND);
			sprintf(send_buf, "AT+SAPBR=3,1,\"APN\",\"%s\"", g_svr_apn);
			if (sim7500e_send_cmd((u8*)send_buf,"OK",500)) {
					if (sim7500e_send_cmd((u8*)send_buf,"OK",500)) {
						ret = 1;
						break;
					}
			}

			if (sim7500e_send_cmd("AT+SAPBR=1,1","OK",500)) {
					if (sim7500e_send_cmd("AT+SAPBR=1,1","OK",500)) {
						ret = 1;
						break;
					}
			}

			if (sim7500e_send_cmd("AT+SAPBR=2,1","OK",500)) {
					if (sim7500e_send_cmd("AT+SAPBR=2,1","OK",500)) {
						ret = 1;
						break;
					}
			}

			if (sim7500e_send_cmd("AT+HTTPINIT","OK",500)) {
					if (sim7500e_send_cmd("AT+HTTPINIT","OK",500)) {
						ret = 1;
						break;
					}
			}

			if (sim7500e_send_cmd("AT+HTTPPARA=\"CID\",1","OK",500)) {
					if (sim7500e_send_cmd("AT+HTTPPARA=\"CID\",1","OK",500)) {
						ret = 1;
						break;
					}
			}

			if (g_mp3_update != 0) {
					sprintf(send_buf, "AT+HTTPPARA=\"URL\",\"%s\"", g_mp3_update_url);
			} else if (g_iap_update != 0) {
					sprintf(send_buf, "AT+HTTPPARA=\"URL\",\"%s\"", g_iap_update_url);
			}

	#if 1// DEBUG
			if (sim7500e_send_cmd("AT+HTTPPARA=\"URL\",\"http://gdlt.sc.chinaz.com/Files/DownLoad/sound1/201701/8224.wav\"","OK",1000)) {
					if (sim7500e_send_cmd("AT+HTTPPARA=\"URL\",\"http://gdlt.sc.chinaz.com/Files/DownLoad/sound1/201701/8224.wav\"","OK",1000)) {
						ret = 1;
						break;
					}
			}
	#else
			if (sim7500e_send_cmd((u8*)send_buf,"OK",500)) {
					if (sim7500e_send_cmd((u8*)send_buf,"OK",500)) {
						ret = 1;
						break;
					}
			}
	#endif

			g_http_action_sta = 0;
			g_dw_size_total = 0;

			time_start = os_jiffies;
			for (i=0; i<250; i++) {
					sim7500e_send_cmd("AT+HTTPACTION=0","+HTTPACTION: ",500);
					if (1 == g_http_action_sta) {
							break;
					}

					if (is_jiffies_timeout(time_start, 60000)) {// 60s
							ret = 1;
							break;
					}

					if (240 == i) {
						ret = 1;
						break;
					}

					delay_ms(5000);

					if (1 == g_http_action_sta) {
							break;
					}
			}
		} while(0);

		OSSemPost(sem_atsend);
		
		if (1 == ret) {
			return 1;
		}

    if (0 == g_dw_size_total) {
        g_dw_size_total = atoi((const char*)(USART1_RX_BUF_BAK+21));
        printf("g_dw_size_total = %d\n", g_dw_size_total);
    }

    return 0;
}

void sim7500e_http_mp3()
{
		u8 err = 0;
    u8 try_cnt = 0;

    if (g_mp3_update != 0) {
        u16 split_size = 0;

        if (1 == g_mp3_update) {
            u8 res = 0;
            FIL f_txt;
            u8 mp3_file[LEN_FILE_NAME+1] = "";

            // try two times
            if (sim7500e_setup_http()) {
                if (sim7500e_setup_http()) return;
            }

            if (strlen((const char*)g_mp3_update_name) > 40) {
                printf("file name is too long\n");
                return;
            }

            sprintf((char*)mp3_file, "0:/MUSIC/%s.mp3", g_mp3_update_name);
            res = f_open(&f_txt,(const TCHAR*)mp3_file,FA_READ|FA_WRITE|FA_CREATE_ALWAYS);
            if (0 == res) {
                f_close(&f_txt);
            } else {
                return;
            }
        }

        if (g_dw_size_total > 0) {
            g_mp3_update++;

            // do http get
            if ((split_size+g_dw_recved_sum) > g_dw_size_total) {
                split_size = g_dw_size_total - g_dw_recved_sum;
            } else {
                split_size = U1_DATA_LEN_ONE;
            }
	
						OSSemPend(sem_atsend,0,&err);
            sprintf(send_buf, "AT+HTTPREAD=%d,%d", g_dw_recved_sum, split_size);
            sim7500e_send_cmd((u8*)send_buf,NULL,500);
						OSSemPost(sem_atsend);
						
            while (0 == DW_RX_STA) {
                if (try_cnt >= 100) {// 5s
                    return;
                }
                delay_ms(50);
                try_cnt++;
            }

            if (sim7500e_file_dw_check()) {
                return;
            }

            printf("g_dw_recved_sum = %d\n", g_dw_recved_sum);
            if (g_dw_recved_sum == g_dw_size_total) {
                g_mp3_update = 0;

                memset(g_mp3_update_md5, 0, LEN_DW_MD5);
                memset(g_mp3_update_url, 0, LEN_DW_URL);
                memset(g_mp3_update_name, 0, LEN_FILE_NAME);

                g_dw_size_total = 0;
                g_dw_recved_sum = 0;
                sim7500e_do_mp3_dw_success_ack();
            }
        }
    }
}

void sim7500e_http_iap()
{
	u8 err = 0;
	u8 try_cnt = 0;

    if (g_iap_update != 0) {
        u16 split_size = 0;

        if (1 == g_iap_update) {
            u8 res = 0;
            FIL f_txt;

            if (sim7500e_setup_http()) {
                if (sim7500e_setup_http()) return;
            }

            res = f_open(&f_txt,(const TCHAR*)"0:/IAP/APP.BIN",FA_READ|FA_WRITE|FA_CREATE_ALWAYS);
            if (0 == res) {
                f_close(&f_txt);
            } else {
                return;
            }
        }

        if (g_dw_size_total > 0) {
            g_iap_update++;

            // do http get
            if ((split_size+g_dw_recved_sum) > g_dw_size_total) {
                split_size = g_dw_size_total - g_dw_recved_sum;
            } else {
                split_size = U1_DATA_LEN_ONE;
            }
						OSSemPend(sem_atsend,0,&err);
            sprintf(send_buf, "AT+HTTPREAD=%d,%d", g_dw_recved_sum, split_size);
            sim7500e_send_cmd((u8*)send_buf,NULL,500);
						OSSemPost(sem_atsend);

            while (0 == DW_RX_STA) {
                if (try_cnt >= 100) {// 5s
                    return;
                }
                delay_ms(50);
                try_cnt++;
            }

            if (sim7500e_file_dw_check()) {
                return;
            }

            printf("g_dw_recved_sum = %d\n", g_dw_recved_sum);
            if (g_dw_recved_sum == g_dw_size_total) {
                g_iap_update = 0;

                memset(g_iap_update_md5, 0, LEN_DW_MD5);
                memset(g_iap_update_url, 0, LEN_DW_URL);

                g_dw_size_total = 0;
                g_dw_recved_sum = 0;

                sim7500e_do_iap_upgrade_ack();
                env_update_iap_req(0x1A1A2B2B);
            }
        }
    }
}

void sim7500e_idle_actions(void)
{
    // during download mode, skip other operations
    // till download success or failed
    if (g_iap_update != 0) {
        sim7500e_http_iap();

        // try twice NG, skip this request
        if (g_iap_update != 0) {
            g_iap_update = 0;

            memset(g_iap_update_md5, 0, LEN_DW_MD5);
            memset(g_iap_update_url, 0, LEN_DW_URL);

            g_dw_size_total = 0;
            g_dw_recved_sum = 0;
        }

        return;
    }

    // during download mode, skip other operations
    // till download success or failed
    if (g_mp3_update != 0) {
        sim7500e_http_mp3();

        // try twice NG, skip this request
        if (g_mp3_update != 0) {
            g_mp3_update = 0;

            memset(g_mp3_update_md5, 0, LEN_DW_MD5);
            memset(g_mp3_update_url, 0, LEN_DW_URL);
            memset(g_mp3_update_name, 0, LEN_FILE_NAME);

            g_dw_size_total = 0;
            g_dw_recved_sum = 0;
        }

        return;
    }

    if (g_calypso_active) {
        sim7500e_do_calypso_report();
        g_calypso_active = 0;
    }

    // always send till received ACK from server
    if (g_drlock_sta_chged&0x80) {// Door Lock Changed
        if (g_drlock_sta_chged&0x01) {// Unlocked
            sim7500e_do_door_unlocked_report();
        } else {// Locked
            sim7500e_do_door_locked_report();
            g_gps_active = 1;
        }
    }

    // always send till received ACK from server
    if (g_dropen_sta_chged&0x80) {// Door Open Changed
        if (g_dropen_sta_chged&0x01) {// Opened
            sim7500e_do_door_opened_report();
        } else {// Closed
            sim7500e_do_door_closed_report();
        }
    }

    // always send till received ACK from server
    if (g_hbrake_sta_chged&0x80) {// Hand Brake Changed
        if (g_hbrake_sta_chged&0x01) {// Unlocked
            sim7500e_do_brake_unlocked_report();
        } else {// Locked
            sim7500e_do_brake_locked_report();
        }
    }

    if (g_bms_charge_sta_chged&0x80) {// Charge Changed
        if (g_bms_charge_sta_chged&0x01) {// Start
            sim7500e_do_charge_start_report();
        } else {// Stop
            sim7500e_do_charge_stop_report();
        }
        g_bms_charge_sta_chged &= 0x7F;
    }

    if (g_invaid_move) {
        g_invaid_move = 0;
        sim7500e_do_invalid_moving_report();
    }

    if (1 == g_gps_active) {
        g_gps_active = 0;
        sim7500e_do_gps_location_report();
    }

    sim7500e_invalid_move_check();
}

void sim7500e_mobit_process(u8 index)
{
	u8 *pTemp = NULL;
	u8 data_lenth = 0;

    LED_R = 1;
	// Received User Data
	pTemp = (u8*)strstr((const char*)(MOBIT_RX_BUF+U1_RX_LEN_ONE*index), PROTOCOL_HEAD);
	if (pTemp) {
		u16 i = 0;
		data_lenth = strlen((const char*)pTemp);

		printf("RECVED TCP(%dB): %s\n", data_lenth, pTemp);
		// write_logs("SIM7000E", (char*)(pTemp), data_lenth, 0);
		
		for (i=0; i<data_lenth; i++) {
			if ('$' == pTemp[i]) {
				pTemp[i] = 0;
			}
		}

		sim7500e_parse_msg((char*)pTemp);
	}

    delay_ms(20);
    LED_R = 0;
}

void sim7500e_mobit_msg_ack(void)
{
    // Two Path to Control SIM700E
    // 1: sim7500e_communication_loop
    // 2: sim7500e_mobit_process
    // Must Be Careful!!!
    // If ACK Confused, Please Move AT-Send from sim7500e_mobit_process into here
    // Warning: send_buf will be used in two place!!!
				
    // CMD_QUERY_PARAMS,
    if (g_need_ack & (1<<QUERY_PARAMS)) {
			g_need_ack &= ~(1<<QUERY_PARAMS);
			sim7500e_do_query_params_ack();
    // CMD_RING_ALARM,
    } else if (g_need_ack & (1<<RING_ALARM)) {
      g_need_ack &= ~(1<<RING_ALARM);
			sim7500e_do_ring_alarm_ack();
    // CMD_UNLOCK_DOOR,
    } else if (g_need_ack & (1<<UNLOCK_DOOR)) {
			g_need_ack &= ~(1<<UNLOCK_DOOR);
			sim7500e_do_unlock_door_ack();
    // CMD_JUMP_LAMP,
    } else if (g_need_ack & (1<<JUMP_LAMP)) {
      g_need_ack &= ~(1<<JUMP_LAMP);
			sim7500e_do_jump_lamp_ack();
    // CMD_ENGINE_START,
    } else if (g_need_ack & (1<<ENGINE_START)) {
      g_need_ack &= ~(1<<ENGINE_START);
			sim7500e_do_engine_start_ack();
    // CMD_DEV_SHUTDOWN,
    } else if (g_need_ack & (1<<DEV_SHUTDOWN)) {
			g_need_ack &= ~(1<<DEV_SHUTDOWN);
			sim7500e_do_dev_shutdown_ack();
    // CMD_QUERY_GPS,
    } else if (g_need_ack & (1<<QUERY_GPS)) {
			g_need_ack &= ~(1<<QUERY_GPS);
			sim7500e_do_query_gps_ack();
    // CMD_IAP_UPGRADE,
    } else if (g_need_ack & (1<<IAP_UPGRADE)) {
      g_need_ack &= ~(1<<IAP_UPGRADE);
			sim7500e_do_iap_upgrade_ack();
    // CMD_MP3_UPDATE,
    } else if (g_need_ack & (1<<MP3_UPDATE)) {
      g_need_ack &= ~(1<<MP3_UPDATE);
    // CMD_MP3_PLAY,
    } else if (g_need_ack & (1<<MP3_PLAY)) {
      g_need_ack &= ~(1<<MP3_PLAY);
			sim7500e_do_mp3_play_ack();
    // CMD_START_TRACE,
    } else if (g_need_ack & (1<<START_TRACE)) {
      g_need_ack &= ~(1<<START_TRACE);
			sim7500e_do_start_trace_ack();
		// CMD_STOP_TRACE,
    } else if (g_need_ack & (1<<STOP_TRACE)) {
      g_need_ack &= ~(1<<STOP_TRACE);
			sim7500e_do_stop_trace_ack();
    // CMD_QUERY_BMS,
    } else if (g_need_ack & (1<<QUERY_BMS)) {
			g_need_ack &= ~(1<<QUERY_BMS);
			sim7500e_do_query_bms_ack();
    // CMD_QUERY_MP3,
    } else if (g_need_ack & (1<<QUERY_MP3)) {
			g_need_ack &= ~(1<<QUERY_MP3);
			sim7500e_do_query_mp3_ack();
    // CMD_QUERY_CAR,
    } else if (g_need_ack & (1<<QUERY_CAR)) {
			g_need_ack &= ~(1<<QUERY_CAR);
			sim7500e_do_query_car_ack();
    // CMD_ENGINE_STOP,
    } else if (g_need_ack & (1<<LOCK_DOOR)) {
      g_need_ack &= ~(1<<LOCK_DOOR);
			sim7500e_do_lock_door_ack();
    // CMD_DOOR_OPENED,
    } else if (g_need_ack & (1<<DOOR_OPENED)) {
      g_need_ack &= ~(1<<DOOR_OPENED);
    // CMD_DOOR_CLOSED,
    } else if (g_need_ack & (1<<DOOR_CLOSED)) {
      g_need_ack &= ~(1<<DOOR_CLOSED);
    // CMD_BRAKE_LOCKED,
    } else if (g_need_ack & (1<<BRAKE_LOCKED)) {
      g_need_ack &= ~(1<<BRAKE_LOCKED);
    // CMD_BRAKE_UNLOCKED,
    } else if (g_need_ack & (1<<BRAKE_UNLOCKED)) {
      g_need_ack &= ~(1<<BRAKE_UNLOCKED);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void sim7500e_communication_loop(u8 mode,u8* ipaddr,u8* port)
{
    u16 timex = 0;

    if (sim7500e_setup_initial()) {
        return;
    }

    while (1) {
        // tcp lost connection, just do re-connect
        if ((2==sim7500dev.tcp_status) || (g_hbeaterrcnt>8)) {
            g_hbeaterrcnt = 0;
            if (sim7500e_setup_connect()) {
                return;// exit loop, re-call setup initial
            }
        }

        if (1 == sim7500dev.tcp_status) {
            if (g_hbeat_active) {
                // during download mode, skip other operations
                // till download success or failed
                if ((g_mp3_update!=0) || (g_iap_update!=0)) {
                    continue;
                }

                g_hbeat_active = 0;
								printf("xxxxxxxxxxxxxxxxxxxxxxxx begin\n");
                sim7500e_query_rssi();
                sim7500e_do_heart_beat_auto();
								printf("xxxxxxxxxxxxxxxxxxxxxxxx end\n");

								// AT Communication Failed
								if (0 == strlen(g_rssi_sim)) {
									g_rssi_empty_cnt++;
									if (10 == g_rssi_empty_cnt) {
										printf("rssi is empty, do reboot\n");
										SoftReset();
									}
								} else {
									g_rssi_empty_cnt = 0;
								}
								
                g_hbeaterrcnt++;
                printf("hbeaterrcnt = %d\r\n",g_hbeaterrcnt);
            } else {
								sim7500e_mobit_msg_ack();

                // 2000ms
                if (0 == (timex%40)) {
                    sim7500e_idle_actions();
                }
            }
        }

        // 1000ms
        if (0 == (timex%20)) {
            LED_G = !LED_G;
        }

        // 5000ms
        if (0 == (timex%100)) {
            printf("main_loop test\n");
        }

        timex++;

        if (timex >= 10000) {
            timex = 0;
        }

        delay_ms(50);
    }
}
