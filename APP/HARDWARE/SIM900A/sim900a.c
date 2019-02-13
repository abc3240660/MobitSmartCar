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

// Door LOCK Sta:
// BIT7: 0-idle, 1-changed
// BIT0: 0-Close, 1-Open
u8 g_drlock_sta_chged = 0;

// DOOR Sta:
u8 g_door_state = 0;

// ECAR ON12V:
u8 g_power_state = 0;

u8 g_iap_update = 0;
u8 g_iap_update_url[LEN_DW_URL+1] = "";

u8 g_mp3_update = 0;
u8 g_mp3_update_md5[LEN_DW_MD5+1]   = "";
u8 g_mp3_update_url[LEN_DW_URL+1]   = "";
u8 g_mp3_update_name[LEN_FILE_NAME+1] = "";

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

u8 g_bms_temp_max = 30;// 30
u8 g_bms_temp_min = 30;// 30
u8 g_bms_battery_vol = 80;// 80%
u16 g_bms_charged_times = 0;// Save into ExFlash

u8 gps_temp_dat1[32] = "";
u8 gps_temp_dat2[32] = "";
u8 gps_temp_dat3[32] = "";
u8 gps_temp_dat4[32] = "";
u8 gps_temp_dat5[32] = "";

u32 g_need_ack = 0;

u8 g_hbeaterrcnt = 0;

u16 g_car_sta = 0;

u8 g_svr_ip[32]  = "47.104.83.177";
u8 g_svr_port[8] = "88";
u8 g_svr_apn[32] = "CMNET";

u32 g_need_iap_flag = 0;
u32 g_iap_sta_flag = 0;

u8 USART1_RX_BUF_BAK[U1_RECV_LEN_ONE];
u8 USART1_RX_BUF_BAK_MOBIT[U1_RECV_LEN_ONE];

void SoftReset(void);
void write_logs(char *module, char *log, u16 size, u8 mode);

extern u8 g_mp3_list[512];
extern u8 g_mp3_play;
extern u8 g_mp3_play_name[LEN_FILE_NAME+1];
extern u8 g_calypso_card_id[CARD_ID_SIZE+1];
extern u8 g_calypso_serial_num[SERIAL_NUM_SIZE+1];

__sim7500dev sim7500dev;

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
	CMD_ENGINE_STOP,
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
u16 g_gps_trace_gap = 0;// default 10s

u8 g_longitude[32] = "";
u8 g_latitude[32] = "";
u8 g_imei_str[32] = "";

u8 g_dw_write_enable = 0;
vu16 g_data_pos = 0;
vu16 g_data_size = 0;
void sim7500e_mobit_process(u8 index);

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
	strx=strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index),(const char*)"CONNECT OK");
	if (NULL == strx) {
		strx=strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index),(const char*)"ALREADY CONNECT");
		if (NULL == strx) {
			strx=strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index),(const char*)"CONNECT FAIL");
			if (NULL == strx) {
				strx=strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index),(const char*)"ERROR");
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
		sim7500dev.tcp_status = 1;// Connect Failed/Error
	}
	
	return strx;
}

u8 sim7500e_ipsta_check(u8 *sta)
{
	if (strstr((const char*)USART1_RX_BUF_BAK, "CLOSED")) {
		*sta = 2;
	}

	if (strstr((const char*)USART1_RX_BUF_BAK, "CONNECT OK")) {
		*sta = 1;
	}
	
	return NULL;
}

u8 sim7500e_file_dw_check(void)
{
	u16 i = 0;
	u16 size = 0;
	u16 size_pos = 0;
	u16 data_pos = 0;
	u8 size_str[8] = "";

	for (i=0; i<U1_RECV_LEN_ONE; i++) {
		if ((':' == USART1_RX_BUF_BAK[i]) && (' ' == USART1_RX_BUF_BAK[i+1])) {
			size_pos = i + 2;
		}

		if ((i >= size_pos) && (size_pos != 0)) {
			if ((0x0D == USART1_RX_BUF_BAK[i]) && (0x0A == USART1_RX_BUF_BAK[i+1])) {
				data_pos = i + 2;
				break;
			}

			if ((i-size_pos) < 8) {
				size_str[i-size_pos] = USART1_RX_BUF_BAK[i];
			} else {
				printf("DW size is too large = %s!\n", size_str);
			}
		}
	}

	size = atoi((const char*)size_str);

	g_data_pos = data_pos;
	g_data_size = size;
	g_dw_write_enable = 1;
	
	while(1) {
		if (0 == g_dw_write_enable) {
			break;
		}
		delay_ms(10);
	}
	
	g_dw_recved_sum += size;
	
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

u8 sim7500e_gps_check(void)
{
	memset(gps_temp_dat4, 0, 32);
	memset(gps_temp_dat5, 0, 32);
	sscanf((const char*)USART1_RX_BUF_BAK, "%[^,],%[^,],%[^,],%[^,],%[^,]", gps_temp_dat1, gps_temp_dat2, gps_temp_dat3, gps_temp_dat4, gps_temp_dat5);
	
	// 如果中途SIM7000E断电重启，那么是有回显的
	// 判断GPS PWR是否因为异常断电导致被关闭了
	// Exception Process
	
	memset(g_latitude, 0, 32);
	memset(g_longitude, 0, 32);
	if (strlen((const char*)gps_temp_dat4) > 5) {
		strcpy((char*)g_latitude, (const char*)gps_temp_dat4);
		strcpy((char*)g_longitude, (const char*)gps_temp_dat5);
		printf("GPS Latitude(%s), Longitude(%s)\n", gps_temp_dat4, gps_temp_dat5);
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

#if 0
	printf("ACK+++");
	
	for (i=0; i<10; i++) {
		if (0 == USART1_RX_BUF[U1_RECV_LEN_ONE*index+i]) {
			break;
		}
		printf("%c", USART1_RX_BUF[U1_RECV_LEN_ONE*index+i]);
	}
	printf("---ACK\n");
#endif

	// printf("SIM7000E-01 Recv Data %s\n", USART1_RX_BUF+U1_RECV_LEN_ONE*index);
	// write_logs("SIM7000E", (char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index), USART1_RX_STA[index]&0X7FFF, 0);

	// CONNECT's actual result ACK maybe recved 150sec later after "OK"
	// very very long time wait, so must wait till recved CONNECT or ERROR
	
	if (strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index), "CLOSED")) {
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
		strx = strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index), (const char*)str);
	}
	
	if (0 == strcmp((const char*)str, "SEND OK")) {
		g_hbeaterrcnt = 0;
	}
	
	// if use strcmp, when SEND OK + ^Mobit will lose one MSG
	if (0 == strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index), PROTOCOL_HEAD)) {// Do not process ^Mobit MSGs
		// Already get "OK", next to get extra info(IMEI / GPS)
		// Go on receiving other packages ASAP
		memcpy(USART1_RX_BUF_BAK, USART1_RX_BUF+U1_RECV_LEN_ONE*index, U1_RECV_LEN_ONE);
		USART1_RX_STA[index] = 0;
	}
	
	return (u8*)strx;
}

void sim7500e_clear_recved_buf()
{
	u8 i = 0;

	// Clear All Previous Recved AT-ACK from SIM7000E Except MOBIT MSGs
	for (i=0; i<U1_RECV_BUF_CNT; i++) {
		if (USART1_RX_STA[i]&0X8000) {
			if (!strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*i), "^MOBIT")) {
				if (strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*i), "CLOSED")) {
					sim7500dev.tcp_status = 2;
				}
				
				USART1_RX_STA[i] = 0;
			}
		}
	}
}

u8 sim7500e_send_cmd(u8 *cmd, u8 *ack, u16 waittime)
{
	u8 i = 0;
	u8 ret = 0;
	u8 res = 0;
	
	sim7500e_clear_recved_buf();

	// Setup TCP Connect
	if (0 == strcmp((const char*)cmd, "AT+CIPSTART")) {
		sim7500dev.tcp_status = 0;// Reset to IDLE
	}
	
	printf("SIM7000E Send Data %s\n", cmd);

	if (strstr((const char*)(cmd), PROTOCOL_HEAD)) {
		write_logs("SIM7000E", (char*)cmd, strlen((char*)cmd), 1);
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
			for (i=0; i<U1_RECV_BUF_CNT; i++) {
				if (USART1_RX_STA[i]&0X8000) {
					if (sim7500e_check_cmd(ack, i)) {
						res = 0;
						ret = 1;
						break;
					} else {
						res = 1;
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
	printf("SEND:%s\n", send);
	
	if(sim7500e_send_cmd("AT+CIPSEND",">",40)==0)
	{
		sim7500e_send_cmd((u8*)send,0,500);
		delay_ms(20);
		sim7500e_send_cmd((u8*)0X1A,"SEND OK",500);
	}else sim7500e_send_cmd((u8*)0X1B,"OK",500);
}

// DEV ACK
void sim7500e_do_engine_start_ack()
{
	CAN1_StartEngine();

	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_ENGINE_START, g_power_state);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_engine_stop_ack()
{
	CAN1_CloseDoor();
	CAN1_StopEngine();

	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_ENGINE_STOP, g_power_state);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_open_door_ack()
{
	CAN1_OpenDoor();

	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_UNLOCK_DOOR, g_door_state);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_jump_lamp_ack()
{
	CAN1_JumpLamp(g_lamp_times);

	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_JUMP_LAMP);

	sim7500e_tcp_send(send_buf_main);
}

// DEV ACK
void sim7500e_do_ring_alarm_ack()
{
	CAN1_RingAlarm(g_ring_times);

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
		g_car_sta &= ~(1<<BIT_HAND_BRAKE);// IDLE
	} else {
		g_car_sta |= (1<<BIT_HAND_BRAKE);// Active
	}

	car_status[0] = (g_car_sta&BIT_HAND_BRAKE)?'1':'0';
	car_status[1] = (g_car_sta&BIT_FAR_LED)?'1':'0';
	car_status[2] = (g_car_sta&BIT_NEAR_LED)?'1':'0';
	car_status[3] = (g_car_sta&BIT_FOG_LED)?'1':'0';
	car_status[4] = (g_car_sta&BIT_CLEAR_LED)?'1':'0';
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
	memset(send_buf_main, 0, LEN_MAX_SEND);

	sim7500e_send_cmd("AT+CGNSINF","OK",40);
	sim7500e_gps_check();
	
	if (strlen((const char*)g_longitude) > 5) {
		sprintf(send_buf_main, "%s,%s,%s,%s,%s,%s,%s,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_GPS, g_longitude, g_latitude);
	} else {
		sprintf(send_buf_main, "%s,%s,%s,%s,%s,F,F,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_GPS);
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
	memset(send_buf_main, 0, LEN_MAX_SEND);
	sprintf(send_buf_main, "%s,%s,%s,%s,%s,%d,%d,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_ACK, CMD_QUERY_BMS, g_bms_battery_vol, g_bms_charged_times, g_bms_temp_max);

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
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%s,%s,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DEV_REGISTER, HW_VERSION, SW_VERSION, g_bms_battery_vol);

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
		g_car_sta &= ~(1<<BIT_HAND_BRAKE);
	} else {
		g_car_sta |= (1<<BIT_HAND_BRAKE);
	}

	car_status[0] = (g_car_sta&BIT_HAND_BRAKE)?'1':'0';
	car_status[1] = (g_car_sta&BIT_FAR_LED)?'1':'0';
	car_status[2] = (g_car_sta&BIT_NEAR_LED)?'1':'0';
	car_status[3] = (g_car_sta&BIT_FOG_LED)?'1':'0';
	car_status[4] = (g_car_sta&BIT_CLEAR_LED)?'1':'0';
	car_status[5] = (g_car_sta&BIT_LEFT_DOOR)?'1':'0';
	car_status[6] = (g_car_sta&BIT_RIGHT_DOOR)?'1':'0';
	car_status[7] = (g_car_sta >> 8) + '0';
	car_status[8] = (g_car_sta&BIT_CHARGE_STA)?'1':'0';

	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%s,%d,%s,%d,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_HEART_BEAT, dev_time, (g_drlock_sta_chged&0x7F), g_rssi_sim, g_bms_battery_vol, car_status);
	
	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_door_closed_report()
{
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DOOR_LOCKED);

	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_door_opened_report()
{
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_DOOR_UNLOCKED);

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
	memset(send_buf, 0, LEN_MAX_SEND);

	sim7500e_send_cmd("AT+CGNSINF","OK",40);
	sim7500e_gps_check();
	
	if (strlen((const char*)g_longitude) > 5) {
		sprintf(send_buf, "%s,%s,%s,%s,%s,%s,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_REPORT_GPS, g_longitude, g_latitude);
	} else {
		sprintf(send_buf, "%s,%s,%s,%s,F,F,0$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_REPORT_GPS);
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
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%d,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_CHARGE_STARTED, g_bms_battery_vol, g_bms_charged_times);

	sim7500e_tcp_send(send_buf);
}

// DEV Auto SEND
void sim7500e_do_charge_stop_report()
{
	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "%s,%s,%s,%s,%d,%d$", PROTOCOL_HEAD, DEV_TAG, g_imei_str, CMD_CHARGE_STOPED, g_bms_battery_vol, g_bms_charged_times);

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

				if (cmd_type != UNKNOWN_CMD) {
					if (0 == data_pos) {
						data_pos = index;
						// printf("data_pos = %d, cmd_type = %d\n", data_pos, cmd_type);
					}
					
					g_need_ack |= (1<<cmd_type);

					// No need Parse extra params
					if (UNLOCK_DOOR == cmd_type) {
						sim7500e_do_open_door_ack();
					} else if (ENGINE_START == cmd_type) {
						sim7500e_do_engine_start_ack();
					} else if (ENGINE_STOP == cmd_type) {
						sim7500e_do_engine_stop_ack();
					} else if (QUERY_PARAMS == cmd_type) {
						sim7500e_do_query_params_ack();
					} else if (QUERY_CAR == cmd_type) {
						sim7500e_do_query_car_ack();
					} else if (DEV_SHUTDOWN == cmd_type) {
						sim7500e_do_dev_shutdown_ack();
					} else if (QUERY_GPS == cmd_type) {
						sim7500e_do_query_gps_ack();
					} else if (QUERY_BMS == cmd_type) {
						sim7500e_do_query_bms_ack();
					} else if (QUERY_MP3 == cmd_type) {
						sim7500e_do_query_mp3_ack();
					} else if (STOP_TRACE == cmd_type) {
						g_gps_trace_gap = 0;
						printf("g_gps_trace_gap = %d\n", g_gps_trace_gap);
						sim7500e_do_stop_trace_ack();
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
					g_hbeat_gap = atoi(split_str);
					printf("g_hbeat_gap = %d\n", g_hbeat_gap);
				}
			} else if (HEART_BEAT == cmd_type) {
				// Do nothing
			} else if (IAP_UPGRADE == cmd_type) {
				g_iap_update = 1;
				memset(g_iap_update_url, 0, LEN_DW_URL);
				strncpy((char*)g_iap_update_url, split_str, LEN_DW_URL);
				printf("g_iap_update_url = %s\n", g_iap_update_url);
				sim7500e_do_iap_upgrade_ack();
			} else if (MP3_PLAY == cmd_type) {
				memset(g_mp3_play_name, 0, LEN_FILE_NAME);
				strncpy((char*)g_mp3_play_name, split_str, LEN_FILE_NAME);
				printf("g_mp3_play_name = %s\n", g_mp3_play_name);
				
				sim7500e_do_mp3_play_ack();
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
				g_gps_trace_gap = atoi(split_str);
				printf("g_gps_trace_gap = %d\n", g_gps_trace_gap);
				sim7500e_do_start_trace_ack();
			} else if (RING_ALARM == cmd_type) {
				g_ring_times = atoi(split_str);
				printf("g_ring_times = %d\n", g_ring_times);
				sim7500e_do_ring_alarm_ack();
			} else if (JUMP_LAMP == cmd_type) {
				g_lamp_times = atoi(split_str);
				printf("g_lamp_times = %d\n", g_lamp_times);
				sim7500e_do_jump_lamp_ack();
			}
		}
		split_str = strtok(NULL, delims);
		index++;
	}
}

u8 sim7500e_setup_connect(void)
{
	u8 i = 0;
	
	sim7500dev.tcp_status=0;// IDLE
	
	if ((strlen((const char*)g_svr_ip)>0) && (strlen((const char*)g_svr_port)>0)) {
		memset(send_buf, 0, LEN_MAX_SEND);
		sprintf(send_buf, "AT+CIPSTART=\"TCP\",\"%s\",%s", g_svr_ip, g_svr_port);
	} else {
		return 1;
	}

	write_logs("SIM7000E", (char*)"TCP CONNECTED", strlen((char*)"TCP CONNECTED"), 2);

	i = 0;
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

	for (i=0; i<5; i++) {
		if (0 == sim7500e_send_cmd("AT","OK",20))break;
		if (4 == i) return 1;
		delay_ms(50);
	}
	
	// Close Echo Display
	if(sim7500e_send_cmd("ATE0","OK",40)) {
		if(sim7500e_send_cmd("ATE0","OK",40)) return 1;
	}

	// Get IMEI
	if(sim7500e_send_cmd("AT+GSN","OK",40)) {
		if(sim7500e_send_cmd("AT+GSN","OK",40)) return 1;
	}
	sim7500e_imei_check();

	// Get ICCID
	if(sim7500e_send_cmd("AT+CCID","OK",40)) {
		if(sim7500e_send_cmd("AT+CCID","OK",40)) return 1;
	}
	sim7500e_iccid_check();
	
	// Open GPS
	if(sim7500e_send_cmd("AT+CGNSPWR=1","OK",40)) {
		if(sim7500e_send_cmd("AT+CGNSPWR=1","OK",40)) return 1;
	}
	
	if(sim7500e_send_cmd("AT+CNMP=13","OK",40)) {
		if(sim7500e_send_cmd("AT+CNMP=13","OK",40)) return 1;
	}
	
	if(sim7500e_send_cmd("AT+CMNB=2","OK",40)) {
		if(sim7500e_send_cmd("AT+CMNB=2","OK",40)) return 1;
	}
	
	if(sim7500e_send_cmd("AT+CGATT?","+CGATT: 1",40)) {
		if(sim7500e_send_cmd("AT+CGATT?","+CGATT: 1",40)) return 1;
	}

	if(sim7500e_send_cmd("AT+CIPSHUT","SHUT OK",200)) {
		if(sim7500e_send_cmd("AT+CIPSHUT","SHUT OK",200)) return 1;
	}
	
	if (0 == strlen((const char*)g_svr_apn)) {
		return 1;
	}

	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "AT+CSTT=\"%s\"", g_svr_apn);
	if(sim7500e_send_cmd((u8*)send_buf,"OK",40)) {
		if(sim7500e_send_cmd((u8*)send_buf,"OK",40)) return 1;
	}
	
	if(sim7500e_send_cmd("AT+CIICR","OK",200)) {
		if(sim7500e_send_cmd("AT+CIICR","OK",200)) return 1;
	}
	
	if (sim7500e_send_cmd("AT+CIFSR",0,200)) {
		return 1;
	}
	
	delay_ms(500);
		
	// Temp Test
	// CAN1_JumpLamp(g_lamp_times);

	return sim7500e_setup_connect();
}

u8 sim7500e_setup_http(void)
{
	sim7500e_send_cmd("AT+HTTPTERM","XXX",100);
	delay_ms(1000);
	sim7500e_send_cmd("AT+SAPBR=0,1","XXX",100);
	delay_ms(1000);
	
	if (0 == strlen((const char*)g_svr_apn)) {
		return 1;
	}

	memset(send_buf, 0, LEN_MAX_SEND);
	sprintf(send_buf, "AT+SAPBR=3,1,\"APN\",\"%s\"", g_svr_apn);
	if (sim7500e_send_cmd((u8*)send_buf,"OK",500)) {
		if (sim7500e_send_cmd((u8*)send_buf,"OK",500)) return 1;
	}
	
	if (sim7500e_send_cmd("AT+SAPBR=1,1","OK",500)) {
		if (sim7500e_send_cmd("AT+SAPBR=1,1","OK",500)) return 1;
	}
	
	if (sim7500e_send_cmd("AT+SAPBR=2,1","OK",500)) {
		if (sim7500e_send_cmd("AT+SAPBR=2,1","OK",500)) return 1;
	}
	
	if (sim7500e_send_cmd("AT+HTTPINIT","OK",500)) {
		if (sim7500e_send_cmd("AT+HTTPINIT","OK",500)) return 1;
	}
	
	if (sim7500e_send_cmd("AT+HTTPPARA=\"CID\",1","OK",500)) {
		if (sim7500e_send_cmd("AT+HTTPPARA=\"CID\",1","OK",500)) return 1;
	}

    if (g_mp3_update != 0) {
        sprintf(send_buf, "AT+HTTPPARA=\"URL\",\"%s\"", g_mp3_update_url);
    } else if (g_iap_update != 0) {
        sprintf(send_buf, "AT+HTTPPARA=\"URL\",\"%s\"", g_iap_update_url);
    }

	//if (sim7500e_send_cmd("AT+HTTPPARA=\"URL\",\"http://gdlt.sc.chinaz.com/Files/DownLoad/sound1/201701/8224.wav\"","OK",1000)) {
	//	if (sim7500e_send_cmd("AT+HTTPPARA=\"URL\",\"http://gdlt.sc.chinaz.com/Files/DownLoad/sound1/201701/8224.wav\"","OK",1000)) return 1;
	//}

	if (sim7500e_send_cmd((u8*)send_buf,"OK",500)) {
		if (sim7500e_send_cmd((u8*)send_buf,"OK",500)) return 1;
	}
	
	if (sim7500e_send_cmd("AT+HTTPACTION=0","+HTTPACTION: 0,200",500)) {
		if (sim7500e_send_cmd("AT+HTTPACTION=0","+HTTPACTION: 0,200",500)) return 1;
	}
	g_dw_size_total = atoi((const char*)(USART1_RX_BUF_BAK+21));
	printf("g_dw_size_total = %d\n", g_dw_size_total);
	
	return 0;
}

void sim7500e_http_mp3()
{
	if (g_mp3_update != 0) {
		u16 split_size = 0;

		if (1 == g_mp3_update) {
			u8 res = 0;
			FIL f_txt;
			u8 mp3_file[LEN_FILE_NAME+1] = "";

			if (sim7500e_setup_http()) {
				return;
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

			sprintf(send_buf, "AT+HTTPREAD=%d,%d", g_dw_recved_sum, split_size);
			if (0 == sim7500e_send_cmd((u8*)send_buf,"+HTTPREAD",500)) {
				if (sim7500e_file_dw_check()) {
					return;
				}
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
	if (g_iap_update != 0) {
		u16 split_size = 0;

		if (1 == g_iap_update) {
			u8 res = 0;
			FIL f_txt;

			if (sim7500e_setup_http()) {
				return;
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

			if ((split_size+g_dw_recved_sum) > g_dw_size_total) {
				split_size = g_dw_size_total - g_dw_recved_sum;
			} else {
				split_size = U1_DATA_LEN_ONE;
			}

			sprintf(send_buf, "AT+HTTPREAD=%d,%d", g_dw_recved_sum, split_size);
			if (0 == sim7500e_send_cmd((u8*)send_buf,"+HTTPREAD",500)) {
				if (sim7500e_file_dw_check()) {
					return;
				}
			}

			printf("g_dw_recved_sum = %d\n", g_dw_recved_sum);
			if (g_dw_recved_sum == g_dw_size_total) {
				g_iap_update = 0;
				sim7500e_do_iap_upgrade_ack();
        env_update_iap_req(0x1A1A2B2B);
			}
		}
	}
}

void sim7500e_idle_actions(void)
{
	if (g_calypso_active) {
		sim7500e_do_calypso_report();
		g_calypso_active = 0;
	}
	if (g_drlock_sta_chged&0x80) {// Door Changed
		if (g_drlock_sta_chged&0x01) {// OPEN
			sim7500e_do_door_opened_report();
		} else {// CLOSE
			sim7500e_do_door_closed_report();
		}
		g_drlock_sta_chged &= 0x7F;
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
	if (g_gps_trace_gap) {
		// every loop delay 10ms
		if (1 == g_gps_active) {
			g_gps_active = 0;
			sim7500e_do_gps_location_report();
		}
	}

	if (g_mp3_update != 0) {
        sim7500e_http_mp3();
        return;
    }

	if (g_iap_update) {
        sim7500e_http_iap();
	}
	
#if 0
	sim7500e_invalid_move_check();
#endif
}

void sim7500e_mobit_process(u8 index)
{
	u8 *pTemp = NULL;
	u8 data_lenth = 0;

	// printf("SIM7000E-03 Recv Data %s\n", USART1_RX_BUF+U1_RECV_LEN_ONE*index);
	// write_logs("SIM7000E", (char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index), USART1_RX_STA[index]&0X7FFF, 0);
			
	//if (g_hbeaterrcnt) {
	//	if (strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index), "SEND OK")) {
	//		g_hbeaterrcnt = 0;
	//	}
	//}
			
	// Received User Data
	pTemp = (u8*)strstr((const char*)(USART1_RX_BUF+U1_RECV_LEN_ONE*index), PROTOCOL_HEAD);
	if (pTemp) {
		u16 i = 0;
		data_lenth = strlen((const char*)pTemp);
				
		memset(USART1_RX_BUF_BAK_MOBIT, 0, U1_RECV_LEN_ONE);
		memcpy(USART1_RX_BUF_BAK_MOBIT, pTemp, data_lenth);

		printf("RECVED1 MSG(%dB): %s\n", data_lenth, USART1_RX_BUF_BAK_MOBIT);
		write_logs("SIM7000E", (char*)(USART1_RX_BUF_BAK_MOBIT), data_lenth, 0);
		
		for (i=0; i<data_lenth; i++) {
			if ('$' == USART1_RX_BUF_BAK_MOBIT[i]) {
				USART1_RX_BUF_BAK_MOBIT[i] = 0;
			}
		}

		sim7500e_parse_msg((char*)USART1_RX_BUF_BAK_MOBIT);
	}
	
	USART1_RX_STA[index] = 0;// Go on receiving other packages ASAP
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
	// CMD_RING_ALARM,
	} else if (g_need_ack & (1<<RING_ALARM)) {
		g_need_ack &= ~(1<<RING_ALARM);
	// CMD_UNLOCK_DOOR,
	} else if (g_need_ack & (1<<UNLOCK_DOOR)) {
		g_need_ack &= ~(1<<UNLOCK_DOOR);
	// CMD_JUMP_LAMP,
	} else if (g_need_ack & (1<<JUMP_LAMP)) {
		g_need_ack &= ~(1<<JUMP_LAMP);
	// CMD_ENGINE_START,
	} else if (g_need_ack & (1<<ENGINE_START)) {
		g_need_ack &= ~(1<<ENGINE_START);
	// CMD_DEV_SHUTDOWN,
	} else if (g_need_ack & (1<<DEV_SHUTDOWN)) {
		g_need_ack &= ~(1<<DEV_SHUTDOWN);
	// CMD_QUERY_GPS,
	} else if (g_need_ack & (1<<QUERY_GPS)) {
		g_need_ack &= ~(1<<QUERY_GPS);
	// CMD_IAP_UPGRADE,
	} else if (g_need_ack & (1<<IAP_UPGRADE)) {
		g_need_ack &= ~(1<<IAP_UPGRADE);
	// CMD_MP3_UPDATE,	
	} else if (g_need_ack & (1<<MP3_UPDATE)) {
		g_need_ack &= ~(1<<MP3_UPDATE);
	// CMD_MP3_PLAY,
	} else if (g_need_ack & (1<<MP3_PLAY)) {
		g_need_ack &= ~(1<<MP3_PLAY);
	// CMD_START_TRACE,
	} else if (g_need_ack & (1<<START_TRACE)) {
		g_need_ack &= ~(1<<START_TRACE);
	// CMD_STOP_TRACE,
	} else if (g_need_ack & (1<<STOP_TRACE)) {
		g_need_ack &= ~(1<<STOP_TRACE);
	// CMD_QUERY_BMS,
	} else if (g_need_ack & (1<<QUERY_BMS)) {
		g_need_ack &= ~(1<<QUERY_BMS);
	// CMD_QUERY_MP3,
	} else if (g_need_ack & (1<<QUERY_MP3)) {
		g_need_ack &= ~(1<<QUERY_MP3);
	// CMD_QUERY_CAR,
	} else if (g_need_ack & (1<<QUERY_CAR)) {
		g_need_ack &= ~(1<<QUERY_CAR);
	// CMD_ENGINE_STOP,
	} else if (g_need_ack & (1<<ENGINE_STOP)) {
		g_need_ack &= ~(1<<ENGINE_STOP);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void sim7500e_communication_loop(u8 mode,u8* ipaddr,u8* port)
{
	u8 rtc_gap = 0;
	u8 connectsta = 0;// 0-idle 1-connected 2-disconnect

	u16 timex = 0;
	
	RTC_TimeTypeDef RTC_TimeStruct_old;
	RTC_TimeTypeDef RTC_TimeStruct_cur;
		
	if (sim7500e_setup_initial()) {
		return;
	}

	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct_old);

	while (1) {
		if (0 == (timex%20)) {// every 200ms
			LED_G = !LED_G;

			// If TCP Lost Connection, Just Re-connect
			if ((2==connectsta) || (g_hbeaterrcnt>8)) {
 				g_hbeaterrcnt = 0;
				
				if (sim7500e_setup_connect()) {
					return;// Exit This Loop, Re-call Setup Initial
				}
			}
		}

		connectsta = sim7500dev.tcp_status;

		if (connectsta == 1) {
			if (g_gps_trace_gap) {
				if (0 == (timex%(g_gps_trace_gap*100))) {// 10ms per loop
					g_gps_active = 1;
				}
			}
			
			RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct_cur);
			if (RTC_TimeStruct_cur.RTC_Seconds > RTC_TimeStruct_old.RTC_Seconds) {
				rtc_gap = RTC_TimeStruct_cur.RTC_Seconds - RTC_TimeStruct_old.RTC_Seconds;
			} else {
				rtc_gap = (60 - RTC_TimeStruct_old.RTC_Seconds) + RTC_TimeStruct_cur.RTC_Seconds;
			}

			if ((0 == (timex%(g_hbeat_gap*100)) || (rtc_gap > g_hbeat_gap))) {// 10ms per loop
				rtc_gap = 0;
				RTC_TimeStruct_old.RTC_Seconds = RTC_TimeStruct_cur.RTC_Seconds;

				// Get RSSI
				if(sim7500e_send_cmd("AT+CSQ","OK",100)) {
					sim7500e_send_cmd("AT+CSQ","OK",100);
				}
				
				sim7500e_rssi_check();
	
				sim7500e_do_heart_beat_auto();

				g_hbeaterrcnt++;
				printf("hbeaterrcnt = %d\r\n",g_hbeaterrcnt);
			} else {
				sim7500e_idle_actions();
			}
		}

		sim7500e_mobit_msg_ack();

		if (timex >= 60000) {
			timex = 0;
		}
		
		if (0 == timex%500) {
			printf("main_loop test\n");
		}

		timex++;
		
		delay_ms(10);
	}
}
