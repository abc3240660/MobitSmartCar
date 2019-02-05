#ifndef __SIM7500E_H__
#define __SIM7500E_H__	 
#include "sys.h"

typedef struct 
{							  
	u8 tcp_status;// 0-idle, 1-Connect OK, 2-Connect Failed/Error
}__sim7500dev; 

extern __sim7500dev sim7500dev;

#define PROTOCOL_HEAD	"^MOBIT"
#define DEV_TAG			"ECAR"
#define SW_VERSION		"V1.0"
#define HW_VERSION		"V1.0"

#define CMD_DEV_ACK		"Re"// DEV ACK

// F407 Send to Server automatically
#define CMD_DEV_REGISTER	"R0"// DEV Host
#define CMD_HEART_BEAT		"H0"// DEV Host
#define CMD_DOOR_LOCKED		"C1"// DEV Host
#define CMD_DOOR_UNLOCKED	"O1"// DEV Host
#define CMD_CALYPSO_UPLOAD	"C3"// DEV Host
#define CMD_INVALID_MOVE	"W1"// DEV Host
#define CMD_REPORT_GPS		"L1"// DEV Host
#define CMD_IAP_SUCCESS		"U1"// DEV Host
#define CMD_CHARGE_STARTED	"B1"// DEV Host
#define CMD_CHARGE_STOPED	"B3"// DEV Host

// F407 Recv from Server and Action / ACK
#define CMD_QUERY_PARAMS	"C0"// DEV ACK
#define CMD_RING_ALARM		"R2"// DEV ACK
#define CMD_UNLOCK_DOOR		"O0"// DEV ACK
#define CMD_JUMP_LAMP		"S2"// DEV ACK
#define CMD_ENGINE_START	"E0"// DEV ACK
#define CMD_DEV_SHUTDOWN    "S0"// DEV ACK
#define CMD_QUERY_GPS   	"L0"// DEV ACK
#define CMD_IAP_UPGRADE   	"U0"// DEV ACK
#define CMD_MP3_UPDATE  	"U2"// DEV ACK
#define CMD_MP3_PLAY    	"P0"// DEV ACK
#define CMD_START_TRACE   	"T0"// DEV ACK
#define CMD_STOP_TRACE   	"T2"// DEV ACK
#define CMD_QUERY_BMS   	"B0"// DEV ACK
#define CMD_QUERY_MP3   	"P2"// DEV ACK
#define CMD_QUERY_CAR   	"C4"// DEV ACK
#define CMD_ENGINE_STOP   	"C6"// DEV ACK

#define LEN_SYS_TIME    32
#define LEN_IMEI_NO     32
#define LEN_BAT_VOL     32
#define LEN_RSSI_VAL    32
#define LEN_MAX_SEND    256
#define LEN_MAX_RECV    32

#define DEBUG_USE 1

// BIT0~7: 0-OFF,1-ON
#define BIT_HAND_BRAKE   (1 << 0) // ShouSha
#define BIT_FAR_LED      (1 << 1) // YuanGuang
#define BIT_NEAR_LED     (1 << 2) // JinGuang
#define BIT_FOG_LED      (1 << 3) // WuDeng
#define BIT_CLEAR_LED    (1 << 4) // ShiKuoDeng
#define BIT_LEFT_DOOR    (1 << 5) // ZuoCheMen
#define BIT_RIGHT_DOOR   (1 << 6) // YouCheMen
#define BIT_CHARGE_STA   (1 << 7) // ChargeSta

// BIT8~9: 0-N,1-P,2-D,3-R
#define BIT_GEAR_STA     (1 << 8) // DangWei

extern u16 g_car_sta;

enum CMD_TYPE {
	DEV_REGISTER = 0,
	HEART_BEAT,
	QUERY_PARAMS,
	RING_ALARM,
	UNLOCK_DOOR,
	DOOR_LOCKED,
	DOOR_UNLOCKED,
	JUMP_LAMP,
	CALYPSO_UPLOAD,
	ENGINE_START,
	INVALID_MOVE,
	REPORT_GPS,
	IAP_SUCCESS,
	CHARGE_STARTED,
	CHARGE_STOPED,
	DEV_SHUTDOWN,
	QUERY_GPS,
	IAP_UPGRADE,
	MP3_UPDATE,
	MP3_PLAY,
	START_TRACE,
	STOP_TRACE,
	QUERY_BMS,
	QUERY_MP3,
	QUERY_CAR,
	ENGINE_STOP,
	UNKNOWN_CMD
};
 
#define swap16(x) (x&0XFF)<<8|(x&0XFF00)>>8	

u8* sim7500e_check_cmd(u8 *str, u8 index);
u8 sim7500e_send_cmd(u8 *cmd,u8 *ack,u16 waittime);
u8 sim7500e_chr2hex(u8 chr);
u8 sim7500e_hex2chr(u8 hex);
void sim7500e_unigbk_exchange(u8 *src,u8 *dst,u8 mode);
void sim7500e_cmsgin_check(void);
void sim7500e_status_check(void);
void sim7500e_communication_loop(u8 mode,u8* ipaddr,u8* port);

#endif/* __SIM7500E_H__ */





