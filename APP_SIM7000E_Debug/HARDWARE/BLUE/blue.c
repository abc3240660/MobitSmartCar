#include "sys.h"
#include "blue.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"
#include "ucos_ii.h"
#include "usart6.h"
#include "can1.h"
#include "common.h"
#include <stdlib.h>

// Mobit Protocol:
//           STX + LEN(1B) + RAND(1B) + KEY(1B) + CMD(1B) + DATA(LEN B) + 						CRC(1B)

// Encode
// CRC8 for STX / LEN / RAND / KEY / CMD / DATA
// ^ for KET / CMD / DATA with (RAND=RAND+0x32)

// Decode
// CRC8 for STX / LEN / RAND / KEY / CMD / DATA
// ^ for KET / CMD / DATA with (RAND=RAND-0x32)

u8 g_key_once = 0;
u8 snd_buf[128] = {0};
const u8 passw_ok[32] = "MobitCar";
// const u8 passw_ok[32] = "01234567";

const char CRC8Table[]={
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};


extern u8 g_svr_ip[32];
extern u8 g_svr_apn[32];
extern u8 g_svr_port[8];

extern vu16 UART6_RX_STA;

void SoftReset(void);

u8 rand_fake()
{
	return 0x55;
}

void hc08_query_sta(void)
{
	// TBD: Add IO for Input
//	if (0 == KEY_HC08_STA) {// IDLE
//		g_key_once = 0;
//	} else {// Connected
//	}
}

u8 hc08_parse_atacks(u8 times, u8* out_buf)
{
	u8 i = 0;
	u8 ret = 1;

	for (i=0; i<times; i++) {
		if (UART6_RX_STA&0X8000) {
			UART6_RX_STA = 0;
			ret = 0;
			break;
		}

		delay_ms(50);
	}

	if (out_buf != NULL) {
		for (i=0; i<UART6_RX_STA&0X7FFF; i++) {
			if (i >= 12) {
				break;
			}
			out_buf[i] = UART6_RX_BUF[i];
		}
	}

	return ret;
}

void hc08_init(void)
{
	// Reset value
	g_key_once = 0;

	// TBD: Add IO for Output
#if 0// HW RST
	PDout(0) = 0;
	delay_ms(200);
	PDout(0) = 1;
#endif
	delay_ms(300);

#ifdef HC08_ENABLE
#if 1
	UART6_SendData("AT+NAME=Mobit", sizeof("AT+NAME=Mobit"));
	if (hc08_parse_atacks(5, NULL)) return;
	delay_ms(200);
	UART6_SendData("AT+ADDR=?", sizeof("AT+ADDR=?"));
	if (hc08_parse_atacks(5, g_mac_addr)) return;
	delay_ms(200);
#else
	UART6_SendData("AT+NAMEMobit\r\n", sizeof("AT+NAMEMobit\r\n"));
#endif
#endif
}

u8 crc8_calc(u8 *data, u8 len)
{
	u8 crc_val = 0;
	
	while (len--) {
		crc_val = CRC8Table[crc_val^*(data++)];
	}

	return crc_val;
}

void get_svr_info(u8 *data, u8 len)
{
	u8 i = 0;
	u8 j = 0;
	u8 split_cnt = 0;

	memset(g_svr_ip, 0, 32);
	memset(g_svr_port, 0, 32);
	memset(g_svr_apn, 0, 32);

	for (i=0; i<len; i++) {
		if (',' == data[i]) {
			j = 0;
			split_cnt++;
			continue;
		}

		if (0 == split_cnt) {
			g_svr_ip[j++] = data[i];
		} else if (1 == split_cnt) {
			g_svr_port[j++] = data[i];
		} else if (2 == split_cnt) {
			g_svr_apn[j++] = data[i];
		}
	}

	printf("g_svr_ip = %s\n", g_svr_ip);
	printf("g_svr_port = %s\n", g_svr_port);
	printf("g_svr_apn = %s\n", g_svr_apn);

	// save into flash
	// sys_env_save();
}

void rand_encode(u8 *data, u8 rand, u8 len)
{
	u8 i = 0;

	for (i=0; i<len; i++) {
		data[i] ^= rand;
	}
}

// 1-CRC NG
// 2-Not yet Get KEY
// 3-KEY NG
// 4-Head NG
// 5-Size NG
// 6-Unknown CMD
// ERR Report
// DEV->APP: A3A4 +   01 +     RAND +      KEY +    10 +     01 + 								CRC (CRC NG)
// DEV->APP: A3A4 +   01 +     RAND +      KEY +    10 +     02 + 								CRC (Please Get KEY first)
// DEV->APP: A3A4 +   01 +     RAND +      KEY +    10 +     03 +	 							  CRC (KEY NG)
void auto_err_report(u8 err_mode)
{
	u8 rand_val = rand_fake();

	snd_buf[0] = 0xA3;
	snd_buf[1] = 0xA4;
	snd_buf[2] = 0x01;
	snd_buf[3] = rand_val + 0x32;
	snd_buf[4] = 0x00;
	snd_buf[5] = 0x10;
	snd_buf[6] = err_mode;
	snd_buf[7] = crc8_calc(snd_buf, 7);

	rand_encode(snd_buf+4, rand_val, snd_buf[2]+2);

	UART6_SendData(snd_buf, 8);
}

void ack_other_actions(u8 cmd, u8 is_action_ok)
{
	u8 rand_val = rand_fake();

	snd_buf[0] = 0xA3;
	snd_buf[1] = 0xA4;
	snd_buf[2] = 0x01;
	snd_buf[3] = rand_val + 0x32;
	snd_buf[4] = g_key_once;
	snd_buf[5] = cmd;

	if (is_action_ok) {
		snd_buf[6] = 0x01;
	} else {
		snd_buf[6] = 0x02;
	}

	snd_buf[7] = crc8_calc(snd_buf, 7);

	rand_encode(snd_buf+4, rand_val, snd_buf[2]+2);

	UART6_SendData(snd_buf, 8);
}

void ack_get_key(u8 is_pw_ok)
{
	u8 rand_val = rand_fake();
	
	// g_key_once  = rand_fake();
	g_key_once  = rand_fake() + 0x28;

	snd_buf[0] = 0xA3;
	snd_buf[1] = 0xA4;
	snd_buf[2] = 0x02;
	snd_buf[3] = rand_val + 0x32;

	if (is_pw_ok) {
		snd_buf[4] = g_key_once;
	} else {
		snd_buf[4] = 0x00;
	}

	snd_buf[5] = 0x01;

	if (is_pw_ok) {
		snd_buf[6] = 0x01;
		snd_buf[7] = g_key_once;
	} else {
		snd_buf[6] = 0x00;
		snd_buf[7] = 0x00;
	}

	snd_buf[8] = crc8_calc(snd_buf, 8);

	rand_encode(snd_buf+4, rand_val, snd_buf[2]+2);

	UART6_SendData(snd_buf, 9);
}

void hc08_msg_process(u8 *data, u16 num)
{
	u8 i = 0;
	u8 key = 0;
	u8 rand_val = 0;

#if 0
	printf("HC08 User MSGs:");
	for (i=0; i<(UART6_RX_STA&0X7FFF); i++) {
		printf("%.2X ", UART6_RX_BUF[i]);
	}
	printf("\n");
#endif

    // Hook for IAP
	if ((0x55 == data[0]) || (0xAA == data[1])) {
        // TBD: Update Flash Flag
        SoftReset();
        return;
    }

	// Check Header
	if ((data[0] != 0xA3) || (data[1] != 0xA4)) {
		auto_err_report(4);
		return;
	}

	// Check Size
	if ((data[2]+7) != num) {
		auto_err_report(5);
		return;
	}

#if 0// TBD: wait to implement
	// Check Crc
	if (data[num-1] != crc8_calc(data, num-1)) {
		auto_err_report(1);
		return;
	}
#endif
	
	memset(snd_buf, 0, 128);

	rand_val = data[3];

	rand_encode(data+4, (rand_val-0x32), data[2]+2);

	// Please Get Key first
	if ((data[5] != 0x01) && (0 == g_key_once)) {
		auto_err_report(2);
		return;
	}
	
	// Check KEY
	if ((data[5] != 0x01) && (data[4] != g_key_once)) {
		auto_err_report(3);
		return;
	}
	
	key  = data[4];

	switch (data[5])
	{
	// Get Key
	// APP->DEV: A3A4 +   08 +     RAND +      00  +    01 +     PW(MobitCar) + 					CRC
	// DEV->APP: A3A4 +   02 +     RAND +      KEY +    01 +     01 + KEY + 						CRC (PW is OK)
	// DEV->APP: A3A4 +   02 +     RAND +      KEY +    01 +     00 + 00 + 							CRC (PW is NG)
	case 0x01:
		if (memcmp(data+6, passw_ok, 8)) {// ERR
			ack_get_key(0);
		} else {// OK
			ack_get_key(1);
		}
		break;
	case 0x20:
		// Set IP/APN
		// APP->DEV: A3A4 +   NN +     RAND +      KEY +    20 +     192.168.1.1,88,mobit.apn + 		CRC
		// DEV->APP: A3A4 +   01 +     RAND +      KEY +    20 +     01 + 								CRC (SET OK)
		// DEV->APP: A3A4 +   01 +     RAND +      KEY +    20 +     02 + 								CRC (SET NG)
		get_svr_info(data+6, data[2]);
		ack_other_actions(data[5], 1);
		// ack_config_srv(1);
		SoftReset();
		break;
	case 0x21:
		// Unlock
		// APP->DEV: A3A4 +   00 +     RAND +      KEY +    21 +     NULL + 							CRC
		// DEV->APP: A3A4 +   01 +     RAND +      KEY +    21 +     01/02 + 							CRC (OK/NG)
		CAN1_OpenDoor();
		ack_other_actions(data[5], 1);
		break;
	case 0x22:
		// Lock
		// APP->DEV: A3A4 +   00 +     RAND +      KEY +    22 +     NULL + 							CRC
		// DEV->APP: A3A4 +   01 +     RAND +      KEY +    22 +     01/02 + 							CRC (OK/NG)
		CAN1_CloseDoor();
		ack_other_actions(data[5], 1);
		break;
	case 0x23:
		// Lamp Jump
		// APP->DEV: A3A4 +   00 +     RAND +      KEY +    23 +     NULL + 							CRC
		// DEV->APP: A3A4 +   01 +     RAND +      KEY +    23 +     01/02 + 							CRC (OK/NG)
		CAN1_JumpLamp(5);
		ack_other_actions(data[5], 1);
		break;
		// Reboot
		// APP->DEV: A3A4 +   00 +     RAND +      KEY +    24 +     NULL + 							CRC
		// DEV->APP: A3A4 +   01 +     RAND +      KEY +    24 +     01/02 + 							CRC (OK/NG)
	case 0x24:
		SoftReset();
		ack_other_actions(data[5], 1);
		break;
	default:
		auto_err_report(6);
		break;
	}
}
