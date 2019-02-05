#ifndef RFID_READ_H

#define RFID_READ_H

#include "sys.h"
#include "stdio.h"

#define RFID_MAX_BUF_SIZE 64
#define RFID_HEADER_SIZE 3
#define SERIAL_NUM_SIZE 16
#define CARD_ID_SIZE 19

typedef enum _RET_RFID {
	RET_RFID_OK = 0,
	RET_RFID_SEND_LEN_VALID = -1,
	RET_RFID_RECV_LEN_VALID = -2,
	RET_RFID_RECV_BUF_LACK  = -3,
	RET_RFID_RECV_DAT_VALID = -4,
	RET_RFID_RECV_TIM_OUT   = -5
} RET_RFID;

extern u8 calypso_serial_num[SERIAL_NUM_SIZE+1];
extern u8 g_calypso_card_id[CARD_ID_SIZE+1];

extern RET_RFID cpr74_read_calypso(void);

#endif // RFID_READ_H
