#include "rfid.h"
#include "usart5.h" 
#include "delay.h"

static u8 cpr74_recv_size = 0;
static u8* pGotVal = NULL;
u8 g_calypso_card_id[CARD_ID_SIZE+1] = "";
u8 g_calypso_serial_num[SERIAL_NUM_SIZE+1] = "";

extern u8 g_calypso_active;

static u8 cpr74_chr2hex(u8 chr)
{
	if(chr>='0'&&chr<='9')return chr-'0';
	if(chr>='A'&&chr<='F')return (chr-'A'+10);
	if(chr>='a'&&chr<='f')return (chr-'a'+10); 
	return 0;
}

static u8 cpr74_hex2chr(u8 hex)
{
	if(hex<=9)return hex+'0';
	if(hex>=10&&hex<=15)return (hex-10+'A'); 
	return '0';
}

static void cpr74_parse_ack(u8 bit_pos, u8 bit_len)
{
	u8 i = 0;
	u8 j = 0;
	u8 offset = 0;
	u8 bit_cnt = 0;
	u8 tmp_val = 0;

	for (i=0; i<(UART5_RX_STA&0X7FFF); i++) {
		for (j=0; j<8; j++) {
			if (bit_pos <= (i*8 + j + 1)) {
				offset = 7 - ((bit_pos+bit_cnt-1)%8);
				tmp_val += ((UART5_RX_BUF[i]>>offset)&0x01) << (3-bit_cnt%4);
				bit_cnt++;
				if (0 == (bit_cnt%4)) {
					if (pGotVal != NULL) {
						if (tmp_val <= 9) {
							pGotVal[bit_cnt/4-1] = (tmp_val - 0x00) + '0';
						} else {
							pGotVal[bit_cnt/4-1] = (tmp_val - 0x0a) + 'A';
						}
					}
					tmp_val = 0;
				}
				if (bit_len == bit_cnt) {
					break;
				}
			}
		}
		if (bit_len == bit_cnt) {
			break;
		}
	}
	
	printf("RFID GET DATA: %s\n", pGotVal);
}

static void cpr74_error_msg(RET_RFID ret)
{
	if (RET_RFID_RECV_DAT_VALID == ret) {
		printf("CPR74 ERR: Header is not 02\n");
	} else if (RET_RFID_RECV_LEN_VALID == ret) {
		if (0x08 == cpr74_recv_size) {
			printf("CPR74 WAR: No Card Detected\n");
		} else {
			printf("CPR74 ERR: Non Supported Card Detected\n");
		}
	}
}

static RET_RFID cpr74_check_ack(void)
{
	u8 data_len = 0;
	RET_RFID ret = RET_RFID_OK;

	if (UART5_RX_STA&0X8000) {
		if (UART5_RX_BUF[0] != 0x02) {// Header Check
			ret = RET_RFID_RECV_DAT_VALID;
		} else {
			data_len = (UART5_RX_BUF[1] << 8) + UART5_RX_BUF[2];

			if ((data_len+1) != (UART5_RX_STA&0X7FFF)) {// Length Check
				ret = RET_RFID_RECV_LEN_VALID;
			}
			if (data_len < 37) {// Length Check
				ret = RET_RFID_RECV_LEN_VALID;
			}
			
			cpr74_recv_size = data_len;
		}
	} 

	return ret;
}

// SEND:02000EFFB2BE8100B201E41D011D
// RECV:02002A00B20002000004004C21808400000095584E3F90000805C0C8400019500101492492FC90001E6E
// NEW-RECV:    020045001F04004C21808400000095584E3F90000805C0C8400019500101492492FC90001FB20CCA0000000000000000000000000000000000000000000000000000900003E8
static RET_RFID cpr74_apdu_stage1()
{
	RET_RFID ret = RET_RFID_OK;

	// Get Card ID
	pGotVal = g_calypso_card_id;

	cpr74_parse_ack(14*4+3, 76);

	return ret;
}

RET_RFID cpr74_read_calypso(void)
{
	RET_RFID ret = RET_RFID_OK;

	memset(g_calypso_card_id, 0, CARD_ID_SIZE+1);

	ret = cpr74_check_ack();
	if (RET_RFID_OK == ret) {
		cpr74_apdu_stage1();
		g_calypso_active = 1;
	} 

	return ret;
}
