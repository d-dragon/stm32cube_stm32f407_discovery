#ifndef __MESSAGE_PARSER_H
#define __MESSAGE_PARSER_H



/* Configuration **************************************************************/
#define DMA_BUF_SIZE        64      /* DMA circular buffer size in bytes */


typedef enum {
	MATLAB_CMD_REPLY = 0,
	MATLAB_CMD_RESTART,
	MATLAB_CMD_SEND_PARAM,
	MATLAB_CMD_GET_POS,
	MATLAB_CMD_REP_POS
}MatLab_Cmd_Type;

typedef enum {
	MSG_TRANSMIT_START = 0,
	MSG_TRANSMIT_CMD_TYPE,
	MSG_TRANSMIT_PAYLOAD,
	MSG_TRANSMIT_CRC,
	MSG_TRANSMIT_IDLE
}Message_Transmit_State;

typedef struct {
	uint8_t len;
	uint8_t cmd_type;
	uint8_t* payload;
	uint8_t crc[2];
}MatLab_Message_TypeDef;
#endif
