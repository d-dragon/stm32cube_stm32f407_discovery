#ifndef __MESSAGE_PARSER_H
#define __MESSAGE_PARSER_H

#include <stdint.h>

/* Configuration **************************************************************/
#define DMA_BUF_SIZE        64      /* DMA circular buffer size in bytes */


typedef enum {
	MATLAB_CMD_REPLY = 0,
	MATLAB_CMD_RESTART,
	MATLAB_CMD_SEND_PARAM,
	MATLAB_CMD_GET_POS,
	MATLAB_CMD_REP_POS
}MatLab_Cmd_Type;

//typedef enum {
//	MSG_TRANSMIT_START = 0,
//	MSG_TRANSMIT_CMD_TYPE,
//	MSG_TRANSMIT_PAYLOAD,
//	MSG_TRANSMIT_CRC,
//	MSG_TRANSMIT_IDLE
//}Message_Transmit_State;

typedef enum {
	MSG_PARSER_SUCCESS = 0,
	MSG_PARSER_INVALID_CMD,
	MSG_PARSER_INVALID_FORMAT,
	MSG_PARSER_CRC_ERROR,
	MSG_PARSER_PAYLOAD_INCORRECT
}Message_Parser_Error_Code;

typedef struct {
	uint8_t len;
	uint8_t *data;
}MatLab_Payload_TypeDef;

typedef struct {
	uint8_t len;
	uint8_t cmd_type;
	MatLab_Payload_TypeDef payload;
	uint8_t crc[2];
} MatLab_Message_TypeDef;

uint8_t MatLab_Message_Parser(MatLab_Message_TypeDef *ml_msg, uint8_t *serial_data, uint8_t data_len);
#endif
