#ifndef __MESSAGE_UTIL_H
#define __MESSAGE_UTIL_H

#include <stdint.h>

/* Configuration **************************************************************/
#define DMA_BUF_SIZE        64      /* DMA circular buffer size in bytes */
#define MSG_LEN_BYTE		1
#define MSG_TYPE_LEN		1
#define MSG_CRC_LEN			2

#define MSG_LEN_IDX			0
#define MSG_TYPE_IDX		1
#define MSG_PAYLOAD_IDX		2

typedef enum {
	MATLAB_CMD_REPLY = 0,
	MATLAB_CMD_RESTART,
	MATLAB_CMD_SEND_PARAM,
	MATLAB_CMD_SET_PWM,
	MATLAB_CMD_GET_ADC,
	MATLAB_CMD_GET_POS,
	MATLAB_CMD_REP_POS
}MatLab_Cmd_Type;

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

uint8_t MatLab_Send_Response(uint8_t msg_type, uint8_t *payload, uint8_t payload_len);
#endif
