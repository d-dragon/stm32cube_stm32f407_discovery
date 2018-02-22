#include "message_parser.h"
#include "usart.h"

/* If parsing failed return NULL and set error code */
MatLab_Message_TypeDef MatLab_Message_Parser(uint8_t *serial_data, uint8_t msg_len, uint8_t *err) {
	MatLab_Message_TypeDef mat_msg;

	uint8_t payload_len = serial_data[0];
	uint8_t i;
	mat_msg.crc[0] = serial_data[msg_len - 2];
	mat_msg.crc[1] = serial_data[msg_len - 1];

	/********* Validate CRC  **********/


	/**********************************/
	uint8_t payload[payload_len];
	for (i = 0; i < payload_len; i++) {
		payload[i] = serial_data[i + 2];
	}
	mat_msg.len = payload_len;
	mat_msg.cmd_type = serial_data[1];
	mat_msg.payload = (uint8_t *)payload;
//	HAL_UART_Transmit_IT(&huart2, (uint8_t *)mat_msg.payload, 2);
	return mat_msg;
}
