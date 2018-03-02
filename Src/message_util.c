#include "message_util.h"
#include "usart.h"

/* set error code */
uint8_t MatLab_Message_Parser(MatLab_Message_TypeDef *ml_msg, uint8_t *serial_data, uint8_t data_len) {

	uint8_t err = MSG_PARSER_SUCCESS;
	/* Parsing raw data to message struct */
	ml_msg->len = serial_data[0];
	ml_msg->cmd_type = serial_data[1];

	ml_msg->crc[0] = serial_data[data_len - 2];
	ml_msg->crc[1] = serial_data[data_len - 1];

	ml_msg->payload.len = ml_msg->len - 1;
	ml_msg->payload.data = &(serial_data[2]);

	/*===============================================================*/

	/********* Validate CRC  **********/


	/**********************************/

	/*********validate message format*********/
	if (ml_msg->len != (data_len - 1 - 2)) {
		err = MSG_PARSER_INVALID_FORMAT;
//		HAL_UART_Transmit(&huart2, (uint8_t *)resp_msg_err, 18, 5);
		return err;
	}

	/*****************************************/

	/* Execute command */


//	HAL_UART_Transmit(&huart2, ml_msg->payload.data, ml_msg->payload.len, 5);
	return err;
//	HAL_UART_Transmit(&huart2, (uint8_t *)payload, payload_len, 5);
//	mat_msg.len = payload_len;
//	mat_msg.cmd_type = serial_data[1];
//	mat_msg.payload = (uint8_t *)payload;
//	HAL_UART_Transmit_IT(&huart2, (uint8_t *)mat_msg.payload, 2);


}

uint8_t MatLab_Send_Response(uint8_t msg_type, uint8_t *payload, uint8_t payload_len) {

	uint8_t msg_len = MSG_LEN_BYTE + MSG_TYPE_LEN + MSG_CRC_LEN + payload_len;
	uint8_t res_msg[msg_len];
	uint8_t i;

	res_msg[MSG_LEN_IDX] = MSG_TYPE_LEN + payload_len;
	res_msg[MSG_TYPE_IDX] = msg_type;
	for(i = 0; i < payload_len; i++) {
		res_msg[MSG_PAYLOAD_IDX + i] = payload[i];
	}

	/* Add CRC to response message */

	/*******************************/

	HAL_UART_Transmit(&huart2, res_msg, msg_len, 5);
	return 0;
}
