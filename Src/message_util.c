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

	switch (ml_msg->cmd_type) {
	case MATLAB_CMD_SEND_PARAM:
		/* TODO - Call SendParam function */
		break;
	case MATLAB_CMD_GET_POS:
		/* TODO - Call GetPos function */
	case MATLAB_CMD_RESTART:
		/* TODO - Call Soft reset function */
		break;
	default:
		err = MSG_PARSER_INVALID_CMD;
//		HAL_UART_Transmit(&huart2, (uint8_t *)resp_msg_err, 18, 5);
		return err;
	}
	/* Execute command */


//	HAL_UART_Transmit(&huart2, ml_msg->payload.data, ml_msg->payload.len, 5);
	return err;
//	HAL_UART_Transmit(&huart2, (uint8_t *)payload, payload_len, 5);
//	mat_msg.len = payload_len;
//	mat_msg.cmd_type = serial_data[1];
//	mat_msg.payload = (uint8_t *)payload;
//	HAL_UART_Transmit_IT(&huart2, (uint8_t *)mat_msg.payload, 2);


}
