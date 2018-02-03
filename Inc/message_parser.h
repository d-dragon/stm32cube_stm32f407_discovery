#ifndef __MESSAGE_PARSER_H
#define __MESSAGE_PARSER_H

typedef enum {
	MATLAB_CMD_RUN_LAB = 0x00,
	MATLAB_CMD_SET_PWM = 0x01,
	MATLAB_CMD_READ_ADC = 0x02
}MatLab_Cmd;

#endif
