/*
 * motor_controller.h
 *
 *  Created on: Apr 10, 2018
 *      Author: phand
 */

#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define K_P_POS 2
#define K_I_POS 6
#define K_D_POS 10
#define ENCODER_MARK 0x00FFU

typedef struct {
	float32_t LastError;
	float32_t Error;
	float32_t K_p;
	float32_t K_i;
	float32_t K_d;
	float32_t K_in;
	uint16_t Tag_Pul;
}PID_Algo_Params_TypeDef;

extern PID_Algo_Params_TypeDef pid_algo_params;

uint8_t Set_Motor_Speed(PID_Algo_Params_TypeDef *pid_params);

#endif /* MOTOR_CONTROLLER_H_ */
