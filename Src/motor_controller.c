/*
 * motor_controller.c
 *
 *  Created on: Apr 10, 2018
 *      Author: phand
 */

#include "motor_controller.h"
#include "gpio.h"
#include "tim.h"

uint8_t Motor_Forward_Drive(uint16_t duty_cycle)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty_cycle);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
}

uint8_t Motor_Reverse_Drive(uint16_t duty_cycle)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty_cycle);
}

uint8_t Read_Encoder_Position()
{
	return (uint8_t)HAL_GPIO_ReadPort(GPIOD);
}

void Control_Motor(uint8_t *data, uint8_t len) {

	/***************************************

	 data map = [0] [1] | [2].....[5] | [6].....[9] | [10]...[13]
			    tag_pul |     k_p	  |     k_i	    |     k_d
			ex:  10 00  | 77 BE 8F 3F | 44 8B 54 40 | 08 AC 7C 3F
	tag_pul = 9
	k_p = 1.123 (0x3f8fbe77)
	k_i = 3.321 (0x40548b44)
	k_d = 0.987 (0x3f7cac08)

	*************************************/

	float32_t kp, ki, kd, result_f;
	uint16_t tag_pul = 0;
	uint16_t duty_cycle, k_in, k_d, current_pos, last_error, error;

	last_error = error = 0;

	k_in = 0;

	tag_pul = data[1];
	tag_pul = tag_pul << 8;
	tag_pul |= data[0];


	kp = *(float32_t*) (data + K_P_POS); // f.e. 1.123 -> data = 77 be 8f 3f
	ki = *(float32_t*) (data + K_I_POS);
	kd = *(float32_t*) (data + K_D_POS);

	current_pos = (uint16_t) Read_Encoder_Position();

	error = tag_pul - current_pos;
	k_in = k_in + error;
	k_d = error - last_error;

	result_f = kp*error + ki*k_in + kd*k_d;

	duty_cycle = (uint16_t)result_f;

	if (duty_cycle > 255) {
		duty_cycle = 255;
	}
//	PWM_Set_Duty(duty_cycle);
	if (result_f > 0) {
		Motor_Forward_Drive(duty_cycle);
	} else if (result_f < 0) {
		Motor_Reverse_Drive(duty_cycle);
	}

	/* For Debugging */
	PWM_Set_Duty(duty_cycle);
}

void PWM_Set_Duty(uint16_t duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_cycle);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty_cycle);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty_cycle);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty_cycle);
}
