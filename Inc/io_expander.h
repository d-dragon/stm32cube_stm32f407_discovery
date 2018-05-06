/*
 * io_expander.h
 *
 *  Created on: May 6, 2018
 *      Author: DELL
 */

#include "main.h"
#include "stm32f4xx_hal.h"

#ifndef IO_EXPANDER_H_
#define IO_EXPANDER_H_

#define	FXL_ADDR			0x44
#define Device_ID			0x01
#define	IO_Direction	0x03
#define Output_State	0x05
#define	Output_HiZ		0x07
#define Input_Def_Sta	0x09
#define	Pull_Enable		0x0B
#define	PU_PD					0x0D
#define Input_Status	0x0F
#define Input_Mask		0x11
#define	Interrupt_Sta	0x13

void Init_IO_Expander(I2C_HandleTypeDef *hi2cx);
int HCTL_RST_HIGH(I2C_HandleTypeDef *hi2cx);
int HCTL_RST_LOW(I2C_HandleTypeDef *hi2cx);

#endif /* IO_EXPANDER_H_ */
