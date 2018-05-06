/*
 * io_expander.c
 *
 *  Created on: May 6, 2018
 *      Author: DELL
 */
#include "io_expander.h"

void Init_IO_Expander(I2C_HandleTypeDef *hi2cx)
{
	uint8_t i2c_transmit[1];

	while (HAL_I2C_IsDeviceReady(hi2cx, FXL_ADDR<<1, 1, 100) != HAL_OK);
	// Set output for FXL_GPIO - IO_Direction = 0x03
	i2c_transmit[0] = 0xFF;
	while (HAL_I2C_Mem_Write(hi2cx, FXL_ADDR<<1, IO_Direction, 1, i2c_transmit, 1, 100) != HAL_OK);
	// Set LOW level for FXL_GPIO - Output_State = 0x05
	i2c_transmit[0] = 0x00;
	while (HAL_I2C_Mem_Write(hi2cx, FXL_ADDR<<1, Output_State, 1, i2c_transmit, 1, 100) != HAL_OK);
	// Set no HiZ output for FXL_GPIO - Output_HiZ = 0x07
	i2c_transmit[0] = 0x00;
	while (HAL_I2C_Mem_Write(hi2cx, FXL_ADDR<<1, Output_HiZ, 1, i2c_transmit, 1, 100) != HAL_OK);
	// Set no Pull-up/Pull-down output for FXL_GPIO - Pull_Enable = 0x0B
	i2c_transmit[0] = 0x00;
	while (HAL_I2C_Mem_Write(hi2cx, FXL_ADDR<<1, Pull_Enable, 1, i2c_transmit, 1, 100) != HAL_OK);
}

int HCTL_RST_HIGH(I2C_HandleTypeDef *hi2cx)
{
  uint8_t res;
  uint8_t value[1] = {0x04};

  res = HAL_I2C_IsDeviceReady(hi2cx, FXL_ADDR<<1, 1, 100);
  if (res != HAL_OK)
  {
    return res;
  }

  res = HAL_I2C_Mem_Write(hi2cx, FXL_ADDR<<1, Output_State, 1, value, 1, 100);
  if (res != HAL_OK)
  {
    return res;
  }

  return res;
}

int HCTL_RST_LOW(I2C_HandleTypeDef *hi2cx)
{
  uint8_t res;
  uint8_t value[1] = {0x00};

  res = HAL_I2C_IsDeviceReady(hi2cx, FXL_ADDR<<1, 1, 100);
  if (res != HAL_OK)
  {
    return res;
  }

  res = HAL_I2C_Mem_Write(hi2cx, FXL_ADDR<<1, Output_State, 1, value, 1, 100);
  if (res != HAL_OK)
  {
    return res;
  }

  return res;
}


