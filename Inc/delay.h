/*
 * delay.h
 *
 *  Created on: May 5, 2018
 *      Author: DELL
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f4xx.h"

/**
 *  \defgroup DelayH Delay Routines
 *  These routines can be found by including delay.hpp. Additionally, you
 *  will have to build delay.cpp in your Makefile.
 *
 *  Note that these routines are only designed for 168Mhz clock rates, and
 *  that delay routines may take longer than desired if the processor is
 *  under heavy load from interrupts.
 *  \{
 */

/**
 *  \brief Delay for a number of milliseconds.
 *  \param ms The number of milliseconds to delay.
 */
void delay_ms(const uint16_t ms);

/**
 *  \brief Delay for a number of microseconds.
 *  \param us The number of microseconds to delay.
 */
void delay_us(const uint16_t us);

/**
 *  \brief Delay for a number of nanoseconds.
 *  \param ns The number of nanoseconds to delay.
 */
void delay_ns(const uint16_t ns);

/**
 *  \}
 */

#endif /* DELAY_H_ */
