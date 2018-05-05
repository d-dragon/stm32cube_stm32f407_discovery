/*
 * delay.c
 *
 *  Created on: May 5, 2018
 *      Author: DELL
 */
#include "delay.h"

/* 168MHz = each clock cycle is 6ns. Loop is always 6 clock cycles?
 * These can get clock stretched if we have interrupts in the background.
 */
void delay_ms(const uint16_t ms)
{
  uint32_t i = ms * 27778;
  while (i-- > 0) {
    asm("nop");
  }
}

void delay_us(const uint16_t us)
{
  uint32_t i = us * 28;
  while (i-- > 0) {
    asm("nop");
  }
}

void delay_ns(const uint16_t ns)
{
  uint32_t i = ns / 36;
  while (i-- > 0) {
    asm("nop");
  }
}
