/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    AVR/hal_lld.c
 * @brief   AVR HAL subsystem low level driver code.
 *
 * @addtogroup HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

uint8_t findBestPrescaler(uint16_t frequency, uint16_t *ratio ,uint8_t *clock_source,uint8_t n)
{
  uint8_t i;
  for(i=0;i<n;i++)
  {
    uint32_t result = F_CPU/ratio[i]/frequency;
    if(result > 256UL)
       return (i-1);
    if(result * ratio[i] * frequency == F_CPU)
      return i;
  };
  
}
/* This function chooses the best clock settings to match the requested CH_FREQUENCY
 * Since this is a 8 bit timer special care is needed when setting the values
 * It tries to reduce rounding errors
 * 
 * 
 */
static void setClock()
{
  uint16_t ratio[]={1024,256,64,8};
  uint8_t clock_source[]={5,4,3,2};
  uint8_t prescaler_index = findBestPrescaler(CH_FREQUENCY,ratio,clock_source,4);
  
  TCCR0B &= ~((1 << CS02)  | (1 << CS01)  | (1 << CS00));
  TCCR0B |=((clock_source[prescaler_index] & 0x07)<<CS00);
  OCR0A   = F_CPU / ratio[prescaler_index] /CH_FREQUENCY - 1;
}
/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {
     /*
   * External interrupts setup, all disabled initially.
   */
  EICRA  = 0x00;
#ifdef EICRB
  EICRB  = 0x00;
#endif
  EIMSK  = 0x00;

  /*
   * Timer 0 setup.
   */
  TCCR0A  = (1 << WGM01) | (0 << WGM00) |                /* CTC mode.        */
            (0 << COM0A1) | (0 << COM0A0) |              /* OC0A disabled.   */
            (0 << COM0B1) | (0 << COM0B0);               /* OC0B disabled.   */
  TCCR0B  = (0 << WGM02) ;				 /* CTC mode.        */
             setClock();  			 
  
  TCNT0   = 0;                                           /* Reset counter.   */
  TIFR0   = (1 << OCF0A);                                /* Reset pending.   */
  TIMSK0  = (1 << OCIE0A);  
}

/** @} */
