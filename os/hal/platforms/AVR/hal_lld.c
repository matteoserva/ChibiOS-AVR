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
/* 0 is failure, 1 is success  but probably there is a better choice,
 * 2 is success and best coice
 */


static uint8_t tryClockPrescaler(uint32_t ratio, uint8_t clock_source)
{
  ratio *= CH_FREQUENCY;
  if(F_CPU / ratio <= 256ULL)
  {
     TCCR0B &= ~((1 << CS02)  | (1 << CS01)  | (1 << CS00));
     TCCR0B |= ((clock_source &0x07) << CS00);
     OCR0A   = F_CPU / ratio - 1;
     if(F_CPU % ratio ==0)
	  return 2;
     return 1;
  }
  return 0;
  
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
  
  
  uint8_t found = 0;
  uint8_t i;
  for(i=0;i<sizeof(clock_source)/sizeof(clock_source[0]);i++)
  {
    uint8_t result =tryClockPrescaler(ratio[i],clock_source[i]);
    found += result;
    if(result == 2)
      return;
  };
  if(found)
    return;
  /* fallback */
  TCCR0B &= ~((1 << CS02)  | (1 << CS01)  | (1 << CS00));
  TCCR0B |=(0 << CS02)  | (1 << CS01)  | (1 << CS00);
  OCR0A   = F_CPU / 64 /CH_FREQUENCY - 1;
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
