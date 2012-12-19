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

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

void spicallback(SPIDriver *spip){
  chprintf(&SD1,"spicallback\n");
  
}
/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */


  halInit();
  chSysInit();
  

   

    static SPIConfig spicfg= {spicallback,NULL};
  
   
  DDRB=(1<<4)|(1<<5)|(1<<7);
  PORTB=0b00010000;
  spiStart(&SPID1,&spicfg);
  sdStart(&SD1, NULL);

  
  
  pwmcnt_t val = 0;
  while(1){
      uint8_t temp;
      spi_lld_select(&SPID1);
      temp = spiPolledExchange(&SPID1, 'a');
      spi_lld_unselect(&SPID1);
      spi_lld_select(&SPID1);
      chprintf(&SD1,"temp1: %x SPCR: %x, SPSR: %x, SPDR: %c, PORTB %x, DDRB: %x\n",temp,SPCR,SPSR,SPDR,PORTB, DDRB);
      
      temp = spiPolledExchange(&SPID1, 'd');
      spi_lld_unselect(&SPID1);
      chprintf(&SD1,"temp1: %x SPCR: %x, SPSR: %x, SPDR: %c\n",temp,SPCR,SPSR,SPDR);
      chThdSleepMilliseconds(500);
  }
}
