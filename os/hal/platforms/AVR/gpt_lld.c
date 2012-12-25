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
 * @file    templates/gpt_lld.c
 * @brief   GPT Driver subsystem low level driver source template.
 *
 * @addtogroup GPT
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
typedef volatile uint8_t * timer_registers[8];
timer_registers timer_registers_table[]={ 
  #if USE_AVR_GPT1 || defined(__DOXYGEN__)
  {&TCCR1A,  &TCCR1B, &OCR1AH,&OCR1AL,&TCNT1H,&TCNT1L,&TIFR1,&TIMSK1},
  #endif
  #if USE_AVR_GPT2 || defined(__DOXYGEN__)
  {&TCCR2A,  &TCCR2B, &OCR2A,&OCR2A,&TCNT2,&TCNT2,&TIFR2,&TIMSK2},
  #endif
  #if USE_AVR_GPT3 || defined(__DOXYGEN__)
  {&TCCR3A,  &TCCR3B, &OCR3AH,&OCR3AL,&TCNT3H,&TCNT3L,&TIFR3,&TIMSK3},
  #endif
  #if USE_AVR_GPT4 || defined(__DOXYGEN__)
  {&TCCR4A,  &TCCR4B, &OCR4AH,&OCR4AL,&TCNT4H,&TCNT4L,&TIFR4,&TIMSK4},
  #endif
  #if USE_AVR_GPT5 || defined(__DOXYGEN__)
  {&TCCR5A,  &TCCR5B, &OCR5AH,&OCR5AL,&TCNT5H,&TCNT5L,&TIFR5,&TIMSK5},
#endif
};



/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
#if USE_AVR_GPT1 || defined(__DOXYGEN__)
GPTDriver GPTD1;
#endif
#if USE_AVR_GPT2 || defined(__DOXYGEN__)
GPTDriver GPTD2;
#endif
#if USE_AVR_GPT3 || defined(__DOXYGEN__)
GPTDriver GPTD3;
#endif
#if USE_AVR_GPT4 || defined(__DOXYGEN__)
GPTDriver GPTD4;
#endif
#if USE_AVR_GPT5 || defined(__DOXYGEN__)
GPTDriver GPTD5;
#endif
/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
static void gpt_lld_serve_interrupt(GPTDriver *gptp) {
  gptp->counter++;
  if(gptp->counter == gptp->period)
  {
	gptp->counter = 0;
	if (gptp->state == GPT_ONESHOT) {
	    gptp->state = GPT_READY;                /* Back in GPT_READY state.     */
	    gpt_lld_stop_timer(gptp);               /* Timer automatically stopped. */
	}
	gptp->callback(gptp);
  }
}

static void gpt_lld_dummy_callback(GPTDriver *gptp)
{
}

uint8_t getTimerIndex(GPTDriver *gptp)
{
  uint8_t index = 0;
    #if USE_AVR_GPT1 || defined(__DOXYGEN__)
    if (gptp == &GPTD1) return index; else index++;
      #endif  
    #if USE_AVR_GPT2 || defined(__DOXYGEN__)
    if (gptp == &GPTD1) return index; else index++;
      #endif 
    #if USE_AVR_GPT3 || defined(__DOXYGEN__)
    if (gptp == &GPTD1) return index; else index++;
      #endif 
    #if USE_AVR_GPT4 || defined(__DOXYGEN__)
    if (gptp == &GPTD1) return index; else index++;
      #endif 
    #if USE_AVR_GPT5 || defined(__DOXYGEN__)
    if (gptp == &GPTD1) return index; else index++;
      #endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if USE_AVR_GPT1 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(TIMER1_COMPA_vect) {
    CH_IRQ_PROLOGUE();
      gpt_lld_serve_interrupt(&GPTD1);
    CH_IRQ_EPILOGUE();
}
#endif

#if USE_AVR_GPT2 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(TIMER2_COMPA_vect) {
    CH_IRQ_PROLOGUE();
      gpt_lld_serve_interrupt(&GPTD2);
    CH_IRQ_EPILOGUE();
}
#endif

#if USE_AVR_GPT3 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(TIMER3_COMPA_vect) {
    CH_IRQ_PROLOGUE();
      gpt_lld_serve_interrupt(&GPTD3);
    CH_IRQ_EPILOGUE();
}
#endif

#if USE_AVR_GPT4 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(TIMER4_COMPA_vect) {
    CH_IRQ_PROLOGUE();
      gpt_lld_serve_interrupt(&GPTD4);
    CH_IRQ_EPILOGUE();
}
#endif

#if USE_AVR_GPT5 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(TIMER5_COMPA_vect) {
    CH_IRQ_PROLOGUE();
      gpt_lld_serve_interrupt(&GPTD5);
    CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {
#if USE_AVR_GPT1 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD1);
#endif
  #if USE_AVR_GPT2 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD2);
#endif
    #if USE_AVR_GPT3 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD3);
#endif
    #if USE_AVR_GPT4 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD4);
#endif
    #if USE_AVR_GPT5 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD5);
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {
  uint8_t psc;

  if (gptp->state == GPT_STOP) {
    /* Clock activation.*/

  }
  /* Configuration.*/

  #if USE_AVR_GPT2 || defined(__DOXYGEN__)
  if(gptp == &GPTD2)
  {
      psc = findBestPrescaler(gptp->config->frequency,ratio_extended,clock_source_extended,PRESCALER_SIZE_EXTENDED);
      gptp->clock_source = clock_source_extended[psc] & 0x07;
      TCCR2A  = (1 << WGM21) | (0 << WGM20);
      TCCR2B  = (0 << WGM22);
      OCR2A = F_CPU / ratio_extended[psc] /gptp->config->frequency - 1;   
      return;
  }
#endif
  
  uint8_t index = getTimerIndex(gptp);
      psc = findBestPrescaler(gptp->config->frequency,ratio_base,clock_source_base,PRESCALER_SIZE_BASE);
      gptp->clock_source = clock_source_base[psc] & 0x07;
      *timer_registers_table[index][0]  = (0 << WGM11) | (0 << WGM10) | (0 << COM1A1) | (0 << COM1A0) |(0 << COM1B1) | (0 << COM1B0);
      *timer_registers_table[index][1]  = (1 << WGM12);
      *timer_registers_table[index][2] = 0;
      *timer_registers_table[index][3] = F_CPU / ratio_base[psc] /gptp->config->frequency - 1;  
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {
  /*nothing to be done*/
  if (gptp->state == GPT_READY) {
    /* Clock de-activation.*/

  }
  gpt_lld_stop_timer(gptp);
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] period    period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t period) {
      gptp->callback = gptp->config->callback;
      gptp->period = period;
      gptp->counter = 0;
  #if USE_AVR_GPT1 || defined(__DOXYGEN__)
  if(gptp == &GPTD1)
  {
      TCNT1   = 0;                                           /* Reset counter.   */
      TIFR1   = (1 << OCF1A);                                /* Reset pending.   */
      TIMSK1  = (1 << OCIE1A);
      TCCR1B  |= (gptp->clock_source <<CS10);
  }
#endif

    #if USE_AVR_GPT2 || defined(__DOXYGEN__)
  if(gptp == &GPTD2)
  {
      TCNT2   = 0;                                           /* Reset counter.   */
      TIFR2   = (1 << OCF2A);                                /* Reset pending.   */
      TIMSK2  = (1 << OCIE2A);
      TCCR2B  |= (gptp->clock_source <<CS20);
  }
#endif

  #if USE_AVR_GPT3 || defined(__DOXYGEN__)
  if(gptp == &GPTD3)
  {
      TCNT3   = 0;                                           /* Reset counter.   */
      TIFR3   = (1 << OCF1A);                                /* Reset pending.   */
      TIMSK3  = (1 << OCIE1A);
      TCCR3B  |= (gptp->clock_source <<CS30);
  }
#endif
  #if USE_AVR_GPT4 || defined(__DOXYGEN__)
  if(gptp == &GPTD4)
  {
      TCNT4   = 0;                                           /* Reset counter.   */
      TIFR4   = (1 << OCF1A);                                /* Reset pending.   */
      TIMSK4  = (1 << OCIE1A);
      TCCR4B  |= (gptp->clock_source <<CS40);
  }
#endif
  #if USE_AVR_GPT5 || defined(__DOXYGEN__)
  if(gptp == &GPTD5)
  {
      TCNT5   = 0;                                           /* Reset counter.   */
      TIFR5   = (1 << OCF1A);                                /* Reset pending.   */
      TIMSK5  = (1 << OCIE1A);
      TCCR5B  |= (gptp->clock_source <<CS50);
  }
#endif
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {
    #if USE_AVR_GPT1 || defined(__DOXYGEN__)
  if(gptp == &GPTD1)
  {
      TCCR1B  &= ~(0x07 <<CS10);
      TIMSK1  &= ~(1 << OCIE1A);
      TIFR1   = (1 << OCF1A); 
  }
#endif
    #if USE_AVR_GPT2 || defined(__DOXYGEN__)
  if(gptp == &GPTD2)
  {
      TCCR2B  &= ~(0x07 <<CS20);
      TIMSK2  &= ~(1 << OCIE2A);
      TIFR2   = (1 << OCF2A); 
  }
#endif
    #if USE_AVR_GPT3 || defined(__DOXYGEN__)
  if(gptp == &GPTD3)
  {
      TCCR3B  &= ~(0x07 <<CS10);
      TIMSK3  &= ~(1 << OCIE1A);
      TIFR3   = (1 << OCF1A); 
  }
#endif
    #if USE_AVR_GPT4 || defined(__DOXYGEN__)
  if(gptp == &GPTD4)
  {
      TCCR4B  &= ~(0x07 <<CS10);
      TIMSK4  &= ~(1 << OCIE1A);
      TIFR4   = (1 << OCF1A); 
  }
#endif
    #if USE_AVR_GPT5 || defined(__DOXYGEN__)
  if(gptp == &GPTD5)
  {
      TCCR5B  &= ~(0x07 <<CS10);
      TIMSK5  &= ~(1 << OCIE1A);
      TIFR5   = (1 << OCF1A); 
  }
#endif
  /*disable timer
   * clear interrupt*/
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {
  gptp->callback = gpt_lld_dummy_callback;
  gpt_lld_start_timer(gptp,interval);
  while(gptp->state != GPT_READY)
     ;
  return;
}

#endif /* HAL_USE_GPT */

/** @} */
