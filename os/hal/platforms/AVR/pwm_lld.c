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
 * @file    templates/pwm_lld.c
 * @brief   PWM Driver subsystem low level driver source template.
 *
 * @addtogroup PWM
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
/** @brief PWM driver identifiers.*/
#if USE_AVR_PWM1 || defined(__DOXYGEN__)
PWMDriver PWMD1;
#endif
#if USE_AVR_PWM2 || defined(__DOXYGEN__)
PWMDriver PWMD2;
#endif
/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
static void pwm_configure_hw_channel(volatile uint8_t * TCCRnA, uint8_t COMnx1,uint8_t COMnx0, pwmmode_t mode)
{
	  *TCCRnA &= ~((1<<COMnx1) | (1<<COMnx0)); 
	  if(PWM_OUTPUT_ACTIVE_HIGH ==mode )
	    *TCCRnA |=  ((1<<COMnx1) | (0<<COMnx0)); //non inverting mode
	  if(PWM_OUTPUT_ACTIVE_LOW ==mode)
	    *TCCRnA |= (1<<COMnx1) | (1<<COMnx0); //inverting mode 
}
/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*
 * interrupt for compare1&2 and clock overflow. pwmd1 & pwmd2
 * 
 * 
 */

CH_IRQ_HANDLER(TIMER1_OVF_vect) {
    CH_IRQ_PROLOGUE();
      PWMD1.config->callback(&PWMD1);
    CH_IRQ_EPILOGUE();
}
CH_IRQ_HANDLER(TIMER1_COMPA_vect) {
    CH_IRQ_PROLOGUE();
      PWMD1.config->channels[0].callback(&PWMD1);
    CH_IRQ_EPILOGUE();
}
CH_IRQ_HANDLER(TIMER1_COMPB_vect) {
    CH_IRQ_PROLOGUE();
      PWMD1.config->channels[1].callback(&PWMD1);
    CH_IRQ_EPILOGUE();
}
CH_IRQ_HANDLER(TIMER2_OVF_vect) {
    CH_IRQ_PROLOGUE();
      PWMD2.config->callback(&PWMD2);
    CH_IRQ_EPILOGUE();
}
CH_IRQ_HANDLER(TIMER2_COMPA_vect) {
    CH_IRQ_PROLOGUE();
      PWMD2.config->channels[0].callback(&PWMD2);
    CH_IRQ_EPILOGUE();
}
CH_IRQ_HANDLER(TIMER2_COMPB_vect) {
    CH_IRQ_PROLOGUE();
      PWMD2.config->channels[1].callback(&PWMD2);
    CH_IRQ_EPILOGUE();
}



/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {

#if USE_AVR_PWM1 || defined(__DOXYGEN__)
	pwmObjectInit(&PWMD1);
	TCCR1A = (0<<WGM11) | (1<<WGM10);   //fast pwm 8 bit  
	TCCR1B = (1<<WGM12) | (0<<WGM13);  //fast pwm 8 bit
#endif
  #if USE_AVR_PWM2 || defined(__DOXYGEN__)
	pwmObjectInit(&PWMD2);
	TCCR2A = (1<<WGM21) | (1<<WGM20);   //fast pwm 8 bit
	TCCR2B = (0<<WGM22);  //fast pwm 8 bit
  #endif
  
  
}

/**
 * @brief   Configures and activates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmp) {
  
  if ( pwmp->state == PWM_STOP) {
    /* Clock activation.*/
    
    
#if USE_AVR_PWM1 || defined(__DOXYGEN__)
      if(pwmp == &PWMD1)
      {//TODO frequency
	  TCCR1B |= (1<<CS12) |(0<<CS11) | (1<<CS10); //parti col no prescaling
	  if(pwmp->config->callback != NULL)
	    TIMSK1 = (1<<TOIE1);
      }
#endif
      
#if USE_AVR_PWM2 || defined(__DOXYGEN__)
      if(pwmp == &PWMD2)
      {
	      TCCR2B |= (0<<CS22) |(0<<CS21) | (1<<CS20); //parti col no prescaling
	      if(pwmp->config->callback != NULL)
		TIMSK2 = (1<<TOIE2);
      }
#endif
  }
  /* Configuration.*/




}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp) {
	#if USE_AVR_PWM1 || defined(__DOXYGEN__)
	if(pwmp == &PWMD1)
	{
	  TCCR1B &= ~((1<<CS12) |(1<<CS11) | (1<<CS10)); //parti col no prescaling
	  TIMSK1 = 0;
	}
#endif
	#if USE_AVR_PWM2 || defined(__DOXYGEN__)
	if(pwmp == &PWMD2)
	{
	    TCCR2B &= ~((1<<CS22) |(1<<CS21) | (1<<CS20)); //parti col no prescaling
	    TIMSK2 = 0;
	}
#endif
}

/**
 * @brief   Changes the period the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    The function has effect at the next cycle start.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] period    new cycle time in ticks
 *
 * @notapi
 */
void pwm_lld_change_period(PWMDriver *pwmp, pwmcnt_t period) {

}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    Depending on the hardware implementation this function has
 *          effect starting on the next cycle (recommended implementation)
 *          or immediately (fallback implementation).
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmp,
                            pwmchannel_t channel,
                            pwmcnt_t width) {
  uint32_t val = width;
  
  val *= 256;
  val /= (uint32_t)pwmp->period;
  if(val > 0x00FF)
    val = 0xFF;
  

  #if USE_AVR_PWM1 || defined(__DOXYGEN__)
  if(pwmp == &PWMD1)
      {   
	pwm_configure_hw_channel(&TCCR1A,7-2*channel,6-2*channel,pwmp->config->channels[channel].mode);
	TIMSK1 |= (1<< (channel + 1));
	if(pwmp->config->channels[channel].callback != NULL)
	switch(channel){
	      case 0: OCR1A = val;break;
	      case 1: OCR1B = val;break;
	    #if PWM_CHANNELS>2
	      case 2: OCR1C = val;break;
	    #endif	  
	}
      }
  
  #endif
  #if USE_AVR_PWM2 || defined(__DOXYGEN__)
  if(pwmp == &PWMD2)
      {
	pwm_configure_hw_channel(&TCCR2A,7-2*channel,6-2*channel,pwmp->config->channels[channel].mode);
	TIMSK2 |= (1<< (channel + 1));
	if(pwmp->config->channels[channel].callback != NULL)
	switch(channel){
	      case 0: OCR2A = val;break;
	      case 1: OCR2B = val;break;	  
	}
      }
  
#endif
  

}

/**
 * @brief   Disables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    Depending on the hardware implementation this function has
 *          effect starting on the next cycle (recommended implementation)
 *          or immediately (fallback implementation).
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel) {
#if USE_AVR_PWM1 || defined(__DOXYGEN__)
  if(pwmp == &PWMD1)
      {   
	pwm_configure_hw_channel(&TCCR1A,7-2*channel,6-2*channel,PWM_OUTPUT_DISABLED);
	TIMSK1 &= ~(1<< (channel + 1));
      }
#endif
#if USE_AVR_PWM2 || defined(__DOXYGEN__)
  if(pwmp == &PWMD2)
      {   
	pwm_configure_hw_channel(&TCCR2A,7-2*channel,6-2*channel,PWM_OUTPUT_DISABLED);
	TIMSK2 &= ~(1<< (channel + 1));
      }
#endif
}

#endif /* HAL_USE_PWM */

/** @} */
