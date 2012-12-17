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

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*
 * interrupt for compare1&2 and clock overflow. pwmd1 & pwmd2
 * 
 * 
 */




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
  TCCR1A = (0<<WGM11) | (1<<WGM10);   //fast pwm 8 bit
	    //(1<<COM1A1) | (0<<COM1A0); //non inverting mode
  
  TCCR1B = (1<<WGM12) | (0<<WGM13);  //fast pwm 8 bit
#endif
  #if USE_AVR_PWM1 || defined(__DOXYGEN__)
    TCCR2A = (1<<WGM21) | (1<<WGM20);   //fast pwm 8 bit
	    //(1<<COM1A1) | (0<<COM1A0); //non inverting mode
  
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
  
  if (pwmp->state == PWM_STOP) {
    /* Clock activation.*/
#if USE_AVR_PWM1 || defined(__DOXYGEN__)
      if(pwmp == &PWMD1)
      {
	  TCCR1B |= (0<<CS12) |(0<<CS11) | (1<<CS10); //parti col no prescaling
      }
#endif
      
#if USE_AVR_PWM2 || defined(__DOXYGEN__)
      if(pwmp == &PWMD2)
      {
	      TCCR2B |= (0<<CS22) |(0<<CS21) | (1<<CS20); //parti col no prescaling
      }
#endif
  }
  /* Configuration.*/
  
  
  for(pwmchannel_t currentChannel=0; currentChannel <PWM_CHANNELS;currentChannel++)
  {
      switch(pwmp->config->channels[currentChannel].mode)
      {
	
	
      }
    
    
    
  }
  
  #if USE_AVR_PWM1 || defined(__DOXYGEN__)
  if(pwmp == &PWMD1)
      {
      TCCR1B |= (0<<CS12) |(0<<CS11) | (1<<CS10); //parti col no prescaling
      }
#endif
      
#if USE_AVR_PWM2 || defined(__DOXYGEN__)
      if(pwmp == &PWMD2)
      {
      TCCR2B |= (0<<CS22) |(0<<CS21) | (1<<CS20); //parti col no prescaling
      }
#endif
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
	}
#endif
	#if USE_AVR_PWM2 || defined(__DOXYGEN__)
	if(pwmp == &PWMD2)
	{
	    TCCR1B &= ~((1<<CS22) |(1<<CS21) | (1<<CS20)); //parti col no prescaling
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
  #if USE_AVR_PWM1 || defined(__DOXYGEN__)
  if(pwmp == &PWMD1)
      {
	
      }
  
  #endif
  #if USE_AVR_PWM2 || defined(__DOXYGEN__)
  if(pwmp == &PWMD2)
      {
	
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

}

#endif /* HAL_USE_PWM */

/** @} */
