
#include "ch.h"
#include "hal.h"

#ifndef _ATMEGA_TIMERS_H_
#define _ATMEGA_TIMERS_H_
uint8_t findBestPrescaler(uint16_t frequency, uint16_t *ratio ,uint8_t *clock_source,uint8_t n);


#endif