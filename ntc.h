#ifndef __NTC_H
#define __NTC_H

#include "stm8s.h"

uint8_t get_temper(uint16_t adc_val, int16_t *t);

#endif

