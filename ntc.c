#include "ntc.h"
#include "ntc_cfg.h"

#define R1      5100    // ohm

uint8_t get_temper(uint16_t adc_val, int16_t *t_out)
{
  uint8_t i;
  
  if (adc_val > NTC_ADC[R_CNT-1] || adc_val < NTC_ADC[0]) 
  {
    *t_out = 0;
    return 1;   // out of range measure или обрыв датчика (R2 = 0)
  }
  
  // find in table
  for (i = 0; i < (R_CNT - 1); i++)
  {
    if (adc_val < NTC_ADC[i+1] && adc_val >= NTC_ADC[i]) break;
  }
  
  double x1 = NTC_ADC[i];
  double x2 = NTC_ADC[i+1];
  double y1 = NTC_T[i];
  double y2 = NTC_T[i+1];
  
  double temper = (((adc_val-x1)*(y1-y2))/(x1-x2)) + y1;
  
  // 21.3 C = 213 C
  *t_out = (int16_t)(temper * 10);
    
  return 0;
}