/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "PetitModbus.h"
#include "ntc.h"
//#include "i2c.h"

#define MB_ADDRESS      17

/* Registers
0 - DIN
1 - Temperature (S1)
2 - Temperature (S2)
3 - AIN

*/


/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void TIM1_Config(void);
static void TIM2_Config(void);

//uint8_t din_register = 0;
uint16_t adc_buffer[3];
uint8_t adc_ready = 0;

void main(void)
{
  int16_t value = 0;
  
  // Clock init
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); 
  
  // GPIO init
  GPIO_DeInit(GPIOD);
  GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOC);

  // uart
  GPIO_Init(GPIOD, (GPIO_Pin_TypeDef)(GPIO_PIN_5 | GPIO_PIN_6), GPIO_MODE_OUT_PP_LOW_FAST); // rx, tx
  GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST); // rw
  
  // din
  GPIO_Init(GPIOC, (GPIO_Pin_TypeDef)(GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7), GPIO_MODE_OUT_PP_LOW_FAST);
  
  // ain
  GPIO_Init(GPIOD, (GPIO_Pin_TypeDef)(GPIO_PIN_2 | GPIO_PIN_3), GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
  
  // TIM2 init (for ADC)
  TIM2_Config();
  
  // TIM1 init (for di)
  TIM1_Config();
  
  // ADC init
  ADC1_DeInit();
  ADC1_ScanModeCmd(ENABLE);
  ADC1_DataBufferCmd(ENABLE);
  
  ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
  
  ADC1_ClearITPendingBit(ADC1_IT_EOC);
   
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
      (ADC1_Channel_TypeDef)(ADC1_CHANNEL_2 | ADC1_CHANNEL_3 | ADC1_CHANNEL_4), // channels
      ADC1_PRESSEL_FCPU_D8,
      ADC1_EXTTRIG_TIM, DISABLE, 
      ADC1_ALIGN_RIGHT, 
      ADC1_SCHMITTTRIG_ALL, DISABLE);

  
  // Modbus init
  InitPetitModbus(MB_ADDRESS);
  
  enableInterrupts();
  
  //ADC1_StartConversion();
  
  /* Infinite loop */
  while (1)
  {
    ProcessPetitModbus();
    
    // Get AIN
    if (adc_ready)
    {
      if (get_temper(adc_buffer[1], &value) == 0) PetitRegisters[1].ActValue = value;
      if (get_temper(adc_buffer[2], &value) == 0) PetitRegisters[2].ActValue = value;
      
      PetitRegisters[3].ActValue = adc_buffer[0];
      adc_ready = 0;
    }
  }
  
}

static void TIM1_Config(void)
{
  // Time base configuration
  TIM1_TimeBaseInit(16000, TIM1_COUNTERMODE_UP, 60-1, 0); // 60 ms
  // Clear TIM1 update flag
  TIM1_ClearFlag(TIM1_FLAG_UPDATE);
  // Enable update interrupt
  TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
  
  // Enable TIM
  TIM1_Cmd(ENABLE);
}

// ~500ms timer
static void TIM2_Config(void)
{
    TIM2_TimeBaseInit(TIM2_PRESCALER_32768, 244-1);   //~ 500 ms
    TIM2_ARRPreloadConfig(ENABLE);
    TIM2_ITConfig(TIM2_IT_UPDATE , ENABLE);
    TIM2_Cmd(ENABLE);
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
