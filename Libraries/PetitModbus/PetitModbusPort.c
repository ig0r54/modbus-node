#include "stm8s.h"
#include "PetitModbus.h"
#include "PetitModbusPort.h"

// This port file for STM8S microcontrollers!

// Modbus RTU Variables
volatile unsigned char   PetitReceiveBuffer[PETITMODBUS_RECEIVE_BUFFER_SIZE];   // Buffer to collect data from hardware
volatile unsigned char   PetitReceiveCounter=0;                                 // Collected data number

// UART Initialize for Microconrollers, yes you can use another phsycal layer!
void PetitModBus_UART_Initialise(void)
{
    UART1_DeInit();
    UART1_Init((u32)9600, \
               UART1_WORDLENGTH_8D, \
               UART1_STOPBITS_1, \
               UART1_PARITY_NO , \
               UART1_SYNCMODE_CLOCK_DISABLE , \
               UART1_MODE_TXRX_ENABLE);
               UART1_ITConfig(UART1_IT_RXNE_OR,ENABLE);
    UART1_Cmd(ENABLE ); 
}

// Timer Initialize for Petit Modbus, 1ms Timer will be good for us!
void PetitModBus_TIMER_Initialise(void)
{
  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 125-1); // 1 ms
  
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}

// This is used for send one character
/*
void PetitModBus_UART_Putch(unsigned char c)
{
    // Loop until the end of transmission
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET); // wait while tx buffer will be empty
    UART1_SendData8(c);
}
*/

// This is used for send string, better to use DMA for it ;)
unsigned char PetitModBus_UART_String(unsigned char *s, unsigned int Length)
{
    unsigned short  DummyCounter;
    
    //Set Max485 in Transmitter mode
    GPIO_WriteHigh(GPIOA, GPIO_PIN_3);

    for(DummyCounter=0; DummyCounter<Length; DummyCounter++)
    {
        UART1_SendData8((uint8_t)(s[DummyCounter]));
	while ((UART1->SR & UART1_SR_TXE ) != UART1_SR_TXE );

        //PetitModBus_UART_Putch(s[DummyCounter]);
    }
    
    //Set Max485 in Receiver mode
    GPIO_WriteLow(GPIOA, GPIO_PIN_3);
    
    return TRUE;
}

/*************************Interrupt Fonction Slave*****************************/
// Call this function into your UART Interrupt. Collect data from it!
// Better to use DMA
void ReceiveInterrupt(unsigned char Data)
{
    PetitReceiveBuffer[PetitReceiveCounter] = Data;
    PetitReceiveCounter++;

    if(PetitReceiveCounter>PETITMODBUS_RECEIVE_BUFFER_SIZE)
        PetitReceiveCounter=0;

    PetitModbusTimerValue=0;
}

// Call this function into 1ms Interrupt or Event!
void PetitModBus_TimerValues(void)
{
    PetitModbusTimerValue++;
}
/******************************************************************************/
