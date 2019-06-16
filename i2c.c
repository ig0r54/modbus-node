#include "stm8s.h"
#include "i2c.h"

// table to store data to be sent
u8 dummy_Tab[MAX_DUMMY]= { 0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90};

// table used to store received data
u8 dummy[MAX_DUMMY];

// Var initialised with u8_NumByte at every add ack to know the initial number of byte to be read or write durring comm
u8 NumByteToRW;


/******************************************************************************
* Function name : main
* Description 	: Main testing loop
* Input param 	: None
* Return 		    : None
* See also 		  : None
*******************************************************************************/
void i2c_init(void) 
{ 
	
  // - System CLK configuration 
  #ifdef FAST_I2C_MODE
    CLK->CKDIVR = 0x00;             // sys clock / 1
  #else
    CLK->CKDIVR = 0x01;             // sys clock / 2
  #endif
 
 // - GPIO initialisation for LED
  GPIOA->DDR |=  0xF0;            // GPIO init
  GPIOA->CR1 |=  0xF0;            // all LEDs push pull outputs
  GPIOA->ODR |=  0xF0;
  
  
  // initialize timer 4 mandatory for I2C timout measurement 
  tim4_hardware_setup();                    
  
  // Initialize I2C for communication
  i2c_hardware_setup();                     
	
  // Enable all interrupt
  enableInterrupts();

  /* main loop */
  while (!I2C_WriteRegister(0x51,SEV_BIT_ADDRESS,STOP,0x05, &dummy_Tab[0]));
  while(1) 
  {
    while (!I2C_WriteRegister(0x51,SEV_BIT_ADDRESS,NOSTOP,0x01, &dummy_Tab[0]));
    while (!I2C_ReadRegister(0x51,SEV_BIT_ADDRESS,NOSTOP,0x03, &dummy[0]));
    
    while (!I2C_WriteRegister(0x51,SEV_BIT_ADDRESS,NOSTOP,0x01, &dummy_Tab[0]));
    while (!I2C_ReadRegister(0x51,SEV_BIT_ADDRESS,NOSTOP,0x02, &dummy[0]));
    
    while (!I2C_WriteRegister(0x51,SEV_BIT_ADDRESS,NOSTOP,0x01, &dummy_Tab[0]));
    while (!I2C_ReadRegister(0x51,SEV_BIT_ADDRESS,NOSTOP,0x01, &dummy[0]));
  }

}

  u8  STATE;                    // curent I2C states machine state
  volatile u8 err_state;  	// error state 
  volatile u8 err_save;   	// I2C->SR2 copy in case of error
  volatile u16 TIM4_tout;       // Timout Value  
 
  u8  u8_regAddr ;					
  u8  u8_Direction;
 
  u8  u8_NumByte_cpy ; 
  u8* pu8_DataBuffer_cpy;
  u16 u16_SlaveAdd_cpy;
  u8  u8_AddType_cpy;
  u8  u8_NoStop_cpy;

/******************************************************************************
* Function name : I2C_Init
* Description 	: Initialize I2C peripheral
* Input param 	: None
* Return 		    : None
* See also 		  : None
*******************************************************************************/
void i2c_hardware_setup(void) 
{
  //define SDA, SCL outputs, HiZ, Open drain, Fast
  GPIOE->ODR |= 0x06;               
  GPIOE->DDR |= 0x06;
  GPIOE->CR2 |= 0x06;

#ifdef FAST_I2C_MODE
  I2C->FREQR = 16;               // input clock to I2C - 16MHz 
  I2C->CCRL = 15;                // 900/62.5= 15, (SCLhi must be at least 600+300=900ns!)
  I2C->CCRH = 0x80;              // fast mode, duty 2/1 (bus speed 62.5*3*15~356kHz)
  I2C->TRISER = 5;               // 300/62.5 + 1= 5  (maximum 300ns)
#else
  I2C->FREQR = 8;                // input clock to I2C - 8MHz
  I2C->CCRL = 40;                // CCR= 40 - (SCLhi must be at least 4000+1000=5000ns!)
  I2C->CCRH = 0;                 // standard mode, duty 1/1 bus speed 100kHz
  I2C->TRISER = 9;               // 1000ns/(125ns) + 1  (maximum 1000ns)
#endif
  I2C->OARL = 0xA0;              // own address A0;
  I2C->OARH |= 0x40;
  I2C->ITR = 3;                  // enable Event & error interrupts 
  I2C->CR2 |= 0x04;              // ACK=1, Ack enable
  I2C->CR1 |= 0x01;              // PE=1
  I2C->CR1 &= ~0x80;             // Stretch enable
	
  // Initialise I2C State Machine
  err_save= 0;
  STATE = INI_00;
  set_tout_ms(0);
  
}

/******************************************************************************
* Function name : ErrProc
* Description 	: Managed Error durring I2C communication to be modified depending of final application
* Input param 	: None
* Return 		    : None
* See also 		  : None.
*******************************************************************************/
void ErrProc (void)
{
      err_save = I2C->SR2 ;
      err_state = STATE;
      I2C->SR2= 0;
      STATE = INI_00;
      set_tout_ms(0);
}

/******************************************************************************
* Function name : I2C_WriteRegister
* Description 	: write defined number bytes to slave memory starting with defined offset
* Input param 	: Slave Address ; Address type (TEN_BIT_ADDRESS or SEV_BIT_ADDRESS) ; STOP/NOSTOP ;
*									Number byte to Write ; address of the application send buffer
* Return 		    : 0 : START Writing not performed -> Communication onging on the bus
*                 1 : START Writing performed 
* See also 		  : None.
*******************************************************************************/
u8 I2C_WriteRegister(u16 u16_SlaveAdd, u8 u8_AddType, u8 u8_NoStop, u8 u8_NumByteToWrite, u8 *pu8_DataBuffer) 
{
      // check if communication on going
      if ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
              return 0 ;
      // check if STATE MACHINE is in state INI_00
      if (STATE != INI_00)
              return 0 ;
      // set ACK
      I2C->CR2 |= I2C_CR2_ACK;
      // reset POS
      I2C->CR2 &= ~I2C_CR2_POS;
      // setup I2C comm. in write
      u8_Direction = WRITE;
      // copy parametters for interrupt routines
      u8_NoStop_cpy = u8_NoStop;
      u16_SlaveAdd_cpy = u16_SlaveAdd;
      u8_NumByte_cpy = u8_NumByteToWrite; 
      pu8_DataBuffer_cpy  = pu8_DataBuffer;
      u8_AddType_cpy = u8_AddType;
      // set comunication Timeout
      set_tout_ms(I2C_TOUT);
      // generate Start
      I2C->CR2 |= I2C_CR2_START;
      STATE = SB_01;
      return 1;
}

/******************************************************************************
* Function name : I2C_ReadRegister
* Description 	: Read defined number bytes from slave memory starting with defined offset
* Input param 	: Slave Address ; Address type (TEN_BIT_ADDRESS or SEV_BIT_ADDRESS) ; STOP/NOSTOP ;
*									Number byte to Read ; address of the application receive buffer
* Return 		    : 0 : START Reading not performed -> Communication onging on the bus
*                 1 : START Reading performed 
* See also 		  : None
*******************************************************************************/
u8 I2C_ReadRegister(u16 u16_SlaveAdd, u8 u8_AddType, u8 u8_NoStop, u8 u8_NumByteToRead, u8 *u8_DataBuffer) 
{
    // check if communication on going
    if (((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY) && (u8_NoStop == 0))
            return 0 ;
    // check if STATE MACHINE is in state INI_00
    if (STATE != INI_00)
            return 0 ;
    // set ACK
    I2C->CR2 |= I2C_CR2_ACK;
    // reset POS
    I2C->CR2 &= ~I2C_CR2_POS;
    // setup I2C comm. in Read
    u8_Direction = READ;
    // copy parametters for interrupt routines
    u8_NoStop_cpy = u8_NoStop;
    u8_AddType_cpy = u8_AddType;
    u16_SlaveAdd_cpy = u16_SlaveAdd;
    u8_NumByte_cpy = u8_NumByteToRead; 
    pu8_DataBuffer_cpy = u8_DataBuffer;
    // set comunication Timeout
    set_tout_ms(I2C_TOUT);
    //generate Start
     I2C->CR2 |= 1;
     STATE = SB_11;
     I2C->ITR |= 3;                  // re-enable interrupt
    return 1;
}

void tim4_hardware_setup(void)
{
    CLK->PCKENR1 |= 4;               // TIM4 clock enable
    
    TIM4->ARR = 0x80;                // init timer4 1ms interrupts
    TIM4->PSCR= 7;
    TIM4->IER = 1;
    TIM4->CR1 |= 1;
}





