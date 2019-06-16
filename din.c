#include "stm8s.h"
#include "din.h"
#include "PetitModbus.h"

#define IN1 (((GPIOC->IDR) >> 3) & 0x01)
#define IN2 (((GPIOC->IDR) >> 5) & 0x01)
#define IN3 (((GPIOC->IDR) >> 6) & 0x01)
#define IN4 (((GPIOC->IDR) >> 7) & 0x01)

uint8_t din1, din2, din3, din4 = 0x00;
uint8_t tmp = 0x0000;

uint8_t cycles = 0;

void din_proc()
{
  din1 = (din1<<1) | IN1;
  din2 = (din2<<1) | IN2;
  din3 = (din3<<1) | IN3;
  din4 = (din4<<1) | IN4;
  cycles++;
  
  if (cycles == 8)
  {
    if (din1 == 0x00) tmp &= ~0x01;
    else if (din1 == 0xff) tmp |= 0x01;
    
    if (din2 == 0x00) tmp &= ~0x02;
    else if (din2 == 0xff) tmp |= 0x02;
    
    if (din3 == 0x00) tmp &= ~0x04;
    else if (din3 == 0xff) tmp |= 0x04;
    
    if (din4 == 0x00) tmp &= ~0x08;
    else if (din4 == 0xff) tmp |= 0x08;
    
    // if register changed
    if (PetitRegisters[0].ActValue != (uint16_t)tmp)
    {
      PetitRegisters[0].ActValue = (uint16_t)tmp;
      tmp = 0x00;
    }
    
    cycles = 0;
  }
  
}