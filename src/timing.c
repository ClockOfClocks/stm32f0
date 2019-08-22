#include "main.h"

volatile uint32_t Millis = 0;

void SysTick_Handler(void)
{
    Millis++;
}

uint32_t millis(void)
{
    return Millis;
}

void delay_ms(uint32_t nTime)
{
    uint32_t curTime = Millis;
    while (nTime - (Millis - curTime) > 0)
        ;
}