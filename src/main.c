#include "main.h"

void _init(void) {}

// Test TIM interruption
uint32_t cc = 0;
bool state = true;

int main()
{
  SysTick_Config(48000); /* 1ms config */
  RCC_Init();
  ConfigureTIM1PWM();
  ConfigureTIM3PWM();
  ConfigureGPIO();

  while (1)
  {
    // put your main code here, to run repeatedly:

    // GPIOC->BSRR = GPIO_BSRR_BR_14;
    // delay_ms(1000);
    // GPIOC->BSRR = GPIO_BSRR_BS_14;
    // delay_ms(2000);
  }
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  TIM1->SR &= ~TIM_SR_UIF; // clear interruption flag

  // if(++cc == 24000){
  //   if(state == true){
  //     GPIOC->BSRR = GPIO_BSRR_BR_14;
  //     state = false;
  //   }else{
  //     GPIOC->BSRR = GPIO_BSRR_BS_14;
  //     state = true;
  //   }
  //   cc=0;
  // }
}

void TIM3_IRQHandler(void)
{
  TIM3->SR &= ~TIM_SR_UIF; // clear interruption flag

  if (++cc == 12000)
  {
    if (state == true)
    {
      GPIOC->BSRR = GPIO_BSRR_BR_14;
      state = false;
    }
    else
    {
      GPIOC->BSRR = GPIO_BSRR_BS_14;
      state = true;
    }
    cc = 0;
  }
}