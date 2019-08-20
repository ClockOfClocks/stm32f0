#include "main.h"

void _init(void){} 

int main() {
    SysTick_Config(48000); /* 1ms config */

    RCC->CR |= ((uint32_t)RCC_CR_HSEON); 												// Enable HSE
    while (!(RCC->CR & RCC_CR_HSERDY));													// Ready start HSE		

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;					// Cloclk Flash memory

    // Clear
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;															// clearn PLLSRC bits
    RCC->CFGR &= ~RCC_CFGR_PLLMUL;               							  // clear PLLMULL bits

    // Setup
    RCC->CFGR |= RCC_CFGR_PLLSRC; 											        // source HSE
    RCC->CFGR &= ~RCC_CFGR_PLLXTPRE; 								            // source HSE/1 = 8 MHz
    RCC->CFGR |= RCC_CFGR_PLLMUL6; 														  // PLL x6: clock = 8 MHz * 6 = 48 MHz

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;														// AHB = SYSCLK/1
    RCC->CFGR |= RCC_CFGR_PPRE_DIV2;														// APB1 = HCLK/2 = 48/2 = 24MHz
    RCC->CFGR |= RCC_CFGR_PPRE_DIV1;														// APB2 = HCLK/1

    RCC->CR |= RCC_CR_PLLON;                      							// enable PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0) {}      							// wait till PLL is ready

    RCC->CFGR &= ~RCC_CFGR_SW;                   							 	// clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_PLL;                 							// select source SYSCLK = PLL
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1) {} 			// wait till PLL is used	
    
    // Setup led
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER14)) | (GPIO_MODER_MODER14_0);

  while(1) {
    // put your main code here, to run repeatedly:
    GPIOC->BSRR = GPIO_BSRR_BR_14;
    delay_ms(1000);
    GPIOC->BSRR = GPIO_BSRR_BS_14;
    delay_ms(2000);
  }
}