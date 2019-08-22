#include "main.h"

void RCC_Init(void)
{
    // Enable HSE
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);

    // Ready start HSE
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    // Clock Flash memory and enable latency
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // Clear PLLSRC & PLLMUL bits
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;
    RCC->CFGR &= ~RCC_CFGR_PLLMUL;

    // Setup
    // source HSE
    RCC->CFGR |= RCC_CFGR_PLLSRC;
    // source HSE/1 = 8 MHz
    RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;
    // PLL x6: clock = 8 MHz * 6 = 48 MHz
    RCC->CFGR |= RCC_CFGR_PLLMUL6;

    // AHB = SYSCLK / 1
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    // APB = HCLK / 1
    RCC->CFGR |= RCC_CFGR_PPRE_DIV2;

    // enable PLL and wait till PLL is ready
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    // clear SW bits and select source SYSCLK = PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    // wait till PLL is used
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1)
    {
    }
}

/**
 *  Configure PWM for cock hand 1
 */
void ConfigureTIM1PWM(void)
{
    /* (1) Enable the peripheral clock of Timer x */
    /* (2) Enable the peripheral clock of GPIOA */
    /* (3) Select alternate function mode on GPIOA pin 8, 9, 10, 11 */
    /* (4) Select AF2 on PA{8:11} in AFRH for TIM1_CH{1:4} */

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;                                             /* (1) */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                              /* (2) */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1);   /* (3), PA8 */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9)) | (GPIO_MODER_MODER9_1);   /* (3), PA9 */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER10)) | (GPIO_MODER_MODER10_1); /* (3), PA10 */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER11)) | (GPIO_MODER_MODER11_1); /* (3), PA11 */

    GPIOA->AFR[1] |= 0x02 << GPIO_AFRH_AFRH0_Pos; /* (4), PA8 -> AF2 */
    GPIOA->AFR[1] |= 0x02 << GPIO_AFRH_AFRH1_Pos; /* (4), PA9 -> AF2 */
    GPIOA->AFR[1] |= 0x02 << GPIO_AFRH_AFRH2_Pos; /* (4), PA10 -> AF2 */
    GPIOA->AFR[1] |= 0x02 << GPIO_AFRH_AFRH3_Pos; /* (4), PA11 -> AF2 */

    /* (1) Set prescaler to 1, so APBCLK/2 = 24MHz */
    /* (2) Set ARR = 1000, as timer clock is 24MHz PWM frequency will be 24kHz */
    /* (3) Set initial value of CCRx = 0 */
    /* (4) Select PWM mode 1 on CH1 and CH3 (OC1M, OC3M = 110) and mode 2 on CH2 and CH4  (OC2M, OC4M = 110),
         enable preload registers (OC1PE, OC2PE, OC3PE, OC4PE) */
    /* (5) Active high polarity (CC{1:4}P = 0, reset value),
         enable the output on OC1 (CC{1:4}E = 1)*/
    /* (6) Enable output (MOE = 1)*/
    /* (7) Enable counter (CEN = 1)
         select edge aligned mode (CMS = 00, reset value)
         select direction as upcounter (DIR = 0, reset value) */
    /* (8) Force update generation (UG = 1) */

    TIM1->PSC = 2 - 1; /* (1) */
    TIM1->ARR = 1000;  /* (2) */

    TIM1->CCR1 = 500; /* (3) */
    TIM1->CCR2 = 500; /* (3) */
    TIM1->CCR3 = 500; /* (3) */
    TIM1->CCR4 = 500; /* (3) */

    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;                    /* (4), CH1 -> PWM mode 1 */
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2PE; /* (4), CH2 -> PWM mode 2 */
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;                    /* (4), CH3 -> PWM mode 1 */
    TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4PE; /* (4), CH4 -> PWM mode 2 */

    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; /* (5) */
    TIM1->BDTR |= TIM_BDTR_MOE;                                                  /* (6) */

    TIM1->CR1 |= TIM_CR1_CEN; /* (7) */
    TIM1->EGR |= TIM_EGR_UG;  /* (8) */

    // Enable interruption
    TIM1->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

/**
 *  Configure PWM for cock hand 2
 */
void ConfigureTIM3PWM(void)
{
    /* (1) Enable the peripheral clock of Timer x */
    /* (2) Enable the peripheral clock of GPIOA and GPIOB */
    /* (3) Select alternate function mode on GPIOA pin 8, 9, 10, 11 */
    /* (4) Select AF1 on PA{6,7} and PB{0,1} in AFRL for TIM3_CH{1:4} */

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                                           /* (1) */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                            /* (2) */
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                                            /* (2) */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER6)) | (GPIO_MODER_MODER6_1); /* (3), PA6 */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER7)) | (GPIO_MODER_MODER7_1); /* (3), PA7 */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER0)) | (GPIO_MODER_MODER0_1); /* (3), PB0 */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER1)) | (GPIO_MODER_MODER1_1); /* (3), PB1 */

    GPIOA->AFR[0] |= 0x01 << GPIO_AFRL_AFRL6_Pos; /* (4), PA6 -> AF1 */
    GPIOA->AFR[0] |= 0x01 << GPIO_AFRL_AFRL7_Pos; /* (4), PA7 -> AF1 */
    GPIOB->AFR[0] |= 0x01 << GPIO_AFRL_AFRL0_Pos; /* (4), PB0 -> AF1 */
    GPIOB->AFR[0] |= 0x01 << GPIO_AFRL_AFRL1_Pos; /* (4), PB1 -> AF1 */

    /* (1) Set prescaler to 1, so APBCLK/2 = 24MHz */
    /* (2) Set ARR = 1000, as timer clock is 24MHz PWM frequency will be 24kHz */
    /* (3) Set initial value of CCRx = 0 */
    /* (4) Select PWM mode 1 on CH1 and CH3 (OC1M, OC3M = 110) and mode 2 on CH2 and CH4  (OC2M, OC4M = 110),
         enable preload registers (OC1PE, OC2PE, OC3PE, OC4PE) */
    /* (5) Active high polarity (CC{1:4}P = 0, reset value),
         enable the output on OC1 (CC{1:4}E = 1)*/
    /* (6) Enable output (MOE = 1)*/
    /* (7) Enable counter (CEN = 1)
         select edge aligned mode (CMS = 00, reset value)
         select direction as upcounter (DIR = 0, reset value) */
    /* (8) Force update generation (UG = 1) */

    TIM3->PSC = 2 - 1; /* (1) */
    TIM3->ARR = 1000;  /* (2) */

    TIM3->CCR1 = 250; /* (3) */
    TIM3->CCR2 = 250; /* (3) */
    TIM3->CCR3 = 250; /* (3) */
    TIM3->CCR4 = 250; /* (3) */

    TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;                    /* (4), CH1 -> PWM mode 1 */
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2PE; /* (4), CH2 -> PWM mode 2 */
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;                    /* (4), CH3 -> PWM mode 1 */
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4PE; /* (4), CH4 -> PWM mode 2 */

    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; /* (5) */
    TIM3->BDTR |= TIM_BDTR_MOE;                                                  /* (6) */

    TIM3->CR1 |= TIM_CR1_CEN; /* (7) */
    TIM3->EGR |= TIM_EGR_UG;  /* (8) */

    // Enable interruption
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
}

void ConfigureGPIO(void)
{
    // LED PC14
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER14)) | (GPIO_MODER_MODER14_0);

    // MotorDrive enable pins PA5, PB15
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER5)) | (GPIO_MODER_MODER5_0);
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER15)) | (GPIO_MODER_MODER15_0);

    // Hall sensor pins PB8, PB9: input pull-up
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;
}
