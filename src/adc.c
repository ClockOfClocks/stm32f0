#include "main.h"

volatile uint16_t adc_error = 0; //initialized at 0 and modified by the functions

/**
  * @brief  Main ADC function.
  * @param  None
  * @retval None
  */
void initADC(void)
{
    SetClockForADC();
    CalibrateADC();
    ConfigureGPIOforADC();
    ConfigureDMA();
    EnableADC();
    ConfigureADC();
    CurrentChannel = 0;         /* Initializes the CurrentChannel */
    ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversions */
    // DisableADC();
}


/**
  * @brief  This function enables the peripheral clocks on GPIO ports A,B,C
  *         configures PA1, PB1 and PC0 in Analog mode.
  *         For portability, some GPIO are again enabled.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureGPIOforADC(void)
{
    /* (1) Enable the peripheral clock of GPIOA */
    /* (2) Select analog mode for PA2 */
    /* (2) Select analog mode for PA3 */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */
    GPIOA->MODER |= GPIO_MODER_MODER2; /* (2) */
    GPIOA->MODER |= GPIO_MODER_MODER3; /* (3) */
}


/**
  * @brief  This function enables the clock in the RCC for the ADC
  *         and start HSI 14MHz dedicated RC oscillator
  * @param  None
  * @retval None
  */
__INLINE void SetClockForADC(void)
{
    /* (1) Enable the peripheral clock of the ADC */
    /* (2) Start HSI14 RC oscillator */
    /* (3) Wait HSI14 is ready */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;        /* (1) */
    RCC->CR2 |= RCC_CR2_HSI14ON;               /* (2) */
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
    {
        /* For robust implementation, add here time-out management */
    }
}

/**
  * @brief  This function performs a self-calibration of the ADC
  * @param  None
  * @retval None
  */
__INLINE void CalibrateADC(void)
{
    /* (1) Ensure that ADEN = 0 */
    /* (2) Clear ADEN */
    /* (3) Launch the calibration by setting ADCAL */
    /* (4) Wait until ADCAL=0 */
    if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
    {
        ADC1->CR &= (uint32_t)(~ADC_CR_ADEN); /* (2) */
    }
    ADC1->CR |= ADC_CR_ADCAL;              /* (3) */
    while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (4) */
    {
        /* For robust implementation, add here time-out management */
    }
}

/**
  * @brief  This function configures the ADC to convert sequentially 4 channels
  *         in continuous mode.
  *         The conversion frequency is 14MHz 
  *         The interrupt on overrun is enabled and the NVIC is configured
  * @param  None
  * @retval None
  */
__INLINE void ConfigureADC(void)
{
    /* (1) Select HSI14 by writing 00 in CKMODE (reset value) */
    /* (2) Select the continuous mode and scanning direction */
    /* (3) Select CHSEL2, CHSEL3 */
    /* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us */
    /* (5) Enable interrupts on EOC, EOSEQ and overrrun */
    //ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
    ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_SCANDIR;                                              /* (2) */
    ADC1->CHSELR = ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL3; /* (3) */
    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;                                 /* (4) */
    ADC1->IER = ADC_IER_EOCIE | ADC_IER_EOSEQIE | ADC_IER_OVRIE;                                    /* (5) */

    /* Configure NVIC for ADC */
    /* (7) Enable Interrupt on ADC */
    /* (8) Set priority for ADC */
    NVIC_EnableIRQ(ADC1_COMP_IRQn);      /* (7) */
    NVIC_SetPriority(ADC1_COMP_IRQn, 0); /* (8) */
}

/**
  * @brief  This function configures the DMA to store the result of an ADC sequence.
  *         The conversion results are stored in N-items array.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureDMA(void)
{
  /* (1) Enable the peripheral clock on DMA */
  /* (2) Enable DMA transfer on ADC - DMACFG is kept at 0 for one shot mode */ 
  /* (3) Configure the peripheral data register address */ 
  /* (4) Configure the memory address */
  /* (5) Configure the number of DMA tranfer to be performs on DMA channel 1 */
  /* (6) Configure increment, size and interrupts */
  /* (7) Enable DMA Channel 1 */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN; /* (2) */
  DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); /* (3) */
  DMA1_Channel1->CMAR = (uint32_t)(ADC_array); /* (4) */
  DMA1_Channel1->CNDTR = NUMBER_OF_ADC_CHANNEL; /* (5) */
  DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                      | DMA_CCR_TEIE | DMA_CCR_TCIE ; /* (6) */  
  DMA1_Channel1->CCR |= DMA_CCR_EN; /* (7) */

  /* Configure NVIC for DMA */
  /* (8) Enable Interrupt on DMA */
  /* (9) Set priority for DMA */
  NVIC_EnableIRQ(DMA1_Channel1_IRQn); /* (8) */
  NVIC_SetPriority(DMA1_Channel1_IRQn,0); /* (9) */
}

/**
  * @brief  This function enables the ADC
  * @param  None
  * @retval None
  */
__INLINE void EnableADC(void)
{
    /* (1) Enable the ADC */
    /* (2) Wait until ADC ready */
    do
    {
        /* For robust implementation, add here time-out management */
        ADC1->CR |= ADC_CR_ADEN; /* (1) */
    } while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */;
}

/**
  * @brief  This function disables the ADC
  * @param  None
  * @retval None
  */
__INLINE void DisableADC(void)
{
    /* (1) Ensure that no conversion on going */
    /* (2) Stop any ongoing conversion */
    /* (3) Wait until ADSTP is reset by hardware i.e. conversion is stopped */
    /* (4) Disable the ADC */
    /* (5) Wait until the ADC is fully disabled */
    if ((ADC1->CR & ADC_CR_ADSTART) != 0) /* (1) */
    {
        ADC1->CR |= ADC_CR_ADSTP; /* (2) */
    }
    while ((ADC1->CR & ADC_CR_ADSTP) != 0) /* (3) */
    {
        /* For robust implementation, add here time-out management */
    }
    ADC1->CR |= ADC_CR_ADDIS;             /* (4) */
    while ((ADC1->CR & ADC_CR_ADEN) != 0) /* (5) */
    {
        /* For robust implementation, add here time-out management */
    }
}


/**
  * @brief  This function handles DMA Channel1 interrupt request.
  *         It manages the ADC and DMA 
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
  if ((DMA1->ISR & DMA_ISR_TCIF1) != 0) /* Test if transfer completed on DMA channel 1 */
  {
    DMA1_Channel1->CCR &= (uint32_t)(~DMA_CCR_EN); /* Disable DMA Channel 1 to write in CNDTR*/
    DMA1_Channel1->CNDTR = NUMBER_OF_ADC_CHANNEL; /* Reload the number of DMA tranfer to be performs on DMA channel 1 */
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* Enable again DMA Channel 1 */
    DMA1->IFCR |= DMA_IFCR_CTCIF1; /* Clear the flag */
    GPIOC->ODR ^= (1<<9);//toggle green led on PC9
    ADC1->CR |= ADC_CR_ADSTART; /* Restart the sequence conversion */
  }
  else if ((DMA1->ISR & DMA_ISR_TEIF1) != 0) /* Test if transfer error on DMA channel 1 */
  {
    adc_error |= ERROR_DMA_XFER; /* Report an error */
    DMA1->IFCR |= DMA_IFCR_CTEIF1; /* Clear the flag */
  }
  else
  {
    adc_error |= ERROR_UNEXPECTED_DMA_IT; /* report unexpected DMA interrupt occurrence */
  }
}


/**
  * @brief  This function handles ADC interrupt request.
  *         It manages the ADC and DMA in case of overrun
  *         the ADC is stopped but not disabled,
  *         the DMA is reinitialized,
  *         The AD conversion is reume till the USER button is pressed
  * @param  None
  * @retval None
  */
void ADC1_COMP_IRQHandler(void)
{
  if ((ADC1->ISR & ADC_ISR_OVR) != 0)  /* Check OVR has triggered the IT */
  {
    GPIOC->BSRR = (1<<8); /* Switch on orange led to report a resume of the conversion  */
    GPIOC->BSRR = (1<<(9+16)); /* Switch off green led to reportit is due to overrun  */
    ADC1->ISR |= ADC_ISR_OVR; /* Clear the pending bit */
    ADC1->CR |= ADC_CR_ADSTP; /* Stop the sequence conversion */
    DMA1_Channel1->CCR &= (uint32_t)(~DMA_CCR_EN); /* Disable DMA Channel 1 to write in CNDTR*/
    DMA1_Channel1->CNDTR = NUMBER_OF_ADC_CHANNEL; /* Reload the number of DMA tranfer to be performs on DMA channel 1 */
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* Enable again DMA Channel 1 */    
  }
  else
  {
    adc_error |= ERROR_UNEXPECTED_ADC_IT; /* Report unexpected ADC interrupt occurrence */
  }
}
