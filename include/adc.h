#ifndef ADC_H
#define ADC_H

/* Error codes used to make the orange led blinking */
#define ERROR_DMA_XFER 0x01
#define ERROR_UNEXPECTED_DMA_IT 0x02
#define ERROR_UNEXPECTED_ADC_IT 0x04
#define ERROR_UNEXPECTED_EXT_IT 0x08

#define NUMBER_OF_ADC_CHANNEL 2

/* Internal voltage reference calibration value address */
#define VREFINT_CAL_ADDR ((uint16_t *)((uint32_t)0x1FFFF7BA))

uint16_t ADC_array[NUMBER_OF_ADC_CHANNEL]; //Array to store the values coming from the ADC and copied by DMA
uint32_t CurrentChannel;                   //index on the ADC_array

void initADC(void);
void SetClockForADC(void);
void CalibrateADC(void);
void ConfigureADC(void);
void ConfigureGPIOforADC(void);
void ConfigureDMA(void);
void EnableADC(void);
void DisableADC(void);

#endif // ADC_H