#include "main.h"

void _init(void) {}

int sdt = 0;

int main()
{
    SysTick_Config(48000); /* 1ms config */
    NVIC_SetPriority(SysTick_IRQn, 0);

    RCC_Init();
    ConfigureTIM1PWM();
    ConfigureTIM3PWM();
    ConfigureGPIO();
    UART_Init();
    ConfigureTIM6();
    initADC();

    Axis_Init();
    Queue q1;
    ax1.queue = &q1;
    queue_init(ax1.queue);

    Queue q2;
    ax2.queue = &q2;
    queue_init(ax2.queue);

    // Enable motor drive
    GPIOA->BSRR = GPIO_BSRR_BS_5;
    GPIOB->BSRR = GPIO_BSRR_BS_15;

    // Axis ready to handle tasks
    ax1.state = AXIS_STATUS_IDLE;
    ax2.state = AXIS_STATUS_IDLE;

    uart1.delay = 5; // read timeout
    uart1.charDelay = 2;

    while (1)
    {
        if (sdt == 1)
        {
            serialDisableTransfer();
        }

        if (is_modbus_frame_ready(&uart1))
        {
            modbus_process_message(&uart1);
            modbus_start_response(&uart1);
        }

        // if (is_hall_sensor_active_2())
        // {
        //   GPIOC->BSRR = GPIO_BSRR_BR_14;
        // }
        // else
        // {
        //   GPIOC->BSRR = GPIO_BSRR_BS_14;
        // }
    }
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    TIM1->SR &= ~TIM_SR_UIF; // clear interruption flag

    axis_loop(&ax1);
}

void TIM3_IRQHandler(void)
{
    TIM3->SR &= ~TIM_SR_UIF; // clear interruption flag

    axis_loop(&ax2);
}
