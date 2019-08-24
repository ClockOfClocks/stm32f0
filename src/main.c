#include "main.h"

void _init(void) {}

int main()
{
  SysTick_Config(48000); /* 1ms config */
  NVIC_SetPriority(SysTick_IRQn, 0);

  RCC_Init();
  ConfigureTIM1PWM();
  ConfigureTIM3PWM();
  ConfigureGPIO();

  Axis_Init();
  Queue q1;
  ax1.queue = &q1;
  queue_init(ax1.queue);

  Queue q2;
  ax2.queue = &q2;
  queue_init(ax2.queue);

  // Test tasks
  // struct AxisTask task0;
  // task0.type = AXIS_TASK_TYPE_CALIBRATION;
  // queue_push(ax2.queue, &task0.n);

  float testSpeed = 20;
  float testAngle = 180;

  struct AxisTask task;
  task.type = AXIS_TASK_TYPE_MOVE;
  task.degree = testAngle;
  task.speed = testSpeed;
  task.relative = false;
  queue_push(ax1.queue, &task.n);

  struct AxisTask task2;
  task2.type = AXIS_TASK_TYPE_MOVE;
  task2.degree = testAngle;
  task2.speed = testSpeed;
  task2.relative = false;
  queue_push(ax2.queue, &task2.n);

  // Enable motor drive
  GPIOA->BSRR = GPIO_BSRR_BS_5;
  GPIOB->BSRR = GPIO_BSRR_BS_15;

  // Axis ready to handle tasks
  ax1.state = AXIS_STATUS_IDLE;
  ax2.state = AXIS_STATUS_IDLE;

  while (1)
  {
    // put your main code here, to run repeatedly:

    if (is_hall_sensor_active_2())
    {
      GPIOC->BSRR = GPIO_BSRR_BR_14;
    }
    else
    {
      GPIOC->BSRR = GPIO_BSRR_BS_14;
    }
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