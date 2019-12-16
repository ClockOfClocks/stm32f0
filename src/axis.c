#include "main.h"

int16_t sin_data[256] = // 256 values per 2pi, +\- 1024
    {0, 24, 49, 73, 98, 122, 146, 170, 195, 219, 242, 266, 290, 313, 336, 359, 382, 405, 427, 449, 471, 492, 514, 535, 555, 575, 595, 615, 634, 653, 671, 689, 707, 724, 740, 757, 773, 788, 803, 817, 831, 844, 857, 870, 881, 893, 903, 914, 923, 932, 941, 949, 956, 963, 970, 975, 980, 985, 989, 992, 995, 997, 998, 999, 1000, 999, 998, 997, 995, 992, 989, 985, 980, 975, 970, 963, 956, 949, 941, 932, 923, 914, 903, 893, 881, 870, 857, 844, 831, 817, 803, 788, 773, 757, 740, 724, 707, 689, 671, 653, 634, 615, 595, 575, 555, 535, 514, 492, 471, 449, 427, 405, 382, 359, 336, 313, 290, 266, 242, 219, 195, 170, 146, 122, 98, 73, 49, 24, 0, -24, -49, -73, -98, -122, -146, -170, -195, -219, -242, -266, -290, -313, -336, -359, -382, -405, -427, -449, -471, -492, -514, -535, -555, -575, -595, -615, -634, -653, -671, -689, -707, -724, -740, -757, -773, -788, -803, -817, -831, -844, -857, -870, -881, -893, -903, -914, -923, -932, -941, -949, -956, -963, -970, -975, -980, -985, -989, -992, -995, -997, -998, -999, -1000, -999, -998, -997, -995, -992, -989, -985, -980, -975, -970, -963, -956, -949, -941, -932, -923, -914, -903, -893, -881, -870, -857, -844, -831, -817, -803, -788, -773, -757, -740, -724, -707, -689, -671, -653, -634, -615, -595, -575, -555, -535, -514, -492, -471, -449, -427, -405, -382, -359, -336, -313, -290, -266, -242, -219, -195, -170, -146, -122, -98, -73, -49, -24};

void Axis_Init(void)
{
  // ax1
  ax1.is_hour_axis = true;
  ax1.state = AXIS_STATUS_INIT;
  ax1.pointer_position = INITIAL_POINTER_POSITION;
  ax1.target_pointer_position = INITIAL_POINTER_POSITION;
  ax1.direction = true;

  ax1.sin_pwm_channel1 = &TIM3->CCR1;
  ax1.sin_pwm_channel2 = &TIM3->CCR2;
  ax1.cos_pwm_channel1 = &TIM3->CCR3;
  ax1.cos_pwm_channel2 = &TIM3->CCR4;

  ax1.is_hall_sensor_active = is_hall_sensor_active_1;

  // ax2
  ax1.is_hour_axis = false;
  ax2.state = AXIS_STATUS_INIT;
  ax2.pointer_position = INITIAL_POINTER_POSITION;
  ax2.target_pointer_position = INITIAL_POINTER_POSITION;
  ax2.direction = true;

  ax2.sin_pwm_channel1 = &TIM1->CCR1;
  ax2.sin_pwm_channel2 = &TIM1->CCR2;
  ax2.cos_pwm_channel1 = &TIM1->CCR3;
  ax2.cos_pwm_channel2 = &TIM1->CCR4;

  ax2.is_hall_sensor_active = is_hall_sensor_active_2;
}

void update_axis_sin_cos(struct Axis *axis)
{
  // clear 12 most bits and shift value right to trim 12 lower bits
  uint8_t sin_pointer = (axis->pointer_position & ~(((1 << 12) - 1) << 20)) >> 12;
  uint8_t cos_pointer = sin_pointer + COS_OFFSET;

  int16_t sin_value = sin_data[sin_pointer];
  int16_t cos_value = sin_data[cos_pointer];

  uint16_t sin_pwm_ccr = (sin_value / 2) + 512;
  *axis->sin_pwm_channel1 = sin_pwm_ccr;
  *axis->sin_pwm_channel2 = sin_pwm_ccr;

  uint16_t cos_pwm_ccr = (cos_value / 2) + 512;
  *axis->cos_pwm_channel1 = cos_pwm_ccr;
  *axis->cos_pwm_channel2 = cos_pwm_ccr;
}

void set_axis_speed(struct Axis *axis, float degreePerSecond)
{
  axis->pointer_diff_per_interruption = (degreePerSecond * ONE_DEGREE_POINTER_DIFF) / INTERRUPTION_FREQ;
}

void move_axis_to(struct Axis *axis, float degree, bool relative)
{
  int32_t diff = (ONE_DEGREE_POINTER_DIFF * degree);

  if (relative)
  {
    axis->target_pointer_position = axis->pointer_position + diff;
  }
  else
  {
    axis->target_pointer_position = INITIAL_POINTER_POSITION + diff;
  }
  axis->direction = axis->target_pointer_position > axis->pointer_position;
}

void extract_task(struct Axis *axis)
{
  if (queue_size(axis->queue) == 0)
  {
    axis->state = AXIS_STATUS_IDLE;
  }
  else
  {
    struct AxisTask *task = queue_entry(queue_peek(axis->queue), struct AxisTask, n);

    switch (task->type)
    {
    case AXIS_TASK_TYPE_MOVE:
      set_axis_speed(axis, task->speed);
      move_axis_to(axis, task->degree, task->relative);
      // Let's move axis
      axis->state = AXIS_STATUS_MOVE;
      break;
    case AXIS_TASK_TYPE_CALIBRATION:
      set_axis_speed(axis, 30);
      move_axis_to(axis, 720, true);
      axis->calibration_state = AXIS_CALIBRATION_INIT;
      // Let's find home
      axis->state = AXIS_STATUS_CALIBRATION;
      break;
    case AXIS_TASK_TYPE_WAIT:
      axis->wait_until_ms = millis() + task->wait_ms;
      axis->state = AXIS_STATUS_WAIT;
      break;

    default:
      break;
    }
    // Remove task from queue
    queue_pop(axis->queue);

    // Clear memory
    free(task);
  }
}

void axis_loop(struct Axis *axis)
{
  switch (axis->state)
  {
  case AXIS_STATUS_IDLE:
    extract_task(axis);
    break;

  case AXIS_STATUS_WAIT:
    if (millis() >= axis->wait_until_ms)
    {
      // continue
      axis->state = AXIS_STATUS_IDLE;
    }
    break;

  case AXIS_STATUS_MOVE:
  case AXIS_STATUS_CALIBRATION:
    if (axis->direction)
    {
      axis->pointer_position += axis->pointer_diff_per_interruption;
      update_axis_sin_cos(axis);
      if (axis->target_pointer_position <= axis->pointer_position)
      {
        // finished
        axis->state = AXIS_STATUS_IDLE;
      }
    }
    else
    {
      axis->pointer_position -= axis->pointer_diff_per_interruption;
      update_axis_sin_cos(axis);
      if (axis->target_pointer_position >= axis->pointer_position)
      {
        // finished
        axis->state = AXIS_STATUS_IDLE;
      }
    }

    if (AXIS_STATUS_CALIBRATION == axis->state)
    {
      bool searchState = axis->calibration_state == AXIS_CALIBRATION_SEARCH;
      if (axis->is_hall_sensor_active(searchState))
      {
        if (searchState)
        {
          axis->calibration_pointer_position = axis->pointer_position;
          axis->calibration_state = AXIS_CALIBRATION_DETECTED;
        }
      }
      else
      {
        if (axis->calibration_state == AXIS_CALIBRATION_INIT)
        {
          // do not track case when magnet detected in the beginning of calibration
          axis->calibration_state = AXIS_CALIBRATION_SEARCH;
        }
        else if (axis->calibration_state == AXIS_CALIBRATION_DETECTED)
        {
          // initial position + position of magnet center
          axis->pointer_position = INITIAL_POINTER_POSITION + (axis->pointer_position - axis->calibration_pointer_position) / 2;

          if(true){ // set home position to the right
            // axis->pointer_position += (ONE_DEGREE_POINTER_DIFF * 45);
          }

          axis->calibration_state = AXIS_CALIBRATION_FINISHED;

          // if no tasks in the queue, move to home position after calibratin finish
          if (queue_size(axis->queue) == 0)
          {
            struct AxisTask* task = malloc (sizeof (struct AxisTask));
            task->type = AXIS_TASK_TYPE_MOVE;
            task->degree = 0;
            task->speed = 20;
            task->relative = false;
            queue_push(axis->queue, &task->n);
          }
          // pick next task
          axis->state = AXIS_STATUS_IDLE;
        }
      }
    }
    break;

  default:
    break;
  }
}

bool is_hall_sensor_active_1(bool enterState)
{
  return ADC_array[1] > (enterState ? 3500 : 3200);
}

bool is_hall_sensor_active_2(bool enterState)
{
  return ADC_array[0] > (enterState ? 3300 : 3000);
}
