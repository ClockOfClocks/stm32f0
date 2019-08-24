#include "stdint.h"
#include "stdbool.h"

#include "queue.h"
#include "timing.h"

#define COS_OFFSET 64

#define INTERRUPTION_FREQ 24000 // 48MHz / 2 (PSC) / 1000 (ARR) = 24kHz;

#define PERIODS_PER_REVOLUTION 180 // 720 (steps per revolution) / 4

#define ONE_DEGREE_POINTER_DIFF 524288 // ( (1 << 20) * PERIODS_PER_REVOLUTION / 360 ) // one_period_pointer_diff * periods_per_revolution / 360

// (12-bit global position) (8-bit sin table position) (12-bit precision)
// 12-bit position is 2^12 / 2 / (720/4) ~= 11.37 revolutions in each direction
#define INITIAL_POINTER_POSITION (1 << 31) // ( ( (1 << 12) / 2 ) << 20 )

#define AXIS_STATUS_INIT 0
#define AXIS_STATUS_CALIBRATION 1
#define AXIS_STATUS_MOVE 2
#define AXIS_STATUS_IDLE 3
#define AXIS_STATUS_WAIT 4

#define AXIS_CALIBRATION_INIT 0
#define AXIS_CALIBRATION_SEARCH 1
#define AXIS_CALIBRATION_DETECTED 2
#define AXIS_CALIBRATION_FINISHED 3

#define AXIS_TASK_TYPE_MOVE 0
#define AXIS_TASK_TYPE_WAIT 1
#define AXIS_TASK_TYPE_CALIBRATION 2

struct Axis
{
   uint32_t pointer_position;
   uint8_t state;
   bool direction;                        // current direction of move, true – positive
   int16_t pointer_diff_per_interruption; // speed
   uint32_t target_pointer_position;
   uint32_t wait_until_ms;
   uint8_t calibration_state;
   uint32_t calibration_pointer_position; // position when magnet detected first time

   volatile uint32_t *sin_pwm_channel1;
   volatile uint32_t *sin_pwm_channel2;
   volatile uint32_t *cos_pwm_channel1;
   volatile uint32_t *cos_pwm_channel2;

   struct Queue *queue;
   bool (*is_hall_sensor_active) (void);
};

struct AxisTask
{
   uint8_t type;
   float degree;
   float speed;
   bool relative;
   uint16_t wait_ms;
   struct QueueNode n;
};

void Axis_Init(void);

void update_axis_sin_cos(struct Axis *axis);
void set_axis_speed(struct Axis *axis, float degreePerSecond);
void move_axis_to(struct Axis *axis, float degree, bool relative);
void extract_task(struct Axis *axis);
void axis_loop(struct Axis *axis);
bool is_hall_sensor_active_1 (void);
bool is_hall_sensor_active_2 (void);
