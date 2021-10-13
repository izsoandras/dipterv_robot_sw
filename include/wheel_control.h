#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#include <FreeRTOS.h>
#include "motor.h"
#include "encoder.h"
#include <math.h>

#define NL_BP_NUM 18

extern float pid_params[3][2][3];
extern float inverse_nonlinearity[3][2][2][NL_BP_NUM];

extern Motor *motors[3];
extern Encoder *encoders[3];

extern const int cycle_period; // [ms]

extern float speed_setpoints[3];
extern float current_speeds[3];


void wheel_control(void *params);

#endif