#ifndef WHEEL_CONTROL
#define WHEEL_CONTROL

#include <FreeRTOS.h>
#include "motor.h"
#include "encoder.h"
#include <math.h>

#define NL_BP_NUM 11

float pid_params[3][2][3] = { // 1D: which motor, 2D: which direction, 3D: current u, prev u, prev y
    {
        {0.114789328973203,	-0.103351645873929, 1},
        {0.114789328973203,	-0.103351645873929, 1}
    },
    {
        {0.114789328973203,	-0.103351645873929, 1},
        {0.114789328973203,	-0.103351645873929, 1}
    },
    {
        {0.114789328973203,	-0.103351645873929, 1},
        {0.114789328973203,	-0.103351645873929, 1}
    }
};
float inverse_nonlinearity[3][2][2][NL_BP_NUM] = { // 1D: which motor, 2D: which direction, 3D: breakpoint or table value, 4D: point idx
    {
        {
            {0, 0.000273685692041052,	0.0699337157284592,	0.0971860965381096,	0.109320686931914,	0.119242453940753,	0.123106691702407,	0.132777233549822,	0.141185275098227,	0.145113372155702,	0.148582159391034},
            {0, 7.39189258805592,	17.1484297366043,	27.3988929750014,	36.0428156954499,	45.5726090821521,	54.6059902361493,	63.6369712329163,	71.7162641155802,	90.8088497255657,	85.5086621870342}
        },
        {
            {0, 0.000273685692041052,	0.0699337157284592,	0.0971860965381096,	0.109320686931914,	0.119242453940753,	0.123106691702407,	0.132777233549822,	0.141185275098227,	0.145113372155702,	0.148582159391034},
            {0, 7.39189258805592,	17.1484297366043,	27.3988929750014,	36.0428156954499,	45.5726090821521,	54.6059902361493,	63.6369712329163,	71.7162641155802,	90.8088497255657,	85.5086621870342}
        }
    },
    {
        {
            {0, 0.000273685692041052,	0.0699337157284592,	0.0971860965381096,	0.109320686931914,	0.119242453940753,	0.123106691702407,	0.132777233549822,	0.141185275098227,	0.145113372155702,	0.148582159391034},
            {0, 7.39189258805592,	17.1484297366043,	27.3988929750014,	36.0428156954499,	45.5726090821521,	54.6059902361493,	63.6369712329163,	71.7162641155802,	90.8088497255657,	85.5086621870342}
        },
        {
            {0, 0.000273685692041052,	0.0699337157284592,	0.0971860965381096,	0.109320686931914,	0.119242453940753,	0.123106691702407,	0.132777233549822,	0.141185275098227,	0.145113372155702,	0.148582159391034},
            {0, 7.39189258805592,	17.1484297366043,	27.3988929750014,	36.0428156954499,	45.5726090821521,	54.6059902361493,	63.6369712329163,	71.7162641155802,	90.8088497255657,	85.5086621870342}
        }
    },
    {
        {
            {0, 0.000273685692041052,	0.0699337157284592,	0.0971860965381096,	0.109320686931914,	0.119242453940753,	0.123106691702407,	0.132777233549822,	0.141185275098227,	0.145113372155702,	0.148582159391034},
            {0, 7.39189258805592,	17.1484297366043,	27.3988929750014,	36.0428156954499,	45.5726090821521,	54.6059902361493,	63.6369712329163,	71.7162641155802,	90.8088497255657,	85.5086621870342}
        },
        {
            {0, 0.000273685692041052,	0.0699337157284592,	0.0971860965381096,	0.109320686931914,	0.119242453940753,	0.123106691702407,	0.132777233549822,	0.141185275098227,	0.145113372155702,	0.148582159391034},
            {0, 7.39189258805592,	17.1484297366043,	27.3988929750014,	36.0428156954499,	45.5726090821521,	54.6059902361493,	63.6369712329163,	71.7162641155802,	90.8088497255657,	85.5086621870342}
        }
    }
};

Motor *motors[3];
Encoder *encoders[3];

const int cycle_period = 10; // [ms]

float speed_setpoints[3] = {0,0,0};
float current_speeds[3] = {0,0,0};
float errors[3] = {0,0,0};
float errors_prev[3] = {0,0,0};

float ctrl_outputs[3] = {0,0,0};
float ctrl_outputs_prev[3] = {0,0,0};

float nonlinearized_outputs[3] = {0,0,0};


void wheel_control(void *params){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(cycle_period);

  int mot_idx = 0;
  int dir_idx = 0;
  int nl_idx = 0;
  while(true){
    
    for(mot_idx = 0; mot_idx <3; mot_idx++){
        // read sensor
        current_speeds[mot_idx] = encoders[mot_idx]->ticks2rad(encoders[mot_idx] ->getCountReset())/(cycle_period/1000.0);

        // calculate error
        errors[mot_idx] = fabs(speed_setpoints[mot_idx]) - current_speeds[mot_idx];

        // select direction
        if(speed_setpoints[mot_idx] == 0){
            motors[mot_idx]->stop();
            continue;
        }else if(speed_setpoints[mot_idx] > 0){
            dir_idx = 0;
        }else{
            dir_idx = 1;
        }

        // calculate controller output value and update state
        if(motors[mot_idx]->isSaturated()){
            ctrl_outputs[mot_idx] = pid_params[mot_idx][dir_idx][0] * errors[mot_idx];
        }else{
            ctrl_outputs[mot_idx] = pid_params[mot_idx][dir_idx][0] * errors[mot_idx] + pid_params[mot_idx][dir_idx][1] * errors_prev[mot_idx] + pid_params[mot_idx][dir_idx][2] * ctrl_outputs_prev[mot_idx];
        }
        
        // handle nonlinearity
        for(nl_idx = 1; nl_idx < NL_BP_NUM; nl_idx++){ // start from 1 because [0,0] is the first element
            if(inverse_nonlinearity[mot_idx][dir_idx][0][nl_idx] > ctrl_outputs[mot_idx])
                break;
        }

        if(nl_idx == NL_BP_NUM){
            nonlinearized_outputs[mot_idx] = 100;
        }else{
            nonlinearized_outputs[mot_idx] = inverse_nonlinearity[mot_idx][dir_idx][1][nl_idx-1] + (inverse_nonlinearity[mot_idx][dir_idx][1][nl_idx] - inverse_nonlinearity[mot_idx][dir_idx][1][nl_idx-1]) / (inverse_nonlinearity[mot_idx][dir_idx][0][nl_idx] - inverse_nonlinearity[mot_idx][dir_idx][0][nl_idx-1]) * (ctrl_outputs[mot_idx] - inverse_nonlinearity[mot_idx][dir_idx][0][nl_idx-1]);
            errors_prev[mot_idx] = errors[mot_idx];
            ctrl_outputs_prev[mot_idx] = ctrl_outputs[mot_idx];
        }

        // perform actuation
        if(dir_idx == 0){
            motors[mot_idx]->rotateCW(nonlinearized_outputs[mot_idx]);
        }else{
            motors[mot_idx]->rotateCCW(nonlinearized_outputs[mot_idx]);
        }
    }



    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

#endif