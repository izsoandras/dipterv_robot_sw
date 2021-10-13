#include "wheel_control.h"

float pid_params[3][2][3] = { // 1D: which motor, 2D: which direction, 3D: current u, prev u, prev y
    {
        {0.320077758022071,	-0.292510591095589, 1},
        {0.320077758022071,	-0.292510591095589, 1}
    },
    {
        {0.320077758022071,	-0.292510591095589, 1},
        {0.320077758022071,	-0.292510591095589, 1}
    },
    {
        {0.320077758022071,	-0.292510591095589, 1},
        {0.320077758022071,	-0.292510591095589, 1},
    }
};
float inverse_nonlinearity[3][2][2][NL_BP_NUM] = { // 1D: which motor, 2D: which direction, 3D: breakpoint or table value, 4D: point idx
    {
        {
            {0, 8.67361737988404e-17,	0.0147833226707626,	0.0314996830907063,	0.107312771083493,	0.352095854166548,	0.541130716903937,	0.655803453502254,	0.763189068210479,	0.884485749342306,	0.995014945791568,	1.08499519138804,	1.16699066032570,	1.24085500627414,	1.27678400974974,	1.27755660619318,	1.37628453767835,	1.65515631043158},
            {0, 4.76190483952312,	9.52380952836509,	14.2857142474282,	33.3333343550043,	38.0952374719551,	42.8571432360672,	47.6190476562062,	52.3809524063895,	57.1428571232214,	61.9047619226635,	66.6666666736451,	71.4285714107282,	76.1904761075709,	80.9523813047118,	85.7142847319142,	90.4761955945002,	95.2380896700792}
        },
        {
            {0, 8.67361737988404e-17,	0.0147833226707626,	0.0314996830907063,	0.107312771083493,	0.352095854166548,	0.541130716903937,	0.655803453502254,	0.763189068210479,	0.884485749342306,	0.995014945791568,	1.08499519138804,	1.16699066032570,	1.24085500627414,	1.27678400974974,	1.27755660619318,	1.37628453767835,	1.65515631043158},
            {0, 4.76190483952312,	9.52380952836509,	14.2857142474282,	33.3333343550043,	38.0952374719551,	42.8571432360672,	47.6190476562062,	52.3809524063895,	57.1428571232214,	61.9047619226635,	66.6666666736451,	71.4285714107282,	76.1904761075709,	80.9523813047118,	85.7142847319142,	90.4761955945002,	95.2380896700792}
        }
    },
    {
        {
            {0, 8.67361737988404e-17,	0.0147833226707626,	0.0314996830907063,	0.107312771083493,	0.352095854166548,	0.541130716903937,	0.655803453502254,	0.763189068210479,	0.884485749342306,	0.995014945791568,	1.08499519138804,	1.16699066032570,	1.24085500627414,	1.27678400974974,	1.27755660619318,	1.37628453767835,	1.65515631043158},
            {0, 4.76190483952312,	9.52380952836509,	14.2857142474282,	33.3333343550043,	38.0952374719551,	42.8571432360672,	47.6190476562062,	52.3809524063895,	57.1428571232214,	61.9047619226635,	66.6666666736451,	71.4285714107282,	76.1904761075709,	80.9523813047118,	85.7142847319142,	90.4761955945002,	95.2380896700792}
        },
        {
            {0, 8.67361737988404e-17,	0.0147833226707626,	0.0314996830907063,	0.107312771083493,	0.352095854166548,	0.541130716903937,	0.655803453502254,	0.763189068210479,	0.884485749342306,	0.995014945791568,	1.08499519138804,	1.16699066032570,	1.24085500627414,	1.27678400974974,	1.27755660619318,	1.37628453767835,	1.65515631043158},
            {0, 4.76190483952312,	9.52380952836509,	14.2857142474282,	33.3333343550043,	38.0952374719551,	42.8571432360672,	47.6190476562062,	52.3809524063895,	57.1428571232214,	61.9047619226635,	66.6666666736451,	71.4285714107282,	76.1904761075709,	80.9523813047118,	85.7142847319142,	90.4761955945002,	95.2380896700792}
        }
    },
    {
        {
            {0, 8.67361737988404e-17,	0.0147833226707626,	0.0314996830907063,	0.107312771083493,	0.352095854166548,	0.541130716903937,	0.655803453502254,	0.763189068210479,	0.884485749342306,	0.995014945791568,	1.08499519138804,	1.16699066032570,	1.24085500627414,	1.27678400974974,	1.27755660619318,	1.37628453767835,	1.65515631043158},
            {0, 4.76190483952312,	9.52380952836509,	14.2857142474282,	33.3333343550043,	38.0952374719551,	42.8571432360672,	47.6190476562062,	52.3809524063895,	57.1428571232214,	61.9047619226635,	66.6666666736451,	71.4285714107282,	76.1904761075709,	80.9523813047118,	85.7142847319142,	90.4761955945002,	95.2380896700792}
        },
        {
            {0, 8.67361737988404e-17,	0.0147833226707626,	0.0314996830907063,	0.107312771083493,	0.352095854166548,	0.541130716903937,	0.655803453502254,	0.763189068210479,	0.884485749342306,	0.995014945791568,	1.08499519138804,	1.16699066032570,	1.24085500627414,	1.27678400974974,	1.27755660619318,	1.37628453767835,	1.65515631043158},
            {0, 4.76190483952312,	9.52380952836509,	14.2857142474282,	33.3333343550043,	38.0952374719551,	42.8571432360672,	47.6190476562062,	52.3809524063895,	57.1428571232214,	61.9047619226635,	66.6666666736451,	71.4285714107282,	76.1904761075709,	80.9523813047118,	85.7142847319142,	90.4761955945002,	95.2380896700792}
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