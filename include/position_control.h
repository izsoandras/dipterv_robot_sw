#include "Functions.h"
#include "wheel_control.h"

float control_vec[3] = {0,0,0};

float control2wheel[3][3] = {
    {34.6410161513775,	20,	2.73692000000000},
    {-7.34788079488412e-15,	-40,	2.73692000000000},
    {-34.6410161513775,	20.0000000000000,	2.73692000000000}
};

void position_control(void *params){
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(cycle_period);
    while(true){
        mul(&control2wheel[0][0], control_vec, speed_setpoints, 3, 3, 1);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
