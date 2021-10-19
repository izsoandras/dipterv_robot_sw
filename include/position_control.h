#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "Functions.h"
#include "wheel_control.h"
#include "math.h"
#include "mykalman.h"
#include "esp_timer.h"
#include "esp_log.h"

bool manual_ctrl_vec = false;

uint8_t pos_cycle = 50; // [ms]

float control_vec[3] = {0, 0, 0};
float state_vec[3] = {0, 0, 0};
float ref_vec[3] = {0, 0, 0};
float global_control_vec[3] = {0, 0, 0};
float P_pos = 0.5;
float P_ori = 1;
float err_vec[3] = {0, 0, 0};
float meas_vec[3] = {0, 0, 0};

float global2robot_rot[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}};

float A[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};
float B[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};
float C[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};
float Rv[3][3] = {
    {-4.38372581696818e-05,	                    0,	    0},
    {                    0,	-4.38372581696818e-05,	    0},
    {                    0,	                    0,	1e-07}
};
float Rz[3][3] = {
    {0.000511966716115089,	                   0,	                   0},
    {                   0,	0.000180664054506985,	                   0},
    {                   0,	                   0,	8.27760425159963e-05}
};

float control2wheel[3][3] = {
    {34.6410161513775, 20, 2.73692000000000},
    {-7.34788079488412e-15, -40, 2.73692000000000},
    {-34.6410161513775, 20.0000000000000, 2.73692000000000}
};

// State estimator
KF state_estimator(&A[0][0], &B[0][0], &C[0][0], &Rv[0][0], &Rz[0][0], zeros, 3, 3, 3);
void init_position_control(float x0[]){
    state_estimator = KF(&A[0][0], &B[0][0], &C[0][0], &Rv[0][0], &Rz[0][0], x0, 3, 3, 3);
    copy(state_vec, x0, 3);
}

void position_control()
{

    state_estimator.update(control_vec, meas_vec);

    if (!manual_ctrl_vec)
    {
        //memcpy(state_vec, state_estimator.x_hat, 3 * sizeof(float));


        sub(ref_vec, state_vec, err_vec, 3);
        if(err_vec[2] > 3.141592654 || err_vec[2] < -3.141592654)
            err_vec[2] = atan2(sin(err_vec[2]), cos(err_vec[2]));   // Wrap to 2pi

        global_control_vec[0] = P_pos * err_vec[0];
        global_control_vec[1] = P_pos * err_vec[1];
        global_control_vec[2] = P_ori * err_vec[2];

        global2robot_rot[0][0] = cos(-state_vec[2]);
        global2robot_rot[0][1] = -sin(-state_vec[2]);
        global2robot_rot[1][0] = sin(-state_vec[2]);
        global2robot_rot[1][1] = cos(-state_vec[2]);

        mul(&global2robot_rot[0][0], global_control_vec, control_vec, 3, 3, 1);
    }

    
    mul(&control2wheel[0][0], control_vec, speed_setpoints, 3, 3, 1);
    ESP_LOGI("state","%f\t%f\t%f", state_vec[0],state_vec[1], state_vec[2]);
}

void position_control_task(void *param)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(pos_cycle);

    while (true)
    {
        // uint64_t start = esp_timer_get_time();
        position_control();

        // uint64_t end = esp_timer_get_time();

        // ESP_LOGI("kf meas", "%fms", (end - start) / 1000.0);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#endif