#include "Functions.h"
#include "wheel_control.h"
#include "math.h"
#include "mykalman.h"

float control_vec[3] = {0,0,0};
float state_vec[3] = {0,0,0};
float ref_vec[3] = {0,0,0};
float global_control_vec[3] = {0,0,0};
float P_pos = 0.4;
float P_ori = 0.4;
float err_vec[3] = {0,0,0};
float meas_vec[3] = {0,0,0};

float global2robot_rot[3][3] = {
                                {1, 0, 0},
                                {0, 1, 0}, 
                                {0, 0, 1}
};

float A[6][6]={};
float B[6][2]={};
float C[2][6]={};
float Rv[6][6]={};
float Rz[2][2]={};

float control2wheel[3][3] = {
    {34.6410161513775,	20,	2.73692000000000},
    {-7.34788079488412e-15,	-40,	2.73692000000000},
    {-34.6410161513775,	20.0000000000000,	2.73692000000000}
};
    // TODO: only 1 KF is needed
    KF state_estimator(&A[0][0], &B[0][0], &C[0][0], &Rv[0][0], &Rz[0][0], 6, 2, 2);

void position_control(){
        
        state_estimator.update(control_vec, meas_vec);

        memcpy(state_vec, pos_estimator.x_hat, 3*sizeof(float));

        sub(ref_vec, state_vec, err_vec, 3);

        global_control_vec[0] = P_pos * err_vec[0];
        global_control_vec[1] = P_pos * err_vec[1];
        global_control_vec[2] = P_ori * err_vec[2];

        global2robot_rot[0][0] = cos(state_vec[2]);
        global2robot_rot[0][1] = -sin(state_vec[2]);
        global2robot_rot[1][0] = cos(state_vec[2]);
        global2robot_rot[1][1] = sin(state_vec[2]);

        mul(&global2robot_rot[0][0], global_control_vec, control_vec, 3, 3, 1);
        mul(&control2wheel[0][0], control_vec, speed_setpoints, 3, 3, 1);
}

void position_control_task(void* param){
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(cycle_period);

    while(true){
        position_control();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
