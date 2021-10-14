#ifndef FUSION_H
#define FUSION_H

#include "mykalman.h"
#include "position_control.h"
#include "MPU9250.h"

MPU9250* imu;

float cam_vec[3] = {0,0,0};
float imu_acc[2] = {0,0};
float imu_head = 0;             // TODO: confirm that heading can't go out frombetwenn 0,2pi
const float head_offset = 0;    // offset of magnetic north from NED 0°

float A_p[6][6]={};
float B_p[6][2]={};
float C_p[2][6]={};
float Rv_p[6][6]={};
float Rz_p[2][2]={};

float A_o[2][2]={};
float B_o[2][1]={};
float C_o[1][2]={};
float Rv_o[2][2]={};
float Rz_o[1][1]={};


KF pos_estimator(&A_p[0][0], &B_p[0][0], &C_p[0][0], &Rv_p[0][0], &Rz_p[0][0], 6, 2, 2);
KF ori_estimator(&A_o[0][0], &B_o[0][0], &C_o[0][0], &Rv_o[0][0], &Rz_o[0][0], 2, 1, 1);

void update_est(){
    imu->update();
    imu_head = imu->getYaw();
    for(uint8_t i = 0; i < 3; i++)
        imu_acc[i] = imu->getAcc(i);

    pos_estimator.update(imu_acc, cam_vec);
    ori_estimator.update(&imu_head, cam_vec+2);

    memcpy(meas_vec,pos_estimator.x_hat, 2*sizeof(float));
    memcpy(meas_vec+2, ori_estimator.x_hat+1, sizeof(float));
}

void fusion_task(){
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(cycle_period);

    while(true){
        update_est();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#endif