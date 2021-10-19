#ifndef FUSION_H
#define FUSION_H

#include "mykalman.h"
#include "position_control.h"
#include "MPU9250.h"
#include "math.h"

MPU9250* imu_fusion;

float cam_vec[3] = {0,0,0};
float imu_acc[2] = {0,0};
float imu_acc_raw[2] = {0,0};
float imu_acc_global[2] = {0,0};
float imu_gyro = 0;
float imu_mag_raw[2] = {0,0};
float imu_mag_korr[2] = {0,0};
float imu_head_earth = 0;             // TODO: confirm that heading can't go out frombetwenn [0;2pi]
float imu_head = 0;
float head_offset = 0;    // offset of magnetic north from NED 0°
const int fusion_cycle = 10;    // [ms]

float local2global_trf[2][2] = {
    {1, 0},
    {0, 1}
};

float mag_korr_trf[2][2] = {
    {0.884275490666799,	0.355036043266962},
    {0.372589849153722,	-0.927996123002465}
};

float acc_bias[2] = {-0.000810458718105195,	0.00281820950827883};

    // acceleration and gyro sensor coordinates are different from the robot coordinates
 // IMU measures in x*g so we have to upscale
float acc_trf[2][2] = {
    {    0, 9.8},
    { -9.8,   0}
};
float gyro_bias = 0.277865106481108;

float A_p[6][6]={
    {1,	0,	-0.0100000000000000,	0,	0,	0},
    {0,	1,	0,	-0.0100000000000000,	0,	0},
    {0,	0,	1,	0,	0,	0},
    {0,	0,	0,	1,	0,	0},
    {0.0100000000000000,	0,	0,	0,	1,	0},
    {0,	0.0100000000000000,	0,	0,	0,	1},
};
float B_p[6][2]={
    {0.0100000000000000,	0},
    {0,	0.0100000000000000},
    {0,	0},
    {0,	0},
    {0,	0},
    {0,	0}
};
float C_p[2][6]={
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1}
};
float Rv_p[6][6]={
    {6.10800239326465e-07,	0,	0,	0,	0,	0},
    {0,	0.000450015361182563,	0,	0,	0,	0},
    {0,	0,	6.10800239326465e-08,	0,	0,	0},
    {0,	0,	0,	4.50015361182563e-05,	0,	0},
    {0,	0,	0,	0,	6.10800239326465e-09,	0},
    {0,	0,	0,	0,	0,	4.50015361182563e-06}
};
float Rz_p[2][2]={
    {0.000511966716115089,	0},
    {0,	0.000180664054506985}
};

float A_o1[2][2]={
    {0, -0.01},
    {0,     1}
};
float B_o1[2][1]={
    {0.01},
    {0}
};
float C_o1[1][2]={
    {1,	0}
};
float Rv_o1[2][2]={
    {2.20799504573120e-06,	0},
    {0,	2.20799504573120e-08}
};
float Rz_o1[1][1]={
    {0.00162561033143775}
};

float A_o2[1][1] = {
    {0}
};
float B_o2[1][1] = {
    {1}
};
float C_o2[1][1] = {
    {1}
};
float Rv_o2[1][1] = {
    {0.00162561033143775}
};
float Rz_o2[1][1] = {
    {8.27760425159963e-05}
};


KF pos_estimator(&A_p[0][0], &B_p[0][0], &C_p[0][0], &Rv_p[0][0], &Rz_p[0][0], zeros, 6, 2, 2);
KF ori_estimator1(&A_o1[0][0], &B_o1[0][0], &C_o1[0][0], &Rv_o1[0][0], &Rz_o1[0][0], zeros, 2, 1, 1);
KF ori_estimator2(&A_o2[0][0], &B_o2[0][0], &C_o2[0][0], &Rv_o2[0][0], &Rz_o2[0][0], zeros, 1, 1, 1);

void init_fusion(float pos0[], float ori0[], float head_offs){
    head_offset = head_offs;
    pos_estimator = KF(&A_p[0][0], &B_p[0][0], &C_p[0][0], &Rv_p[0][0], &Rz_p[0][0], pos0, 6, 2, 2);
    ori_estimator1 = KF(&A_o1[0][0], &B_o1[0][0], &C_o1[0][0], &Rv_o1[0][0], &Rz_o1[0][0], ori0, 2, 1, 1);
    ori_estimator2 = KF(&A_o2[0][0], &B_o2[0][0], &C_o2[0][0], &Rv_o2[0][0], &Rz_o2[0][0], ori0, 1, 1, 1);
}

void imu_corr(){
    // heading from magnetometer data
    imu_mag_raw[0] = imu_fusion->getMagX();
    imu_mag_raw[1] = imu_fusion->getMagY();

    mul(&mag_korr_trf[0][0], imu_mag_raw, imu_mag_korr, 2, 2, 1);
    imu_head_earth = atan2(imu_mag_korr[1], imu_mag_korr[0]);
    imu_head = imu_head_earth - head_offset;
   
    // prepare accelerometer data
    imu_acc_raw[0] = imu_fusion->getAccX();
    imu_acc_raw[1] = imu_fusion->getAccY();
    mul(&acc_trf[0][0], imu_acc_raw, imu_acc, 2, 2, 1);

    // prepare gyro data
    imu_gyro = (imu_fusion->getGyroZ() - gyro_bias) * 3.141592654/180;    // IMU measures in degrees but I prefer radians
}

void update_est(){
    // update sensor
    imu_fusion->update();

    imu_corr();

    // transform acceleromater measurement to global coordinates
    local2global_trf[0][0] = cos(state_vec[2]);
    local2global_trf[0][1] = -sin(state_vec[2]);
    local2global_trf[1][0] = sin(state_vec[2]);
    local2global_trf[1][1] = cos(state_vec[2]);

    mul(&local2global_trf[0][0], imu_acc, imu_acc_global, 2, 2, 1);

    //update the estimators
    pos_estimator.update(imu_acc, cam_vec);
    ori_estimator1.update(&imu_gyro, &imu_head);
    ori_estimator2.update(ori_estimator1.x_hat, cam_vec+2);

    // send the estimations to position control
    memcpy(meas_vec,pos_estimator.x_hat, 2*sizeof(float));
    memcpy(meas_vec+2, ori_estimator2.x_hat+1, sizeof(float));
}

void fusion_task(void * params){
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(fusion_cycle);

    while(true){
        update_est();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#endif