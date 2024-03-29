#ifndef MYKALMAN_H
#define MYKALMAN_H
#include "Functions.h"
#include "esp_log.h"

float zeros[] = {0,0,0,0,0,0};

class KF{
    public:
        float* x_hat;   // corrected, estimated state
        float* x_over;  // predicted state
        float* A;       // state transition matrix
        float* B;       // control matrix
        float* C;       // observation matrix
        float* Rv;      // Process noise covariance matrix
        float* Rz;      // Measurement covariance matrix
        float* S;       // Estimate covariance matrix (uncertainity)
        float* G;       // Kalman Gain
        float* M;       // Extrapolated (predicted) covariance matrix

        uint16_t dimX;
        uint16_t dimU;
        uint16_t dimY;

        float* Ctrans;
        float* Atrans;
        float* Yprev;

        float* dimX_temp;
        float* dimXX_temp;

        
        float* dimXY_temp;
        float* dimYY_temp;
        float* dimY_temp;
        bool isNewY = true;

    KF(const float A[],const float B[],const float C[],const float Rv[],const float Rz[], const float x0[], const uint16_t dimX,const uint16_t dimU,const uint16_t dimY);
    void update(float* u, float* y);
};

KF::KF(const float A[],const float B[],const float C[],const float Rv[],const float Rz[], const float x0[],const uint16_t dimX,const uint16_t dimU,const uint16_t dimY){
    x_hat = new float[dimX];
    copy(x_hat, x0, dimX);
    x_over = new float[dimX];

    this->A = new float[dimX*dimX];
    copy(this->A, A, dimX*dimX);

    this->B = new float[dimX*dimU];
    copy(this->B, B, dimX*dimU);

    this->C = new float[dimY*dimX];
    copy(this->C, C, dimY*dimX);

    this->Rv = new float[dimX*dimX];
    copy(this->Rv, Rv, dimX*dimX);

    this->Rz = new float[dimY*dimY];
    copy(this->Rz, Rz, dimY*dimY);

    this->S = new float[dimX*dimX];
    this->M = new float[dimX*dimX];
    this->G = new float[dimX*dimY];

    Ctrans = new float[dimX*dimY];
    copy(this->Ctrans, C, dimY*dimX);
    tran(Ctrans, dimY, dimX);

    Atrans = new float[dimX*dimX];
    copy(this->Atrans, A, dimX*dimX);
    tran(Atrans, dimX, dimX);

    Yprev = new float[dimY];

    this->dimX = dimX;
    this->dimU = dimU;
    this->dimY = dimY;

    dimX_temp = new float[dimX];
    dimXX_temp = new float[dimX*dimX];
    dimXY_temp = new float[dimX*dimY];
    dimYY_temp = new float[dimY*dimY];
    dimY_temp = new float[dimY];
}

void KF::update(float* u, float* y){

    // Prediction
    // x_over =  A*x_hat + B*u
    mul(A, x_hat, x_over, dimX, dimX, 1);
    mul(B, u, dimX_temp, dimX, dimU, 1);
    add(x_over, dimX_temp, x_over, dimX, 1);

    // M = A*S*A' + Rv
    mul(S, Atrans, dimXX_temp, dimX, dimX, dimX);
    mul(A,dimXX_temp,M, dimX, dimX, dimX);
    add(M, Rv, M, dimX, dimX);

    for(uint8_t i = 0; i < dimY && isNewY; i++){
        isNewY = Yprev[i] == y[i];
    }

    // Update
    if(isNewY){
        // Serial.print("New");

        // G = M*C'*pinv(C*M*C' + Rz)
        mul(M,Ctrans, dimXY_temp, dimX,dimX, dimY);
        mul(C,dimXY_temp, dimYY_temp, dimY,dimX, dimY);
        add(dimYY_temp, Rz, dimYY_temp, dimY, dimY);
        pinv(dimYY_temp, dimY, dimY);
        mul(Ctrans,dimYY_temp, dimXY_temp, dimX,dimY,dimY);
        mul(M,dimXY_temp, G, dimX,dimX, dimY);
        
        // S= M - G*C*M
        mul(C,M, dimXY_temp, dimY, dimX, dimX); // Taking advantage of that a row continous dimX*dimY matrix has the same size as a dimY*dimX matrix
        mul(G,dimXY_temp, dimXX_temp, dimX,dimY, dimX);
        sub(M, dimXX_temp, S, dimX*dimX);


        // x_hat = x_over + G*(y - C*x_over)
        mul(C,x_over, dimY_temp, dimY, dimX, 1);
        sub(y, dimY_temp, dimY_temp, dimY);
        mul(G, dimY_temp, dimX_temp, dimX, dimY, 1);
        add(x_over, dimX_temp, x_hat, 3, 1);

    }else{
        copy(S, M, dimX*dimX);
        copy(x_hat, x_over, dimX);
    }
}

#endif