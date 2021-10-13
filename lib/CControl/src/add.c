/*
 * mul.c
 *
 *  Created on: 13 oct 2021
 *      Author: Andras Izso
 */

#include "Functions.h"

/*
 * C = A+B
 * A [row*col]
 * B [row*col]
 */
void add(float A[], float B[], float C[], uint16_t row, uint16_t col){
    uint16_t i;
    uint16_t j;
    for(i = 0; i < row; i++){
        for(j = 0; j < col; j++){
            C[i*col + j] = A[i*col + j] + B[i*col + j];
        }
    }
}