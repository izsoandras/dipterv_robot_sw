/*
 * mul.c
 *
 *  Created on: 13 oct 2021
 *      Author: Andras Izso
 */

#include "Functions.h"

/*
 * C = A-B
 * A [row*col]
 * B [row*col]
 */
void sub(float A[], float B[], float C[], uint16_t length){
    for(uint16_t i=0; i < length; i++){
        C[i] = A[i] - B[i];
    }
}