/*
 * mul.c
 *
 *  Created on: 13 oct 2021
 *      Author: Andras Izso
 */

#include "Functions.h"

/*
 * A = B
 */
void copy(float dist[],const float orig[], uint16_t length){
    for(uint16_t i = 0; i < length; i++){
        dist[i] = orig[i];
    }
}