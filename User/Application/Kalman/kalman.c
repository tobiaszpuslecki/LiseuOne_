/*
 * kalman.c
 *
 *  Created on: 30.10.2019
 *      Author: tobing
 */

#include "kalman.h"


void kalmanUpdate(kalman_t *instance, float measurement)
{
   instance->error   = instance->error + instance->processNoise;
   instance->gain    = instance->error / (instance->error + instance->measurementNoise);
   instance->value  = instance->value + instance->gain * (measurement - instance->value);
   instance->error   = (1 - instance->gain) * instance->error;
}
