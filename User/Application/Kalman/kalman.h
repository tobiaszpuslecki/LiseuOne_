/*
 * kalman.h
 *
 *  Created on: 30.10.2019
 *      Author: tobing
 */

#ifndef APPLICATION_KALMAN_KALMAN_H_
#define APPLICATION_KALMAN_KALMAN_H_

typedef struct {
  float processNoise;
  float measurementNoise;
  float value;
  float error;
  float gain;
} kalman_t;

void kalmanUpdate(kalman_t *instance, float measurement);

#endif /* APPLICATION_KALMAN_KALMAN_H_ */
