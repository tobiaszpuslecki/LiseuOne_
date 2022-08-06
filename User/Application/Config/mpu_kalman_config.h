/*
 * kalman_config.h
 *
 *  Created on: 30.10.2019
 *      Author: tobing
 */

#ifndef APPLICATION_CONFIG_MPU_KALMAN_CONFIG_H_
#define APPLICATION_CONFIG_MPU_KALMAN_CONFIG_H_

#include "Kalman/kalman.h"

//kalman_t k_accX = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 0};
//kalman_t k_accY = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 0};
//kalman_t k_accZ = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 0};
//
//kalman_t k_pitch = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 0};
//kalman_t k_roll = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 0};

#define MPU_PROCCESSNOISE 0.01
#define MPU_MEASUREMENTNOISE 0.25
#define MPU_ERROR 1
#define MPU_GAIN 0




#endif /* APPLICATION_CONFIG_MPU_KALMAN_CONFIG_H_ */
