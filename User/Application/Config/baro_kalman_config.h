/*
 * kalman_config.h
 *
 *  Created on: 30.10.2019
 *      Author: tobing
 */

#ifndef APPLICATION_CONFIG_BARO_KALMAN_CONFIG_H_
#define APPLICATION_CONFIG_BARO_KALMAN_CONFIG_H_

#include "Kalman/kalman.h"

//kalman_t k_pressure = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 101300};
//kalman_t k_temperature = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 2300};
//kalman_t k_altitude = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 0};

#define BARO_PROCCESSNOISE 0.01
#define BARO_MEASUREMENTNOISE 0.25
#define BARO_ERROR 1
#define BARO_GAIN 0



#endif /* APPLICATION_CONFIG_BARO_KALMAN_CONFIG_H_ */
