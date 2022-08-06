/*
 * mpu.h
 *
 *  Created on: 17.12.2019
 *      Author: tobing
 */

#ifndef APPLICATION_MPU_MPU_H_
#define APPLICATION_MPU_MPU_H_

#include "MPU9250/MPU9250.h"
#include "Config/mpu_kalman_config.h"

kalman_t k_accX;
kalman_t k_accY;
kalman_t k_accZ;

kalman_t k_pitch;
kalman_t k_roll;


uint8_t MPUInit(uint16_t properMPUName);
void readAccelerometer(int16_t * destination);
void readGyroscope(int16_t * destination);
uint8_t MPUInited();



uint8_t MPUInitFlag;

#endif /* APPLICATION_MPU_MPU_H_ */
