/*
 * mpu.c
 *
 *  Created on: 17.12.2019
 *      Author: tobing
 */

#include "mpu.h"
//#include "Config/mpu_kalman_config.h"


uint8_t MPUInit(uint16_t properMPUName)
{
	// 	Ascale Gscale Mscale may be configured in mpu9250.c

	 //uint16_t properMPUName = 0x71;
	resetMPU9250();
	initMPU9250();
	uint8_t mpuInitRes = readMpuName();

	if(mpuInitRes == properMPUName)
	{
		k_accX.processNoise = MPU_PROCCESSNOISE;
		k_accX.measurementNoise = MPU_MEASUREMENTNOISE;
		k_accX.error = MPU_ERROR;
		k_accX.gain = MPU_GAIN;
		k_accX.value = 0;

		k_accY.processNoise = MPU_PROCCESSNOISE;
		k_accY.measurementNoise = MPU_MEASUREMENTNOISE;
		k_accY.error = MPU_ERROR;
		k_accY.gain = MPU_GAIN;
		k_accY.value = 0;

		k_accZ.processNoise = MPU_PROCCESSNOISE;
		k_accZ.measurementNoise = MPU_MEASUREMENTNOISE;
		k_accZ.error = MPU_ERROR;
		k_accZ.gain = MPU_GAIN;
		k_accZ.value = 0;

		k_pitch.processNoise = MPU_PROCCESSNOISE;
		k_pitch.measurementNoise = MPU_MEASUREMENTNOISE;
		k_pitch.error = MPU_ERROR;
		k_pitch.gain = MPU_GAIN;
		k_pitch.value = 0;

		k_roll.processNoise = MPU_PROCCESSNOISE;
		k_roll.measurementNoise = MPU_MEASUREMENTNOISE;
		k_roll.error = MPU_ERROR;
		k_roll.gain = MPU_GAIN;
		k_roll.value = 0;


		int16_t acc[3];
		readAccelData(&acc);

		k_accX.value = acc[0];
		k_accY.value = acc[1];
		k_accZ.value = acc[2];

		MPUInitFlag=1;
		return 0;

	}
	else
	{
		MPUInitFlag = 0;
		return 1;
	}
}

uint8_t MPUInited()
{
	if(MPUInitFlag)
		return 1;
	else
		return 0;
}

void readAccelerometer(int16_t * destination)
{
	readAccelData(destination);
}
void readGyroscope(int16_t * destination)
{
	readGyroData(destination);
}


