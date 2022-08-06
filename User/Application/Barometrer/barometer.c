/*
 * barometer.c
 *
 *  Created on: 30.10.2019
 *      Author: tobing
 */

#include "barometer.h"
//#include "Config/baro_kalman_config.h"


uint8_t BarometerInit()
{

	// enum with list of possible modes is in ms5611.h
  	bt_struct.initialized = 0;
	baroResult = MS5611_init(&hi2c1, &bt_struct, MS5611_ULTRA_HIGH_RES);
	if( MS5611_OK != baroResult )
	{
		return 1;
	}
	else
	{
		k_pressure.processNoise = BARO_PROCCESSNOISE;
		k_pressure.measurementNoise = BARO_MEASUREMENTNOISE;
		k_pressure.error = BARO_ERROR;
		k_pressure.gain = BARO_GAIN;
		k_pressure.value = 101300;

		k_temperature.processNoise = BARO_PROCCESSNOISE;
		k_temperature.measurementNoise = BARO_MEASUREMENTNOISE;
		k_temperature.error = BARO_ERROR;
		k_temperature.gain = BARO_GAIN;
		k_temperature.value = 2300;

		k_altitude.processNoise = BARO_PROCCESSNOISE;
		k_altitude.measurementNoise = BARO_MEASUREMENTNOISE;
		k_altitude.error = BARO_ERROR;
		k_altitude.gain = BARO_GAIN;
		k_altitude.value = 0;


		HAL_Delay(200);
		MS5611_requestTemperature();
		HAL_Delay(100);
		MS5611_getTemperature( &bt_struct );

		k_temperature.value = bt_struct.temperature;

		MS5611_requestPressure();
		HAL_Delay(100);
		MS5611_getPressure( &bt_struct );

		k_pressure.value = bt_struct.pressure;

		MS5611_setLevelZero( &bt_struct );
		pressureLevelZero = bt_struct.pressure;

		return 0;
	}
}

uint8_t barometerInitialized()
{
	if(MS5611_OK == baroResult)
		return 1;
	else
		return 0;
}

void requestPressure()
{
	MS5611_requestPressure();
}

void requestTemperature()
{
	MS5611_requestTemperature();
}

uint32_t getPressure()
{
	  MS5611_getPressure( &bt_struct );
	  //MS5611_requestPressure();

	  return bt_struct.pressure;
}

uint16_t getTemperature()
{
	  MS5611_getTemperature( &bt_struct );
	  //MS5611_requestPressure();

	  return bt_struct.temperature;
}

float getAltitude(uint32_t pres)
{
	return (BARO_CONST * (1.0 - pow(pres / pressureLevelZero, BARO_PWR_INDEX)));
}


#pragma GCC push_options
#pragma GCC optimize ("O3")
void delayUS(uint32_t us) {
	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
	volatile uint32_t start = DWT->CYCCNT;
	do  {
	} while(DWT->CYCCNT - start < cycles);
}
#pragma GCC pop_options

void readBarometerValues()
{
	// cannot be used in interrupt!!!
	// approx. time 23ms with 10 in delays
	// with MS5611_ULTRA_HIGH_RES flag, delays must be 8ms minimum
	MS5611_requestPressure();
	//HAL_Delay(8);
	delayUS(8000000);
	MS5611_getPressure( &bt_struct );
	MS5611_requestTemperature();
	//HAL_Delay(8);
	delayUS(8000000);
	MS5611_getTemperature( &bt_struct );

}

void calculateAltitude()
{
	bt_struct.altitude = getAltitude(bt_struct.pressure);
}






