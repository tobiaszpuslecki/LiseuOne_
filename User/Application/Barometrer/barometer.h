/*
 * barometer.h
 *
 *  Created on: 30.10.2019
 *      Author: tobing
 */

#ifndef APPLICATION_BAROMETER_BAROMETER_H
#define APPLICATION_BAROMETER_BAROMETER_H

#include "i2c.h"
#include "MS5611/ms5611.h"
#include "Config/baro_kalman_config.h"

kalman_t k_pressure;
kalman_t k_temperature;
kalman_t k_altitude;


MS5611_result baroResult;
MS5611 bt_struct;
float pressureLevelZero;

uint32_t pressure;
uint16_t temperature;
float altitude;

uint8_t BarometerInit();
uint8_t barometerInitialized();
uint32_t getPressure();
float getAltitude(uint32_t pres);
uint16_t getTemperature();

void readBarometerValues();

void calculateAltitude();




void requestPressure();
void requestTemperature();


#endif /* APPLICATION_BAROMETER_BAROMETER_H */


