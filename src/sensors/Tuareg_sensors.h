#ifndef TUAREG_SENSORS_H_INCLUDED
#define TUAREG_SENSORS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "analog_sensors.h"

#define SENSORS_REQUIRED_CALIBRATION_VERSION 4


/*
ASENSOR_VALIDITY_THRES of consecutive valid captures an analog sensor has to provide until he is considered valid

the counter will be initialized with ASENSOR_VALIDITY_FASTINIT to provide sensor data for startup
*/
#define ASENSOR_VALIDITY_THRES 150
#define ASENSOR_VALIDITY_FASTINIT 100


/**
default sensor values

if an analog sensor is not available, use these defaults
*/
#define MAP_DEFAULT_KPA 100.0
#define BARO_DEFAULT_KPA 101.3
#define TPS_DEFAULT_DEG 30.0
#define O2_DEFAULT_AFR 10.0
#define IAT_DEFAULT_C 20.0
#define CLT_DEFAULT_C 85.0
#define VBAT_DEFAULT_V 12.8
#define KNOCK_DEFAULT 0.0
#define GEAR_DEFAULT 0



void init_Sensors();


VF32 Tuareg_update_MAP_sensor();
VF32 Tuareg_update_BARO_sensor();
VF32 Tuareg_update_KNOCK_sensor();
VF32 Tuareg_update_VBAT_sensor();
VF32 Tuareg_update_CLT_sensor();
VF32 Tuareg_update_IAT_sensor();
VF32 Tuareg_update_TPS_sensor();
VF32 Tuareg_update_O2_sensor();
gears_t Tuareg_update_GEAR_sensor();

#endif // TUAREG_SENSORS_H_INCLUDED
