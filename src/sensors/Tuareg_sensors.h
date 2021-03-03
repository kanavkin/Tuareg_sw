#ifndef TUAREG_SENSORS_H_INCLUDED
#define TUAREG_SENSORS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "sensors.h"

#define SENSORS_REQUIRED_CALIBRATION_VERSION 2


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
#define MAP_DEFAULT_KPA 100
#define BARO_DEFAULT_KPA 100
#define TPS_DEFAULT_DEG 45
#define O2_DEFAULT_AFR 14.5
#define IAT_DEFAULT_C 20
#define CLT_DEFAULT_C 85
#define VBAT_DEFAULT_V 14
#define KNOCK_DEFAULT 0
#define GEAR_DEFAULT 0



volatile sensor_interface_t * init_Sensors();


VF32 Tuareg_update_MAP_sensor();
gears_t Tuareg_update_GEAR_sensor();
VF32 Tuareg_update_BARO_sensor();
VF32 Tuareg_update_KNOCK_sensor();
VF32 Tuareg_update_VBAT_sensor();
VF32 Tuareg_update_CLT_sensor();
VF32 Tuareg_update_IAT_sensor();
VF32 Tuareg_update_TPS_sensor();
VF32 Tuareg_update_ddt_TPS();
VF32 Tuareg_update_O2_sensor();
VF32 Tuareg_update_MAP_sensor();

#endif // TUAREG_SENSORS_H_INCLUDED
