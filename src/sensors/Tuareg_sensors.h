#ifndef TUAREG_SENSORS_H_INCLUDED
#define TUAREG_SENSORS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "sensors.h"

#define SENSORS_REQUIRED_CALIBRATION_VERSION 2

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
