#ifndef SENSOR_CALIBRATION_H_INCLUDED
#define SENSOR_CALIBRATION_H_INCLUDED

#include "Tuareg_types.h"


/***************************************************************************************************************************************************
*   analog sensor calibration page
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Sensor_Calibration_t {

    U8 Version;

    F32 IAT_calib_M;
    F32 IAT_calib_N;
    U16 IAT_min_valid;
    U16 IAT_max_valid;

    F32 CLT_calib_M;
    F32 CLT_calib_N;
    U16 CLT_min_valid;
    U16 CLT_max_valid;

    F32 TPS_calib_M;
    F32 TPS_calib_N;
    U16 TPS_min_valid;
    U16 TPS_max_valid;

    F32 MAP_calib_M;
    F32 MAP_calib_N;
    U16 MAP_min_valid;
    U16 MAP_max_valid;

    F32 BARO_calib_M;
    F32 BARO_calib_N;
    U16 BARO_min_valid;
    U16 BARO_max_valid;

    F32 O2_calib_M;
    F32 O2_calib_N;
    U16 O2_min_valid;
    U16 O2_max_valid;

    F32 VBAT_calib_M;
    F32 VBAT_calib_N;
    U16 VBAT_min_valid;
    U16 VBAT_max_valid;

    F32 KNOCK_calib_M;
    F32 KNOCK_calib_N;
    U16 KNOCK_min_valid;
    U16 KNOCK_max_valid;

    F32 GEAR_calib_M;
    F32 GEAR_calib_N;
    U16 GEAR_min_valid;
    U16 GEAR_max_valid;

} Sensor_Calibration_t;

#define SENSOR_CALIBRATION_SIZE 109


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/
extern volatile Sensor_Calibration_t Sensor_Calibration;


exec_result_t load_Sensor_Calibration();
exec_result_t store_Sensor_Calibration();

void show_Sensor_Calibration(USART_TypeDef * Port);

exec_result_t modify_Sensor_Calibration(U32 Offset, U32 Value);

void send_Sensor_Calibration(USART_TypeDef * Port);

/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/


#endif // SENSOR_CALIBRATION_H_INCLUDED
