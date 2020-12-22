#ifndef SENSOR_CALIBRATION_H_INCLUDED
#define SENSOR_CALIBRATION_H_INCLUDED

#include "Tuareg_types.h"


/***************************************************************************************************************************************************
*   analog sensor calibration page
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Sensor_Calibration_t {

    F32 IAT_calib_M;
    F32 IAT_calib_N;

    F32 CLT_calib_M;
    F32 CLT_calib_N;

    F32 TPS_calib_M;
    F32 TPS_calib_N;

    F32 MAP_calib_M;
    F32 MAP_calib_N;

    F32 BARO_calib_M;
    F32 BARO_calib_N;

    F32 O2_calib_M;
    F32 O2_calib_N;

    F32 VBAT_calib_M;
    F32 VBAT_calib_N;

    F32 KNOCK_calib_M;
    F32 KNOCK_calib_N;

    U8 Version;

} Sensor_Calibration_t;

#define SENSOR_CALIBRATION_SIZE 65


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
