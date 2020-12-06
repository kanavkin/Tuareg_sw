#ifndef SENSOR_CALIBRATION_H_INCLUDED
#define SENSOR_CALIBRATION_H_INCLUDED

#include "Tuareg_types.h"


/***************************************************************************************************************************************************
*   analog sensor calibration page
***************************************************************************************************************************************************/
typedef struct _Sensor_Calibration_t {

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


/***************************************************************************************************************************************************
*   data offset constants for modification via CLI
***************************************************************************************************************************************************/
typedef enum {

    SENSOR_CALIB_CLI_IAT_M,
    SENSOR_CALIB_CLI_IAT_N,
    SENSOR_CALIB_CLI_CLT_M,
    SENSOR_CALIB_CLI_CLT_N,
    SENSOR_CALIB_CLI_TPS_M,
    SENSOR_CALIB_CLI_TPS_N,
    SENSOR_CALIB_CLI_MAP_M,
    SENSOR_CALIB_CLI_MAP_N,
    SENSOR_CALIB_CLI_BARO_M,
    SENSOR_CALIB_CLI_BARO_N,
    SENSOR_CALIB_CLI_O2_M,
    SENSOR_CALIB_CLI_O2_N,
    SENSOR_CALIB_CLI_VBAT_M,
    SENSOR_CALIB_CLI_VBAT_N,
    SENSOR_CALIB_CLI_KNOCK_M,
    SENSOR_CALIB_CLI_KNOCK_N


} Sensor_Calibration_CLI_offset;


/***************************************************************************************************************************************************
*   data offsets for for eeprom layout based on the stored data size
***************************************************************************************************************************************************/
typedef enum {

    SENSOR_CALIB_EE_VERSION =0,     // 1 byte

    SENSOR_CALIB_EE_IAT_M =1,       // 4 bytes 1..3
    SENSOR_CALIB_EE_IAT_N =4,       // 4 bytes 4..7
    SENSOR_CALIB_EE_CLT_M =8,       // 4 bytes 8..11
    SENSOR_CALIB_EE_CLT_N =12,      // 4 bytes 12..15
    SENSOR_CALIB_EE_TPS_M =16,      // 4 bytes 16..19
    SENSOR_CALIB_EE_TPS_N =20,      // 4 bytes 20..23
    SENSOR_CALIB_EE_MAP_M =24,      // 4 bytes 24..27
    SENSOR_CALIB_EE_MAP_N =28,      // 4 bytes 28..31
    SENSOR_CALIB_EE_BARO_M =32,     // 4 bytes 32..35
    SENSOR_CALIB_EE_BARO_N =36,     // 4 bytes 36..39
    SENSOR_CALIB_EE_O2_M =40,       // 4 bytes 40..43
    SENSOR_CALIB_EE_O2_N =44,       // 4 bytes 44..47
    SENSOR_CALIB_EE_VBAT_M =48,     // 4 bytes 48..51
    SENSOR_CALIB_EE_VBAT_N =52,     // 4 bytes 52..55
    SENSOR_CALIB_EE_KNOCK_M =56,    // 4 bytes 56..59
    SENSOR_CALIB_EE_KNOCK_N =60,    // 4 bytes 60..63

    SENSOR_CALIB_EE_NEXT_FREE_OFFSET =64    // next free offset after the page

} Sensor_Calibration_Eeprom_offset;


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/
extern volatile Sensor_Calibration_t Sensor_Calibration;


exec_result_t load_Sensor_Calibration();
exec_result_t write_Sensor_Calibration();

void show_Sensor_Calibration(USART_TypeDef * Port);

exec_result_t modify_Sensor_Calibration(U32 Offset, U32 Value);


/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/


#endif // SENSOR_CALIBRATION_H_INCLUDED
