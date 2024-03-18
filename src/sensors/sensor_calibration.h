#ifndef SENSOR_CALIBRATION_H_INCLUDED
#define SENSOR_CALIBRATION_H_INCLUDED

#include "Tuareg_types.h"
#include "analog_sensors.h"

/***************************************************************************************************************************************************
*   analog sensor calibration page
*
*   the analog sensor calibration can not be packed because of the sensor parameters, which are passed by pointers, leading to unaligned accesses
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Sensor_Calibration_t {
//typedef struct _Sensor_Calibration_t {

    U8 Version;

    /*
    holds the parameter sets for analog sensors
    their TS config item order shall be as defined in asensors_t
    */
    volatile asensor_parameters_t Asensor_Parameters[ASENSOR_COUNT];

} Sensor_Calibration_t;

//#define SENSOR_CALIBRATION_SIZE 181


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/
extern volatile Sensor_Calibration_t Sensor_Calibration;

exec_result_t load_Sensor_Calibration();


exec_result_t store_Sensor_Calibration();
void show_Sensor_Calibration(USART_TypeDef * Port);
exec_result_t modify_Sensor_Calibration(U32 Offset, U32 Value);
void send_Sensor_Calibration(USART_TypeDef * Port);


exec_result_t store_InvTableCLT();
void show_InvTableCLT(USART_TypeDef * Port);
exec_result_t modify_InvTableCLT(U32 Offset, U32 Value);
void send_InvTableCLT(USART_TypeDef * Port);
F32 getValue_InvTableCLT(VU32 Raw);




/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/


#endif // SENSOR_CALIBRATION_H_INCLUDED
