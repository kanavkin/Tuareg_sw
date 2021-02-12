

#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_pages.h"
#include "config_tables.h"
#include "sensor_calibration.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"


///the sensor calibration page
volatile Sensor_Calibration_t Sensor_Calibration;

volatile U8 * const pSensor_Calibration_data= (volatile U8 *) &Sensor_Calibration;
const U32 cSensor_Calibration_size= sizeof(Sensor_Calibration);


/**
*
* reads sensor calibration data from eeprom
*
*/
exec_result_t load_Sensor_Calibration()
{
   return Eeprom_load_data(EEPROM_SENSOR_CALIBRATION_BASE, pSensor_Calibration_data, cSensor_Calibration_size);
}


/**
*
* writes sensor calibration data to eeprom
*
*/
exec_result_t store_Sensor_Calibration()
{
    return Eeprom_update_data(EEPROM_SENSOR_CALIBRATION_BASE, pSensor_Calibration_data, cSensor_Calibration_size);
}



void show_Sensor_Calibration(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nSensor Calibration:");

    /*
    Version
    */
    print(Port, "\r\nVersion: ");
    printf_U(Port, Sensor_Calibration.Version, NO_PAD | NO_TRAIL);

    /*
    IAT
    */
    print(Port, "\r\nIAT M N: ");
    printf_F32(Port, Sensor_Calibration.IAT_calib_M);
    printf_F32(Port, Sensor_Calibration.IAT_calib_N);

    /*
    CLT
    */
    print(Port, "\r\nCLT M N: ");
    printf_F32(Port, Sensor_Calibration.CLT_calib_M);
    printf_F32(Port, Sensor_Calibration.CLT_calib_N);

    /*
    TPS
    */
    print(Port, "\r\nTPS M N: ");
    printf_F32(Port, Sensor_Calibration.TPS_calib_M);
    printf_F32(Port, Sensor_Calibration.TPS_calib_N);

    /*
    MAP
    */
    print(Port, "\r\nMAP M N: ");
    printf_F32(Port, Sensor_Calibration.MAP_calib_M);
    printf_F32(Port, Sensor_Calibration.MAP_calib_N);

    /*
    BARO
    */
    print(Port, "\r\nBARO M N: ");
    printf_F32(Port, Sensor_Calibration.BARO_calib_M);
    printf_F32(Port, Sensor_Calibration.BARO_calib_N);

    /*
    O2
    */
    print(Port, "\r\nO2 M N: ");
    printf_F32(Port, Sensor_Calibration.O2_calib_M);
    printf_F32(Port, Sensor_Calibration.O2_calib_N);

    /*
    VBAT
    */
    print(Port, "\r\nVBAT M N: ");
    printf_F32(Port, Sensor_Calibration.VBAT_calib_M);
    printf_F32(Port, Sensor_Calibration.VBAT_calib_N);

    /*
    KNOCK
    */
    print(Port, "\r\nKNOCK M N: ");
    printf_F32(Port, Sensor_Calibration.KNOCK_calib_M);
    printf_F32(Port, Sensor_Calibration.KNOCK_calib_N);

}


/**
replace a calibration value
*/
exec_result_t modify_Sensor_Calibration(U32 Offset, U32 Value)
{
    if(Offset >= cSensor_Calibration_size)
    {
        return EXEC_ERROR;
    }

    *(pSensor_Calibration_data + Offset)= (U8) Value;

    return EXEC_OK;
}


/**
this function implements the TS interface binary config page read command for Sensor Calibration
*/
void send_Sensor_Calibration(USART_TypeDef * Port)
{
   UART_send_data(Port, pSensor_Calibration_data, cSensor_Calibration_size);
}
