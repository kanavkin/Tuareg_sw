

#include "table.h"
#include "eeprom.h"
#include "eeprom_layout.h"
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


volatile t2D_t InvTableCLT;


/**
*
* reads sensor calibration data from eeprom
*
*/
exec_result_t load_Sensor_Calibration()
{
    exec_result_t load_result;

    //bring up eeprom
    Eeprom_init();

    load_result= Eeprom_load_data(EEPROM_SENSOR_CALIBRATION_BASE, pSensor_Calibration_data, cSensor_Calibration_size);

    ASSERT_EXEC_OK(load_result);

    //load_result= load_t2D_data(&(InvTableCLT.data), EEPROM_SENSOR_INVTABLECLT_BASE);
    load_result= load_t2D(&InvTableCLT, EEPROM_SENSOR_INVTABLECLT_BASE);

    return load_result;
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



/***************************************************************************************************************************************************
*   Sensor Calibration CLT inverse transfer function - lookup Table - InvTableCLT
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> TPS angle in ° (no offset, no scaling)
*
***************************************************************************************************************************************************/

const F32 cInvCLTMult= 2.0;

exec_result_t store_InvTableCLT()
{
    //return store_t2D_data(&(InvTableCLT.data), EEPROM_SENSOR_INVTABLECLT_BASE);
    return store_t2D(&InvTableCLT, EEPROM_SENSOR_INVTABLECLT_BASE);
}


void show_InvTableCLT(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nSensor Calibration CLT lookup Table (x2 K)\r\n");

    show_t2D_data(TS_PORT, &(InvTableCLT.data));
}


exec_result_t modify_InvTableCLT(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(InvTableCLT.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for InvTableCLT
*/
void send_InvTableCLT(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(InvTableCLT.data));
}


/**
returns the coolant temperature in K
*/
VF32 getValue_InvTableCLT(VU32 Raw)
{
    return cInvCLTMult * getValue_t2D(&InvTableCLT, Raw);
}
