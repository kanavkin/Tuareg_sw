#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "base_calc.h"
#include "table.h"
#include "eeprom.h"
#include "eeprom_layout.h"

#include "eeprom_layout.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "ignition_config.h"

///the Ignition Setup Page
volatile Ignition_Setup_t Ignition_Setup;

// Ignition Tables
volatile t3D_t ignAdvTable_TPS;
volatile t3D_t ignAdvTable_MAP;

volatile t2D_t ignDwellTable;


volatile U8 * const pIgnition_Setup_data= (volatile U8 *) &Ignition_Setup;
const U32 cIgnition_Setup_size= sizeof(Ignition_Setup);



/**
*
* reads ignition config (setup and tables) data from eeprom
*
*/
exec_result_t load_Ignition_Config()
{
    exec_result_t load_result;

    //bring up eeprom
    Eeprom_init();

    load_result= Eeprom_load_data(EEPROM_IGNITION_SETUP_BASE, pIgnition_Setup_data, cIgnition_Setup_size);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t3D_data(&(ignAdvTable_TPS.data), EEPROM_IGNITION_ADVTPS_BASE);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(ignDwellTable.data), EEPROM_IGNITION_DWELLTABLE_BASE);

   return load_result;
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_Ignition_Config()
{
    //ignition control module has all vital defaults compiled in


}


/***************************************************************************************************************************************************
*   Ignition Setup
***************************************************************************************************************************************************/


/**
*
* writes ignition config data to eeprom
*
*/
exec_result_t store_Ignition_Setup()
{
    return Eeprom_update_data(EEPROM_IGNITION_SETUP_BASE, pIgnition_Setup_data, cIgnition_Setup_size);
}


void show_Ignition_Setup(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nIgnition Config:");

    //U8 Version
    print(Port, "\r\nVersion: ");
    printf_U(Port, Ignition_Setup.Version, NO_PAD);

    //dynamic_min_rpm
    print(Port, "\r\ndynamic ignition function minimum rpm: ");
    printf_U(Port, Ignition_Setup.dynamic_min_rpm, NO_PAD);

    //dynamic_ignition_base_position
    print(Port, "\r\ndynamic ignition base position: ");
    printf_crkpos(Port, Ignition_Setup.dynamic_ignition_base_position);

    //U16 spark_duration_us
    print(Port, "\r\nspark duration (us): ");
    printf_U(Port, Ignition_Setup.spark_duration_us, NO_PAD);


    //cold_idle_cutoff_rpm
    print(Port, "\r\ncold idle cutoff (rpm): ");
    printf_U(Port, Ignition_Setup.cold_idle_cutoff_rpm, NO_PAD);

    //cold_idle_cutoff_CLT_K
    print(Port, "\r\ncold idle cutoff CLT (K): ");
    printf_U(Port, Ignition_Setup.cold_idle_cutoff_CLT_K, NO_PAD);

    //cold_idle_ignition_advance_deg
    print(Port, "\r\ncold idle ignition advance (deg): ");
    printf_U(Port, Ignition_Setup.cold_idle_ignition_advance_deg, NO_PAD);

    //cold_idle_dwell_target_us
    print(Port, "\r\ncold idle dwell target (us): ");
    printf_U(Port, Ignition_Setup.cold_idle_dwell_target_us, NO_PAD);

    //cranking_ignition_position
    print(Port, "\r\ncranking ignition position: ");
    printf_crkpos(Port, Ignition_Setup.cranking_ignition_position);

    //cranking_dwell_position
    print(Port, "\r\ncranking dwell position: ");
    printf_crkpos(Port, Ignition_Setup.cranking_dwell_position);

    //flags
    print(Port, "\r\nfeature enabled flags: dynamic-cranking-cold_idle-sequential-2 coils: ");

    UART_Tx(TS_PORT, (Ignition_Setup.flags.dynamic_controls_enabled? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Ignition_Setup.flags.cranking_controls_enabled? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Ignition_Setup.flags.cold_idle_enabled? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Ignition_Setup.flags.sequential_mode_enabled? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Ignition_Setup.flags.second_coil_installed? '1' :'0'));
}


/**
replace an ignition configuration value
*/
exec_result_t modify_Ignition_Setup(U32 Offset, U32 Value)
{
    if(Offset >= cIgnition_Setup_size)
    {
        return EXEC_ERROR;
    }

    *(pIgnition_Setup_data + Offset)= (U8) Value;

    return EXEC_OK;

}

/**
this function implements the TS interface binary config page read command for Ignition Config
*/
void send_Ignition_Setup(USART_TypeDef * Port)
{
    UART_send_data(Port, pIgnition_Setup_data, cIgnition_Setup_size);
}



/***************************************************************************************************************************************************
*   Ignition Advance Table (TPS)
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> TPS angle in ° (no offset, no scaling)
* z-Axis -> advance angle in ° BTDC (no offset, no scaling)
***************************************************************************************************************************************************/


exec_result_t store_ignAdvTable_TPS()
{
    return store_t3D_data(&(ignAdvTable_TPS.data), EEPROM_IGNITION_ADVTPS_BASE);
}


void show_ignAdvTable_TPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nIgnition Advance Table (TPS):\r\n");

    show_t3D_data(TS_PORT, &(ignAdvTable_TPS.data));
}


exec_result_t modify_ignAdvTable_TPS(U32 Offset, U32 Value)
{
    //modify_t3D_data provides offset range check!
    return modify_t3D_data(&(ignAdvTable_TPS.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for ignAdvTable_TPS
*/
void send_ignAdvTable_TPS(USART_TypeDef * Port)
{
    send_t3D_data(Port, &(ignAdvTable_TPS.data));
}


VU32 getValue_ignAdvTable_TPS(VU32 Rpm, VF32 TPS)
{
    return (VU32) getValue_t3D(&ignAdvTable_TPS, Rpm, TPS);
}


/***************************************************************************************************************************************************
*   Ignition Dwell time table - ignDwellTable
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> Dwell time in us (no offset, table values are in 48 us increments)
***************************************************************************************************************************************************/
exec_result_t store_ignDwellTable()
{
    return store_t2D_data(&(ignDwellTable.data), EEPROM_IGNITION_DWELLTABLE_BASE);
}


void show_ignDwellTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nDwell table (x48 us):\r\n");

    show_t2D_data(TS_PORT, &(ignDwellTable.data));
}


exec_result_t modify_ignDwellTable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(ignDwellTable.data), Offset, Value);
}

/**
this function implements the TS interface binary config page read command for ignDwellTable
*/
void send_ignDwellTable(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(ignDwellTable.data));
}

/**
returns the dwell time in us
*/
VU32 getValue_ignDwellTable(VU32 Rpm)
{
    //ignDwellTable stores the Dwell time in 48 us increments
    return (VU32) 48 * getValue_t2D(&ignDwellTable, Rpm);
}
