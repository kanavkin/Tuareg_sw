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

#include "fueling_config.h"
#include "Fueling_syslog_locations.h"

///the Fueling Setup Page
volatile Fueling_Setup_t Fueling_Setup;

volatile U8 * const pFueling_Setup_data= (volatile U8 *) &Fueling_Setup;
const U32 cFueling_Setup_size= sizeof(Fueling_Setup);


// Fueling Tables
volatile table_t AccelCompTableTPS;
volatile table_t AccelCompTableMAP;
volatile table_t WarmUpCompTable;
volatile table_t InjectorTimingTable;
volatile table_t CrankingFuelTable;
volatile table_t BAROtable;
volatile map_t ChargeTempMap;


/**
* reads Fueling config (setup and tables) data from eeprom
*
*/
exec_result_t load_Fueling_Config()
{
    //bring up eeprom first
    Eeprom_init();

    ASSERT_EXEC_OK( load_Fueling_Setup() );

    ASSERT_EXEC_OK( load_AccelCompTableTPS() );
    ASSERT_EXEC_OK( load_AccelCompTableMAP() );

    ASSERT_EXEC_OK( load_WarmUpCompTable() );

    ASSERT_EXEC_OK( load_InjectorTimingTable() );

    ASSERT_EXEC_OK( load_CrankingFuelTable() );

    ASSERT_EXEC_OK( load_BAROtable() );

    ASSERT_EXEC_OK( load_ChargeTempMap() );

    return EXEC_OK;
}


/***************************************************************************************************************************************************
*   Fueling Setup
***************************************************************************************************************************************************/


exec_result_t load_Fueling_Setup()
{
    return Eeprom_load_data(EEPROM_FUELING_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);
}

exec_result_t store_Fueling_Setup()
{
    return Eeprom_update_data(EEPROM_FUELING_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);
}


void show_Fueling_Setup(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling Config:");

    //U8 Version
    print(Port, "\r\nVersion: ");
    printf_U(Port, Fueling_Setup.Version, NO_PAD);


    //U16 cylinder_volume_ccm
    print(Port, "\r\ncylinder volume (ccm):");
    printf_U(Port, Fueling_Setup.cylinder_volume_ccm, NO_PAD);


    //injection_reference_pos
    print(Port, "\r\ndefault injection reference position: ");
    printf_crkpos(Port, Fueling_Setup.injection_reference_pos);


    //U16 injector1_rate_mgps
    print(Port, "\r\ninjector #1 flow rate (mg/s):");
    printf_U(Port, Fueling_Setup.injector1_rate_mgps, NO_PAD);

    //U16 injector2_rate_mgps
    print(Port, "\r\ninjector #2 flow rate (mg/s):");
    printf_U(Port, Fueling_Setup.injector2_rate_mgps, NO_PAD);

    //U8 max_injector_duty_cycle_pct
    print(Port, "\r\nmaximum injector duty cycle (%):");
    printf_U(Port, Fueling_Setup.max_injector_duty_cycle_pct, NO_PAD);


    //F32 accel_comp_thres_MAP_low
    print(Port, "\r\nacceleration compensation turn on MAP rate -low- (kPa/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres_MAP_low);

    //F32 accel_comp_thres_MAP_high
    print(Port, "\r\nacceleration compensation turn on MAP rate -high- (kPa/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres_MAP_high);

    //F32 accel_comp_thres_TPS_low
    print(Port, "\r\nacceleration compensation turn on TPS rate -low- (deg/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres_TPS_low);

    //F32 accel_comp_thres_TPS_high
    print(Port, "\r\nacceleration compensation turn on TPS rate -high- (deg/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres_TPS_high);

    //U16 accel_comp_thres_rpm
    print(Port, "\r\nMAP/TPS rate switching threshold (rpm):");
    printf_U(Port, Fueling_Setup.accel_comp_thres_rpm, NO_PAD);

    //F32 decel_comp_thres_TPS
    print(Port, "\r\ndeceleration compensation turn on TPS rate (deg/s):");
    printf_F32(Port, Fueling_Setup.decel_comp_thres_TPS);

    //F32 decel_comp_thres_MAP
    print(Port, "\r\ndeceleration compensation turn on MAP rate (kPa/s):");
    printf_F32(Port, Fueling_Setup.decel_comp_thres_MAP);

    //F32 accel_comp_taper_factor
    print(Port, "\r\nacceleration compensation taper factor (<1!):");
    printf_F32(Port, Fueling_Setup.accel_comp_taper_factor);

    //U16 accel_comp_scaling_thres_rpm
    print(Port, "\r\nacceleration compensation rpm scaling begin (rpm):");
    printf_U(Port, Fueling_Setup.accel_comp_scaling_thres_rpm, NO_PAD);

    //U16 accel_comp_scaling_max_rpm
    print(Port, "\r\nacceleration compensation rpm scaling maximum (rpm):");
    printf_U(Port, Fueling_Setup.accel_comp_scaling_thres_rpm, NO_PAD);

    //U8 cold_accel_pct
    print(Port, "\r\ncold engine acceleration compensation bonus fuel (%):");
    printf_U(Port, Fueling_Setup.cold_accel_pct, NO_PAD);

    //U16 decel_comp_ug
    print(Port, "\r\ndeceleration compensation (ug):");
    printf_U(Port, Fueling_Setup.decel_comp_ug, NO_PAD);

    //U8 accel_comp_cycles
    print(Port, "\r\nacceleration compensation duration (events):");
    printf_U(Port, Fueling_Setup.accel_comp_cycles, NO_PAD);

    //U8 decel_comp_cycles
    print(Port, "\r\ndeceleration compensation duration (events):");
    printf_U(Port, Fueling_Setup.decel_comp_cycles, NO_PAD);

    //U8 accel_comp_taper_thres
    print(Port, "\r\nacceleration compensation taper begin (remaining events):");
    printf_U(Port, Fueling_Setup.accel_comp_taper_thres, NO_PAD);


    //U8 afterstart_comp_pct
    print(Port, "\r\nafter start enrichment (%):");
    printf_U(Port, Fueling_Setup.afterstart_comp_pct, NO_PAD);

    //U8 afterstart_comp_cycles
    print(Port, "\r\nafter start enrichment duration (events):");
    printf_F32(Port, Fueling_Setup.afterstart_comp_cycles);

    //U16 afterstart_thres_K
    print(Port, "\r\nafter start enrichment CLT threshold (K):");
    printf_U(Port, Fueling_Setup.afterstart_thres_K, NO_PAD);


    //dry cranking
    print(Port, "\r\ndry cranking TPS threshold:");
    printf_U(Port, Fueling_Setup.dry_cranking_TPS_thres, NO_PAD);


    //features
    print(Port, "\r\nfeature enabled features: AE-legacyAE-sequential-WUE-ASE-seq-dry: ");

    UART_Tx(Port, (Fueling_Setup.features.load_transient_comp_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.legacy_AE_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.sequential_mode_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.WUE_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.ASE_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.dry_cranking_enabled? '1' :'0'));

}


/**
replace an Fueling configuration value
*/
exec_result_t modify_Fueling_Setup(U32 Offset, U32 Value)
{
    if(Offset >= cFueling_Setup_size)
    {
        return EXEC_ERROR;
    }

    *(pFueling_Setup_data + Offset)= (U8) Value;

    return EXEC_OK;

}

/**
this function implements the TS interface binary config page read command for Fueling Config
*/
void send_Fueling_Setup(USART_TypeDef * Port)
{
    UART_send_data(Port, pFueling_Setup_data, cFueling_Setup_size);
}



/***************************************************************************************************************************************************
*   Fueling acceleration compensation table - AccelCompTableTPS
*
* x-Axis -> TPS change rate in °/s (no offset, no scaling)
* y-Axis -> Acceleration compensation value in ug (no offset, no scaling)
***************************************************************************************************************************************************/

exec_result_t load_AccelCompTableTPS()
{
    return load_table(&AccelCompTableTPS, EEPROM_FUELING_ACCELCOMPTPS_BASE);
}

exec_result_t store_AccelCompTableTPS()
{
    return store_table(&AccelCompTableTPS, EEPROM_FUELING_ACCELCOMPTPS_BASE);
}


void show_AccelCompTableTPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling acceleration compensation table (TPS):\r\n");

    show_table(Port, &AccelCompTableTPS);
}


exec_result_t modify_AccelCompTableTPS(U32 Offset, U32 Value)
{
    //modify_table provides offset range check!
    return modify_table(&AccelCompTableTPS, Offset, Value);
}


/**
this function implements the TS interface binary config page read command for AccelCompTableTPS
*/
void send_AccelCompTableTPS(USART_TypeDef * Port)
{
    send_table(Port, &AccelCompTableTPS);
}


/**
returns the acceleration compensation value in ug
*/
F32 getValue_AccelCompTableTPS(F32 Ddt_TPS)
{
    return getValue_table(&AccelCompTableTPS, Ddt_TPS);
}


/***************************************************************************************************************************************************
*   Fueling acceleration compensation table - AccelCompTableMAP
*
* x-Axis -> TPS change rate in °/s (no offset, no scaling)
* y-Axis -> Acceleration compensation value in ug (no offset, no scaling)
***************************************************************************************************************************************************/

exec_result_t load_AccelCompTableMAP()
{
    return load_table(&AccelCompTableMAP, EEPROM_FUELING_ACCELCOMPMAP_BASE);
}

exec_result_t store_AccelCompTableMAP()
{
    return store_table(&AccelCompTableMAP, EEPROM_FUELING_ACCELCOMPMAP_BASE);
}


void show_AccelCompTableMAP(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling acceleration compensation table (MAP):\r\n");

    show_table(Port, &AccelCompTableMAP);
}


exec_result_t modify_AccelCompTableMAP(U32 Offset, U32 Value)
{
    return modify_table(&AccelCompTableMAP, Offset, Value);
}


/**
this function implements the TS interface binary config page read command for AccelCompTableMAP
*/
void send_AccelCompTableMAP(USART_TypeDef * Port)
{
    send_table(Port, &AccelCompTableMAP);
}


/**
returns the acceleration compensation value in ug
*/
F32 getValue_AccelCompTableMAP(F32 Ddt_MAP)
{
    return getValue_table(&AccelCompTableMAP, Ddt_MAP);
}


/***************************************************************************************************************************************************
*   Fueling Warm up Enrichment compensation table - WarmUpCompTable
*
* x-Axis -> CLT in K (no offset, no scaling)
* y-Axis -> Warm up compensation value in % (no offset, scaling * 10)
***************************************************************************************************************************************************/

const F32 cWarmUpDivider= 10.0;

exec_result_t load_WarmUpCompTable()
{
    return load_table(&WarmUpCompTable, EEPROM_FUELING_WARMUPCOMP_BASE);
}

exec_result_t store_WarmUpCompTable()
{
    return store_table(&WarmUpCompTable, EEPROM_FUELING_WARMUPCOMP_BASE);
}


void show_WarmUpCompTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nWarm up Enrichment table (%):\r\n");

    show_table(Port, &WarmUpCompTable);
}


exec_result_t modify_WarmUpCompTable(U32 Offset, U32 Value)
{
    //modify_table provides offset range check!
    return modify_table(&WarmUpCompTable, Offset, Value);
}


/**
this function implements the TS interface binary config page read command for WarmUpCompTable
*/
void send_WarmUpCompTable(USART_TypeDef * Port)
{
    send_table(Port, &WarmUpCompTable);
}


/**
returns the Warm up Enrichment compensation in percent
*/
F32 getValue_WarmUpCompTable(F32 CLT_K)
{
    return getValue_table(&WarmUpCompTable, CLT_K) / cWarmUpDivider;
}


/***************************************************************************************************************************************************
*   Injector dead time table - InjectorTimingTable
*
* x-Axis -> System Voltage in mV (no offset, no scaling)
* y-Axis -> Injector dead time in us (no offset, table values are in 24 us increments)
***************************************************************************************************************************************************/

exec_result_t load_InjectorTimingTable()
{
    return load_table(&InjectorTimingTable, EEPROM_FUELING_INJECTORTIMING_BASE);
}

exec_result_t store_InjectorTimingTable()
{
    return store_table(&InjectorTimingTable, EEPROM_FUELING_INJECTORTIMING_BASE);
}


void show_InjectorTimingTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nInjector timing table:\r\n");

    show_table(Port, &InjectorTimingTable);
}


exec_result_t modify_InjectorTimingTable(U32 Offset, U32 Value)
{
    //modify_table provides offset range check!
    return modify_table(&InjectorTimingTable, Offset, Value);
}


/**
this function implements the TS interface binary config page read command for InjectorTimingTable
*/
void send_InjectorTimingTable(USART_TypeDef * Port)
{
    send_table(Port, &InjectorTimingTable);
}


/**
returns the injector dead time in its intervals
*/
F32 getValue_InjectorTimingTable(F32 Bat_V)
{
    return getValue_table(&InjectorTimingTable, 1000.0 * Bat_V);
}


/***************************************************************************************************************************************************
*   Cranking base fuel mass table - CrankingFuelTable
*
* x-Axis -> CLT in K (no offset, no scaling)
* y-Axis -> Cranking base fuel amount in ug (no offset, table values are in 256 ug increments)
***************************************************************************************************************************************************/

exec_result_t load_CrankingFuelTable()
{
    return load_table(&CrankingFuelTable, EEPROM_FUELING_CRANKINGTABLE_BASE);
}

exec_result_t store_CrankingFuelTable()
{
    return store_table(&CrankingFuelTable, EEPROM_FUELING_CRANKINGTABLE_BASE);
}


void show_CrankingFuelTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nCranking fuel table (%):\r\n");

    show_table(Port, &CrankingFuelTable);
}


exec_result_t modify_CrankingFuelTable(U32 Offset, U32 Value)
{
    return modify_table(&CrankingFuelTable, Offset, Value);
}


/**
this function implements the TS interface binary config page read command for CrankingFuelTable
*/
void send_CrankingFuelTable(USART_TypeDef * Port)
{
    send_table(Port, &CrankingFuelTable);
}


/**
returns the Cranking base fuel mass in its increments
*/
F32 getValue_CrankingFuelTable(F32 CLT_K)
{
    return getValue_table(&CrankingFuelTable, CLT_K);
}




/***************************************************************************************************************************************************
*   Barometric pressure correction - BAROtable
*
* x-Axis -> Barometric pressure in hPa (no offset, no scaling)
* y-Axis -> correction  in % (offset := 100, no scaling)
***************************************************************************************************************************************************/

const F32 cBAROtableOffset_pct= 100.0;


exec_result_t load_BAROtable()
{
    return load_table(&BAROtable, EEPROM_FUELING_BARO_BASE);
}

exec_result_t store_BAROtable()
{
    return store_table(&BAROtable, EEPROM_FUELING_BARO_BASE);
}


void show_BAROtable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nBARO correction table:\r\n");

    show_table(Port, &BAROtable);
}


exec_result_t modify_BAROtable(U32 Offset, U32 Value)
{
    return modify_table(&BAROtable, Offset, Value);
}


/**
this function implements the TS interface binary config page read command for BAROtable
*/
void send_BAROtable(USART_TypeDef * Port)
{
    send_table(Port, &BAROtable);
}


/**
returns the Barometric pressure correction factor in %
*/
F32 getValue_BAROtable(F32 BARO_kPa)
{
    VF32 raw;

    raw= getValue_table(&BAROtable, 10.0 * BARO_kPa);
    return raw - cBAROtableOffset_pct;
}



/***************************************************************************************************************************************************
*   charge temperature table - ChargeTempMap
*
* x-Axis -> IAT in K (no offset, no scaling)
* y-Axis -> CLT in K (no offset, no scaling)
* z-Axis -> charge temperature in K (offset := 173,15K, no scaling)
***************************************************************************************************************************************************/

const F32 cChargeTempOffset_K= 173.15;

exec_result_t load_ChargeTempMap()
{
    return map_load(&ChargeTempMap, EEPROM_FUELING_CHARGETEMP_BASE);
}

exec_result_t store_ChargeTempMap()
{
    return map_store(&ChargeTempMap, EEPROM_FUELING_CHARGETEMP_BASE);
}

void show_ChargeTempMap(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\ncharge temperature table (K):\r\n");

    map_show(Port, &ChargeTempMap);
}

exec_result_t modify_ChargeTempMap(U32 Offset, U32 Value)
{
    //map_modify provides offset range check!
    return map_modify(&ChargeTempMap, Offset, Value);
}

/**
this function implements the TS interface binary config page read command for ChargeTempMap
*/
void send_ChargeTempMap(USART_TypeDef * Port)
{
    map_send(Port, &ChargeTempMap);
}


/**
returns the charge temperature in K
*/
F32 getValue_ChargeTempMap(F32 IAT_K, F32 CLT_K)
{
    F32 temp_raw;
    exec_result_t result;

    //look up map
    result= map_get(&ChargeTempMap, IAT_K, CLT_K, &temp_raw);

    if(result == EXEC_OK)
    {
        return temp_raw + cChargeTempOffset_K;
    }
    else
    {
        Fatal(TID_FUELING_CONFIG, FUELING_LOC_CONFIG_GETCHARGETEMP_ERR);
        return 0.0;
    }
}





