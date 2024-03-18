#ifndef EEPROM_LAYOUT_H_INCLUDED
#define EEPROM_LAYOUT_H_INCLUDED

#include "Tuareg_config.h"
#include "decoder_config.h"
#include "ignition_config.h"
#include "sensor_calibration.h"

#include "map.h"
#include "table.h"

/**
this is the last permitted eeprom address
defining addresses beyond this address will lead to system error
*/
#define EEPROM_FINAL_ADDRESS 8000
#define EEPROM_WARNING_LEVEL 6000



/**
Tuareg configuration
*/
#define EEPROM_TUAREG_CONFIG_BASE 1

#define EEPROM_TUAREG_CONFIG_RESERVED_SPACE 70


/**
sensor calibration data
*/
#define EEPROM_SENSOR_CALIBRATION_BASE (EEPROM_TUAREG_CONFIG_BASE + EEPROM_TUAREG_CONFIG_RESERVED_SPACE)

#define EEPROM_SENSOR_CALIBRATION_RESERVED_SPACE 200


/**
CLT inverse transfer function - lookup Table - InvTableCLT
*/
#define EEPROM_SENSOR_INVTABLECLT_BASE (EEPROM_SENSOR_CALIBRATION_BASE + EEPROM_SENSOR_CALIBRATION_RESERVED_SPACE)


/**
decoder configuration
*/
#define EEPROM_DECODER_CONFIG_BASE (EEPROM_SENSOR_INVTABLECLT_BASE + TABLE_STORAGE_SIZE_B)

#define EEPROM_DECODER_CONFIG_RESERVED_SPACE 30


/*******************************************************************************************
*
*   ignition
*
********************************************************************************************/

/**
ignition configuration
*/
#define EEPROM_IGNITION_SETUP_BASE (EEPROM_DECODER_CONFIG_BASE + EEPROM_DECODER_CONFIG_RESERVED_SPACE)

#define EEPROM_IGNITION_SETUP_RESERVED_SPACE 30

/**
Ignition dwell table
*/
#define EEPROM_IGNITION_DWELLTABLE_BASE (EEPROM_IGNITION_SETUP_BASE + EEPROM_IGNITION_SETUP_RESERVED_SPACE)

/*******************************************************************************************
*
*   fueling
*
********************************************************************************************/

/**
Fueling configuration
*/
#define EEPROM_FUELING_SETUP_BASE (EEPROM_IGNITION_DWELLTABLE_BASE + TABLE_STORAGE_SIZE_B)

#define EEPROM_FUELING_SETUP_RESERVED_SPACE 80

/**
Fueling acceleration compensation table - AccelCompTableTPS
*/
#define EEPROM_FUELING_ACCELCOMPTPS_BASE (EEPROM_FUELING_SETUP_BASE + EEPROM_FUELING_SETUP_RESERVED_SPACE)

/**
Fueling acceleration compensation table - AccelCompTableMAP
*/
#define EEPROM_FUELING_ACCELCOMPMAP_BASE (EEPROM_FUELING_ACCELCOMPTPS_BASE + TABLE_STORAGE_SIZE_B)

/**
Fueling Warm up enrichment compensation table - WarmUpCompTable
*/
#define EEPROM_FUELING_WARMUPCOMP_BASE (EEPROM_FUELING_ACCELCOMPMAP_BASE + TABLE_STORAGE_SIZE_B)

/**
Injector dead time table - InjectorTimingTable
*/
#define EEPROM_FUELING_INJECTORTIMING_BASE (EEPROM_FUELING_WARMUPCOMP_BASE + TABLE_STORAGE_SIZE_B)


/**
Barometric pressure correction - BAROtable
*/
#define EEPROM_FUELING_BARO_BASE (EEPROM_FUELING_INJECTORTIMING_BASE + TABLE_STORAGE_SIZE_B)

/**
charge temperature table - ChargeTempMap
*/
#define EEPROM_FUELING_CHARGETEMP_BASE (EEPROM_FUELING_BARO_BASE + TABLE_STORAGE_SIZE_B)

/**
Cranking base fuel mass table - CrankingFuelTable
/// TODO (oli#1#03/18/24): debug info: table relocated here from after InjectorTimingTable, invalid readout after reset continues

*/
#define EEPROM_FUELING_CRANKINGTABLE_BASE (EEPROM_FUELING_CHARGETEMP_BASE + MAP_STORAGE_SIZE_B)

/*******************************************************************************************
*
*   control sets
*
********************************************************************************************/

/**
MAP based Control Set
*/
#define EEPROM_CTRLSET_MAP_BASE (EEPROM_FUELING_CRANKINGTABLE_BASE + TABLE_STORAGE_SIZE_B)

/**
TPS based Control Set
*/
#define EEPROM_CTRLSET_TPS_BASE (EEPROM_CTRLSET_MAP_BASE + CTRLSET_STORAGE_SIZE_B)

/**
TPS based Control Set (for LIMP mode)
*/
#define EEPROM_CTRLSET_TPS_LIMP_BASE (EEPROM_CTRLSET_TPS_BASE + CTRLSET_STORAGE_SIZE_B)


/*************************************************/


/**
tachometer table - TachTable
2D
*/
#define EEPROM_TACHTABLE_BASE (EEPROM_CTRLSET_TPS_LIMP_BASE + CTRLSET_STORAGE_SIZE_B)




/*************************************************/


/**
Fault Log - Fault_Log
*/
#define EEPROM_FAULT_LOG_BASE (EEPROM_TACHTABLE_BASE + TABLE_STORAGE_SIZE_B)


/**
This is the last used eeprom address -> memory dump will be read until here
*/
#define EEPROM_STORAGE_END (EEPROM_FAULT_LOG_BASE + 20)


#if (EEPROM_STORAGE_END > EEPROM_WARNING_LEVEL)
#warning Eeprom utilization warning threshold reached
#endif

#if (EEPROM_STORAGE_END > EEPROM_FINAL_ADDRESS)
#error Eeprom layout does not fit the physical storage size
#endif

#endif // EEPROM_LAYOUT_H_INCLUDED
