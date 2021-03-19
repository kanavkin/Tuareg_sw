#ifndef EEPROM_LAYOUT_H_INCLUDED
#define EEPROM_LAYOUT_H_INCLUDED

#include "Tuareg_config.h"
#include "decoder_config.h"
#include "ignition_config.h"
#include "sensor_calibration.h"


//see ct3D_data_size for address calculation!!! 320
#define TABLE3D_RESERVED_SPACE 320


/**
Tuareg configuration
*/
#define EEPROM_TUAREG_CONFIG_BASE 1

#define EEPROM_TUAREG_CONFIG_RESERVED_SPACE 70


/**
sensor calibration data
*/
#define EEPROM_SENSOR_CALIBRATION_BASE (EEPROM_TUAREG_CONFIG_BASE + EEPROM_TUAREG_CONFIG_RESERVED_SPACE)

#define EEPROM_SENSOR_CALIBRATION_RESERVED_SPACE 120


/**
decoder configuration
*/
#define EEPROM_DECODER_CONFIG_BASE (EEPROM_SENSOR_CALIBRATION_BASE + EEPROM_SENSOR_CALIBRATION_RESERVED_SPACE)

#define EEPROM_DECODER_CONFIG_RESERVED_SPACE 20


/**
ignition configuration
*/
#define EEPROM_IGNITION_SETUP_BASE (EEPROM_DECODER_CONFIG_BASE + EEPROM_DECODER_CONFIG_RESERVED_SPACE)

#define EEPROM_IGNITION_SETUP_RESERVED_SPACE 30


/**
Ignition advance table 3D (TPS based)
*/
#define EEPROM_IGNITION_ADVTPS_BASE (EEPROM_IGNITION_SETUP_BASE + EEPROM_IGNITION_SETUP_RESERVED_SPACE)

/**
Ignition advance table 3D (MAP based)
*/
//currently not implemented



/**
Fueling configuration
*/
#define EEPROM_FUELING_SETUP_BASE (EEPROM_IGNITION_SETUP_BASE + EEPROM_IGNITION_SETUP_RESERVED_SPACE + TABLE3D_RESERVED_SPACE)

#define EEPROM_FUELING_SETUP_RESERVED_SPACE 40

/**
Fueling VE Table 3D (TPS based) - VeTable_TPS
*/
#define EEPROM_FUELING_VETPS_BASE (EEPROM_FUELING_SETUP_BASE + EEPROM_FUELING_SETUP_RESERVED_SPACE)

/**
Fueling VE Table 3D (MAP based) - VeTable_MAP
*/
#define EEPROM_FUELING_VEMAP_BASE (EEPROM_FUELING_VETPS_BASE + TABLE3D_RESERVED_SPACE)

/**
Fueling AFR target Table 3D (TPS based) - AfrTable_TPS
*/
#define EEPROM_FUELING_AFRTPS_BASE (EEPROM_FUELING_VEMAP_BASE + TABLE3D_RESERVED_SPACE)



/**
This is the last used eeprom address -> memory dump will be read until here
*/
#define EEPROM_STORAGE_END 1000


#endif // EEPROM_LAYOUT_H_INCLUDED
