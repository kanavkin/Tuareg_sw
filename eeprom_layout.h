#ifndef EEPROM_LAYOUT_H_INCLUDED
#define EEPROM_LAYOUT_H_INCLUDED

#include "Tuareg_config.h"
#include "decoder_config.h"
#include "ignition_config.h"
#include "sensor_calibration.h"


/**
Tuareg configuration
*/
#define EEPROM_TUAREG_CONFIG_BASE 1



/**
sensor calibration data
*/
#define EEPROM_SENSOR_CALIBRATION_BASE 100


/**
decoder configuration
*/
#define EEPROM_DECODER_CONFIG_BASE 200


/**
ignition configuration
*/
#define EEPROM_IGNITION_SETUP_BASE 220


/**
Ignition advance table (TPS based)
*/
#define EEPROM_IGNITION_ADVTPS_BASE 300

/**
Ignition advance table (MAP based)
*/

/**
This is the last used eeprom address -> memory dump will be read until here
*/
#define EEPROM_STORAGE_END 1000


#endif // EEPROM_LAYOUT_H_INCLUDED
