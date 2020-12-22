#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include "Tuareg_types.h"
#include "config_pages.h"

#include "decoder_config.h"
#include "ignition_config.h"
#include "sensor_calibration.h"


#define CURRENT_DATA_VERSION    5
#define PAGE_SIZE 64





/**
config handling
*/
exec_result_t config_tables_write();
exec_result_t config_load();

exec_result_t check_config();

void config_load_essentials();


#endif // CONFIG_H_INCLUDED
