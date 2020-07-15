#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include "config_pages.h"


#define CURRENT_DATA_VERSION    5
#define PAGE_SIZE 64


/**
give access to configPages
*/
extern volatile configPage1_t configPage1;
extern volatile configPage2_t configPage2;
extern volatile configPage3_t configPage3;
extern volatile configPage4_t configPage4;
extern volatile configPage9_t configPage9;
extern volatile configPage10_t configPage10;
extern volatile configPage11_t configPage11;
extern volatile configPage12_t configPage12;
extern volatile configPage13_t configPage13;


//extern const U16 configPage_size[12];



/**
config handling
*/
U32 config_write();
U32 config_load();

U32 migrate_configData();

U32 load_DecoderConfig();
void load_essential_DecoderConfig();
U32 write_DecoderConfig();

U32 load_IgnitionConfig();
void load_essential_IgnitionConfig();
U32 write_IgnitionConfig();

U32 load_SensorCalibration();
U32 write_SensorCalibration();

void config_load_essentials();


#endif // CONFIG_H_INCLUDED
