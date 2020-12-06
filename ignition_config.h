#ifndef IGNITION_CONFIG_H_INCLUDED
#define IGNITION_CONFIG_H_INCLUDED

#include "Tuareg_ignition.h"

/***************************************************************************************************************************************************
*   ignition configuration page
***************************************************************************************************************************************************/
typedef struct _Ignition_Config_t_ {

    //rev limiter function
    U16 max_rpm;

    //dynamic ignition function
    U16 dynamic_min_rpm;
    crank_position_t dynamic_ignition_base_position;
    U16 dynamic_dwell_target_us;

    //cold idle ignition advance function
    U16 cold_idle_cutoff_rpm;
    U16 cold_idle_cutoff_CLT_K;
    U8 cold_idle_ignition_advance_deg;
    U16 cold_idle_dwell_target_us;

    //static ignition setup for cranking
    crank_position_t cranking_ignition_position;
    crank_position_t cranking_dwell_position;

    //how many coils are installed?
    coil_setup_t coil_setup;

    U16 spark_duration_us;

    U8 Version;


} Ignition_Config_t;


/***************************************************************************************************************************************************
*   data offset constants for modification via CLI
***************************************************************************************************************************************************/
typedef enum {

    IGNITION_CONFIG_CLI_MAX_RPM,

    IGNITION_CONFIG_CLI_DYN_MIN_RPM,
    IGNITION_CONFIG_CLI_DYN_IGN_BASE_POS,
    IGNITION_CONFIG_CLI_DYN_DWELL_TGT,

    IGNITION_CONFIG_CLI_COLD_IDLE_CUTOFF_RPM,
    IGNITION_CONFIG_CLI_COLD_IDLE_CUTOFF_CLT,
    IGNITION_CONFIG_CLI_COLD_IDLE_IGN_ADV,
    IGNITION_CONFIG_CLI_COLD_IDLE_DWELL_TGT,

    IGNITION_CONFIG_CLI_CRANKING_IGN_POS,
    IGNITION_CONFIG_CLI_CRANKING_DWELL_POS,

    IGNITION_CONFIG_CLI_COIL_SETUP,

    IGNITION_CONFIG_CLI_SPARK_DURATION

} Ignition_Config_CLI_offset;


/***************************************************************************************************************************************************
*   data offsets for for eeprom layout based on the stored data size
***************************************************************************************************************************************************/
typedef enum {

    IGNITION_CONFIG_EE_VERSION =0,                  // 1 byte

    IGNITION_CONFIG_EE_MAX_RPM =1,                  // 2 bytes 1..2

    IGNITION_CONFIG_EE_DYN_MIN_RPM =3,              // 2 bytes 3..4
    IGNITION_CONFIG_EE_DYN_IGN_BASE_POS =5,         // 1 byte
    IGNITION_CONFIG_EE_DYN_DWELL_TGT =6,            // 2 bytes 6..7

    IGNITION_CONFIG_EE_COLD_IDLE_CUTOFF_RPM =8,     // 2 bytes 8..9
    IGNITION_CONFIG_EE_COLD_IDLE_CUTOFF_CLT =10,    // 2 bytes 10..11
    IGNITION_CONFIG_EE_COLD_IDLE_IGN_ADV =12,       // 2 bytes 12..13
    IGNITION_CONFIG_EE_COLD_IDLE_DWELL_TGT =14,     // 2 bytes 14..15

    IGNITION_CONFIG_EE_CRANKING_IGN_POS =16,        // 1 byte
    IGNITION_CONFIG_EE_CRANKING_DWELL_POS =17,      // 1 byte

    IGNITION_CONFIG_EE_COIL_SETUP =18,              // 1 byte

    IGNITION_CONFIG_EE_SPARK_DURATION =19,          // 2 bytes 19..20

    IGNITION_CONFIG_EE_NEXT_FREE_OFFSET =21         // next free offset after the page

} Ignition_Config_Eeprom_offset;



/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/
extern volatile Ignition_Config_t Ignition_Config;

exec_result_t load_Ignition_Config();
void load_essential_Ignition_Config();
exec_result_t write_Ignition_Config();

void show_Ignition_Config();
exec_result_t modify_Ignition_Config(U32 Offset, U32 Value);

/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
#define ASSERT_CRANK_POS(pos) if((pos) >= CRK_POSITION_COUNT) return EXEC_ERROR
#define ASSERT_IGNITION_SETUP(setup) if((setup) >= COILS_COUNT) return EXEC_ERROR


#endif // IGNITION_CONFIG_H_INCLUDED
