#ifndef TUAREG_CONSOLE_H_INCLUDED
#define TUAREG_CONSOLE_H_INCLUDED

#include "TunerStudio.h"


typedef union
{
     U16 all_flags;

     struct
     {
        U16 calib_mod :1;
        U16 ignition_mod :1;
        U16 fueling_mod :1;
        U16 decoder_mod :1;
        U16 tsetup_mod :1;

        U16 ctrlset_map_mod :1;
        U16 ctrlset_tps_mod :1;
        U16 ctrlset_tps_limp_mod :1;

        U16 faultlog_clear :1;

        U16 burn_permission :1;
     };

} cli_permission_flags_t;


#define TS_CMD_WATCHDOG_S 3

typedef struct _Tuareg_console_t
{
    //Tuner Studio variables
    U32 ts_cmd_watchdog;
    U32 param_offset;
    U32 param_count;
    TS_page_t ts_active_page;
    U8 ts_secl;
    bool ts_connected;
    bool params_valid;

    //CLI variables
    U32 active_cmd;
    cli_permission_flags_t cli_permissions;

} Tuareg_console_t ;


extern volatile Tuareg_console_t Tuareg_console;


void Tuareg_update_console();
void Tuareg_init_console();

void cli_show_help();

void cli_cyclic_update();

void cli_showPage(U32 Page);

void cli_setPermissions(U32 Value);


#endif // TUAREG_CONSOLE_H_INCLUDED
