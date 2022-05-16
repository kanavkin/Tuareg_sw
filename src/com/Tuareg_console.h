#ifndef TUAREG_CONSOLE_H_INCLUDED
#define TUAREG_CONSOLE_H_INCLUDED


#include "TunerStudio.h"

#include "process_table.h"
#include "Tuareg_ignition.h"
#include "Tuareg_ignition_controls.h"

typedef union
{
     U8 all_flags;

     struct
     {
        U8 calib_mod_permission :1;
        U8 ignition_mod_permission :1;
        U8 fueling_mod_permission :1;
        U8 decoder_mod_permission :1;
        U8 tsetup_mod_permission :1;
        U8 burn_permission :1;
        U8 faultlog_permission :1;
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
