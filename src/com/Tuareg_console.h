#ifndef TUAREG_CONSOLE_H_INCLUDED
#define TUAREG_CONSOLE_H_INCLUDED


#include "TunerStudio.h"

#include "process_table.h"
#include "Tuareg_ignition.h"
#include "Tuareg_ignition_controls.h"


typedef struct
{

    U8 legacy_mod_permission :1;

    U8 calib_mod_permission :1;
    U8 ignition_mod_permission :1;
    U8 decoder_mod_permission :1;
    U8 tsetup_mod_permission :1;

    U8 burn_permission :1;

} cli_permission_t ;



#define TS_CMD_WATCHDOG_S 3

typedef struct _Tuareg_console_t
{
    //Tuner Studio variables
    TS_page_t ts_active_page;
    U32 ts_cmd_watchdog;
    U32 ts_getOutputChannels_count;
    U8 secl;

    //CLI variables
    U8 active_cmd;
    cli_permission_t cli_permissions;

} Tuareg_console_t ;


extern volatile Tuareg_console_t Tuareg_console;


void Tuareg_update_console();
void Tuareg_init_console();

extern void cli_show_help();

void cli_update_watchdog();
void cli_update_secl();

extern void cli_showPage(U32 Page);

extern void cli_checkPermissions(U32 Value);


#endif // TUAREG_CONSOLE_H_INCLUDED
