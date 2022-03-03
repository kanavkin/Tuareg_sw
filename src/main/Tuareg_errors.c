
#include "Tuareg.h"

#include "dash_logic.h"

#define ERRORS_DEBUGMSG

#ifdef ERRORS_DEBUGMSG
//#warning Errors debug outputs enabled
#pragma message "Errors debug outputs enabled"
#include "debug_port_messages.h"
#include "uart_printf.h"
#endif // ERRORS_DEBUGMSG



/**
Puts the system to a safe state when a critical error has been detected

This function has to be very restrictive.
BUT
An error could upset the program logic in a way that RunMode will not be updated to FATAL.

So vital actors operation is inhibited already here.

Correcting config parameter errors through the console shall be possible.

*/
void Fatal(Tuareg_ID Id, U8 Location)
{

    Tuareg.errors.fatal_error= true;
    Tuareg.flags.run_inhibit= true;
    Tuareg.flags.service_mode= false;

    Syslog_Error(Id, Location);
    Syslog_Error(TID_TUAREG, TUAREG_LOC_FATAL_ERROR);
    log_Fault(Id, Location);

    //turn off actors
    Tuareg_deactivate_vital_actors();
    set_fuel_pump_unpowered();

    //disable decoder
    disable_Decoder();


    #ifdef ERRORS_DEBUGMSG
    DebugMsg_Error("FATAL ERROR -- in module:");

    printf_U(DEBUG_PORT, Id, NO_PAD);
    print_Tuareg_ID_label(DEBUG_PORT, Id);
    print(DEBUG_PORT, " location: ");
    printf_U(DEBUG_PORT, Location, NO_PAD | NO_TRAIL);
    #endif // ERRORS_DEBUGMSG

    //interim solution
    dash_set_mil(MIL_PERMANENT);
}


/**
Assertion of a vital condition

-> not recoverable
-> programming error
-> system shutdown

*/
void VitalAssert(bool Condition, Tuareg_ID Id, U8 Location)
{
    if(!Condition)
    {
        Fatal(Id, Location);
    }
}


/**
Assertion of a condition required to normal system operation

-> avoiding the use of this function in Limp mode
-> parameter / config error
-> console must be kept functional to correct parameter errors

*/
void rrrr(bool Condition, Tuareg_ID Id, U8 Location)
{
    if(!Condition)
    {
        Limp(Id, Location);
    }
}


/**
Activates the systems limited operation strategy when a critical error has been detected
*/
void Limp(Tuareg_ID Id, U8 Location)
{
    //Fatal mode will persist until reboot
    if(Tuareg.errors.fatal_error == true)
    {
        return;
    }

    Tuareg.flags.limited_op= true;
    Tuareg.flags.service_mode= false;

    Syslog_Error(Id, Location);
    Syslog_Error(TID_TUAREG, TUAREG_LOC_ENTER_LIMP_MODE);
    log_Fault(Id, Location);

    #ifdef ERRORS_DEBUGMSG
    DebugMsg_Error("LIMP -- ");
    printf_U(DEBUG_PORT, Id, NO_PAD);
    printf_U(DEBUG_PORT, Location, NO_PAD | NO_TRAIL);
    #endif // ERRORS_DEBUGMSG

    //interim solution
    dash_set_mil(MIL_PERMANENT);
}

