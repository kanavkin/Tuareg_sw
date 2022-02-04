
#include "Tuareg.h"

#include "dash_logic.h"

//#define ERRORS_DEBUGMSG

#ifdef ERRORS_DEBUGMSG
#warning Errors debug outputs enabled
#include "debug_port_messages.h"
#endif // ERRORS_DEBUGMSG



/**
Puts the system to a safe state when a critical error has been detected

This function has to be very restrictive.
BUT
An error could upset the program logic in a way that RunMode will not be updated to FATAL.

So vital actors operation is inhibited already here.
*/
void Fatal(Tuareg_ID Id, U8 Location)
{
    __disable_irq();

    Tuareg.errors.fatal_error= true;
    Tuareg.flags.run_inhibit= true;

    Syslog_Error(Id, Location);
    Syslog_Error(TID_TUAREG, TUAREG_LOC_FATAL_ERROR);
    log_Fault(Id, Location);

    #ifdef ERRORS_DEBUGMSG
    DebugMsg_Error("FATAL --");
    printf_U(DEBUG_PORT, Id, NO_PAD);
    printf_U(DEBUG_PORT, Location, NO_PAD | NO_TRAIL);
    #endif // ERRORS_DEBUGMSG

    //interim solution
    dash_set_mil(MIL_PERMANENT);
}


void Assert(bool Condition, Tuareg_ID Id, U8 Location)
{
    if(!Condition)
    {
        Fatal(Id, Location);
    }
}


/**
Activates the systems limited operation strategy when a critical error has been detected
*/
void Limp(Tuareg_ID Id, U8 Location)
{
    Tuareg.flags.limited_op= true;

    Syslog_Error(Id, Location);
    Syslog_Error(TID_TUAREG, TUAREG_LOC_ENTER_LIMP_MODE);
    log_Fault(Id, Location);

    #ifdef ERRORS_DEBUGMSG
    DebugMsg_Error("LIMP --");
    printf_U(DEBUG_PORT, Id, NO_PAD);
    printf_U(DEBUG_PORT, Location, NO_PAD | NO_TRAIL);
    #endif // ERRORS_DEBUGMSG

    //interim solution
    dash_set_mil(MIL_PERMANENT);
}

