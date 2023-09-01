#include <Tuareg_platform.h>
#include <Tuareg.h>

//#define TUAREG_DEBUG_OUTPUT

#ifdef TUAREG_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // TUAREG_DEBUG_OUTPUT



/******************************************************************************************************************************
INIT
******************************************************************************************************************************/




/******************************************************************************************************************************
Update - periodic update function

called every 10 ms from systick timer (100 Hz)

this function will be executed in all system states and has to take care of all system errors


******************************************************************************************************************************/




/******************************************************************************************************************************

******************************************************************************************************************************/


exec_result_t Tuareg_update_control_strategy()
{
    /*
    At least one method to determine engine load is required to operate the engine

    -> while booting all errors will be present
    -> non-running modes are not affected
    -> while cranking a static ignition profile and fueling is used
    */
    if( (Tuareg.flags.run_inhibit == false) && (Tuareg.pDecoder->flags.standstill == false) && (Tuareg.flags.cranking == false) &&
       (Tuareg.errors.sensor_MAP_error == true) && (Tuareg.errors.sensor_TPS_error == true) )
    {
        //engine operation is not allowed
        Fatal(TID_TUAREG, TUAREG_LOC_UPDSTRGY_SRC_ERROR);

        return EXEC_ERROR;
    }

    /**
    - select MAP sensor for lower engine speed if it has not failed - use TPS sensor instead
    - select TPS for higher engine speed
    - higher engine speed will be rejected by the limp mode triggered rev limiter
    */
    Tuareg.Tuareg_controls.Flags.SPD_ctrl= (   (Tuareg.errors.sensor_MAP_error == false) && (Tuareg.pDecoder->crank_rpm < Tuareg_Setup.spd_max_rpm) && (Tuareg.flags.limited_op == false) ) ||
                                                    (Tuareg.errors.sensor_TPS_error == true);


    return EXEC_OK;
}


/******************************************************************************************************************************

main function to be called


******************************************************************************************************************************/


exec_result_t Tuareg_update_controls()
{
    ctrlset_req_t Request;
    exec_result_t Result;

    //mark current controls as invalid
    Tuareg.Tuareg_controls.Flags.valid= false;


    /******************************************
    select a proper control strategy
    ******************************************/
    ASSERT_EXEC_OK( Tuareg_update_control_strategy() );


    /******************************************************
    get global control data from the commanded control set
    ******************************************************/

    //X axis is always rpm
    Request.X= Tuareg.pDecoder->crank_rpm;


    if(Tuareg.flags.limited_op == true)
    {
        //LIMP TPS control
        Request.Y= Tuareg.process.TPS_deg;

        //perfom look up
        Result= ctrlset_get(&Control_TPS_Limp, &Request);

        VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_TPSLIMP_ERROR);

        //export set designator
        Tuareg.Tuareg_controls.Set= CTRLSET_TPS_LIMP;

    }
    if(Tuareg.Tuareg_controls.Flags.SPD_ctrl == true)
    {
        //MAP control
        Request.Y= Tuareg.process.MAP_kPa;

        //perfom look up
        Result= ctrlset_get(&Control_MAP, &Request);

        VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_MAP_ERROR);

        //export set designator
        Tuareg.Tuareg_controls.Set= CTRLSET_MAP_STD;
    }
    else
    {
        //TPS control
        Request.Y= Tuareg.process.TPS_deg;

        //perfom look up
        Result= ctrlset_get(&Control_TPS, &Request);

        VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_TPS_ERROR);

        //export set designator
        Tuareg.Tuareg_controls.Set= CTRLSET_TPS_STD;

    }

    //copy data over
    Tuareg.Tuareg_controls.AFRtgt= Request.AFRtgt;
    Tuareg.Tuareg_controls.VE= Request.VE;
    Tuareg.Tuareg_controls.IgnAdv= Request.IgnAdv;

    //mark controls as valid
    Tuareg.Tuareg_controls.Flags.valid= true;


    return Result;
}
