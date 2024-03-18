#include <Tuareg_platform.h>
#include <Tuareg.h>

//#define TUAREG_DEBUG_OUTPUT

#ifdef TUAREG_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // TUAREG_DEBUG_OUTPUT



/******************************************************************************************************************************
built in defaults
******************************************************************************************************************************/
const F32 cDefault_AFR_target= 14.7;
const F32 cMin_AFR_target= 3.0;
const F32 cMax_AFR_target= 30.0;
const F32 cMin_VE_val= 0.0;
const F32 cMax_VE_val= 125.0;
const F32 cMin_Adv_val= 0.0;
const F32 cMax_Adv_val= 70.0;


/******************************************************************************************************************************
one of MAP / TPS sensor fails -> Limp mode (~ 4000 rpm)
MAP failure -> TPS Limp
TPS failure -> MAP

not to be called while engine cranking
******************************************************************************************************************************/
void update_control_strategy()
{
    //check if vital sensors are available
    if((Tuareg.errors.sensor_MAP_error == false) && (Tuareg.errors.sensor_TPS_error == false))
    {
        //select MAP sensor for lower engine speed above cranking
        Tuareg.Controls.Flags.SPD_ctrl= (Tuareg.Decoder.crank_rpm < Tuareg_Setup.spd_max_rpm);

        //check if a smooth transition is necessary
        Tuareg.Controls.Flags.smooth_transition= ( (Tuareg_Setup.flags.SmoothTrans_ena == true) && (Tuareg.flags.limited_op == false) &&
                                             (Tuareg.Decoder.crank_rpm > subtract_U32(Tuareg_Setup.spd_max_rpm, Tuareg_Setup.smooth_transition_radius_rpm)) &&
                                             (Tuareg.Decoder.crank_rpm < Tuareg_Setup.spd_max_rpm + Tuareg_Setup.smooth_transition_radius_rpm) );
    }
    else if(Tuareg.errors.sensor_MAP_error == true)
    {
        //select TPS (limp)
        Tuareg.Controls.Flags.SPD_ctrl= false;
        Tuareg.Controls.Flags.smooth_transition= false;
    }
    else
    {
        //select MAP sensor as load source
        Tuareg.Controls.Flags.SPD_ctrl= true;
        Tuareg.Controls.Flags.smooth_transition= false;
    }

}


/******************************************************************************************************************************
main function to be called
******************************************************************************************************************************/
void Tuareg_update_controls()
{
    ctrlset_req_t Request_TPS, Request_MAP;
    exec_result_t Result;
    F32 VE, AFRtgt, IgnAdv;

    //mark current controls as invalid
    Tuareg.Controls.Flags.valid= false;


    /**
    update process data first
    */
    Tuareg_update_process_data();


    /******************************************
    exit if engine operation is not allowed
    ******************************************/
    if((Tuareg.flags.run_allow == false) || (Tuareg.Decoder.flags.period_valid == false))
    {
        Tuareg_clear_controls();
        return;
    }


    /****************************************************************************************************
    Check if the engine is cranking

    The ignition and fueling modules will perform special procedures for engine cranking
    which will not rely on control data
    ******************************************************************************************************/
    if(Tuareg.Decoder.crank_rpm > Cranking_End_rpm)
    {
        //not cranking
        Tuareg.Controls.Flags.cranking= false;

        /******************************************
        select a proper control strategy
        ******************************************/
        update_control_strategy();


        /******************************************************
        get global control data from the commanded control set
        ******************************************************/

        //prepare both request objects for MAP and TPS
        Request_MAP.X= Tuareg.Decoder.crank_rpm;
        Request_MAP.Y= Tuareg.process.MAP_kPa;
        Request_TPS.X= Tuareg.Decoder.crank_rpm;
        Request_TPS.Y= Tuareg.process.TPS_deg;

        //check if a smooth transition has been commanded
        if(Tuareg.Controls.Flags.smooth_transition == true)
        {
            //perfom look up for MAP control set
            Result= ctrlset_get(&Control_MAP, &Request_MAP);
            VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_MAP_ERROR);

            //perfom look up from regular TPS control set
            Result= ctrlset_get(&Control_TPS, &Request_TPS);
            VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_TPS_ERROR);

            //calculate the average from both maps
            VE= (Request_MAP.VE + Request_TPS.VE) / 2.0;
            AFRtgt= (Request_MAP.AFRtgt + Request_TPS.AFRtgt) / 2.0;
            IgnAdv= (Request_MAP.IgnAdv + Request_TPS.IgnAdv) / 2.0;

        }
        else if(Tuareg.Controls.Flags.SPD_ctrl == false)
        {
            //check if a degraded situation is present
            if(Tuareg.flags.limited_op == true)
            {
                /**
                TPS LIMP control set
                */

                //perfom look up
                Result= ctrlset_get(&Control_TPS_Limp, &Request_TPS);
                VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_TPSLIMP_ERROR);

                //export set designator
                Tuareg.Controls.Set= CTRLSET_TPS_LIMP;
            }
            else
            {
                /**
                TPS
                */

                //perfom look up from regular TPS control set
                Result= ctrlset_get(&Control_TPS, &Request_TPS);
                VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_TPS_ERROR);

                //export set designator
                Tuareg.Controls.Set= CTRLSET_TPS_STD;
            }

            //copy data over for processing
            VE= Request_TPS.VE;
            AFRtgt= Request_TPS.AFRtgt;
            IgnAdv= Request_TPS.IgnAdv;

        }
        else
        {
            /**
            MAP control set
            */

            //perfom look up
            Result= ctrlset_get(&Control_MAP, &Request_MAP);
            VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_MAP_ERROR);

            //export set designator
            Tuareg.Controls.Set= CTRLSET_MAP_STD;

            //copy data over for processing
            VE= Request_MAP.VE;
            AFRtgt= Request_MAP.AFRtgt;
            IgnAdv= Request_MAP.IgnAdv;
        }


        /**
        VE range check
        */
        if(VE < cMin_VE_val)
        {
            //clamp VE
            Tuareg.Controls.VE_pct= cMin_VE_val;

            //a robust but ugly strategy to proceed without a valid VE
            Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_VE_MIN_VAL);
        }
        else if(VE > cMax_VE_val)
        {
            //clamp VE
            Tuareg.Controls.VE_pct= cMax_VE_val;

            //a robust but ugly strategy to proceed without a valid VE
            Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_VE_MAX_VAL);
        }
        else
        {
            //use value from lookup
            Tuareg.Controls.VE_pct= VE;
        }


        /**
        AFR range check and substitution logic
        to avoid unsafe engine operation in a degraded situation, a default AFT target has to be applied
        */
        if((AFRtgt < cMin_AFR_target) || (AFRtgt > cMax_AFR_target))
        {
            //use fallback value
            Tuareg.Controls.AFRtgt= cDefault_AFR_target;
            Tuareg.Controls.Flags.AFR_fallback= true;

            //a robust but ugly strategy to proceed without a valid AFR
            Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_AFR_INVALID);
        }
        else if(Tuareg.flags.limited_op == true)
        {
            //use fallback value, too
            Tuareg.Controls.AFRtgt= cDefault_AFR_target;
            Tuareg.Controls.Flags.AFR_fallback= true;
        }
        else
        {
            //use value from lookup
            Tuareg.Controls.AFRtgt= AFRtgt;
        }


        /**
        Ignition Advance Angle range check
        */
        if(IgnAdv < cMin_Adv_val)
        {
            //clamp angle
            Tuareg.Controls.IgnAdv_deg= cMin_Adv_val;

            //a robust but ugly strategy to proceed without a valid angle
            Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_ADV_MIN_VAL);
        }
        else if(IgnAdv > cMax_Adv_val)
        {
            //clamp angle
            Tuareg.Controls.IgnAdv_deg= cMax_Adv_val;

            //a robust but ugly strategy to proceed without a valid VE
            Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_ADV_MAX_VAL);
        }
        else
        {
            //use value from lookup
            Tuareg.Controls.IgnAdv_deg= IgnAdv;
        }


    }
    else
    {
        /**
        engine is cranking
        */
        Tuareg.Controls.Flags.cranking= true;

        //control data dummy output
        Tuareg.Controls.VE_pct= cMin_VE_val;
        Tuareg.Controls.AFRtgt= cDefault_AFR_target;
        Tuareg.Controls.Flags.AFR_fallback= true;
        Tuareg.Controls.IgnAdv_deg= cMin_Adv_val;
    }


    //controls are valid
    Tuareg.Controls.Flags.valid= true;

    //update ignition controls
    Tuareg_update_ignition_controls();

    //update fueling controls
    Tuareg_update_fueling_controls();
}


/******************************************************************************************************************************
helper function for cleanup
******************************************************************************************************************************/
void Tuareg_clear_controls()
{
    //test
    memclr_boctok((void *) &(Tuareg.Controls), sizeof(Tuareg_controls_t));
}
