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

******************************************************************************************************************************/
void Tuareg_update_control_strategy(volatile Tuareg_controls_t * pControls)
{
/// TODO (oli#1#10/20/23): check this logic

    //if the MAP sensor is available
    if(Tuareg.errors.sensor_MAP_error == false)
    {
        //select MAP sensor for lower engine speed or if the TPS sensor has failed
        pControls->Flags.SPD_ctrl= ((Tuareg.pDecoder->crank_rpm < Tuareg_Setup.spd_max_rpm) && (Tuareg.flags.cranking == false))  || (Tuareg.errors.sensor_TPS_error == true);
    }
    else
    {
        //run_allow will be reset within 200ms if the TPS sensor has failed, too
        pControls->Flags.SPD_ctrl= false;
    }

}


/******************************************************************************************************************************
main function to be called
******************************************************************************************************************************/
void Tuareg_update_controls(volatile Tuareg_controls_t * pControls)
{
    ctrlset_req_t Request;
    exec_result_t Result;


    //mark current controls as invalid
    pControls->Flags.valid= false;


    /******************************************
    exit if engine operation is not allowed
    ******************************************/
    if(Tuareg.flags.run_allow == false)
    {
        return;
    }


    /******************************************
    select a proper control strategy
    ******************************************/
    Tuareg_update_control_strategy(pControls);


    /******************************************************
    get global control data from the commanded control set
    ******************************************************/

    //X axis is always rpm
    Request.X= Tuareg.pDecoder->crank_rpm;


    if(pControls->Flags.SPD_ctrl == false)
    {
        //TPS based control requested

        //check if a degraded situation is present
        if(Tuareg.flags.limited_op == true)
        {
            //LIMP TPS control
            Request.Y= Tuareg.process.TPS_deg;

            //perfom look up
            Result= ctrlset_get(&Control_TPS_Limp, &Request);

            VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_TPSLIMP_ERROR);

            //export set designator
            pControls->Set= CTRLSET_TPS_LIMP;
        }
        else
        {
            //regular TPS control
            Request.Y= Tuareg.process.TPS_deg;

            //perfom look up
            Result= ctrlset_get(&Control_TPS, &Request);

            VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_TPS_ERROR);

            //export set designator
            pControls->Set= CTRLSET_TPS_STD;
        }
    }
    else
    {
        //MAP control
        Request.Y= Tuareg.process.MAP_kPa;

        //perfom look up
        Result= ctrlset_get(&Control_MAP, &Request);

        VitalAssert(Result == EXEC_OK, TID_TUAREG_CONTROLS, TUAREG_LOC_CTRLS_UPDATE_MAP_ERROR);

        //export set designator
        pControls->Set= CTRLSET_MAP_STD;
    }

    /**
    VE range check
    */
    if(Request.VE < cMin_VE_val)
    {
        //clamp VE
        pControls->VE_pct= cMin_VE_val;

        //a robust but ugly strategy to proceed without a valid VE
        Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_VE_MIN_VAL);
    }
    else if(Request.VE > cMax_VE_val)
    {
        //clamp VE
        pControls->VE_pct= cMax_VE_val;

        //a robust but ugly strategy to proceed without a valid VE
        Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_VE_MAX_VAL);
    }
    else
    {
        //use value from lookup
        pControls->VE_pct= Request.VE;
    }


    /**
    AFR range check and substitution logic
    to avoid unsafe engine operation in a degraded situation, a default AFT target has to be applied
    */
    if((Request.AFRtgt < cMin_AFR_target) || (Request.AFRtgt > cMax_AFR_target))
    {
        //use fallback value
        pControls->AFRtgt= cDefault_AFR_target;
        pControls->Flags.AFR_fallback= true;

        //a robust but ugly strategy to proceed without a valid AFR
        Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_AFR_INVALID);
    }
    else if(Tuareg.flags.limited_op == true)
    {
        //use fallback value, too
        pControls->AFRtgt= cDefault_AFR_target;
        pControls->Flags.AFR_fallback= true;
    }
    else
    {
        //use value from lookup
        pControls->AFRtgt= Request.AFRtgt;
    }


    /**
    Ignition Advance Angle range check
    */
    if(Request.IgnAdv < cMin_Adv_val)
    {
        //clamp angle
        pControls->IgnAdv_deg= cMin_Adv_val;

        //a robust but ugly strategy to proceed without a valid angle
        Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_ADV_MIN_VAL);
    }
    else if(Request.IgnAdv > cMax_Adv_val)
    {
        //clamp angle
        pControls->IgnAdv_deg= cMax_Adv_val;

        //a robust but ugly strategy to proceed without a valid VE
        Limp(TID_TUAREG_CONTROLS, TUAREG_LOC_UPDCTRL_ADV_MAX_VAL);
    }
    else
    {
        //use value from lookup
        pControls->IgnAdv_deg= Request.IgnAdv;
    }


    /**
    all good, mark controls as valid
    */
    pControls->Flags.valid= true;
}
