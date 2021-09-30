#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"
#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

VU32 scheduler_diag[SCHEDIAG_COUNT];
VU32 ignition_diag[IGNDIAG_COUNT];
VU32 tuareg_diag[TDIAG_COUNT];
VU32 decoder_diag[DDIAG_COUNT];
VU32 sensors_diag[SNDIAG_COUNT];
VU32 fueling_diag[FDIAG_COUNT];


/******************************************************************************************************
scheduler diag
*******************************************************************************************************/
void scheduler_diag_log_event(scheduler_diag_t event)
{
    if(event < SCHEDIAG_COUNT)
    {
        scheduler_diag[event] += 1;
    }
}


void print_scheduler_diag(USART_TypeDef * Port)
{
    U32 cnt, column = 1;

    print(Port, "\r\nScheduler diag: \r\n");

    for(cnt=0; cnt < SCHEDIAG_COUNT; cnt++)
    {
        printf_U(Port, scheduler_diag[cnt], PAD_10);

        if(column == 4)
        {
            print(Port, "\r\n");
            column = 1;
        }
        else
        {
            column++;
        }
    }
}


void print_scheduler_diag_legend(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nScheduler Diagnostics Legend:");
    print(Port, "\r\n for ICH1, ICH2, FCH1, FCH2");
    print(Port, "\r\nRESET");
    print(Port, "\r\nSET 1INT");
    print(Port, "\r\nSET 2INT");
    print(Port, "\r\nREALLOC");
    print(Port, "\r\nREALLOC_COMPLETED");
    print(Port, "\r\nALLOC_CUR");
    print(Port, "\r\nALLOC_PREL");
    print(Port, "\r\nALLOC_UPD");
    print(Port, "\r\nALLOC");
    print(Port, "\r\nTRIGG");
    print(Port, "\r\nDELAY_MININT1, DELAY_MININT2, WRAP");

}


/******************************************************************************************************
ignition diag
*******************************************************************************************************/
void ignition_diag_log_event(ignition_diag_t event)
{
    if(event < IGNDIAG_COUNT)
    {
        ignition_diag[event] += 1;
    }
}


void print_ignition_diag(USART_TypeDef * Port)
{
    U32 cnt, column = 1;

    print(Port, "\r\nTuareg ignition diag: \r\n");

    for(cnt=0; cnt < IGNDIAG_COUNT; cnt++)
    {
        printf_U(Port, ignition_diag[cnt], PAD_10);

        if(column == DIAG_PRINT_LEGEND_COLUMNS)
        {
            print(Port, "\r\n");
            column = 1;
        }
        else
        {
            column++;
        }
    }
}


void print_ignition_diag_legend(USART_TypeDef * Port)
{
    print(TS_PORT, "\r\nCRKPOSH_CALLS, CRKPOSH_INHIBIT, CRKPOSH_CTRLS_INVALID, CRKPOSH_IGNPOS, CRKPOSH_IGN1SCHED");
    print(TS_PORT, "\r\nCRKPOSH_IGN2SCHED, CRKPOSH_IGN1_UNPOWER, CRKPOSH_IGN2_UNPOWER, CRKPOSH_IGN1_POWER, CRKPOSH_IGN2_POWER");
    print(TS_PORT, "\r\nUPDIGNCTRL_CALLS, UPDIGNCTRL_REVLIM, UPDIGNCTRL_DYN, UPDIGNCTRL_DYN_FAIL");
}


/******************************************************************************************************
Tuareg main diag
*******************************************************************************************************/
void tuareg_diag_log_event(tuareg_diag_t event)
{
    if(event < TDIAG_COUNT)
    {
        tuareg_diag[event] += 1;
    }
}


void print_tuareg_diag(USART_TypeDef * Port)
{
    U32 cnt;

    print(Port, "\r\nTuareg diag: \r\n");

    for(cnt=0; cnt < TDIAG_COUNT; cnt++)
    {
        printf_U(Port, tuareg_diag[cnt], PAD_10);

        if((cnt != 0) && ((cnt % 5) == 0))
        {
            print(Port, "\r\n");
        }

    }
}


void print_tuareg_diag_legend(USART_TypeDef * Port)
{
    print(TS_PORT, "\r\nDECODER_IRQ, DECODER_UPDATE, DECODER_TIMEOUT, DECODER_PASSIVE, IGNITION_IRQ,");
    print(TS_PORT, "\r\nTRIG_IGN_CALLS, TRIG_COIL_DWELL1, TRIG_COIL_DWELL2, TRIG_COIL_IGN1, TRIG_COIL_IGN2,");
    print(TS_PORT, "\r\nPROCESSDATA_CALLS, IGNITIONCALC_CALLS, TSTUDIO_CALLS, MODECTRL, KILL_SIDESTAND,");
    print(TS_PORT, "\r\nKILL_RUNSWITCH, ENTER_INIT, ENTER_HALT, RUNNING_HALT_TR, STB_HALT_TR,");
    print(TS_PORT, "\r\nINIT_HALT_TR, ENTER_RUNNING, CRANKING_RUNNING_TR, HALT_RUNNING_TR, ENTER_STB,");
    print(TS_PORT, "\r\nRUNNING_STB_TR, CRANKING_STB_TR, HALT_STB_TR, ENTER_CRANKING, ENTER_MTEST");
    print(TS_PORT, "\r\nINVALID_RUNMODE");
}


/******************************************************************************************************
decoder diag
*******************************************************************************************************/
void decoder_diag_log_event(decoder_diag_t Event)
{
    if(Event < DDIAG_COUNT)
    {
        decoder_diag[Event] += 1;
    }
}


void print_decoder_diag(USART_TypeDef * Port)
{
    U32 cnt, column = 1;

    print(Port, "\r\nDecoder diag: \r\n");

    for(cnt=0; cnt < DDIAG_COUNT; cnt++)
    {
        printf_U(Port, decoder_diag[cnt], PAD_10);

        if(column == DIAG_PRINT_LEGEND_COLUMNS)
        {
            print(Port, "\r\n");
            column = 1;
        }
        else
        {
            column++;
        }
    }
}


void print_decoder_diag_legend(USART_TypeDef * Port)
{
    print(TS_PORT, "\r\nCRK_EXTI_EVENTS, CRKPOS_INIT, CRKPOS_ASYNC, CRKPOS_ASYNC_KEY, CRKPOS_ASYNC_GAP, ");
    print(TS_PORT, "\r\nCRKPOS_SYNC, SYNCHK_ASYN_FAIL, SYNCHK_ASYN_PASS, SYNCHK_SYN_FAIL, SYNCHK_SYN_PASS,");
    print(TS_PORT, "\r\nUPDATE_IRQ_CALLS, CRK_NOISEF_EVENTS, CAM_NOISEF_EVENTS, TIM_UPDATE_EVENTS, TIMEOUT_EVENTS,");

    print(TS_PORT, "\r\nCAM_EXTI_EVENTS, CISHDL_PRECOND_FAIL, ENA_CIS, CISUPD_CALLS, CISUPD_PRECOND_FAIL,");
    print(TS_PORT, "\r\nCISUPD_TRIGGERED, CISUPD_PHASE_FAIL, CISUPD_PHASE_PASS");
}


/******************************************************************************************************
sensors diag
*******************************************************************************************************/
void sensors_diag_log_event(sensors_diag_t Event)
{
    if(Event < SNDIAG_COUNT)
    {
        sensors_diag[Event] += 1;
    }
}


void print_sensors_diag(USART_TypeDef * Port)
{
    U32 cnt, column = 1;

    print(Port, "\r\nSensors diag: \r\n");

    for(cnt=0; cnt < SNDIAG_COUNT; cnt++)
    {
        printf_U(Port, sensors_diag[cnt], PAD_10);

        if(column == DIAG_PRINT_LEGEND_COLUMNS)
        {
            print(Port, "\r\n");
            column = 1;
        }
        else
        {
            column++;
        }
    }
}


void print_sensors_diag_legend(USART_TypeDef * Port)
{
    print(TS_PORT, "\r\nREAD_DSENSORS_CALLS, ADCIRQ_CALLS, ADCIRQ_INJECTEDGR_CALLS, DMAIRQ_CALLS, DMAIRQ_CH1_CALLS");
}



/******************************************************************************************************
fueling diag
*******************************************************************************************************/
void fueling_diag_log_event(fueling_diag_t event)
{
    if(event < FDIAG_COUNT)
    {
        fueling_diag[event] += 1;
    }
}


void print_fueling_diag(USART_TypeDef * Port)
{
    U32 cnt;

    print(Port, "\r\nfueling diag: \r\n");

    for(cnt=0; cnt < FDIAG_COUNT; cnt++)
    {
        printf_U(Port, fueling_diag[cnt], PAD_10);

        if((cnt != 0) && ((cnt % 5) == 0))
        {
            print(Port, "\r\n");
        }

    }
}


void print_fueling_diag_legend(USART_TypeDef * Port)
{
    print(TS_PORT, "\r\nUPD_CTRLS_CALLS, UPD_CTRLS_OP_PRECOND_FAIL, UPD_CTRLS_CRANKING, UPD_CTRLS_RUNNING, UPD_CTRLS_SEQ,");
    print(TS_PORT, "\r\nUPD_CTRLS_BATCH, UPD_CTRLS_SPD, UPD_CTRLS_ALPHAN, UPD_CTRLS_VE_INVAL, UPD_CTRLS_AFTTGT_INVAL,");
    print(TS_PORT, "\r\nCRKPOSH_CALLS, CRKPOSH_VIT_PRECOND_FAIL, CRKPOSH_INJBEG_POS, CRKPOSH_SEQ_ERROR, CRKPOSH_INJBEG1_SEQ, ");
    print(TS_PORT, "\r\nCRKPOSH_INJBEG2_SEQ, CRKPOSH_INJBEG_BATCH, UPD_BASE_FMAS_CRK_DRY, UPD_ACCELCOMP_RPMSCALED, UPD_ACCELCOMP_ACCEL,");
    print(TS_PORT, "\r\nUPD_ACCELCOMP_DECEL, UPD_ACCELCOMP_TAPERED, UPD_TGTFMASS_PRECOND_FAIL, UPD_TGTFMASS_ASE_ACT, UPD_TGTFMASS_WUE_ACT,");
    print(TS_PORT, "\r\nUPD_TGTFMASS_AE_ACT, UPD_TGTFMASS_ZERO");
}


