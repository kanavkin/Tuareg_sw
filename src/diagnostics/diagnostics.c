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

        if(column == 8)
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
    print(Port, "\r\nICH1 - SET, SET_CURRC, SET_NEXTC_PRELOAD, SET_NEXTC_UPDATE, SET_RETRIGD, TRIGGERED, RESET");
    print(Port, "\r\nICH2 - SET, SET_CURRC, SET_NEXTC_PRELOAD, SET_NEXTC_UPDATE, SET_RETRIGD, TRIGGERED, RESET");
    print(Port, "\r\nFCH1 - SET, SET_CURRC, SET_NEXTC_PRELOAD, SET_NEXTC_UPDATE, SET_RETRIGD, TRIGGERED, RESET");
    print(Port, "\r\nFCH2 - SET, SET_CURRC, SET_NEXTC_PRELOAD, SET_NEXTC_UPDATE, SET_RETRIGD, TRIGGERED, RESET");

    print(Port, "\r\nSCHEDIAG_DELAY_CLIPPED, SCHEDIAG_DELAY_BYPASS, WRAP");
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
    print(TS_PORT, "\r\nCRKPOSH_CALLS, CRKPOSH_PRECOND_FAIL, CRKPOSH_IGNPOS, CRKPOSH_IGN1SCHED_UNPOWER, CRKPOSH_IGN2SCHED_UNPOWER, ");
    print(TS_PORT, "\r\nCRKPOSH_IGN1_UNPOWER, CRKPOSH_IGN2_UNPOWER, CRKPOSH_IGN1_POWER, CRKPOSH_IGN2_POWER, IRQ3H_CALLS,");
    print(TS_PORT, "\r\nIRQ3H_PRECOND_FAIL, IRQ3H_IGN1SCHED_POWER, IRQ3H_IGN2SCHED_POWER, IRQ3H_IGN1_POWER, IRQ3H_IGN2_POWER,");
    print(TS_PORT, "\r\n UPDIGNCTRL_CALLS, UPDIGNCTRL_REVLIM, UPDIGNCTRL_DYN, UPDIGNCTRL_DYN_FAIL, IGNITION_LOC_SEQUENTIAL_FAIL");
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
    print(TS_PORT, "\r\nCAM_EXTI_EVENTS, CISHDL_PRECOND_FAIL, CISUPD_PRECOND_FAIL, CISUPD_INTERVAL_FAIL, CISUPD_PHASE_FAIL,");
    print(TS_PORT, "\r\nCISUPD_PHASE_PASS");
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
    print(TS_PORT, "\r\nCRKPOSH_CALLS, CRKPOSH_PRECOND_FAIL, CRKPOSH_IGNPOS, CRKPOSH_IGN1SCHED_UNPOWER, CRKPOSH_IGN2SCHED_UNPOWER, ");
    print(TS_PORT, "\r\nCRKPOSH_IGN1_UNPOWER, CRKPOSH_IGN2_UNPOWER, CRKPOSH_IGN1_POWER, CRKPOSH_IGN2_POWER, IRQ3H_CALLS,");
    print(TS_PORT, "\r\nIRQ3H_PRECOND_FAIL, IRQ3H_IGN1SCHED_POWER, IRQ3H_IGN2SCHED_POWER, IRQ3H_IGN1_POWER, IRQ3H_IGN2_POWER,");
    print(TS_PORT, "\r\n UPDIGNCTRL_CALLS, UPDIGNCTRL_REVLIM, UPDIGNCTRL_DYN, UPDIGNCTRL_DYN_FAIL, IGNITION_LOC_SEQUENTIAL_FAIL");
}


/******************************************************************************************************
fueling diag
*******************************************************************************************************/
void fueling_diag_log_event(fueling_diag_t event)
{
    if(event < TDIAG_COUNT)
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
    print(TS_PORT, "\r\n,");
}
