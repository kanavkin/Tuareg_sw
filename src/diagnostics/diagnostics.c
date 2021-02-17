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

        if(column == 7)
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

    print(Port, "\r\nSCHEDIAG_DELAY_CLIPPED, SCHEDIAG_DELAY_BYPASS");
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
    print(TS_PORT, "\r\nIGNDIAG_CRKPOSH_CALLS, IGNDIAG_CRKPOSH_PRECOND_FAIL, IGNDIAG_CRKPOSH_IGNPOS, IGNDIAG_CRKPOSH_IGN1SCHED_UNPOWER, IGNDIAG_CRKPOSH_IGN2SCHED_UNPOWER, ");
    print(TS_PORT, "\r\nIGNDIAG_CRKPOSH_IGN1_UNPOWER, IGNDIAG_CRKPOSH_IGN2_UNPOWER, IGNDIAG_CRKPOSH_IGN1_POWER, IGNDIAG_CRKPOSH_IGN2_POWER, IGNDIAG_IRQ3H_CALLS,");
    print(TS_PORT, "\r\nIGNDIAG_IRQ3H_PRECOND_FAIL, IGNDIAG_IRQ3H_IGN1SCHED_POWER, IGNDIAG_IRQ3H_IGN2SCHED_POWER, IGNDIAG_IRQ3H_IGN1_POWER, IGNDIAG_IRQ3H_IGN2_POWER,");
    print(TS_PORT, "\r\n IGNDIAG_UPDIGNCTRL_CALLS, IGNDIAG_UPDIGNCTRL_REVLIM, IGNDIAG_UPDIGNCTRL_DYN, IGNDIAG_UPDIGNCTRL_DYN_FAIL");
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


void tuareg_diag_log_parameter(tuareg_diag_t Parameter, U32 Value)
{
    if(Parameter < TDIAG_COUNT)
    {
        tuareg_diag[Parameter]= Value;
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
    print(TS_PORT, "\r\nTDIAG_DECODER_IRQ, TDIAG_DECODER_UPDATE, TDIAG_DECODER_TIMEOUT, TDIAG_DECODER_PASSIVE, TDIAG_IGNITION_IRQ,");
    print(TS_PORT, "\r\nTDIAG_TRIG_IGN_CALLS, TDIAG_TRIG_COIL_DWELL1, TDIAG_TRIG_COIL_DWELL2, TDIAG_TRIG_COIL_IGN1, TDIAG_TRIG_COIL_IGN2,");
    print(TS_PORT, "\r\nTDIAG_PROCESSDATA_CALLS, TDIAG_IGNITIONCALC_CALLS, TDIAG_TSTUDIO_CALLS, TDIAG_MODECTRL, TDIAG_KILL_SIDESTAND,");
    print(TS_PORT, "\r\nTDIAG_KILL_RUNSWITCH, TDIAG_ENTER_INIT, TDIAG_ENTER_HALT, TDIAG_RUNNING_HALT_TR, TDIAG_STB_HALT_TR,");
    print(TS_PORT, "\r\nTDIAG_INIT_HALT_TR, TDIAG_ENTER_RUNNING, TDIAG_CRANKING_RUNNING_TR, TDIAG_HALT_RUNNING_TR, TDIAG_ENTER_STB,");
    print(TS_PORT, "\r\nTDIAG_RUNNING_STB_TR, TDIAG_CRANKING_STB_TR, TDIAG_HALT_STB_TR, TDIAG_ENTER_CRANKING, TDIAG_ENTER_MTEST");
    print(TS_PORT, "\r\nTDIAG_INVALID_RUNMODE");
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


void decoder_diag_log_parameter(decoder_diag_t Parameter, U32 Value)
{
    if(Parameter < DDIAG_COUNT)
    {
        decoder_diag[Parameter]= Value;
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
    print(TS_PORT, "\r\nDDIAG_CRK_EXTI_EVENTS, DDIAG_CRKPOS_INIT, DDIAG_CRKPOS_ASYNC, DDIAG_CRKPOS_ASYNC_KEY, DDIAG_CRKPOS_ASYNC_GAP, ");
    print(TS_PORT, "\r\nDDIAG_CRKPOS_SYNC, DDIAG_SYNCHK_ASYN_FAIL, DDIAG_SYNCHK_ASYN_PASS, DDIAG_SYNCHK_SYN_FAIL, DDIAG_SYNCHK_SYN_PASS,");
    print(TS_PORT, "\r\nDDIAG_UPDATE_IRQ_CALLS, DDIAG_CRK_NOISEF_EVENTS, DDIAG_CAM_NOISEF_EVENTS, DDIAG_TIM_UPDATE_EVENTS, DDIAG_TIMEOUT_EVENTS,");
    print(TS_PORT, "\r\nDDIAG_CAM_EXTI_EVENTS, DDIAG_CISHDL_PRECOND_FAIL, DDIAG_CISUPD_PRECOND_FAIL, DDIAG_CISUPD_INTERVAL_FAIL, DDIAG_CISUPD_PHASE_FAIL,");
    print(TS_PORT, "\r\nDDIAG_CISUPD_PHASE_PASS");
}


