#include "stm32f4xx.h"
#include "boctok_types.h"

#include "debug.h"

typedef enum {

    FAULT_HDLR_NONE,

    FAULT_HDLR_HARD,
    FAULT_HDLR_NMI,
    FAULT_HDLR_MEMMGR,
    FAULT_HDLR_BUS,
    FAULT_HDLR_USAGE,
    FAULT_HDLR_SVC,
    FAULT_HDLR_DEBUGMON,
    FAULT_HDLR_PENDSV,
    FAULT_HDLR_WWDG,
    FAULT_HDLR_FPU,

    FAULT_HDLR_COUNT

} fault_handler_name_t;

volatile fault_handler_name_t Pending_Fault= FAULT_HDLR_NONE;

typedef struct _scb_dump_t {

    VU32 usage_faults;
    VU32 bus_faults;
    VU32 memory_faults;


} scb_dump_t;

volatile scb_dump_t SCB_dump;

void getScbDump()
{
    SCB_dump.usage_faults= (SCB->CFSR & SCB_CFSR_USGFAULTSR_Msk) >> SCB_CFSR_USGFAULTSR_Pos;
    SCB_dump.bus_faults= (SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk) >> SCB_CFSR_BUSFAULTSR_Pos;
    SCB_dump.memory_faults= (SCB->CFSR & SCB_CFSR_MEMFAULTSR_Msk) >> SCB_CFSR_MEMFAULTSR_Pos;


}




typedef struct _program_state_t {

    VU32 r0;
    VU32 r1;
    VU32 r2;
    VU32 r3;
    VU32 r12;

    VU32 link_reg;
    VU32 program_counter;
    VU32 program_status_reg;

} program_state_t;

volatile program_state_t Program_State;






void getStackRegisters(U32 *Stack)
{
    Program_State.r0= Stack[0];
    Program_State.r1= Stack[1];
    Program_State.r2= Stack[2];
    Program_State.r3= Stack[3];
    Program_State.r12= Stack[4];

    Program_State.link_reg= Stack[5];
    Program_State.program_counter= Stack[6];
    Program_State.program_status_reg= Stack[7];

    getScbDump();

    //SWO_PrintString("test1\r\n", 0);
    //SWO_PrintString("test2\r\n", 0);
    //SWO_PrintString("test3\r\n", 0);

    __ASM volatile("BKPT #01");
    for( ;; );
}




/*
void printErrorMsg(const char * errMsg)
{
   while(*errMsg != '')
   {
      ITM_SendChar(*errMsg);
      ++errMsg;
   }
}
*/




void HardFault_Handler(void)
{
    Pending_Fault= FAULT_HDLR_HARD;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word getStackRegisters    \n"
    );
}


void NMI_Handler(void)
{
    Pending_Fault= FAULT_HDLR_NMI;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler3_address_const                            \n"
        " bx r2                                                     \n"
        " handler3_address_const: .word getStackRegisters    \n"
    );
}

void MemManage_Handler(void)
{
    Pending_Fault= FAULT_HDLR_MEMMGR;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler4_address_const                            \n"
        " bx r2                                                     \n"
        " handler4_address_const: .word getStackRegisters    \n"
    );
}

void BusFault_Handler(void)
{
    Pending_Fault= FAULT_HDLR_BUS;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler5_address_const                            \n"
        " bx r2                                                     \n"
        " handler5_address_const: .word getStackRegisters    \n"
    );
}

void UsageFault_Handler(void)
{
    Pending_Fault= FAULT_HDLR_USAGE;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler6_address_const                            \n"
        " bx r2                                                     \n"
        " handler6_address_const: .word getStackRegisters    \n"
    );
}


void SVC_Handler(void)
{
    Pending_Fault= FAULT_HDLR_SVC;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler7_address_const                            \n"
        " bx r2                                                     \n"
        " handler7_address_const: .word getStackRegisters    \n"
    );
}

void DebugMon_Handler(void)
{
    Pending_Fault= FAULT_HDLR_DEBUGMON;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler8_address_const                            \n"
        " bx r2                                                     \n"
        " handler8_address_const: .word getStackRegisters    \n"
    );
}

void PendSV_Handler(void)
{
    Pending_Fault= FAULT_HDLR_PENDSV;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler9_address_const                            \n"
        " bx r2                                                     \n"
        " handler9_address_const: .word getStackRegisters    \n"
    );
}

void WWDG_IRQHandler(void)
{
    Pending_Fault= FAULT_HDLR_WWDG;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler10_address_const                            \n"
        " bx r2                                                     \n"
        " handler10_address_const: .word getStackRegisters    \n"
    );
}

void FPU_IRQHandler(void)
{
    Pending_Fault= FAULT_HDLR_FPU;

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler11_address_const                            \n"
        " bx r2                                                     \n"
        " handler11_address_const: .word getStackRegisters    \n"
    );
}


