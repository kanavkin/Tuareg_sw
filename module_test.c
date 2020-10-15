#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "module_test.h"
#include "debug.h"
#include "scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "ignition_hw.h"


/**
variables for module test
*/

VU32 mtest_scheduler_delay = MODULE_TEST_SCHEDULER_DELAY_MIN;
VU32 mtest_scheduler_state =0;
VU32 mtest_scheduler_tests_started =0;
VU32 mtest_scheduler_tests_finished =0;


void moduletest_main_action()
{





}

void moduletest_irq2_action()
{





}

void moduletest_irq3_action()
{





}

void moduletest_moduleinit_action()
{





}

void moduletest_initmsg_action()
{

                UART_Send(DEBUG_PORT, "\r \n \r \n . \r \n . \r \n . \r \n \r \n *** WARNING This is Tuareg module test unit, no functional release *** \r \n");

                #ifdef TUAREG_MODULE_TEST_SCHEDULER
                UART_Send(DEBUG_PORT, "\r\n*** Scheduler test ***");
                #endif // TUAREG_MODULE_TEST
}


void moduletest_scheduler_main_action()
{
     if(mtest_scheduler_state == 0)
        {
            mtest_scheduler_state= 0xFF;

            dwt_set_begin();

            //ready for test
            set_ignition_ch1(COIL_DWELL);
            scheduler_set_channel(IGN_CH1, COIL_IGNITION, mtest_scheduler_delay);

            mtest_scheduler_tests_started++;

            //increase delay
            if((mtest_scheduler_delay + MODULE_TEST_SCHEDULER_DELAY_INCREMENT) < MODULE_TEST_SCHEDULER_DELAY_MAX)
            {
                mtest_scheduler_delay += MODULE_TEST_SCHEDULER_DELAY_INCREMENT;
            }
        }
}

void moduletest_scheduler_irq3_action()
{
    //scheduler cycle end
    dwt_set_end();

    mtest_scheduler_tests_finished++;

    //print result
    UART_Send(DEBUG_PORT, "\r\nscheduler tests started, finished, target delay (us): ");
    UART_Print_U(DEBUG_PORT, mtest_scheduler_tests_started, TYPE_U32, NO_PAD);
    UART_Print_U(DEBUG_PORT, mtest_scheduler_tests_finished, TYPE_U32, NO_PAD);
    UART_Print_U(DEBUG_PORT, mtest_scheduler_delay, TYPE_U32, NO_PAD);

    print_dwt_delay();

    print_scheduler_diag(DEBUG_PORT);

    mtest_scheduler_state =0;




}
