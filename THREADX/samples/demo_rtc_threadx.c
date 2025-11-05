/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     demo_rtc_threadx.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     18-Feb-2021
 * @brief    TestApp to verify RTC Driver using Threadx as an operating system.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Includes ----------------------------------------------------------------- */

/* System Includes */
#include <stdio.h>
#include <inttypes.h>
#include "tx_api.h"

/* Project Includes */
/* include for RTC Driver */
#include "Driver_RTC.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

/* RTC Driver instance 0 */
extern ARM_DRIVER_RTC Driver_RTC0;
static ARM_DRIVER_RTC *RTCdrv = &Driver_RTC0;

void rtc_demo_Thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                 1024
#define TX_RTC_ALARM_EVENT              0x01

TX_THREAD               rtc_thread;
TX_EVENT_FLAGS_GROUP    event_flags_rtc;


/**
  \fn           void alarm_callback(uint32_t event)
  \brief        rtc alarm callback
  \return       none
*/
static void alarm_callback(uint32_t event)
{
    if (event & ARM_RTC_EVENT_ALARM_TRIGGER) {
        /* Received RTC Alarm: Wake-up Thread. */
        tx_event_flags_set(&event_flags_rtc, TX_RTC_ALARM_EVENT, TX_OR);
    }
}

/**
  \fn           void rtc_demo_Thread_entry(ULONG thread_input)
  \brief        RTC demo Thread:
            This thread initializes the RTC. And then in a loop,
            reads the current counter value and sets up an alarm to ring in the future.
            The alarms are setup with increasing timeouts starting with 5 counts and
            ending with 25 counts.
  \param[in]        thread_input : thread input
  \return       none
*/
void rtc_demo_Thread_entry(ULONG thread_input)
{
    UINT  val      = 0;
    UINT  iter     = 5;
    UINT  timeout  = 5;
    ULONG events   = 0;
    INT   ret      = 0;
    ARM_DRIVER_VERSION version;
    ARM_RTC_CAPABILITIES capabilities;

    printf("\r\n >>> RTC demo threadX starting up!!! <<< \r\n");

    version = RTCdrv->GetVersion();
    printf("\r\n RTC version api:%X driver:%X...\r\n",version.api, version.drv);

    capabilities = RTCdrv->GetCapabilities();
    if(!capabilities.alarm) {
        printf("\r\n Error: RTC alarm capability is not available.\n");
        return;
    }

    /* Initialize RTC driver */
    ret = RTCdrv->Initialize(alarm_callback);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC init failed\n");
        return;
    }

    /* Enable the power for RTC */
    ret = RTCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC Power up failed\n");
        goto error_uninitialize;
    }

    while (iter--) {
        ret = RTCdrv->ReadCounter(&val);
        if(ret != ARM_DRIVER_OK) {
            printf("\r\n Error: RTC read failed\n");
            goto error_poweroff;
        }

        printf("\r\n Setting alarm after %" PRIu32 " counts into the future: \r\n", timeout);
        ret = RTCdrv->Control(ARM_RTC_SET_ALARM, val + timeout);
        if(ret != ARM_DRIVER_OK) {
            printf("\r\n Error: RTC Could not set alarm\n");
            goto error_poweroff;
        }

        /* wait till alarm event comes in isr callback */
        ret = tx_event_flags_get(&event_flags_rtc, TX_RTC_ALARM_EVENT, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS) {
            printf("Error: RTC tx_event_flags_get\n");
            goto error_poweroff;
        }

        printf("\r\n Received alarm \r\n");
        timeout += 5;
    }

error_poweroff:

    /* Power off RTC peripheral */
    ret = RTCdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize RTC driver */
    ret = RTCdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC Uninitialize failed.\r\n");
    }

    printf("\r\n XXX RTC demo thread exiting XXX...\r\n");
}


/* Define main entry point.  */
int main()
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}


/* Define what the initial system looks like.  */

void tx_application_define(void *first_unused_memory)
{
    UINT status;

    /* Put system definition stuff in here, e.g. thread creates and other assorted
        create information.  */

    /* Create the event flags group used by RTC thread */
    status = tx_event_flags_create(&event_flags_rtc, "event flags RTC");
    if (status != TX_SUCCESS) {
        printf("Could not create event flags\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&rtc_thread, "rtc_thread", rtc_demo_Thread_entry, 0,
            first_unused_memory, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS) {
        printf("Could not create thread\n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
