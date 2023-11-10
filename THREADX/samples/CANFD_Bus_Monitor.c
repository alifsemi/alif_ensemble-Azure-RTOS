/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     CANFD_Bus_Monitor.c
 * @author   Shreehari H K
 * @email    shreehari.hk@alifsemi.com
 * @version  V1.0.0
 * @date     01-Sept-2023
 * @brief    ThreadX demo application for CANFD.
 *           - Performs Bus Monitor test (Listen only mode).
 * @bug      None
 * @Note     None
 ******************************************************************************/

#include <stdio.h>
#include "tx_api.h"

#include <RTE_Components.h>
#include CMSIS_device_header
#include "pinconf.h"
#include "Driver_CAN.h"

#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

#include "se_services_port.h"

/* It is recommended to use the bit rate and bit segments
 * as specified in the Hardware reference manual for proper communication.
 *
 * Nominal bit rate 500kbps, Fast bit rate 2Mbps, 20MHz CANFD clock are set
 * for this example
*/
#define CANFD_NOMINAL_BITRATE               500000U
#define CANFD_BIT_TIME_PROP_SEG             2U
#define CANFD_BIT_TIME_SEG1                 30U
#define CANFD_BIT_TIME_SEG2                 8U
#define CANFD_BIT_TIME_SJW                  8U
#define CANFD_FAST_BITRATE                  2000000U
#define CANFD_FAST_BIT_TIME_PROP_SEG        1U
#define CANFD_FAST_BIT_TIME_SEG1            7U
#define CANFD_FAST_BIT_TIME_SEG2            2U
#define CANFD_FAST_BIT_TIME_SJW             2U

#define CANFD_NOMINAL_BITTIME_SEGMENTS      ((CANFD_BIT_TIME_PROP_SEG << 0U)      | \
                                            (CANFD_BIT_TIME_SEG1 << 8U)           | \
                                            (CANFD_BIT_TIME_SEG2 << 16U)          | \
                                            (CANFD_BIT_TIME_SJW << 24U))

#define CANFD_FAST_BITTIME_SEGMENTS         ((CANFD_FAST_BIT_TIME_PROP_SEG << 0U) | \
                                            (CANFD_FAST_BIT_TIME_SEG1 << 8U)      | \
                                            (CANFD_FAST_BIT_TIME_SEG2 << 16U)     | \
                                            (CANFD_FAST_BIT_TIME_SJW << 24U))

/* Object filter settings */
#define CANFD_OBJECT_FILTER_CODE            0x5A5U
#define CANFD_OBJECT_FILTER_MASK            0U

#define CANFD_MAX_MSG_SIZE                  64U

/* Define for ThreadX notification objects */
#define CANFD_RX_SUCCESS                    0x01U
#define CANFD_ERROR                         0x02U
#define CANFD_ALL_NOTIFICATIONS             (CANFD_RX_SUCCESS | CANFD_ERROR)

/* Define the ThreadX object control blocks...  */
#define THREAD_STACK_SIZE                   1024U

TX_THREAD               canfd_thread;
TX_EVENT_FLAGS_GROUP    event_flags_canfd;

/* CANFD instance object */
extern ARM_DRIVER_CAN  Driver_CANFD;
static ARM_DRIVER_CAN* CANFD_instance    = &Driver_CANFD;

/* File Global variables */
volatile bool is_msg_read                = false;
volatile bool bus_error                  = false;
volatile bool bus_off                    = false;
volatile bool passive_mode               = false;
uint8_t       rx_obj_id                  = 255U;
ARM_CAN_MSG_INFO rx_msg_header;
volatile uint8_t rx_msg_size             = 8U;
uint8_t rx_data[CANFD_MAX_MSG_SIZE + 1U];

/* A map between Data length code to the payload size */
const uint8_t canfd_len_dlc_map[0x10U] =
              {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

/* Support functions */
static void canfd_process_rx_message(void);
static void canfd_check_error(void);

/**
 * @fn      static int32_t pinmux_config(void)
 * @brief   CANFD Rx and Tx pinmux configuration.
 * @note    none
 * @param   none
 * @retval  execution status.
 */
static int32_t pinmux_config(void)
{
    int32_t ret_val = 0U;

    /* pinmux configurations for CANFD pins */
    ret_val = pinconf_set(PORT_7, PIN_0, PINMUX_ALTERNATE_FUNCTION_7,
                         (PADCTRL_READ_ENABLE | PADCTRL_SCHMITT_TRIGGER_ENABLE |
                          PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if(ret_val)
    {
        printf("ERROR: Failed to configure PINMUX for CANFD Rx \r\n");
        return ret_val;
    }

    ret_val = pinconf_set(PORT_7, PIN_1, PINMUX_ALTERNATE_FUNCTION_7,
                         (PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA |
                          PADCTRL_SCHMITT_TRIGGER_ENABLE));
    if(ret_val)
    {
        printf("ERROR: Failed to configure PINMUX for CANFD Tx \r\n");
        return ret_val;
    }
    return ret_val;
}

/**
 * @fn      void cb_unit_event(uint32_t event)
 * @brief   CANFD Callback function for events
 * @note    none
 * @param   event: CANFD event
 * @retval  none
 */
void cb_unit_event(uint32_t event)
{
    if(event == ARM_CAN_EVENT_UNIT_ACTIVE)
    {
        passive_mode = false;
    }
    else if(event == ARM_CAN_EVENT_UNIT_BUS_OFF)
    {
        /* Set bus off flag when bus is OFF */
        bus_off = true;
    }
    else if(event == ARM_CAN_EVENT_UNIT_WARNING)
    {
        /* Set bus error flag when bus warning occurred */
        bus_error = true;
    }
    else if(event == ARM_CAN_EVENT_UNIT_PASSIVE)
    {
        /* Set passive mode flag when bus passive error occurred */
        passive_mode = true;
    }

    if(bus_error || passive_mode || bus_off)
    {
        /* Communication error occurred - Notify the task */
        tx_event_flags_set(&event_flags_canfd, CANFD_ERROR, TX_OR);
    }
}

/**
 * @fn      void cb_object_event(uint32_t obj_idx, uint32_t event)
 * @brief   CANFD Callback function for particular object events
 * @note    none
 * @param   obj_idx : Object ID
 * @param   event   : CANFD event
 * @retval  none
 */
void cb_object_event(uint32_t obj_idx, uint32_t event)
{
    if((event & ARM_CAN_EVENT_RECEIVE) || (event & ARM_CAN_EVENT_RECEIVE_OVERRUN))
    {
        /* Sets msg_rx_complete if the Receive Object matches */
        if(obj_idx == rx_obj_id)
        {
            /* Sets the flag to indicate that the received msg is not read */
            is_msg_read = false;

            /* Rx Success - Notify the task*/
            tx_event_flags_set(&event_flags_canfd, CANFD_RX_SUCCESS, TX_OR);
        }
    }
}

/**
 * @fn      void canfd_lom_demo_task(ULONG thread_input)
 * @brief   CANFD Listen only mode Demo
 * @note    none.
 * @param   thread_input : Input for thread
 * @retval  none
 */
void canfd_lom_demo_task(ULONG thread_input)
{
    int32_t ret_val                 = ARM_DRIVER_OK;
    UINT    event_ret               = 0U;
    ARM_CAN_CAPABILITIES              can_capabilities;
    ARM_CAN_OBJ_CAPABILITIES          can_obj_capabilities;
    uint8_t iter                    = 0U;
    ULONG task_notified_value       = 0U;
    uint32_t error_code             = 0U;
    uint32_t service_error_code     = 0U;

    /* Initialize the SE services */
    se_services_port_init();

    /* Enables the HFOSC clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_HFOSC,
                                              true,
                                              &service_error_code);
    if(error_code)
    {
        printf("SE Error: HFOSC clk enable = %d\n", (int)error_code);
        return;
    }


    /* Enables the 160MHz clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_160M,
                                              true,
                                              &service_error_code);
    if(error_code)
    {
        printf("SE Error: 160 MHz clk enable = %d\n", (int)error_code);
        return;
    }


    printf("*** CANFD Listen only mode Demo app is starting ***\n");

    ret_val = pinmux_config();
    if(ret_val != ARM_DRIVER_OK)
    {
        printf("Error in pin-mux configuration\n");
        return;
    }

    /* Get CANFD capabilities */
    can_capabilities = CANFD_instance->GetCapabilities();
    printf("Num of objects supported: %d\r\n", can_capabilities.num_objects);

    /* Initializing CANFD Access struct */
    ret_val = CANFD_instance->Initialize(cb_unit_event, cb_object_event);
    if(ret_val != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to initialize the CANFD \n");
        return;
    }

    /* Powering up CANFD */
    ret_val = CANFD_instance->PowerControl(ARM_POWER_FULL);
    if(ret_val != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to Power up the CANFD \n");
        goto uninitialise_canfd;
    }

    /* Setting CANFD to FD mode */
    if(can_capabilities.fd_mode == 1U)
    {
        CANFD_instance->Control(ARM_CAN_SET_FD_MODE, ENABLE);
        if(ret_val != ARM_DRIVER_OK)
        {
           printf("ERROR: CANFD Enabling FD mode failed\r\n");
           goto power_off_canfd;
        }
    }

    /* Initializing up CANFD module */
    ret_val = CANFD_instance->SetMode(ARM_CAN_MODE_INITIALIZATION);
    if(ret_val != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to set CANFD to INIT mode \r\n");
        goto power_off_canfd;
    }
    /* Setting bit rate for CANFD */
    ret_val = CANFD_instance->SetBitrate(ARM_CAN_BITRATE_NOMINAL,
                                         CANFD_NOMINAL_BITRATE,
                                         CANFD_NOMINAL_BITTIME_SEGMENTS);
    if(ret_val != ARM_DRIVER_OK)
    {
       printf("ERROR: Failed to set CANFD Nominal Bitrate\r\n");
       goto power_off_canfd;
    }
    /* Setting bit rate for CANFD */
    if(can_capabilities.fd_mode == 1U)
    {
        ret_val = CANFD_instance->SetBitrate(ARM_CAN_BITRATE_FD_DATA,
                                             CANFD_FAST_BITRATE,
                                             CANFD_FAST_BITTIME_SEGMENTS);
        if(ret_val != ARM_DRIVER_OK)
        {
           printf("ERROR: Failed to set CANFD Fast Bitrate\r\n");
           goto power_off_canfd;
        }
    }

    /* Assign IDs to Rx object*/
    for(iter = 0U; iter < can_capabilities.num_objects; iter++)
    {
        can_obj_capabilities = CANFD_instance->ObjectGetCapabilities(iter);
        if((can_obj_capabilities.rx == 1U) && (rx_obj_id == 255U))
        {
            rx_obj_id = iter;
        }
    }

    ret_val = CANFD_instance->ObjectConfigure(rx_obj_id, ARM_CAN_OBJ_RX);
    if(ret_val != ARM_DRIVER_OK)
    {
       printf("ERROR: Object Rx configuration failed\r\n");
       goto power_off_canfd;
    }
    /* Setting Object filter of CANFD */
    ret_val = CANFD_instance->ObjectSetFilter(rx_obj_id,
                                              ARM_CAN_FILTER_ID_EXACT_ADD,
                                              CANFD_OBJECT_FILTER_CODE,
                                              CANFD_OBJECT_FILTER_MASK);
    if(ret_val == ARM_DRIVER_ERROR_SPECIFIC)
    {
       printf("ERROR: No free Filter available\r\n");
    }
    else if(ret_val != ARM_DRIVER_OK)
    {
       printf("ERROR: Failed to set CANFD Object filter\r\n");
       goto power_off_canfd;
    }

    /* Setting CANFD to Normal mode */
    ret_val = CANFD_instance->SetMode(ARM_CAN_MODE_MONITOR);
    if(ret_val != ARM_DRIVER_OK)
    {
       printf("ERROR: Failed to set CANFD to Listen only mode\r\n");
       goto power_off_canfd;
    }

    /* wait for receive/error callback. */
    event_ret = tx_event_flags_get(&event_flags_canfd,
                                   CANFD_ALL_NOTIFICATIONS, TX_OR_CLEAR,
                                   &task_notified_value, TX_WAIT_FOREVER);
    if(event_ret != TX_SUCCESS)
    {
        printf("Error: CANFD event flags\n");
        goto power_off_canfd;
    }

    /* Checks if both callbacks are successful */
    if(task_notified_value & CANFD_RX_SUCCESS)
    {
        /* Invokes received message process function */
        canfd_process_rx_message();
    }
    else if(task_notified_value & CANFD_ERROR)
    {
        /* Invoke the below function to check on errors */
        canfd_check_error();
    }

power_off_canfd:
/* Powering OFF CANFD module */
    if(CANFD_instance->PowerControl(ARM_POWER_OFF) != ARM_DRIVER_OK)
    {
       printf("ERROR in CANFD power off\r\n");
    }

uninitialise_canfd:
    /*  Un-initialising CANFD module */
    if(CANFD_instance->Uninitialize() != ARM_DRIVER_OK)
    {
        printf("ERROR in CANFD un-initialization\r\n");
    }

    /* Disables the HFOSC clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_HFOSC,
                                              false,
                                              &service_error_code);
    if(error_code)
    {
        printf("SE Error: HFOSC clk disable = %d\n", (int)error_code);
        return;
    }

    /* Disables the 160MHz clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_160M,
                                              false,
                                              &service_error_code);
    if(error_code)
    {
        printf("SE Error: 160 MHz clk disable = %d\n", (int)error_code);
        return;
    }

    printf("*** CANFD Listen only Mode Demo is ended ***\r\n");
}

/**
 * @fn      int main()
 * @brief   main function of the CANFD Application
 * @note    none
 * @param   none
 * @retval  software execution status
 */
int main()
{
    /* System Initialization */
   SystemCoreClockUpdate();

#if defined(RTE_Compiler_IO_STDOUT_User)
    int32_t ret;
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
#endif

   /* Enter the ThreadX kernel.  */
   tx_kernel_enter();

   return 0;
}

/**
 * @fn      void tx_application_define(void *first_unused_memory)
 * @brief   ThreadX entry point function
 * @note    none
 * @param   first_unused_memory : First unused memory
 * @retval  None
 */
void tx_application_define(void *first_unused_memory)
{
    UINT status;

    /* Put system definition stuff in here, e.g. thread creates and other assorted
        create information.  */

    /* Create the event flags group used by CANFD thread */
    status = tx_event_flags_create(&event_flags_canfd, "CANFD Events");
    if(status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&canfd_thread, "CANFD_LOM", canfd_lom_demo_task,
                              0U, first_unused_memory, THREAD_STACK_SIZE, 1U, 1U,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if(status != TX_SUCCESS)
    {
        printf("Unable to Create LOM Task\n");
        return;
    }
}
/**
 * @fn      void canfd_check_error(void)
 * @brief   Checks for the errors in CANFD
 * @note    none
 * @param   none
 * @retval  none
 */
void canfd_check_error(void)
{
    ARM_CAN_STATUS cur_sts;

    if(bus_error)
    {
        /* Getting the current CANFD status */
        cur_sts = CANFD_instance->GetStatus();
        /* In LOM if an ACK error occurs  */
        if(cur_sts.last_error_code == ARM_CAN_LEC_ACK_ERROR)
        {
            /*  Reading arrived CANFD Message */
            if(CANFD_instance->MessageRead(rx_obj_id, &rx_msg_header,
                                           rx_data, rx_msg_size) != ARM_DRIVER_OK)
            {
                printf("Error: Message reception failed\r\n");
            }
            else
            {
                /* Sets the below to process the received message */
                is_msg_read = true;
                canfd_process_rx_message();
            }
        }
        else
        {
            printf("Error in CANFD-->Error code: %d\r\n", cur_sts.last_error_code);
        }
        bus_error = false;
    }
    /* If bus is off then raise an error */
    if(bus_off)
    {
        printf("Error: CAN Bus if Off\r\n");
        bus_off = false;
    }
    /* If canfd is in passive mode then raise an error */
    if(passive_mode)
    {
        printf("Error: CANFD In Error Passive mode:\r\n");
    }
}

/**
 * @fn      void canfd_process_rx_message(void)
 * @brief   Processes the received messages
 * @note    none
 * @param   none
 * @retval  none
 */
void canfd_process_rx_message(void)
{
    uint8_t iter = 0U;

    if(!is_msg_read)
    {
        /*  Reading arrived CANFD Message */
        if(CANFD_instance->MessageRead(rx_obj_id, &rx_msg_header,
                                       rx_data, rx_msg_size) != ARM_DRIVER_OK)
        {
            printf("Error: Message reception failed\r\n");
            return;
        }
    }
    /* Checking if a new message is received. If yes
     * performs the below operations */
    if(rx_msg_header.rtr == 1U)
    {
        printf("Rx msg:\r\n    Type:Remote frame, Id:%lu",
               (rx_msg_header.id & (~ARM_CAN_ID_IDE_Msk)));
    }
    else
    {
        printf("Rx msg:\r\n    Type:Data frame, ");

        /* Checks if expected Rx msg length is equal to actual length */
        if(rx_msg_size == canfd_len_dlc_map[rx_msg_header.dlc])
        {
            /* If any error is present in the Rx message */
            if(rx_msg_header.esi)
            {
                printf("\r\n    Error Occurred in Rx message \r\n");
                return;
            }
            printf("Id:%lu, Len:%d:\r\n    Data:",
                   (rx_msg_header.id & (~ARM_CAN_ID_IDE_Msk)), rx_msg_size);
            for(iter = 0; iter < rx_msg_size; iter++)
            {
                printf("%c", rx_data[iter]);
            }
        }
        else
        {
            printf("Error: Rx msg length is not as expected");
        }
    }
    printf("\r\n");
}
