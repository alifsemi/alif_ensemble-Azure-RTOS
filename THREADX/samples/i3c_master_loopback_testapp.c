/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/**************************************************************************//**
 * @file     i3c_masterside_loopback_testapp.c
 * @author   Prabhakar kumar
 * @email    prabhakar.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     27-May-2023
 * @brief    TestApp to verify master loop back test
 *           - Data transmitted from master and slave receive's it.And,
 *             same data is transmitted from slave and master receive it.
 *
 *           - I3C master configuration
 *             Select appropriate i3c Speed mode as per i2c or i3c slave device.
 *             I3C_BUS_MODE_PURE                             : Only Pure I3C devices
 *             I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 Mbps
 *             I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 Kbps
 *             I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 Kbps
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


/* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Project Includes */
/* I3C Driver */
#include "Driver_I3C.h"
#include "system_utils.h"

/* PINMUX Driver */
#include "pinconf.h"

/* For Release build disable printf and semihosting */
#define DISABLE_PRINTF

#ifdef DISABLE_PRINTF
    #define printf(fmt, ...) (0)
    /* Also Disable Semihosting */
    #if __ARMCC_VERSION >= 6000000
            __asm(".global __use_no_semihosting");
    #elif __ARMCC_VERSION >= 5000000
            #pragma import(__use_no_semihosting)
    #else
            #error Unsupported compiler
    #endif

    void _sys_exit(int return_code) {
            while (1);
    }
#endif

/* i3c Driver instance 0 */
extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

/* I3C slave target address */
#define I3C_SLV_TAR           (0x64)

/* transmit buffer from i3c */
uint8_t tx_data[1];
/* receive buffer from i3c */
uint8_t rx_data[1];

void i3c_master_loopback_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE             1024
#define DEMO_BYTE_POOL_SIZE         9120

TX_THREAD               I3C_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags_i3c;

/* i3c callback events */
typedef enum {
    I3C_CB_EVENT_SUCCESS        = (1 << 0),
    I3C_CB_EVENT_ERROR          = (1 << 1),
    I3C_CB_EVENT_MST_TX_DONE    = (1 << 2),
    I3C_CB_EVENT_MST_RX_DONE    = (1 << 3),
    I3C_CB_EVENT_SLV_TX_DONE    = (1 << 4),
    I3C_CB_EVENT_SLV_RX_DONE    = (1 << 5),
    I3C_CB_EVENT_DYN_ADDR_ASSGN = (1 << 6)
}I3C_CB_EVENTS;

/**
  \fn          INT hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   void
  \return      0:success; -1:failure
*/
int32_t hardware_init(void)
{
    /* I3C_SDA_B */
    pinconf_set(PORT_1, PIN_2, PINMUX_ALTERNATE_FUNCTION_3, \
            PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
            PADCTRL_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS);

    /* I3C_SCL_B */
    pinconf_set( PORT_1, PIN_3, PINMUX_ALTERNATE_FUNCTION_3, \
            PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
            PADCTRL_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS);

    return ARM_DRIVER_OK;
}

/**
  \fn          void I3C_callback(UINT event)
  \brief       i3c isr callback
  \param[in]   event: i3c Event
  \return      none
*/
void I3C_callback(UINT event)
{
    if (event & ARM_I3C_EVENT_TRANSFER_DONE)
    {
        /* Transfer Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_SUCCESS, TX_OR);
    }

    if (event & ARM_I3C_EVENT_TRANSFER_ERROR)
    {
        /* Transfer Error: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_ERROR, TX_OR);
    }
    if (event & ARM_I3C_EVENT_MST_TX_DONE)
    {
        /* Master Transfer Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_MST_TX_DONE, TX_OR);
    }
    if (event & ARM_I3C_EVENT_MST_RX_DONE)
    {
        /* Master Receive Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_MST_RX_DONE, TX_OR);
    }
    if (event & ARM_I3C_EVENT_SLV_TX_DONE)
    {
        /* Slave Transfer Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_SLV_TX_DONE, TX_OR);
    }
    if (event & ARM_I3C_EVENT_SLV_RX_DONE)
    {
        /* Slave Receive Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_SLV_RX_DONE, TX_OR);
    }
    if (event & ARM_I3C_EVENT_SLV_DYN_ADDR_ASSGN)
    {
        /* Dynamic Address Assign Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_DYN_ADDR_ASSGN, TX_OR);
    }
}

/**
  \fn          void i3c_master_loopback_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify i3c slave mode
               This demo thread does:
                 - initialize i3c driver;
                 - set i3c speed mode to I3C_BUS_MODE_PURE;
                 - Data transmitted from master and slave receive's it.And,
                   same data is transmitted from slave and master receive it.
  \param[in]   thread_input : thread input
  \return      none
*/
void i3c_master_loopback_demo_thread_entry(ULONG thread_input)
{
    INT   ret    = 0;
    INT   len    = 0;

    ULONG actual_events = 0;
    ARM_DRIVER_VERSION version;

    /* Array of slave address :
     *       Dynamic Address for i3c and
     *       Static  Address for i2c
     */
    UCHAR slave_addr[1] =
    {
        0, /* I3C Accero  Dynamic Address: To be updated later using MasterAssignDA */
    };

    printf("\r\n \t\t >>> Master loop back demo with Azure RTOS ThreadX starting up!!! <<< \r\n");

    /* Get i3c driver version. */
    version = I3Cdrv->GetVersion();
    printf("\r\n i3c version api:0x%X driver:0x%X \r\n",  \
                           version.api, version.drv);

    /* Initialize i3c hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != 0)
    {
        printf("\r\n Error: i3c hardware_init failed.\r\n");
        return;
    }

    /* Initialize I3C driver */
    ret = I3Cdrv->Initialize(I3C_callback);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Initialize failed.\r\n");
        return;
    }

    /* Power up I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Power Up failed.\r\n");
        goto error_uninitialize;
    }

    /* i3c Speed Mode Configuration:
     *  I3C_BUS_MODE_PURE                             : Only Pure I3C devices
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 Mbps
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 Kbps
     *  I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 Kbps
     */
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,  \
                          I3C_BUS_MODE_PURE);

    PMU_delay_loop_us(1000);

    /* Attach all i3c slave using dynamic address */

    /* Assign Dynamic Address to i3c slave */
    printf("\r\n >> i3c: Get dynamic addr for static addr:0x%X.\r\n",I3C_SLV_TAR);

    ret = I3Cdrv->MasterAssignDA(&slave_addr[0], I3C_SLV_TAR);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C MasterAssignDA failed.\r\n");
        goto error_poweroff;
    }
    printf("\r\n >> i3c: Received dyn_addr:0x%X for static addr:0x%X. \r\n",   \
                                 slave_addr[0],I3C_SLV_TAR);

    /* Delay */
    PMU_delay_loop_us(1000);

    /* set tx_data to 0 */
    tx_data[0] =  0x00;

    while(1)
    {
        len = 1;

        /* Delay */
        PMU_delay_loop_us(100);

        tx_data[0] += 1;

        /* Master transmit */
        ret = I3Cdrv->MasterTransmit(slave_addr[0],&tx_data[0], len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Master Transmit failed. \r\n");
            goto error_poweroff;
        }

        ret = tx_event_flags_get(&event_flags_i3c, \
                           I3C_CB_EVENT_MST_TX_DONE |I3C_CB_EVENT_ERROR,\
                           TX_OR_CLEAR,                               \
                           &actual_events,                            \
                           TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf("Error: I3C tx_event_flags_get failed.\n");
            goto error_poweroff;
        }

        /* Delay */ /* FIXME use tx_thread_sleep */
        PMU_delay_loop_us(1000);

        /* Reset rx_data buffer */
        rx_data[0] = 0x00;

        /* Master receive */
        ret = I3Cdrv->MasterReceive(slave_addr[0], &rx_data[0], len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Master Receive failed. \r\n");
            goto error_poweroff;
        }

        ret = tx_event_flags_get(&event_flags_i3c, \
                           I3C_CB_EVENT_MST_RX_DONE|I3C_CB_EVENT_ERROR, \
                           TX_OR_CLEAR,                               \
                           &actual_events,                            \
                           TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf("Error: I3C rx_event_flags_get failed.\n");
            goto error_poweroff;
        };

        /* tx_data and rx_data doesn't match stop */
        if(tx_data[0] != rx_data[0])
        {
           printf("\nFAILED >>> Stopped at while(1)\n");
           while(1);
        }

    }

error_poweroff:

    /* Power off I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
         printf("\r\n Error: I3C Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize I3C driver */
    ret = I3Cdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Uninitialize failed.\r\n");
    }

    printf("\r\n XXX I3C demo thread exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

/* Define what the initial system looks like.  */
void tx_application_define(void *first_unused_memory)
{
    CHAR    *pointer = TX_NULL;
    INT      status  = 0;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Create the event flags group used by i3c thread  */
    status = tx_event_flags_create(&event_flags_i3c, "event flags I3C");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&I3C_thread, "I3C_thread", i3c_master_loopback_demo_thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread \n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
