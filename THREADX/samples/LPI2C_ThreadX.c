/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/******************************************************************************
 * @file     LPI2C_ThreadX.c
 * @author   Shreehari H K
 * @email    shreehari.hk@alifsemi.com
 * @version  V1.0.0
 * @date     07-Sept-2024
 * @brief    TestApp to verify I2C Master and LPI2C Slave functionality
 *           using ThreadX without any operating system.
 * @bug      None
 * @note     Code will verify:
 *            1.)Master transmit and Slave receive
 *            2.)Master receive  and Slave transmit
 *                I2C0 instance is taken as Master and
 *                LPI2C(Slave-only) instance is taken as Slave.
 *
 *           Hardware Connection:
 *           I2C0 SDA(P3_5) -> LPI2C SDA(P5_3)
 *           I2C0 SCL(P3_4) -> LPI2C SCL(P5_2)
 ******************************************************************************/
/* Include */
#include <stdio.h>
#include <string.h>
#include "tx_api.h"

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I2C.h"
#include "pinconf.h"

#if !defined(M55_HE)
#error "This Demo application works only on RTSS_HE"
#endif

#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

/* I2C Driver instance */
extern ARM_DRIVER_I2C Driver_I2C0;
static ARM_DRIVER_I2C *I2C_mstdrv = &Driver_I2C0;

extern ARM_DRIVER_I2C Driver_LPI2C;
static ARM_DRIVER_I2C *LPI2C_slvdrv = &Driver_LPI2C;

/* Define the ThreadX object control blocks...  */
#define THREAD_STACK_SIZE      1024U
#define MST_THREAD_PRIORITY    1U
#define SLV_THREAD_PRIORITY    (MST_THREAD_PRIORITY + 1U)

static TX_THREAD               master_thread;
static TX_THREAD               slave_thread;
static TX_EVENT_FLAGS_GROUP    event_flags_master;
static TX_EVENT_FLAGS_GROUP    event_flags_slave;

/* LPI2C callback events */
typedef enum _LPI2C_CB_EVENT{
    LPI2C_CB_EVENT_SUCCESS    = (1 << 0),
    LPI2C_CB_EVENT_ERROR      = (1 << 1)
}LPI2C_CB_EVENT;

#define TAR_ADDRS         (0x40)   /* Target(Slave) Address, use by Master */
#define RESTART           (0x01)
#define STOP              (0x00)

/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT            21

/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT            22

/* Master parameter set */

/* Master TX Data (Any random value). */
static uint8_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT+1] ={"Test_Message_to_Slave"};

/* master receive buffer */
static uint8_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT+1];

/* Master parameter set END  */

/* Slave parameter set */

/* slave receive buffer */
static uint8_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT+1];

/* Slave TX Data (Any random value). */
static uint8_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT+1]={"Test_Message_to_master"};

/* Slave parameter set END */

/**
 * @fn      static void i2c_mst_tranfer_callback(uint32_t event)
 * @brief   I2C Callback function for events
 * @note    none
 * @param   event: I2C event
 * @retval  none
 */
static void i2c_mst_tranfer_callback(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE)
    {
        /* xfer success - Notify the task*/
        tx_event_flags_set(&event_flags_master, LPI2C_CB_EVENT_SUCCESS, TX_OR);
    }

    else
    {
        /* xfer failure - Notify the task*/
        tx_event_flags_set(&event_flags_master, LPI2C_CB_EVENT_ERROR, TX_OR);
    }
}

/**
 * @fn      static void i2c_slv_transfer_callback(uint32_t event)
 * @brief   LPI2C Callback function for events
 * @note    none
 * @param   event: LPI2C event
 * @retval  none
 */
static void i2c_slv_transfer_callback(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE)
    {
        /* xfer success - Notify the task*/
        tx_event_flags_set(&event_flags_slave, LPI2C_CB_EVENT_SUCCESS, TX_OR);
    }

    else
    {
        /* xfer failure - Notify the task*/
        tx_event_flags_set(&event_flags_slave, LPI2C_CB_EVENT_ERROR, TX_OR);
    }
}
/**
 * @fn      static void pinmux_config(void)
 * @brief   I2C and LPI2C SCL and SDA pinmux configuration.
 * @note    Pinmux for B0
 * @param   none
 * @retval  none
 */
static void pinmux_config()
{
    /* LPI2C_SDA_B */
    pinconf_set(PORT_5, PIN_3, PINMUX_ALTERNATE_FUNCTION_4,
                (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                 PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));

    /* LPI2C_SCL_B */
    pinconf_set(PORT_5, PIN_2, PINMUX_ALTERNATE_FUNCTION_5,
                (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                 PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));

    /* I2C0_SDA_B */
    pinconf_set(PORT_3, PIN_5, PINMUX_ALTERNATE_FUNCTION_5,
                (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                 PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));

    /* I2C0_SCL_B */
    pinconf_set(PORT_3, PIN_4, PINMUX_ALTERNATE_FUNCTION_5,
                (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                 PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));

}

/**
 * @fn      static void i2c_master_task(ULONG thread_input)
 * @brief   Performs Master data comm through I2C
 * @note    none
 * @param   pvParameters : Input to thread
 * @retval  none
 */
static void i2c_master_task(ULONG thread_input)
{
    int32_t   ret          = 0;
    uint8_t   iter         = 0;
    ARM_DRIVER_VERSION version;
    ULONG mst_notified_val = 0U;
    ULONG event_ret = 0;

    ARG_UNUSED(thread_input);

    /* Pinmux */
    pinmux_config();

    version = I2C_mstdrv->GetVersion();
    printf("\r\n I2C version api:0x%X driver:0x%X...\r\n",
            version.api, version.drv);

    /* Initialize I2C driver */
    ret = I2C_mstdrv->Initialize(i2c_mst_tranfer_callback);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: Master init failed\n");
        goto master_error_uninitialize;
    }

    /* Power control I2C */
    ret = I2C_mstdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: Master Power up failed\n");
        goto master_error_uninitialize;
    }

    /* Set I2C bus-speed to (400kHz) fast mode */
    ret = I2C_mstdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: Master control failed\n");
        goto master_error_poweroff;
    }

    tx_thread_sleep(5);

    printf("\n----------------Master transmit/slave receive--------------\n");

    I2C_mstdrv->MasterTransmit(TAR_ADDRS, MST_TX_BUF,
                               MST_BYTE_TO_TRANSMIT, STOP);

    /* wait for Master callback. */
    event_ret = tx_event_flags_get(&event_flags_master,
                      (LPI2C_CB_EVENT_SUCCESS   |
                       LPI2C_CB_EVENT_ERROR),
                       TX_OR_CLEAR,
                       &mst_notified_val,
                       TX_WAIT_FOREVER);

    if(event_ret != TX_SUCCESS)
    {
        printf("\r\nError: Event Timed Out\r\n");
    }

    if(mst_notified_val & LPI2C_CB_EVENT_ERROR)
    {
        printf("\r\nError: Master Tx failed\r\n");
        goto master_error_poweroff;
    }

    tx_thread_sleep(5);

    printf("\n----------------Master receive/slave transmit--------------\n");

    for(iter = 0; iter < SLV_BYTE_TO_TRANSMIT; iter++)
    {
        I2C_mstdrv->MasterReceive(TAR_ADDRS, &MST_RX_BUF[iter], 1, STOP);

        /* wait for Master callback. */
        tx_event_flags_get(&event_flags_master,
                          (LPI2C_CB_EVENT_SUCCESS   |
                           LPI2C_CB_EVENT_ERROR),
                           TX_OR_CLEAR,
                           &mst_notified_val,
                           TX_WAIT_FOREVER);

        if(mst_notified_val & LPI2C_CB_EVENT_ERROR)
        {
            printf("\r\nError: Master Rx failed\r\n");
            goto master_error_poweroff;
        }
    }

    tx_thread_sleep(5);

    /* Compare received data. */
    if (memcmp(&SLV_TX_BUF, &MST_RX_BUF, SLV_BYTE_TO_TRANSMIT))
    {
        printf("\n Error: Master receive/Slave transmit failed\n");
        goto master_error_poweroff;
    }

    printf("\n >>> LPI2C transfer completed without any error\n");

master_error_poweroff:
    /* Power off peripheral */
    ret = I2C_mstdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK)
    {
       printf("\r\n Error: Master Power OFF failed.\r\n");
    }

master_error_uninitialize:
    /* Un-initialize I2C driver */
    ret = I2C_mstdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
      printf("\r\n Error: Master Uninitialize failed.\r\n");
    }
    printf("\r\n  Master Task exiting...\r\n");

    tx_thread_delete(&master_thread);
}

/**
 * @fn      static void i2c_slave_task(ULONG thread_input)
 * @brief   Performs Slave data comm through LPI2C
 * @note    none
 * @param   thread_input : Input to thread
 * @retval  none
 */
static void i2c_slave_task(ULONG thread_input)
{
    int32_t   ret          = 0;
    ARM_DRIVER_VERSION version;
    ULONG slv_notified_val = 0U;

    ARG_UNUSED(thread_input);

    version = LPI2C_slvdrv->GetVersion();
    printf("\r\n LPI2C version api:0x%X driver:0x%X...\r\n",
            version.api, version.drv);

    /* Initialize LPI2C driver */
    ret = LPI2C_slvdrv->Initialize(i2c_slv_transfer_callback);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: Slave init failed\n");
        goto slave_error_uninitialize;
    }

    /* Power control LPI2C */
    ret = LPI2C_slvdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: Slave Power up failed\n");
        goto slave_error_uninitialize;
    }

    /* Perform LPI2C reception */
    LPI2C_slvdrv->SlaveReceive(SLV_RX_BUF, MST_BYTE_TO_TRANSMIT);

    /* wait for Slave callback. */
    tx_event_flags_get(&event_flags_slave,
                      (LPI2C_CB_EVENT_SUCCESS   |
                       LPI2C_CB_EVENT_ERROR),
                       TX_OR_CLEAR,
                       &slv_notified_val,
                       TX_WAIT_FOREVER);

    if(slv_notified_val & LPI2C_CB_EVENT_ERROR)
    {
        printf("\r\nError: Slave Tx failed\r\n");
        goto slave_error_poweroff;
    }

    /* Compare received data. */
    if (memcmp(&SLV_RX_BUF, &MST_TX_BUF, MST_BYTE_TO_TRANSMIT))
    {
        printf("\n Error: Master transmit/Slave receive failed \n");
        goto slave_error_poweroff;
    }

    LPI2C_slvdrv->SlaveTransmit(SLV_TX_BUF, SLV_BYTE_TO_TRANSMIT);

    /* wait for Slave callback. */
    tx_event_flags_get(&event_flags_slave,
                      (LPI2C_CB_EVENT_SUCCESS   |
                       LPI2C_CB_EVENT_ERROR),
                       TX_OR_CLEAR,
                       &slv_notified_val,
                       TX_WAIT_FOREVER);

    if(slv_notified_val & LPI2C_CB_EVENT_ERROR)
    {
        printf("\r\nError: Slave Rx failed\r\n");
        goto slave_error_poweroff;
    }

slave_error_poweroff:
    ret = LPI2C_slvdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK)
    {
       printf("\r\n Error: Slave Power OFF failed.\r\n");
    }

slave_error_uninitialize:
    ret = LPI2C_slvdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
      printf("\r\n Error: Slave Uninitialize failed.\r\n");
    }
    printf("\r\n  Slave Task exiting...\r\n");

    tx_thread_delete(&slave_thread);
}

/**
 * @fn      int main(void)
 * @brief   Entry function of LPI2C
 * @note    none
 * @param   none
 * @retval  none
 */
int main (void)
{
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
    /* Put system definition stuff in here, e.g. thread creates and
       other assorted create information.  */

    /* Create the event flags group used by Master thread */
    status = tx_event_flags_create(&event_flags_master, "Master Events");
    if(status != TX_SUCCESS)
    {
        printf("Could not create Master event flags\n");
        return;
    }

    /* Create the event flags group used by Slave thread */
    status = tx_event_flags_create(&event_flags_slave, "Slave Events");
    if(status != TX_SUCCESS)
    {
        printf("Could not create Slave event flags\n");
        return;
    }

    /* Create the Master task.  */
    status = tx_thread_create(&master_thread, "Master Thread", i2c_master_task,
                              0U, first_unused_memory, THREAD_STACK_SIZE,
                              MST_THREAD_PRIORITY, MST_THREAD_PRIORITY,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if(status != TX_SUCCESS)
    {
        printf("Unable to Create Master Task\n");
        return;
    }

    /* Create the Slave task.  */
    status = tx_thread_create(&slave_thread, "Slave Thread", i2c_slave_task,
                              0U, (first_unused_memory + THREAD_STACK_SIZE),
                              THREAD_STACK_SIZE, SLV_THREAD_PRIORITY,
                              SLV_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if(status != TX_SUCCESS)
    {
        printf("Unable to Create Slave Task\n");
        return;
    }
}
