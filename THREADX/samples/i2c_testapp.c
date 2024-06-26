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
 * @file     i2c_testapp.c
 * @brief    ThreadX demo application to verify I2C Master and
 *           Slave functionality ThreadX as operating system
 *
 *           Code will verify below cases:
 *           1) Master transmit 30 bytes and Slave receive 30 bytes
 *           2) Slave transmit 29 bytes and Master receive 29 bytes
 *           I2C1 instance is taken as Master (PIN P7_2 and P7_3)
 *           I2C0 instance is taken as Slave  (PIN P0_2 and P0_3)
 *
 *           Hardware setup:
 *           - Connecting GPIO pins of I2C1 TO I2C0 instances
 *             SDA pin P7_2(J15) to P0_2(J11)
 *             SCL pin P7_3(J15) to P0_3(J11).
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "tx_api.h"

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I2C.h"
#include "pinconf.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/* I2C Driver instance */
extern ARM_DRIVER_I2C Driver_I2C1;
static ARM_DRIVER_I2C *I2C_MstDrv = &Driver_I2C1;

extern ARM_DRIVER_I2C Driver_I2C0;
static ARM_DRIVER_I2C *I2C_SlvDrv = &Driver_I2C0;

void i2c_demo_Thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                 1024
#define DEMO_BYTE_POOL_SIZE             9120
#define TX_I2C_MST_TRANSFER_DONE        0x01
#define TX_I2C_ADDR_NACKED              0X02
#define TX_I2C_TRANSFER_INCOMPLETE      0X03
#define TX_I2C_SLV_TRANSFER_DONE        0x04

TX_THREAD               i2c_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags_i2c;

/* Defining Address modes */
#define ADDRESS_MODE_7BIT   1                   /* I2C 7 bit addressing mode     */
#define ADDRESS_MODE_10BIT  2                   /* I2C 10 bit addressing mode    */
#define ADDRESS_MODE        ADDRESS_MODE_10BIT   /* Current Addressing mode       */

#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    #define TAR_ADDRS       (0X2D0)  /* 10 bit Target(Slave) Address, use by Master */
    #define SAR_ADDRS       (0X2D0)  /* 10 bit Slave Own Address,     use by Slave  */
#else
    #define TAR_ADDRS       (0X40)   /* 7 bit Target(Slave) Address, use by Master  */
    #define SAR_ADDRS       (0X40)   /* 7 bit Slave Own Address,     use by Slave   */
#endif

#define RESTART             (0X01)
#define STOP                (0X00)

/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT            30

/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT            29

/* Master parameter set */

/* Master TX Data (Any random value). */
uint8_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT] =
{
    "!*!Test Message from Master!*!"
};

/* master receive buffer */
uint8_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT];

/* Master parameter set END  */


/* Slave parameter set */

/* slave receive buffer */
uint8_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT];

/* Slave TX Data (Any random value). */
uint8_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT] =
{
    "!*!Test Message from Slave!*!"
};

/* Slave parameter set END */

static void i2c_mst_transfer_callback(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE)
    {
        /* Master Transfer done. Wake-up Thread. */
        tx_event_flags_set(&event_flags_i2c, TX_I2C_MST_TRANSFER_DONE, TX_OR);
    }

    if (event & ARM_I2C_EVENT_ADDRESS_NACK)
    {
        /* Master Transfer done. Wake-up Thread. */
        tx_event_flags_set(&event_flags_i2c, TX_I2C_ADDR_NACKED, TX_OR);
    }

    if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE)
    {
        /* Master Transfer done. Wake-up Thread. */
        tx_event_flags_set(&event_flags_i2c, TX_I2C_TRANSFER_INCOMPLETE, TX_OR);
    }
}

static void i2c_slv_transfer_callback(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE)
    {
        /* Slave Transfer done. Wake-up Thread. */
        tx_event_flags_set(&event_flags_i2c, TX_I2C_SLV_TRANSFER_DONE, TX_OR);
    }
}

/* Pinmux for B0 */
void hardware_init()
{
    /* I2C0_SDA_A */
    pinconf_set(PORT_0, PIN_2, PINMUX_ALTERNATE_FUNCTION_3,
         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));

    /* I2C0_SCL_A */
    pinconf_set(PORT_0, PIN_3, PINMUX_ALTERNATE_FUNCTION_3,
         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));

    /* I2C1_SDA_C */
    pinconf_set(PORT_7, PIN_2, PINMUX_ALTERNATE_FUNCTION_5,
         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));

    /* I2C1_SCL_C */
    pinconf_set(PORT_7, PIN_3, PINMUX_ALTERNATE_FUNCTION_5,
         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));
}

void i2c_demo_Thread_entry(ULONG thread_input)
{
    ULONG events   = 0;
    int   ret      = 0;
    ARM_DRIVER_VERSION version;
    ARM_I2C_CAPABILITIES capabilities;

    printf("\r\n >>> I2C demo thread starting up!!! <<< \r\n");

    /* Pinmux */
    hardware_init();

    version = I2C_MstDrv->GetVersion();
    printf("\r\n I2C version api:0x%X driver:0x%X...\r\n",version.api, version.drv);

    /* Initialize I2C driver */
    ret = I2C_MstDrv->Initialize(i2c_mst_transfer_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C master init failed\n");
        return;
    }

    /* Initialize I2C driver */
    ret = I2C_SlvDrv->Initialize(i2c_slv_transfer_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C slave init failed\n");
        return;
    }

    /* Power control I2C */
    ret = I2C_MstDrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C Master Power up failed\n");
        goto error_poweroff;
    }

    /* Power control I2C */
    ret = I2C_SlvDrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C Slave Power up failed\n");
        goto error_poweroff;
    }

    /* I2C Master Control */
    ret = I2C_MstDrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C Master Control failed\n");
        goto error_uninitialize;
    }

    /* I2C Slave Control */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_SlvDrv->Control(ARM_I2C_OWN_ADDRESS,
                             (SAR_ADDRS | ARM_I2C_ADDRESS_10BIT));
#else
    ret = I2C_SlvDrv->Control(ARM_I2C_OWN_ADDRESS, SAR_ADDRS);
#endif
    if(ret != ARM_DRIVER_OK){
         printf("\r\n Error: I2C Slave Control failed\n");
         goto error_uninitialize;
    }

    printf("\n----------------Master transmit/slave receive-----------------------\n");

    /* Clear event */
    events = 0;

    /* I2C Slave Receive */
    ret = I2C_SlvDrv->SlaveReceive(SLV_RX_BUF, MST_BYTE_TO_TRANSMIT);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Slave Receive failed\n");
        goto error_uninitialize;
    }

    /* delay */
    tx_thread_sleep(10);

    /* I2C Master Transmit */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    I2C_MstDrv->MasterTransmit((TAR_ADDRS | ARM_I2C_ADDRESS_10BIT),
                               MST_TX_BUF, MST_BYTE_TO_TRANSMIT, STOP);
#else
    I2C_MstDrv->MasterTransmit(TAR_ADDRS, MST_TX_BUF,
                               MST_BYTE_TO_TRANSMIT, STOP);
#endif
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Master Transmit failed\n");
        goto error_uninitialize;
    }

    /* wait till master/slave isr callback */
    ret = tx_event_flags_get(&event_flags_i2c, TX_I2C_MST_TRANSFER_DONE |
                             TX_I2C_ADDR_NACKED | TX_I2C_TRANSFER_INCOMPLETE,
                             TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS) {
        printf("Error: I2C tx_event_flags_get\n");
        goto error_poweroff;
    }

    if (events == TX_I2C_ADDR_NACKED)
    {
        printf("\r\n Error: Slave NACKED\n");
        goto error_uninitialize;
    }
    else if (events == TX_I2C_TRANSFER_INCOMPLETE)
    {
        printf("\r\n Error: Transfer incomplete\n");
        goto error_uninitialize;
    }

    /* wait till master/slave isr callback */
    ret = tx_event_flags_get(&event_flags_i2c, TX_I2C_SLV_TRANSFER_DONE,
                             TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS)
    {
        printf("Error: I2C tx_event_flags_get\n");
        goto error_poweroff;
    }

    /* Compare received data. */
    if(memcmp(&SLV_RX_BUF, &MST_TX_BUF, MST_BYTE_TO_TRANSMIT))
    {
        printf("\n Error: Master transmit/slave receive failed\n");
        printf("\n ---Stop--- \r\n wait forever >>> \n");
        while(1);
    }

    printf("\n----------------Master receive/slave transmit-----------------------\n");

    /* Clear event */
    events = 0;

    /* I2C Master Receive */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_MstDrv->MasterReceive((TAR_ADDRS | ARM_I2C_ADDRESS_10BIT),
                                    MST_RX_BUF, SLV_BYTE_TO_TRANSMIT, STOP);
#else
    ret = I2C_MstDrv->MasterReceive(TAR_ADDRS, MST_RX_BUF,
                                    SLV_BYTE_TO_TRANSMIT, STOP);
#endif

    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Master Receive failed\n");
        goto error_uninitialize;
    }

    /* I2C Slave Transmit */
    ret = I2C_SlvDrv->SlaveTransmit(SLV_TX_BUF, SLV_BYTE_TO_TRANSMIT);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Slave Transmit failed\n");
        goto error_uninitialize;
    }

    /* wait till master/slave isr callback */
    ret = tx_event_flags_get(&event_flags_i2c, TX_I2C_MST_TRANSFER_DONE |
                             TX_I2C_ADDR_NACKED | TX_I2C_TRANSFER_INCOMPLETE,
                             TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS)
    {
        printf("Error: I2C tx_event_flags_get\n");
        goto error_poweroff;
    }

    if (events == TX_I2C_ADDR_NACKED)
    {
        printf("\r\n Error: Slave NACKED\n");
        goto error_uninitialize;
    }
    else if (events == TX_I2C_TRANSFER_INCOMPLETE)
    {
        printf("\r\n Error: Transfer incomplete\n");
        goto error_uninitialize;
    }

    /* wait till master/slave isr callback */
    ret = tx_event_flags_get(&event_flags_i2c, TX_I2C_SLV_TRANSFER_DONE,
                             TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS)
    {
        printf("Error: I2C tx_event_flags_get\n");
        goto error_poweroff;
    }

    /* Compare received data. */
    if(memcmp(&SLV_TX_BUF, &MST_RX_BUF, SLV_BYTE_TO_TRANSMIT))
    {
        printf("\n Error: Master receive/slave transmit failed\n");
        printf("\n ---Stop--- \r\n wait forever >>> \n");
        while(1);
    }

    ret =I2C_MstDrv->Uninitialize();
    if (ret == ARM_DRIVER_OK)
    {
        printf("\r\nI2C Master Unintialize\n");
        goto error_uninitialize;
    }
    ret =I2C_SlvDrv->Uninitialize();
    if (ret == ARM_DRIVER_OK)
    {
        printf("\r\nI2C Slave Unintialize\n");
        goto error_uninitialize;
    }

    printf("\n >>> I2C Communication completed without any error <<< \n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    while(1);

error_poweroff:
    /* Power off I2C peripheral */
    ret = I2C_MstDrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
         printf("\r\n Error: I2C Master Power OFF failed.\r\n");
    }
    ret = I2C_SlvDrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
         printf("\r\n Error: I2C Slave Power OFF failed.\r\n");
    }

error_uninitialize:
    /* Un-initialize I2C driver */
    ret = I2C_MstDrv->Uninitialize();
    if(ret == ARM_DRIVER_OK)
    {
        printf("\r\n I2C Master Uninitialize\r\n");
    }
    ret = I2C_SlvDrv->Uninitialize();
    if(ret == ARM_DRIVER_OK)
    {
        printf("\r\n I2C Slave Uninitialize\r\n");
    }
    printf("\r\n I2C demo thread exiting...\r\n");
}

/* Define main entry point.  */
int main()
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

/* Define what the initial system looks like.  */
void tx_application_define(void *first_unused_memory)
{
    CHAR    *pointer = TX_NULL;
    INT status;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Put system definition stuff in here, e.g. thread creates and other assorted
        create information.  */

    /* Create the event flags group used by I2C thread */
    status = tx_event_flags_create(&event_flags_i2c, "event flags I2C");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread 0.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&i2c_thread, "i2c_thread", i2c_demo_Thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread\n");
        return;
    }
}
