/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     SPI0_SPI1_testapp.c
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     29-May-2023
 * @brief    ThreadX demo application for SPI0 and SPI1.
 *           - Data transfer between SPI0(master) and SPI1(slave).
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
#include "tx_api.h"
#include <stdio.h>
#include "Driver_SPI.h"
#include "pinconf.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


#define DEMO_BYTE_POOL_SIZE             (1024)
#define TRANSFER_THREAD_STACK_SIZE      (512)

UCHAR                                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_BYTE_POOL                            memory_pool;
TX_THREAD                               spi0_spi1_transfer_thread;
TX_EVENT_FLAGS_GROUP                    spi_event_flag;
ULONG                                   events;

#define SPI0_CALLBACK_EVENT   (1 << 0)
#define SPI1_CALLBACK_EVENT   (1 << 1)

/* Use below macro to specify transfer type
 * 1 - Uses SPI Transfer function
 * 0 - Uses SPI Send & Receive function
 * */
#define DATA_TRANSFER_TYPE  1

#define  SPI1           1    /* SPI1 instance */
#define  SPI0           0    /* SPI0 instance */

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI1);
ARM_DRIVER_SPI *ptrSPI1 = &ARM_Driver_SPI_(SPI1);

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI0);
ARM_DRIVER_SPI *ptrSPI0 = &ARM_Driver_SPI_(SPI0);

/**
 * @fn      static int32_t pinmux_config(void)
 * @brief   SPI1 & SPI0 pinmux configuration.
 * @note    none.
 * @param   none.
 * @retval  execution status.
 */
static int32_t pinmux_config(void)
{
    int32_t ret = ARM_DRIVER_OK;

    /* pinmux configurations for SPI0 pins (using B version pins) */
    ret = pinconf_set(PORT_5, PIN_0, PINMUX_ALTERNATE_FUNCTION_4, PADCTRL_READ_ENABLE);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI0_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_5, PIN_1, PINMUX_ALTERNATE_FUNCTION_4, 0);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI0_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_5, PIN_3, PINMUX_ALTERNATE_FUNCTION_3, 0);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI0_CLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_5, PIN_2, PINMUX_ALTERNATE_FUNCTION_4, 0);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI0_SS_PIN\n");
        return ret;
    }

    /* pinmux configurations for SPI1 pins (using B version pins) */
    ret = pinconf_set(PORT_8, PIN_3, PINMUX_ALTERNATE_FUNCTION_2, 0);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI1_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_8, PIN_4, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_READ_ENABLE);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI1_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_8, PIN_5, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_READ_ENABLE);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI1_CLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_6, PIN_4, PINMUX_ALTERNATE_FUNCTION_4, PADCTRL_READ_ENABLE);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI1_SS_PIN\n");
        return ret;
    }

    return ret;
}

/**
 * @fn      static void SPI0_cb_func (uint32_t event)
 * @brief   SPI0 callback function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  none.
 */
static void SPI0_cb_func (uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        tx_event_flags_set(&spi_event_flag, SPI0_CALLBACK_EVENT, TX_OR);
    }
}

/**
 * @fn      static void SPI1_cb_func (uint32_t event)
 * @brief   SPI1 callback function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  none.
 */
static void SPI1_cb_func (uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        tx_event_flags_set(&spi_event_flag, SPI1_CALLBACK_EVENT, TX_OR);
    }
}

/**
 * @fn      static void spi0_spi1_transfer(ULONG thread_input)
 * @brief   demo application function for data transfer.
 * @note    none.
 * @param   thread_input.
 * @retval  none.
 */
static void spi0_spi1_transfer(ULONG thread_input)
{
    uint32_t spi0_tx_buff, spi1_rx_buff = 0;
    int32_t ret = ARM_DRIVER_OK;
    uint32_t status;
    uint32_t spi1_control, spi0_control;
#if DATA_TRANSFER_TYPE
    uint32_t spi1_tx_buff, spi0_rx_buff = 0;
#endif

    /*
     * H/W connections on devkit:
     * short SPI0 MISO (P5_0 -> J12-13 pin) and SPI1 MISO (P8_3 -> J14-15 pin).
     * short SPI0 MOSI (P5_1 -> J12-15 pin) and SPI1 MOSI (P8_4 -> J14-17 pin).
     * short SPI0 SCLK (P5_3 -> J14-5 pin) and SPI1 SCLK (P8_5 -> J14-19 pin).
     * short SPI0 SS (P5_2 -> J12-17 pin) and SPI1 SS (P6_4 -> J12-22 pin).
     * */

    printf("*** Demo ThreadX app using SPI0 & SPI1 is starting ***\n");

    ret = pinmux_config();
    if (ret)
    {
        printf("Error in pinmux configuration\n");
        return;
    }

    /* SPI0 Configuration as master */
    ret = ptrSPI0->Initialize(SPI0_cb_func);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to initialize the SPI0\n");
        return;
    }

    ret = ptrSPI0->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to power SPI0\n");
        goto error_spi0_uninitialize;
    }

    spi0_control = (ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_DATA_BITS(32));

    /* Baudrate is 1MHz */
    ret = ptrSPI0->Control(spi0_control, 1000000);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure SPI0\n");
        goto error_spi0_power_off;
    }

    /* SPI1 Configuration as slave */
    ret = ptrSPI1->Initialize(SPI1_cb_func);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to initialize the SPI1\n");
        goto error_spi0_power_off;
    }

    ret = ptrSPI1->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to power SPI1\n");
        goto error_spi1_uninitialize;
    }

    spi1_control = (ARM_SPI_MODE_SLAVE | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_DATA_BITS(32));

    ret = ptrSPI1->Control(spi1_control, NULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure SPI1\n");
        goto error_spi1_power_off;
    }

    ret = ptrSPI0->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to enable the slave select of SPI0\n");
        goto error_spi1_power_off;
    }

#if DATA_TRANSFER_TYPE
    spi0_tx_buff = 0xAAAAAAAA;
    spi1_tx_buff = 0x55555555;

    ret = ptrSPI1->Transfer(&spi1_tx_buff, &spi1_rx_buff, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed SPI1 to configure as tx_rx\n");
        goto error_spi1_power_off;
    }

    ret = ptrSPI0->Transfer(&spi0_tx_buff, &spi0_rx_buff, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed SPI0 to configure as tx_rx\n");
        goto error_spi1_power_off;
    }

#else
    spi0_tx_buff = 0x12345678;

    ret = ptrSPI1->Receive(&spi1_rx_buff, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: SPI1 Failed to configure as receive only\n");
        goto error_spi1_power_off;
    }

    ret = ptrSPI0->Send(&spi0_tx_buff, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: SPI0 Failed to configure as send only\n");
        goto error_spi1_power_off;
    }
#endif

    status = tx_event_flags_get (&spi_event_flag, SPI1_CALLBACK_EVENT|SPI0_CALLBACK_EVENT, TX_AND_CLEAR, &events, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) {
        printf("ERROR : event not received, timeout happened \n");
        goto error_spi0_power_off;
    }

    while (!((ptrSPI0->GetStatus().busy == 0) && (ptrSPI1->GetStatus().busy == 0)));
    printf("Data Transfer completed\n");

    printf("SPI1 received value : 0x%x\n", spi1_rx_buff);
#if DATA_TRANSFER_TYPE
    printf("SPI0 received value : 0x%x\n", spi0_rx_buff);
#endif

error_spi1_power_off :
    ret = ptrSPI1->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR in SPI1 power off\n");
    }

error_spi1_uninitialize :
    ret = ptrSPI1->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR in SPI1 un-initialization\n");
    }

error_spi0_power_off :
    ret = ptrSPI0->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR in SPI0 power off\n");
    }

error_spi0_uninitialize :
    ret = ptrSPI0->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR in SPI0 un-initialization\n");
    }

    printf("*** Demo ThreadX app using SPI0 & SPI1 is ended ***\n");
}

/* Define main entry point.  */
int main ()
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
void tx_application_define (void *first_unused_memory)
{
    CHAR *pointer = TX_NULL;
    UINT ret;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    ret = tx_byte_pool_create (&memory_pool, "memory pool", memory_area, DEMO_BYTE_POOL_SIZE);

    if (ret != TX_SUCCESS) {
        printf("failed to create to byte Pool\r\n");
    }

    /* Put system definition stuff in here, e.g. thread creates and other assorted
       create information.  */
    /* Create a event flag for spi group.  */
    ret = tx_event_flags_create (&spi_event_flag, "SPI_EVENT_FLAG");

    if (ret != TX_SUCCESS) {
        printf("failed to create spi event flag\r\n");
    }

    /* Allocate the stack for thread 0.  */
    ret = tx_byte_allocate (&memory_pool, (VOID **) &pointer, TRANSFER_THREAD_STACK_SIZE, TX_NO_WAIT);

    if (ret != TX_SUCCESS) {
        printf("failed to allocate stack for led blink demo thread\r\n");
    }

    /* Create the main thread.  */
    ret = tx_thread_create (&spi0_spi1_transfer_thread, "TRANSFER DEMO THREAD", spi0_spi1_transfer, 0, pointer, TRANSFER_THREAD_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (ret != TX_SUCCESS) {
        printf("failed to create led blink demo thread\r\n");
    }
}
