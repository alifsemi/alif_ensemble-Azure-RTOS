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
 * @file     MRAM_Threadx.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     01-June-2023
 * @brief    TestApp to verify MRAM(On-Chip NVM(Non-Volatile Memory) ) interface
 *            using Threadx as an operating system.
 *            Verify Read/Write to/from MRAM.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
/* ThredX Include */
#include "tx_api.h"

/* System Includes */
#include <stdio.h>
#include "string.h"
#include <RTE_Components.h>
#include CMSIS_device_header

/* Project Includes */
/* include for MRAM Driver */
#include "Driver_MRAM.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif  /* RTE_CMSIS_Compiler_STDOUT */
#include "app_utils.h"

/* for Unused Arguments. */
#ifndef ARG_UNUSED
#define ARG_UNUSED(arg)     ((void)arg)
#endif


/* MRAM Driver */
extern ARM_DRIVER_MRAM Driver_MRAM;
static ARM_DRIVER_MRAM *MRAM_drv = &Driver_MRAM;

void MRAM_Thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE           1024
#define DEMO_BYTE_POOL_SIZE       9120

TX_THREAD                         MRAM_thread;
TX_BYTE_POOL                      byte_pool_0;
UCHAR                             memory_area[DEMO_BYTE_POOL_SIZE];

/* Valid MRAM address-offset. */
#define MRAM_ADDR_OFFSET           (0x100000)

#define MRAM_TEST_SIZE            (512*1024)                     /* 512KB size. */
#define MRAM_TEST_LOOPS           (MRAM_TEST_SIZE / BUFFER_SIZE) /* 512KB/64KB = 8 loops. */

/* Buffer size and value which needs to write to MRAM. */
#define BUFFER_SIZE   0x10000 /* any random size.(for demo purpose size taken as 64KB) */
#define BUFFER_VALUE  0x19    /* any random value. */

UCHAR buff_TX[BUFFER_SIZE] = {0x00};
UCHAR buff_RX[BUFFER_SIZE] = {0x00};


/**
 * @function    void MRAM_thread_entry(ULONG thread_input)
 * @brief       TestApp to verify MRAM(On-Chip NVM) interface
 *              using Threadx as an operating system.
 *                 - Verify Read/Write to/from MRAM.
 *                 - write 1MB data to MRAM
 *                 - read back and compare wrote data
 * @note        none
 * @param       none
 * @retval      none
 */
void MRAM_Thread_entry(ULONG thread_input)
{
    INT ret;
    ARM_DRIVER_VERSION version;

    UINT addr        = MRAM_ADDR_OFFSET;
    INT cmp          = 0;
    INT err_cnt      = 0;

    /* Fill buffer data which needs to write to MRAM. */
    memset(buff_TX, BUFFER_VALUE, sizeof(buff_TX));

    printf("\r\n >>> MRAM testApp starting up!!!...<<< \r\n");

    /* MRAM driver version. */
    version = MRAM_drv->GetVersion();
    printf("\r\n MRAM version: api:0x%X driver:0x%X...\r\n",version.api, version.drv);
    ARG_UNUSED(version);

    /* Initialize MRAM driver */
    ret = MRAM_drv->Initialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in MRAM Initialize.\r\n");
        return;
    }

    /* Power up peripheral */
    ret = MRAM_drv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in MRAM Power Up.\r\n");
        goto error_uninitialize;
    }

    /* write data to MRAM (for demo purpose write 512KB data (64KB x 8) ) */
    for(int i = 0; i < MRAM_TEST_LOOPS; i++)
    {
        ret = MRAM_drv->ProgramData(addr, buff_TX, BUFFER_SIZE);
        if(ret != BUFFER_SIZE)
        {
            printf("\r\n Error: MRAM ProgramData failed: addr:0x%X data:%0x.\r\n", \
                                    addr, (uint32_t)buff_TX);
            goto error_poweroff;
        }
        addr += BUFFER_SIZE;
    }

    /* read back and compare wrote data from MRAM. */
    addr = MRAM_ADDR_OFFSET;

    for(int i = 0; i < MRAM_TEST_LOOPS; i++)
    {
        ret = MRAM_drv->ReadData(addr, buff_RX, BUFFER_SIZE);
        if(ret != BUFFER_SIZE)
        {
            printf("\r\n Error: MRAM ReadData failed: addr:0x%X data:%0x.\r\n", \
                                    addr, (uint32_t)buff_RX);
            goto error_poweroff;
        }

        /* compare write and read. */
        cmp = memcmp(buff_TX, buff_RX, BUFFER_SIZE);
        if(cmp != 0)
        {
            printf("\r\n Error: MRAM write-read failed: addr:0x%X data:%0x.\r\n", \
                                    addr, (uint32_t)buff_RX);
            err_cnt++;
        }
        addr += BUFFER_SIZE;
    }

    /* check for error. */
    if(err_cnt)
    {
        printf("\r\n waiting in Error. \r\n");
        while(1);
    }

    printf("\r\n MRAM Write-Read test completed!!! \r\n");
    printf("\r\n waiting here forever...\r\n");
    while(1);

error_poweroff:
    /* Received error, Power off MRAM peripheral */
    ret = MRAM_drv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in MRAM Power OFF.\r\n");
    }

error_uninitialize:
    /* Received error, Un-initialize MRAM driver */
    ret = MRAM_drv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in MRAM Uninitialize.\r\n");
    }

    printf("\r\n XXX MRAM demo thread exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
    #if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
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
    INT      status  = 0;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not allocate memory for thread stack.\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&MRAM_thread, "MRAM_thread", MRAM_Thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create MRAM demo thread \n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
