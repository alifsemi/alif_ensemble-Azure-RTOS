/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     Flash_MRAM_Threadx.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     23-June-2022
 * @brief    TestApp to verify Flash(On-Chip NVM MRAM) interface using
 *             Threadx as an operating system.
 *             Verify Read/Write/Erase to/from MRAM.
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
/* include for MRAM Flash Driver */
#include "Driver_Flash.h"

/* For Release build disable printf and semihosting */
#define DISABLE_SEMIHOSTING

#ifdef DISABLE_SEMIHOSTING
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


int _sys_open(void *p){

   return 0;
}


int _sys_close(void *p){

   return 0;
}


int _sys_read(void *p){

   return 0;
}

int _sys_write(void *p){

   return 0;
}

int _sys_istty(void *p){

   return 0;
}

int _sys_seek(void *p){

   return 0;
}

int _sys_flen(void *p){

    return 0;
}

void _ttywrch(int ch){

}

#endif /* DISABLE_SEMIHOSTING */

/* for Unused Arguments. */
#ifndef ARG_UNUSED
#define ARG_UNUSED(arg)     ((void)arg)
#endif

/* MRAM Flash Driver instance(default 0th instance for MRAM) */
#define FLASH_MRAM_DRV_INSTANCE          0

/* MRAM Flash Driver */
extern ARM_DRIVER_FLASH ARM_Driver_Flash_(FLASH_MRAM_DRV_INSTANCE);

/* MRAM Flash Driver instance */
static ARM_DRIVER_FLASH *FLASH_MRAM_drv = &ARM_Driver_Flash_(FLASH_MRAM_DRV_INSTANCE);

void Flash_MRAM_Thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE           1024
#define DEMO_BYTE_POOL_SIZE       9120

TX_THREAD                         MRAM_thread;
TX_BYTE_POOL                      byte_pool_0;
UCHAR                             memory_area[DEMO_BYTE_POOL_SIZE];

/* Valid MRAM Flash address-offset. */
#define MRAM_ADDR_OFFSET           (0x100000)

/* Buffer size and value which needs to write to MRAM. */
#define BUFFER_SIZE   0x10000 /* any random size.(for demo purpose size taken as 64KB) */
#define BUFFER_VALUE  0xA5    /* any random value. */

UCHAR buff_TX[BUFFER_SIZE] = {0x00};
UCHAR buff_RX[BUFFER_SIZE] = {0x00};


/**
 * @function    void Flash_MRAM_thread_entry(ULONG thread_input)
 * @brief       TestApp to verify Flash(On-Chip NVM MRAM) interface
 *              using Threadx as an operating system.
 *                 - Verify Read/Write/Erase to/from MRAM.
 *                 - write 1MB data to MRAM flash
 *                 - read back and compare wrote data
 *                 - erase one MRAM sector
 * @note        none
 * @param       none
 * @retval      none
 */
void Flash_MRAM_Thread_entry(ULONG thread_input)
{
	INT ret;
	ARM_DRIVER_VERSION version;
	ARM_FLASH_INFO *info;

	UINT addr        = MRAM_ADDR_OFFSET;
	INT sector_size  = 0;
	INT erased_value = 0;
	INT cmp          = 0;
	INT err_cnt      = 0;

	/* Fill buffer data which needs to write to MRAM. */
	memset(buff_TX, BUFFER_VALUE, sizeof(buff_TX));

	printf("\r\n >>> MRAM Flash testApp starting up!!!...<<< \r\n");

	/* MRAM Flash driver version. */
	version = FLASH_MRAM_drv->GetVersion();
	printf("\r\n MRAM Flash version: api:0x%X driver:0x%X...\r\n",version.api, version.drv);
	ARG_UNUSED(version);

	/* MRAM Flash information. */
	info = FLASH_MRAM_drv->GetInfo();
	sector_size  = info->sector_size;
	erased_value = info->erased_value;
	printf("\r\n MRAM Flash info: sector-size:0x%X sector-count:0x%X...\r\n", \
								info->sector_size, info->sector_count);

	/* Initialize MRAM Flash driver */
	ret = FLASH_MRAM_drv->Initialize(NULL);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error in MRAM Flash Initialize.\r\n");
		return;
	}

	/* Power up Flash peripheral */
	ret = FLASH_MRAM_drv->PowerControl(ARM_POWER_FULL);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error in MRAM Flash Power Up.\r\n");
		goto error_uninitialize;
	}


	/* write data to MRAM (for demo purpose write 1MB data (64KB x 16) ) */
	for(int i = 0; i < 16; i++)
	{
		ret = FLASH_MRAM_drv->ProgramData(addr, buff_TX, BUFFER_SIZE);
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

	for(int i = 0; i < 16; i++)
	{
		ret = FLASH_MRAM_drv->ReadData(addr, buff_RX, BUFFER_SIZE);
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

	addr = MRAM_ADDR_OFFSET;

	/* Erase one MRAM sector. */
	ret = FLASH_MRAM_drv->EraseSector(addr);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: MRAM EraseSector failed: addr:0x%X \r\n", addr);
		goto error_poweroff;
	}

	/* check is it erased one sector? */
	ret = FLASH_MRAM_drv->ReadData(addr, buff_RX, sector_size);
	if(ret != sector_size)
	{
		printf("\r\n Error: MRAM Sector ReadData failed: addr:0x%X data:%0x.\r\n", \
									addr, (uint32_t)buff_RX);
		goto error_poweroff;
	}

	/* erase buff_TX and compare. */
	memset(buff_TX, erased_value, sector_size);

	cmp = memcmp(buff_TX, buff_RX, sector_size);
	if(cmp != 0)
	{
		printf("\r\n Error: MRAM Erase sector failed: addr:0x%X \r\n", addr);
		err_cnt++;
	}

	/* check for error. */
	if(err_cnt)
	{
		printf("\r\n waiting in Error. \r\n");
		while(1);
	}

	printf("\r\n MRAM Write-Read-Erase test completed!!! \r\n");
	printf("\r\n waiting here forever...\r\n");
	while(1);

error_poweroff:
	/* Received error, Power off MRAM Flash peripheral */
	ret = FLASH_MRAM_drv->PowerControl(ARM_POWER_OFF);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error in MRAM Flash Power OFF.\r\n");
	}

error_uninitialize:
	/* Received error, Un-initialize MRAM Flash driver */
	ret = FLASH_MRAM_drv->Uninitialize();
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error in MRAM Flash Uninitialize.\r\n");
	}

	printf("\r\n XXX MRAM Flash demo thread exiting XXX...\r\n");
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

	/* Allocate the stack for thread.  */
	status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
	if (status != TX_SUCCESS)
	{
		printf("Could not allocate memory for thread stack.\n");
		return;
	}

	/* Create the main thread.  */
	status = tx_thread_create(&MRAM_thread, "MRAM_thread", Flash_MRAM_Thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (status != TX_SUCCESS)
	{
		printf("Could not create MRAM Flash demo thread \n");
		return;
	}
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
