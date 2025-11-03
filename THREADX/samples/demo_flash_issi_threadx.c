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
 * @file     demo_flash_issi_azurertos.c
 * @author   Khushboo Singh
 * @email    khushboo.singh@alifsemi.com
 * @version  V1.0.0
 * @date     31-Oct-2022
 * @brief    TestApp to verify ISSI Flash interface using
 *             Threadx as an operating system.
 *             Verify Read/Write/Erase to/from ISSI flash.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include "string.h"
#include "tx_api.h"

#include <RTE_Components.h>
#include CMSIS_device_header

#include "pinconf.h"
#include "board_config.h"
#include "Driver_Flash.h"
#include "Driver_IO.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

/* ISSI Flash Driver instance */
#define FLASH_ISSI_DRV_INSTANCE          1

extern ARM_DRIVER_FLASH ARM_Driver_Flash_(FLASH_ISSI_DRV_INSTANCE);
#define ptrFLASH (&ARM_Driver_Flash_(FLASH_ISSI_DRV_INSTANCE))

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_ISSI_FLASH_RESET_GPIO_PORT);
ARM_DRIVER_GPIO       *GPIODrv = &ARM_Driver_GPIO_(BOARD_ISSI_FLASH_RESET_GPIO_PORT);

void demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE           (BUFFER_SIZE * 2U)

TX_THREAD                         demo_thread;

#define FLASH_ADDR                0x00U
#define BUFFER_SIZE               1024U

USHORT buff_write[BUFFER_SIZE];
USHORT buff_read[BUFFER_SIZE];

/**
 * @fn      static INT setup_PinMUX(void)
 * @brief   Set up PinMUX and PinPAD
 * @note    none
 * @param   none
 * @retval  -1: On Error
 *          0 : On Success
 */
static INT setup_pinmux(void)
{
    INT ret;

    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return APP_ERROR;
    }

    ret = GPIODrv->Initialize(BOARD_ISSI_FLASH_RESET_GPIO_PIN, 0);
    if (ret != ARM_DRIVER_OK) {
        return APP_ERROR;
    }

    ret = GPIODrv->PowerControl(BOARD_ISSI_FLASH_RESET_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return APP_ERROR;
    }

    ret = GPIODrv->SetDirection(BOARD_ISSI_FLASH_RESET_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return APP_ERROR;
    }

    ret = GPIODrv->SetValue(BOARD_ISSI_FLASH_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return APP_ERROR;
    }

    ret = GPIODrv->SetValue(BOARD_ISSI_FLASH_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return APP_ERROR;
    }

    return APP_SUCCESS;
}

/**
 * @function    void demo_thread_entry(ULONG thread_input)
 * @brief       TestApp to verify Flash interface
 *                using Threadx as an operating system.
 *                 - Verify Read/Write/Erase to/from flash.
 * @note        none
 * @param       none
 * @retval      none
 */
void demo_thread_entry(ULONG thread_input)
{
    INT status;
    UINT ret, index, iter = 0, count = 0;
    ARM_DRIVER_VERSION version;
    ARM_FLASH_INFO *flash_info;

    (void) thread_input;

    /* Prepare the data for writing to flash */
    for (index = 0; index < BUFFER_SIZE; index++) {
        buff_write[index] = index % 65536;
    }

    printf("OSPI Flash Initialization\n");

    ret = setup_pinmux();
    if (ret != ARM_DRIVER_OK) {
        printf("Set up pinmux failed\n");
        goto error_pinmux;
    }

    /* Get version of the flash */
    version = ptrFLASH->GetVersion();

    printf("\r\n FLASH version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize the flash */
    status = ptrFLASH->Initialize(NULL);
    if (status != ARM_DRIVER_OK) {
        printf("Flash initialization failed\n");
        goto error_uninitialize;
    }

    status = ptrFLASH->PowerControl(ARM_POWER_FULL);
    if (status != ARM_DRIVER_OK) {
        printf("Flash Power control failed\n");
        goto error_poweroff;
    }

    /* Get Flash Info.*/
    flash_info = ptrFLASH->GetInfo();

    printf("\r\n FLASH Info :\n Sector ulCount : %" PRIu32 "\n Sector Size : %" PRIu32
           " Bytes\n Page Size : %" PRIu32 "\n "
           "Program Unit : %" PRIu32 "\n "
           "Erased Value : 0x%" PRIx8 " \r\n",
           flash_info->sector_count,
           flash_info->sector_size,
           flash_info->page_size,
           flash_info->program_unit,
           flash_info->erased_value);

    printf("\nErasing the chip\n");

    /* Erase the chip */
    status = ptrFLASH->EraseChip();
    if (status != ARM_DRIVER_OK) {
        printf("Chip erase failed\n");
        goto error_poweroff;
    }

    printf("starting reading erased data\n");

    iter = 0;

    /* Read 2KB data after erase and check if it is erased completely */
    status = ptrFLASH->ReadData(FLASH_ADDR, buff_read, BUFFER_SIZE);
    if (status != BUFFER_SIZE) {
        printf("Data not read completely\n");
        goto error_poweroff;
    }

    /* Verify the read data */
    while (iter < BUFFER_SIZE) {
        if (buff_read[iter] != (flash_info->erased_value << 8 | flash_info->erased_value)) {
            count++;
        }
        iter++;
    }

    printf("Total errors after reading erased chip = %d\n", count);

    printf("Starting writing\n");

    /* Write 2 KB data to the flash */
    status = ptrFLASH->ProgramData(FLASH_ADDR, buff_write, BUFFER_SIZE);
    if (status != BUFFER_SIZE) {
        printf("Data not written completely\n");
        goto error_poweroff;
    }

    printf("Finished writing\n");

    iter = 0;
    count = 0;

    printf("Starting reading after writing\n");

    /* Read 2KB data after writing to flash */
    status = ptrFLASH->ReadData(FLASH_ADDR, buff_read, BUFFER_SIZE);
    if (status != BUFFER_SIZE) {
        printf("Data not read completely\n");
        goto error_poweroff;
    }

    while (iter < BUFFER_SIZE) {
        if (buff_read[iter] != buff_write[iter]) {
            count++;
        }
        iter++;
    }

    printf("Total errors after reading data written to flash = %" PRIu32 "\n", count);

    iter = 0;
    count = 0;

    /* Erase 4KB sector */
    status = ptrFLASH->EraseSector(FLASH_ADDR);
    if (status != ARM_DRIVER_OK) {
        printf("Sector erase failed\n");
        goto error_poweroff;
    }

    printf("starting reading after erasing a sector\n");

    /* Read 2KB data after erasing a sector */
    status = ptrFLASH->ReadData(FLASH_ADDR, buff_read, BUFFER_SIZE);
    if (status != BUFFER_SIZE) {
        printf("Data not read completely\n");
        goto error_poweroff;
    }

    while (iter < BUFFER_SIZE) {
        if (buff_read[iter] != (flash_info->erased_value << 8 | flash_info->erased_value)) {
            count++;
        }
        iter++;
    }

    printf("Total errors after erasing a sector =  %" PRIu32 "\n", count);

    WAIT_FOREVER_LOOP

error_poweroff :
    status = ptrFLASH->PowerControl(ARM_POWER_OFF);
    if (status != ARM_DRIVER_OK) {
        printf("Flash Power control failed\n");
    }

error_uninitialize :
    status = ptrFLASH->Uninitialize();
    if (status != ARM_DRIVER_OK) {
        printf("Flash un-initialization failed\n");
    }

error_pinmux :

    printf("\r\n ISSI Flash demo thread exiting...\r\n");
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
    UCHAR    *pointer = first_unused_memory;
    INT      status  = 0;

    /* Create the main thread.  */
    status = tx_thread_create(&demo_thread, "demo_thread", demo_thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (status != TX_SUCCESS) {
        printf("Could not create Flash demo thread \n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
