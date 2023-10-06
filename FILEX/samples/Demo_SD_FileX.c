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
 * @file     Demo_SD_FileX.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    Test App for FileX.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
/* System Includes */
#include "stdio.h"

/* ThreadX and FileX Includes */
#include "tx_api.h"
#include "fx_api.h"
#include "fx_sd_driver.h"

/* include for Pin Mux config */
#include "pinconf.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


#define TEST_FILE "TestFile34.txt"
/* Define Test Requirement <Test File Name> */
//#define FILE_CREATE_TEST TEST_FILE
//#define FILE_READ_TEST TEST_FILE
#define FILE_WRITE_TEST TEST_FILE

#define K (1024)

/* Tasks Pool size, stack size, and pointers */
#define STACK_POOL_SIZE (40*K)
#define SD_STACK_SIZE (10*K)
#define SD_BLK_SIZE 512
#define NUM_BLK_TEST 20
#define SD_TEST_ITTR_CNT 10

TX_THREAD mySD_Thread;
TX_BYTE_POOL StackPool;
unsigned char *p_sdStack = NULL;
uint32_t count1, count2, total_cnt=0;

/* Buffer for FileX FX_MEDIA sector cache. This must be large enough for at least one sector , which are typically 512 bytes in size. */
UCHAR media_memory[SD_BLK_SIZE*NUM_BLK_TEST] __attribute__((section("sd_dma_buf"))) __attribute__((aligned(32)));
UCHAR filebuffer[SD_BLK_SIZE*NUM_BLK_TEST] __attribute__((section("sd_dma_buf"))) __attribute__((aligned(32)));
FX_MEDIA sd_card;
FX_FILE test_file;

/**
  \fn           mySD_Thread_entry(ULONG args)
  \brief        ThreadX and FileX integrated SD driver Test Function
  \param[in]    args - Thread argument
  \return       none
  */
void mySD_Thread_entry(ULONG args)
{
    UINT        status;
    ULONG       actual;
    ULONG       startCnt, EndCnt;

    pinconf_set(PORT_7, PIN_0, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE); //cmd
    pinconf_set(PORT_7, PIN_1, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE); //clk
    pinconf_set(PORT_5, PIN_0, PINMUX_ALTERNATE_FUNCTION_7, PADCTRL_READ_ENABLE); //d0
#ifdef SDMMC_4BIT_MODE
    pinconf_set(PORT_5, PIN_1, PINMUX_ALTERNATE_FUNCTION_7, PADCTRL_READ_ENABLE); //d1
    pinconf_set(PORT_5, PIN_2, PINMUX_ALTERNATE_FUNCTION_7, PADCTRL_READ_ENABLE); //d2
    pinconf_set(PORT_5, PIN_3, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE); //d3
#endif
#ifdef SDMMC_8BIT_MODE
    pinconf_set(PORT_5, PIN_4, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE); //d4
    pinconf_set(PORT_5, PIN_5, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE); //d5
    pinconf_set(PORT_5, PIN_6, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE); //d6
    pinconf_set(PORT_5, PIN_7, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE); //d7
#endif

    /* Open the SD disk. and initialize SD controller */
    status =  fx_media_open(&sd_card, "SD_DISK", _fx_sd_driver, 0, (VOID *)media_memory, sizeof(media_memory));

    /* Check the media open status.  */
    if (status != FX_SUCCESS)
    {
        printf("media open fail status = %d...\n",status);
        while(1);
    }
    printf("SD Mounted Successfully...\n");
#ifdef FILE_CREATE_TEST
    /* Create a file called TEST.TXT in the root directory.  */
    status =  fx_file_create(&sd_card, FILE_CREATE_TEST);

    /* Check the create status.  */
    if (status != FX_SUCCESS)
    {
        /* Check for an already created status. This is expected on the
           second pass of this loop!  */
        if (status != FX_ALREADY_CREATED)
        {
            while(1);
        }
    }

    /* Open the test file.  */
    status =  fx_file_open(&sd_card, &test_file, FILE_CREATE_TEST, FX_OPEN_FOR_READ);

    /* Check the file open status.  */
    if (status != FX_SUCCESS)
    {

        /* Error opening file, break the loop.  */
        while(1);
    }

    /* Seek to the beginning of the test file.  */
    status =  fx_file_seek(&test_file, 0);

    /* Check the file seek status.  */
    if (status != FX_SUCCESS)
    {

        /* Error performing file seek, break the loop.  */
        while(1);
    }

#elif defined(FILE_READ_TEST)
    /* Open the test file.  */
    status =  fx_file_open(&sd_card, &test_file, FILE_READ_TEST, FX_OPEN_FOR_READ);

    /* Check the file open status.  */
    if (status != FX_SUCCESS)
    {
        /* Error opening file, break the loop.  */
        printf("File open status: %d\n",status);
        printf("%s File not Found...\n",FILE_READ_TEST);
        while(1);
    }

    /* Seek to the beginning of the test file.  */
    status =  fx_file_seek(&test_file, 0);

    /* Check the file seek status.  */
    if (status != FX_SUCCESS)
    {
        printf("File seek status: %d\n",status);
        /* Error performing file seek, break the loop.  */
        while(1);
    }

    printf("Reading Data from File...%s\n",FILE_READ_TEST);
    memset(filebuffer,'\0',sizeof(filebuffer));

    while(1){

        status =  fx_file_read(&test_file, filebuffer, (SD_BLK_SIZE * NUM_BLK_TEST), &actual);

        /* Check the file read status.  */
        if (status != FX_SUCCESS)
        {
            if(status == FX_END_OF_FILE){
                printf("End of File\n");
                break;
            }
            else{
                /* Error performing file read, break the loop.  */
                printf("File read status: %d\n",status);
                break;
            }
        }

        printf("actual size = %lu\n %s\n",actual, (const char *)filebuffer);

    }

#elif defined(FILE_WRITE_TEST)

    /* Create a file called FILE_WRITE_TEST in the root directory.  */
    status =  fx_file_create(&sd_card, FILE_WRITE_TEST);

    /* Check the create status.  */
    if (status != FX_SUCCESS)
    {
        /* Check for an already created status. This is expected on the
           second pass of this loop!  */
        printf("File create status: %d\n",status);
        if (status != FX_ALREADY_CREATED)
        {
            while(1);
        }
    }

    /* Open the test file.  */
    status = fx_file_open(&sd_card, &test_file, FILE_WRITE_TEST, FX_OPEN_FOR_WRITE);

    /* Check the file open status.  */
    if (status != FX_SUCCESS)
    {
        printf("File open status: %d\n",status);
        /* Error opening file, break the loop.  */
        while(1);
    }

    /* Seek to the beginning of the test file.  */
    status = fx_file_seek(&test_file, 0);

    /* Check the file seek status.  */
    if (status != FX_SUCCESS)
    {
        printf("File seek status: %d\n",status);
        /* Error performing file seek, break the loop.  */
        while(1);
    }

    printf("Writing Data in File...%s\n",FILE_WRITE_TEST);
    memset(filebuffer, '\0', sizeof(filebuffer));
    for(int i = 0; i<SD_TEST_ITTR_CNT; i++){

        memset(filebuffer, 'D', (SD_BLK_SIZE * NUM_BLK_TEST));

        /* Write a string to the test file.  */
        status =  fx_file_write(&test_file, (void *)filebuffer, (SD_BLK_SIZE * NUM_BLK_TEST));

        tx_thread_sleep(1);

        /* Check the file write status.  */
        if (status != FX_SUCCESS)
        {
            printf("ittr: %d File write status: %d\n",i, status);

            /* Error writing to a file, break the loop.  */
            if (status == FX_NO_MORE_SPACE){
                printf("ittr: %d No More Space...\nFlushing out data...\n",i);
                break;
            }
        }
    }
#else
#error "No Test Defined...\n"

#endif

    printf("Closing File...%s\n",TEST_FILE);
    /* Close the test file.  */
    status =  fx_file_close(&test_file);

    /* Check the file close status.  */
    if (status != FX_SUCCESS)
    {
        printf("File close status: %d\n",status);
        /* Error closing the file, break the loop.  */
        while(1);
    }

    printf("Closing Media...\n");
    /* Close the media.  */
    status =  fx_media_close(&sd_card);

    /* Check the media close status.  */
    if (status != FX_SUCCESS)
    {
        /* Error closing the media, break the loop.  */
        printf("Media close status: %d\n",status);
        while(1);
    }
    printf("File R/W Test Completed!!!\n");

}

void tx_application_define(void *first_unused_memory){

    /* Tasks memory allocation and creation */
    tx_byte_pool_create(&StackPool, "Stack_pool", first_unused_memory, STACK_POOL_SIZE);

    tx_byte_allocate(&StackPool, (void **)&p_sdStack, SD_STACK_SIZE, TX_NO_WAIT);
    tx_thread_create(&mySD_Thread, "SD_thread", mySD_Thread_entry, NULL, p_sdStack, SD_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

    /* FileX Initialization */
    fx_system_initialize();

    return;
}

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

    tx_kernel_enter();

    return 0;
}

