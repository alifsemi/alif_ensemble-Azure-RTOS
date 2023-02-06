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
#include "Driver_PINMUX_AND_PINPAD.h"

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

    int _sys_open(void *p){

        return 0;
    }

    void _ttywrch(int ch){

    }
#endif

/* Define Test Requirement <Test File Name> */
//#define FILE_CREATE_TEST "TEST4.txt"
#define FILE_READ_TEST "TestFile63.txt"
//#define FILE_WRITE_TEST "TestFile63.txt"

#define K (1024)

/* Tasks Pool size, stack size, and pointers */
#define STACK_POOL_SIZE (40*K)
#define SD_STACK_SIZE (10*K)

TX_THREAD mySD_Thread;
TX_BYTE_POOL StackPool;
unsigned char *p_sdStack = NULL;

/* Buffer for FileX FX_MEDIA sector cache. This must be large enough for at least one sector , which are typically 512 bytes in size. */
volatile UCHAR media_memory[512*4] __attribute__((section("sd_dma_buf"))) __attribute__((aligned(32)));
unsigned char filebuffer[512*4] __attribute__((aligned(32)));
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

    /* Board Pin mux Configurations */
    PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_10, PINMUX_ALTERNATE_FUNCTION_3); //CMD
    PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_11, PINMUX_ALTERNATE_FUNCTION_3); //CLK
    PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_12, PINMUX_ALTERNATE_FUNCTION_4); //D0
#ifdef SD_4BIT_MODE
    PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_13, PINMUX_ALTERNATE_FUNCTION_4); //D1
    PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_14, PINMUX_ALTERNATE_FUNCTION_3); //D2
    PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_15, PINMUX_ALTERNATE_FUNCTION_4); //D3
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

    /* Close the test file.  */
    status =  fx_file_close(&test_file);

    /* Check the file close status.  */
    if (status != FX_SUCCESS)
    {

        /* Error closing the file, break the loop.  */
        while(1);
    }

    /* Close the media.  */
    status =  fx_media_close(&sd_card);

    /* Check the media close status.  */
    if (status != FX_SUCCESS)
    {

        /* Error closing the media, break the loop.  */
        while(1);
    }
#endif

#ifdef FILE_READ_TEST
    /* Open the test file.  */
    status =  fx_file_open(&sd_card, &test_file, FILE_READ_TEST, FX_OPEN_FOR_READ);

    /* Check the file open status.  */
    if (status != FX_SUCCESS)
    {
        /* Error opening file, break the loop.  */
        printf("File open status: %d\n",status);
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

    for(int i = 0;i<80;i++)
    {
        memset(filebuffer,'\0',512);

        status =  fx_file_read(&test_file, filebuffer, 511, &actual);

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

        printf("%s\n",(const char *)filebuffer);
    }

    /* Close the test file.  */
    status =  fx_file_close(&test_file);

    /* Check the file close status.  */
    if (status != FX_SUCCESS)
    {
        printf("File close status: %d\n",status);
        /* Error closing the file, break the loop.  */
        while(1);
    }

    /* Close the media.  */
    status =  fx_media_close(&sd_card);

    /* Check the media close status.  */
    if (status != FX_SUCCESS)
    {
        /* Error closing the media, break the loop.  */
        printf("Media close status: %d\n",status);
        while(1);
    }
#endif

#ifdef FILE_WRITE_TEST

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
    for(int i = 0;i<100;i++){
        sprintf(filebuffer,"Hello World Target Write Test : %d\n",i);

        /* Write a string to the test file.  */
        status =  fx_file_write(&test_file, (void *)filebuffer, strlen(filebuffer));

        /* Check the file write status.  */
        if (status != FX_SUCCESS)
        {
            printf("File write status: %d\n",status);
            /* Error writing to a file, break the loop.  */
            if (status == FX_NO_MORE_SPACE){
                printf("No More Space...\nFlushing out data...\n");
                break;
            }

            while(1);
        }
    }
    /* Close the test file.  */
    status =  fx_file_close(&test_file);

    /* Check the file close status.  */
    if (status != FX_SUCCESS)
    {
        printf("File close status: %d\n",status);
        /* Error closing the file, break the loop.  */
        while(1);
    }

    /* Close the media.  */
    status =  fx_media_close(&sd_card);

    /* Check the media close status.  */
    if (status != FX_SUCCESS)
    {
        printf("Media close status: %d\n",status);
        /* Error closing the media, break the loop.  */
        while(1);
    }
#endif
    printf("File R/W Test Complete!!!\n");

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

int main(){

    tx_kernel_enter();

    return 0;
}
