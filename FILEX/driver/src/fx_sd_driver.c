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
 * @file     fx_sd_driver.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    FileX SD Driver entry, Based on FileX Ram Driver example code.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Include necessary system files.  */
#include "RTE_Device.h"
#include "fx_sd_driver.h"
#include "fx_sd_driver_private.h"
#ifdef SDMMC_PRINTF_DEBUG
#include "stdio.h"
#endif

#define SDMMC_XFER_DONE_EVENT 1

const diskio_t  *p_SD_Driver = &SD_Driver;
sd_cardinfo_t card_info;
#ifdef SDMMC_IRQ_MODE
TX_EVENT_FLAGS_GROUP sd_event;

VOID sd_fx_cb(uint32_t status)
{
    if (status == FX_SUCCESS)
        tx_event_flags_set(&sd_event, SDMMC_XFER_DONE_EVENT, TX_OR);
}
#endif

/* The SD driver relies on the fx_media_format call to be made prior to
   the fx_media_open call. The following call will format the default
   32KB SD drive, with a sector size of 128 bytes per sector.

   fx_media_format(&sd_disk,
   _fx_sd_driver,          // Driver entry
   sd_disk_memory,         // SD disk memory pointer
   media_memory,           // Media buffer pointer
   sizeof(media_memory),   // Media buffer size
   "MY_SD_DISK",           // Volume Name
   1,                      // Number of FATs
   32,                     // Directory Entries
   0,                      // Hidden sectors
   256,                    // Total sectors
   128,                    // Sector size
   1,                      // Sectors per cluster
   1,                      // Heads
   1);                     // Sectors per track

*/

VOID  _fx_sd_driver(FX_MEDIA *media_ptr)
{

    ULONG partition_start;
    ULONG partition_size;
    UCHAR status, retry_cnt = 1;
    sd_param_t sd_param;
#ifdef SDMMC_IRQ_MODE
    ULONG actual_event, wait_ticks = 50;
#endif

    /* There are several useful/important pieces of information contained in
       the media structure, some of which are supplied by FileX and others
       are for the driver to setup. The following is a summary of the
       necessary FX_MEDIA structure members:

       FX_MEDIA Member                    Meaning

       fx_media_driver_request             FileX request type. Valid requests from
       FileX are as follows:

       FX_DRIVER_READ
       FX_DRIVER_WRITE
       FX_DRIVER_FLUSH
       FX_DRIVER_ABORT
       FX_DRIVER_INIT
       FX_DRIVER_BOOT_READ
       FX_DRIVER_RELEASE_SECTORS
       FX_DRIVER_BOOT_WRITE
       FX_DRIVER_UNINIT

       fx_media_driver_status              This value is RETURNED by the driver.
       If the operation is successful, this
       field should be set to FX_SUCCESS for
       before returning. Otherwise, if an
       error occurred, this field should be
       set to FX_IO_ERROR.

       fx_media_driver_buffer              Pointer to buffer to read or write
       sector data. This is supplied by
       FileX.

       fx_media_driver_logical_sector      Logical sector FileX is requesting.

       fx_media_driver_sectors             Number of sectors FileX is requesting.

       The following is a summary of the optional FX_MEDIA structure members:

       FX_MEDIA Member                              Meaning

       fx_media_driver_info                Pointer to any additional information
       or memory. This is optional for the
       driver use and is setup from the
       fx_media_open call. The SD disk uses
       this pointer for the SD disk memory
       itself.

       fx_media_driver_write_protect       The DRIVER sets this to FX_TRUE when
       media is write protected. This is
       typically done in initialization,
       but can be done anytime.

       fx_media_driver_free_sector_update  The DRIVER sets this to FX_TRUE when
       it needs to know when clusters are
       released. This is important for FLASH
       wear-leveling drivers.

       fx_media_driver_system_write        FileX sets this flag to FX_TRUE if the
       sector being written is a system sector,
       e.g., a boot, FAT, or directory sector.
       The driver may choose to use this to
       initiate error recovery logic for greater
       fault tolerance.

       fx_media_driver_data_sector_read    FileX sets this flag to FX_TRUE if the
       sector(s) being read are file data sectors,
       i.e., NOT system sectors.

       fx_media_driver_sector_type         FileX sets this variable to the specific
       type of sector being read or written. The
       following sector types are identified:

    FX_UNKNOWN_SECTOR
        FX_BOOT_SECTOR
        FX_FAT_SECTOR
        FX_DIRECTORY_SECTOR
        FX_DATA_SECTOR
        */

        /* Process the driver request specified in the media control block.  */
        switch (media_ptr -> fx_media_driver_request)
        {

            case FX_DRIVER_READ:
                {

                    if(card_info.cardtype == SDMMC_CARD_SDSC)
                    {
                        uint32_t sec_cnt = media_ptr -> fx_media_driver_sectors;
                        for(uint32_t idx = 0; sec_cnt; sec_cnt--, idx++)
                        {

                            status = p_SD_Driver -> disk_read(media_ptr -> fx_media_driver_logical_sector + idx +
                                    media_ptr -> fx_media_hidden_sectors, 1,
                                    (UCHAR *)media_ptr -> fx_media_driver_buffer + (idx * SDMMC_BLK_SIZE_512_Msk));

                            status = tx_event_flags_get(&sd_event, SDMMC_XFER_DONE_EVENT, TX_OR_CLEAR, &actual_event, wait_ticks);
                        }

                        RTSS_InvalidateDCache_by_Addr(media_ptr -> fx_media_driver_buffer, media_ptr -> fx_media_driver_sectors * SDMMC_BLK_SIZE_512_Msk);

                        /* Check status of SD Read.  */
                        if (status == FX_SUCCESS)
                        {
                            /* Successful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                        }
                        else
                        {
                            /* Unsuccessful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                        }

                    }
                    else
                    {
                        status = p_SD_Driver -> disk_read(media_ptr -> fx_media_driver_logical_sector +
                                media_ptr -> fx_media_hidden_sectors, media_ptr -> fx_media_driver_sectors,
                                (UCHAR *)media_ptr -> fx_media_driver_buffer);
#ifdef SDMMC_IRQ_MODE
                        status = tx_event_flags_get(&sd_event, SDMMC_XFER_DONE_EVENT, TX_OR_CLEAR, &actual_event, wait_ticks);

                        RTSS_InvalidateDCache_by_Addr(media_ptr -> fx_media_driver_buffer, media_ptr -> fx_media_driver_sectors * SDMMC_BLK_SIZE_512_Msk);
#endif
                        /* Check status of SD Read.  */
                        if (status == FX_SUCCESS)
                        {
                            /* Successful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                        }
                        else
                        {
                            /* Unsuccessful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                        }
                    }

                    break;
                }

            case FX_DRIVER_WRITE:
                {
#ifdef SDMMC_IRQ_MODE
                    RTSS_CleanDCache_by_Addr(media_ptr -> fx_media_driver_buffer, media_ptr -> fx_media_driver_sectors * SDMMC_BLK_SIZE_512_Msk);
#endif

                    if(card_info.cardtype == SDMMC_CARD_SDSC)
                    {
                        uint32_t sec_cnt = media_ptr -> fx_media_driver_sectors;
                        for(uint32_t idx = 0; sec_cnt; sec_cnt--, idx++)
                        {

                            status = p_SD_Driver -> disk_write(media_ptr -> fx_media_driver_logical_sector + idx +
                                    media_ptr -> fx_media_hidden_sectors, 1,
                                    (UCHAR *)media_ptr -> fx_media_driver_buffer + (idx * SDMMC_BLK_SIZE_512_Msk));

                            status = tx_event_flags_get(&sd_event, SDMMC_XFER_DONE_EVENT, TX_OR_CLEAR, &actual_event, wait_ticks);
                        }

                        /* Check status of SD Read.  */
                        if (status == FX_SUCCESS)
                        {
                            /* Successful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                        }
                        else
                        {
                            /* Unsuccessful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                        }

                    }
                    else
                    {
                        status = p_SD_Driver -> disk_write((media_ptr -> fx_media_driver_logical_sector +
                                    media_ptr -> fx_media_hidden_sectors), media_ptr -> fx_media_driver_sectors,
                                (UCHAR *)media_ptr -> fx_media_driver_buffer);
#ifdef SDMMC_IRQ_MODE
                        status = tx_event_flags_get(&sd_event, SDMMC_XFER_DONE_EVENT, TX_OR_CLEAR, &actual_event, wait_ticks);
#endif
                        /* Check status of SD Write.  */
                        if (status == FX_SUCCESS)
                        {
                            /* Successful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                        }
                        else
                        {
                            /* Unsuccessful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                        }
                    }

                    break;
                }

            case FX_DRIVER_FLUSH:
                {
                    /* Return driver success.  */
                    media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                    break;
                }

            case FX_DRIVER_ABORT:
                {
                    /* Return driver success.  */
                    media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                    break;
                }

            case FX_DRIVER_INIT:
                {
                    /* FLASH drivers are responsible for setting several fields in the
                       media structure, as follows:

                       media_ptr -> fx_media_driver_free_sector_update
                       media_ptr -> fx_media_driver_write_protect

                       The fx_media_driver_free_sector_update flag is used to instruct
                       FileX to inform the driver whenever sectors are not being used.
                       This is especially useful for FLASH managers so they don't have
                       maintain mapping for sectors no longer in use.

                       The fx_media_driver_write_protect flag can be set anytime by the
                       driver to indicate the media is not writable.  Write attempts made
                       when this flag is set are returned as errors.  */

                    /* Perform basic initialization here... since the boot record is going
                       to be read subsequently and again for volume name requests.  */

                    sd_param.dev_id         = SDMMC_DEV_ID;
                    sd_param.clock_id       = RTE_SDC_CLOCK_SELECT;
                    sd_param.bus_width      = RTE_SDC_BUS_WIDTH;
                    sd_param.dma_mode       = RTE_SDC_DMA_SELECT;
                    sd_param.app_callback   = sd_fx_cb;

                    status = p_SD_Driver->disk_initialize(&sd_param);

                    /* Check status of SD initialize.  */
                    if (status == FX_SUCCESS)
                    {
                        /* Successful driver request.  */
                        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
#ifdef SDMMC_PRINTF_DEBUG
                        printf("Card Initialized Successfully...\n");
#endif
#ifdef SDMMC_IRQ_MODE
                        status = tx_event_flags_create(&sd_event, "SD_EVENT");
                        if (status != FX_SUCCESS)
                        {

                            /* Unsuccessful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                        }

                        /* get the card info */
                        p_SD_Driver->disk_info(&card_info);
#endif
                    }
                    else
                    {

                        /* Unsuccessful driver request.  */
                        media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                    }

                    break;
                }

            case FX_DRIVER_UNINIT:
                {

                    /* There is nothing to do in this case for the SD driver.  For actual
                       devices some shutdown processing may be necessary.  */

                    p_SD_Driver->disk_uninitialize(SDMMC_DEV_ID);

                    status = tx_event_flags_delete(&sd_event);
                    if(status)
                    {
#ifdef SDMMC_PRINTF_DEBUG
                        printf("Unable to delete sd event... status:%d\n",status);
#endif
                        media_ptr -> fx_media_driver_status = status;

                    }
                    else
                    {
                        memset(&sd_event, '\0', sizeof(sd_event));

                        /* Successful driver request.  */
                        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                    }
                    break;
                }

            case FX_DRIVER_BOOT_READ:
                {
#ifdef SDMMC_IRQ_MODE
RETRY:
#endif
#ifdef SDMMC_PRINTF_DEBUG
                    printf("Reading MBR...\n");
#endif
                    status = p_SD_Driver -> disk_read(0, media_ptr -> fx_media_driver_sectors,
                            (UCHAR *)media_ptr -> fx_media_driver_buffer);

                    /* Check status of SD Read.  */
                    if (status != FX_SUCCESS)
                    {

                        /* Unsuccessful driver request.  */
                        media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                        return;
                    }

#ifdef SDMMC_IRQ_MODE
                    status = tx_event_flags_get(&sd_event, SDMMC_XFER_DONE_EVENT, TX_OR_CLEAR, &actual_event, wait_ticks);

                    if ( status != TX_SUCCESS)
                    {
                        if(retry_cnt--)
                            goto RETRY;
                    }

                    RTSS_InvalidateDCache_by_Addr(media_ptr -> fx_media_driver_buffer, media_ptr -> fx_media_driver_sectors * SDMMC_BLK_SIZE_512_Msk);
#endif
                    /* Determine if we have the partition... */
                    partition_start =  0;

                    status =  _fx_partition_offset_calculate(media_ptr -> fx_media_driver_buffer, 0,
                            &partition_start, &partition_size);

                    /* Check partition read error.  */
                    if (status)
                    {

                        /* Unsuccessful driver request.  */
                        media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                        return;
                    }

                    /* Now determine if there is a partition...   */
                    if (partition_start)
                    {

                        status = p_SD_Driver -> disk_read(partition_start,1,(UCHAR *)media_ptr -> fx_media_driver_buffer);
#ifdef SDMMC_IRQ_MODE
                        status = tx_event_flags_get(&sd_event, SDMMC_XFER_DONE_EVENT, TX_OR_CLEAR, &actual_event, wait_ticks);

                        RTSS_InvalidateDCache_by_Addr(media_ptr -> fx_media_driver_buffer, 1 * SDMMC_BLK_SIZE_512_Msk);
#endif
                        /* Check status of SD Read.  */
                        if (status != FX_SUCCESS)
                        {
                            /* Unsuccessful driver request.  */
                            media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                            return;
                        }

                    }

                    /* Successful driver request.  */
                    media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                    break;
                }

            case FX_DRIVER_BOOT_WRITE:
                {
#ifdef SDMMC_IRQ_MODE
                    /* Clean the DCache */
                    RTSS_CleanDCache_by_Addr(media_ptr -> fx_media_driver_buffer, media_ptr -> fx_media_driver_sectors * SDMMC_BLK_SIZE_512_Msk);
#endif
                    status = p_SD_Driver -> disk_write((media_ptr -> fx_media_hidden_sectors ),
                            media_ptr -> fx_media_driver_sectors, (UCHAR *)media_ptr -> fx_media_driver_buffer);

                    /* Check status of SD Write.  */
                    if (status == FX_SUCCESS)
                    {
                        /* Successful driver request.  */
                        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                    }
                    else
                    {

                        /* Unsuccessful driver request.  */
                        media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                    }

                    break ;
                }

            default:
                {
                    /* Invalid driver request.  */
                    media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                    break;
                }
        }
}

