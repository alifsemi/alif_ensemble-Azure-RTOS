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
 * @file     sd.h
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    exposed SD Driver variables and APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef _SD_H_
#define _SD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sd_core.h"

//#define PRINTF_DEBUG
#define SD_4BIT_MODE

/**
 * @brief  SD driver status enum definition
 */
typedef enum{
    SD_OK,
    SD_HOST_INIT_ERR,
    SD_CARD_INIT_ERR,
    SD_RD_ERR,
    SD_WR_ERR,
    SD_TIMEOUT_ERR
}sd_drv_status_t;

/**
 * @brief  SD Card status enum definition
 */
typedef enum{
    SD_INIT = -1,
    SD_IDLE,
    SD_READY,
    SD_IDENT,
    SD_STBY,
    SD_TRAN,
    SD_DATA,
    SD_RCV,
    SD_PRG,
    SD_DIS,
    SD_RESV
}sd_status_t;

/**
 * @brief  SD Card Information Structure definition
 */
typedef struct{
    uint32_t CardType;          /*!< Specifies the card Type                        */
    uint32_t CardVersion;       /*!< Specifies the card version                     */
    uint32_t RelCardAdd;        /*!< Specifies the Relative Card Address            */
    uint32_t SectorCount;       /*!< Specifies the Card Capacity in blocks          */
    uint32_t SectorSize;        /*!< Specifies one block size in bytes              */
    uint32_t LogBlockNbr;       /*!< Specifies the Card logical Capacity in blocks  */
    uint32_t LogBlockSize;      /*!< Specifies logical block size in bytes          */
    uint32_t BusSpeed;          /*!< Clock                                          */
    uint16_t Class;             /*!< Specifies the class of the card class          */
    uint8_t isCardPresent;      /*!< is card present flag                           */
}SD_CardInfoTypeDef;

/**
 * @brief  SD command structure definition
 */
typedef struct{
    uint32_t arg;           /*!< SD Command Argument        */
    uint16_t xfer_mode;     /*!< SD Command transfer mode   */
    uint8_t cmdidx;         /*!< SD Command index           */
}SD_CmdTypeDef;

/**
 * @brief  Global SD Handle Information Structure definition
 */
typedef struct{
    SD_TypeDef          *sd_hc;         /*!< SD controller registers base address   */
    SD_CmdTypeDef       sd_cmd;         /*!< SD Command info                        */
    SD_CardInfoTypeDef  SdCard;         /*!< SD Card information                    */
    uint32_t            HC_Caps;        /*!< Host Controller capabilities           */
    __IO uint32_t       Context;        /*!< SD transfer context                    */
    __IO uint32_t       ErrorCode;      /*!< SD Card Error codes                    */
    uint32_t            CSD[4];         /*!< SD card specific data table            */
    uint32_t            CID[4];         /*!< SD card identification number table    */
    sd_status_t         State;          /*!< SD card State                          */
    uint16_t            HC_Version;     /*!< Host controller version                */
    uint8_t             BusWidth;       /*!< 1Bit, 4Bit, 8Bit Mode                  */
}SD_HandleTypeDef;

/**
 * @brief  Disk IO Driver structure definition
 */
typedef struct
{
    sd_drv_status_t (*disk_initialize) (uint8_t);                                       /*!< Initialize Disk Drive  */
    sd_status_t     (*disk_status)     (SD_HandleTypeDef *);                                /*!< Get Disk Status        */
    sd_drv_status_t (*disk_read)       (uint32_t, uint16_t, volatile unsigned char *);  /*!< Read Sector(s)         */
    sd_drv_status_t (*disk_write)      (uint32_t, uint32_t, volatile unsigned char *);  /*!< Write Sector(s)        */
}Diskio_TypeDef;

extern const Diskio_TypeDef SD_Driver;

/* SD Driver function forward declaration */
sd_status_t SD_status(SD_HandleTypeDef *);
sd_drv_status_t SD_init(uint8_t);
sd_drv_status_t SD_host_init(SD_HandleTypeDef *);
sd_drv_status_t SD_card_init(SD_HandleTypeDef *);
uint8_t getCmdRspType(uint8_t);
hc_status_t HC_SendCMD(SD_HandleTypeDef *, SD_CmdTypeDef *);
hc_status_t HC_reset(SD_HandleTypeDef *, uint8_t);
hc_status_t HC_reset(SD_HandleTypeDef *, uint8_t);
hc_status_t HC_SetBusPower(SD_HandleTypeDef *, uint8_t);
hc_status_t HC_SetClkFreq(SD_HandleTypeDef *, uint16_t);
hc_status_t HC_IdentifyCard(SD_HandleTypeDef *);
hc_status_t HC_GetCardIFCond(SD_HandleTypeDef *);
hc_status_t HC_GetCardOPCond(SD_HandleTypeDef *);
hc_status_t HC_GetCardCID(SD_HandleTypeDef *);
hc_status_t HC_GetCardCSD(SD_HandleTypeDef *);
sd_drv_status_t SD_write(uint32_t, uint32_t, volatile unsigned char *);
sd_drv_status_t SD_read(uint32_t, uint16_t, volatile unsigned char *);
sd_drv_status_t SDErrorHandler();
hc_status_t HC_ConfigDMA(SD_HandleTypeDef *, uint8_t);
hc_status_t HC_SetBus_Width(SD_HandleTypeDef *, uint8_t);

#ifdef __cplusplus
}
#endif

#endif
