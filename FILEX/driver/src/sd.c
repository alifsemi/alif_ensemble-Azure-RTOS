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
 * @file     sd.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    SD Driver APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "system_utils.h"
#include "global_map.h"
#include "sd.h"
#include "string.h"

/* Global SD Driver Callback definitions */
const Diskio_TypeDef SD_Driver =
{
    SD_init,
    SD_status,
    SD_read,
    SD_write,
};

/* Global SD Handle */
static SD_HandleTypeDef Hsd;

/**
  \fn           SDErrorHandler
  \brief        SD Driver error Handler [TODO]
  \param[in]    SD error number
  \return       sd driver state
  */
sd_drv_status_t SDErrorHandler(){
    return ~SD_OK;
}

/**
  \fn           SD_host_init
  \brief        initialize host controller
  \param[in]    SD Global Handle pointer
  \return       sd driver status
  */
sd_drv_status_t SD_host_init(SD_HandleTypeDef *pHsd){

    uint8_t PowerLevel;

    /* set some default values */
    pHsd->sd_hc     = (SDIO_TypeDef *)SDMMC_BASE;
    pHsd->State     = SD_INIT;

    /* Get the Host Controller version */
    pHsd->HC_Version = *((volatile uint16_t *)(HC_VERSION_REG)) & HC_VERSION_REG_MASK;

    /* Get the Host Controller Capabilities */
    pHsd->HC_Caps = pHsd->sd_hc->CAPABILITIES1;

    /* Disable the SD Voltage supply */
    HC_SetBusPower(pHsd, 0x0);

    /* Soft reset Host controller cmd and data lines */
    HC_reset(pHsd, (uint8_t)(SD_SW_RST_ALL_MASK));

    /* Get the host voltage capability */
    if ((pHsd->HC_Caps & HOST_SD_CAP_VOLT_3V3_MASK) != 0U) {
        PowerLevel = SD_PC_BUS_VSEL_3V3_MASK;
    } else if ((pHsd->HC_Caps & HOST_SD_CAP_VOLT_3V0_MASK) != 0U) {
        PowerLevel = SD_PC_BUS_VSEL_3V0_MASK;
    } else if ((pHsd->HC_Caps & HOST_SD_CAP_VOLT_1V8_MASK) != 0U) {
        PowerLevel = SD_PC_BUS_VSEL_1V8_MASK;
    } else {
        PowerLevel = 0U;
    }

    HC_SetBusPower(pHsd, PowerLevel);

    HC_ConfigDMA(pHsd, (uint8_t)(SD_DMA_SEL | SD_DMA_SEL_1BIT_MODE));

    return SD_OK;
}

/**
  \fn           SD_card_init
  \brief        initialize card
  \param[in]    sd global handle pointer
  \return       sd driver status
  */
sd_drv_status_t SD_card_init(SD_HandleTypeDef *pHsd){

    uint32_t reg;
    sd_drv_status_t status = SD_OK;

    /* Default settings */
    pHsd->BusWidth          = SD_1_BIT_WIDTH;
    pHsd->SdCard.CardType   = SD_CARD_SDHC;
    pHsd->SdCard.BusSpeed   = SD_CLK_400_KHZ;

    HC_SetBusPower(pHsd, (uint8_t)(SD_PC_BUS_VSEL_3V3_MASK | SD_PC_BUS_PWR_VDD1_MASK));

    reg = SD_CLK_GEN_SEL_MASK | SD_UPPER_FREQ_SEL | SD_PLL_EN_MASK | \
          SD_CLK_EN_MASK | SD_INTERNAL_CLK_EN_MASK;
    HC_SetClkFreq(pHsd, (uint16_t)reg);

    pHsd->State = SD_IDLE;

    PMU_delay_loop_us(100);

    /* Reset the command structure */
    pHsd->sd_cmd.cmdidx       = 0;
    pHsd->sd_cmd.arg          = 0;
    pHsd->sd_cmd.xfer_mode    = 0;

    /* Check and wait till the card is present and Reset It */
    if(HC_IdentifyCard(pHsd) != HC_OK){
        /* Card Not present */
        status = SD_TIMEOUT_ERR;
        goto exit;
    }

    /* Reset the SD/UHS Cards */
    pHsd->sd_cmd.cmdidx       = CMD0;
    pHsd->sd_cmd.arg          = 0;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        SDErrorHandler();
    }

    /* Get the card interface condition */
    if(HC_GetCardIFCond(pHsd) != HC_OK){
        status = SD_CARD_INIT_ERR;
        goto exit;
    }

    /* Get the card operating condition */
    if(HC_GetCardOPCond(pHsd) != HC_OK){
        status = SD_CARD_INIT_ERR;
        goto exit;
    }

    /* Get the card ID CMD2 */
    if(HC_GetCardCID(pHsd) != HC_OK){
        status = SD_CARD_INIT_ERR;
        goto exit;
    }

    /* Get the CSD register */
    if(HC_GetCardCSD(pHsd)!= HC_OK){
        status = SD_CARD_INIT_ERR;
        goto exit;
    }

    /* Change the Card State from Identification to Ready */
    pHsd->State = SD_STBY;

    /* Select the card to transition to transfer state */
    pHsd->sd_cmd.cmdidx       = CMD7;
    pHsd->sd_cmd.arg          = pHsd->SdCard.RelCardAdd;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        SDErrorHandler();
    }

    /* Change the Card State from Ready to Tran */
    pHsd->State = SD_TRAN;

#ifdef SD_4BIT_MODE
    if(HC_SetBus_Width(pHsd, SD_4_BIT_WIDTH) != HC_OK){
        return SD_CARD_INIT_ERR;
    }
#endif

    if(pHsd->sd_hc->BLOCK_SIZE != SD_BLK_SIZE_512_MASK){
        pHsd->sd_hc->BLOCK_SIZE = SD_BLK_SIZE_512_MASK;
        /* set the block size */
        pHsd->sd_cmd.cmdidx       = CMD16;
        pHsd->sd_cmd.arg          = SD_BLK_SIZE_512_MASK;
        if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
            SDErrorHandler();
        }
    }

    reg = SD_FREQ_SEL | SD_CLK_GEN_SEL_MASK | SD_PLL_EN_MASK |
          SD_CLK_EN_MASK | SD_INTERNAL_CLK_EN_MASK;
    HC_SetClkFreq(pHsd, (uint16_t)reg);

exit:
    return status;
}

/**
  \fn           SD_init
  \brief        main SD initialize function
  \param[in]    device ID
  \return       sd driver status
  */
sd_drv_status_t SD_init(uint8_t devId){

    sd_drv_status_t errcode = SD_OK;

    /* Initialize Host controller */
    errcode = SD_host_init(&Hsd);

    if(errcode != SD_OK){
        return SD_HOST_INIT_ERR;
    }

    /* Initialize Card */
    errcode = SD_card_init(&Hsd);

    if(errcode != SD_OK)
        return SD_CARD_INIT_ERR;

    return SD_OK;
}

/**
  \fn           SD_status
  \brief        SD Status
  \param[in]    Global SD handle pointer
  \return       sd driver status
  */
sd_status_t SD_status(SD_HandleTypeDef *pHsd){
    uint32_t status;

    /* Check current card status */
    pHsd->sd_cmd.cmdidx       = CMD13;
    pHsd->sd_cmd.arg          = pHsd->SdCard.RelCardAdd;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        SDErrorHandler();
    }
    status = pHsd->sd_hc->RESP01;

    status = (status & SD_STATUS_MASK) >> SD_STATUS_POS;

#ifdef PRINTF_DEBUG
    printf("Card Status: %x\n",status);
#endif

    return status;
}

/**
  \fn           sd_drv_status_t SD_read(uint32_t sec, uint32_t BlkCnt, volatile unsigned char * DestBuff)
  \brief        read sd sector
  \param[in]    sec - input sector number to read
  \param[in]    BlkCnt - number of block to readd
  \param[in]    DestBuff - Destination buffer pointer
  \return       sd driver status
  */
sd_drv_status_t SD_read(uint32_t sec, uint16_t BlkCnt, volatile unsigned char *DestBuff){

    SD_HandleTypeDef *pHsd =  &Hsd;
    uint32_t pstate = 0, dma_irq=0;
    uint32_t status;
    int timeout_cnt = 1000 * BlkCnt;

    if(DestBuff == NULL)
        return SD_RD_ERR;

#ifdef PRINTF_DEBUG
    printf("SD READ Dest Buff: 0x%p Sec: %u, Block Count: %u\n",DestBuff,sec,BlkCnt);
#endif

    status = SD_status(pHsd);
    if(status != SD_TRAN && status != SD_DATA && status != SD_RCV && status != SD_PRG)
        return SD_RD_ERR;

    pHsd->sd_hc->HOST_CTRL1 |= SD_LED_ON; //led caution on

    /* Configure DMA */
    pHsd->sd_hc->ADMA_SA_LOW = (uint32_t)LocalToGlobal(DestBuff);
    pHsd->sd_hc->BLOCKCOUNT = BlkCnt;

    /* Change the Card State from Tran to Data */
    pHsd->State = SD_DATA;

    if(BlkCnt == 1){
        /* read single block request */
        pHsd->sd_cmd.cmdidx       = CMD17;
        pHsd->sd_cmd.arg          = sec;
        pHsd->sd_cmd.xfer_mode    = XFER_MODE_DATA_XFER_RD_MASK |
                                    XFER_MODE_AUTO_CMD12 << XFER_MODE_AUTO_CMD_EN_Pos |
                                    XFER_MODE_DMA_EN_MASK;

        if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
            SDErrorHandler();
        }

    }else{
        /* read multiple block */
        pHsd->sd_cmd.cmdidx       = CMD18;
        pHsd->sd_cmd.arg          = sec;
        pHsd->sd_cmd.xfer_mode    = XFER_MODE_DATA_XFER_RD_MASK | XFER_MODE_MULTI_BLK_SEL_MASK |
                                    XFER_MODE_AUTO_CMD23 << XFER_MODE_AUTO_CMD_EN_Pos |
                                    XFER_MODE_DMA_EN_MASK;
        if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
            SDErrorHandler();
        }
    }

    /* check for transfer active state */
    while(timeout_cnt){
        dma_irq = pHsd->sd_hc->NORMAL_INT_STAT;
        dma_irq = (dma_irq & NORMAL_INT_STAT_XFER_COMPLETE_MASK) >> NORMAL_INT_STAT_XFER_COMPLETE_POS;

        if(dma_irq)
            break;

        PMU_delay_loop_us(1);
        timeout_cnt--;
    }

    pHsd->sd_hc->HOST_CTRL1 ^= SD_LED_ON; //led caution off

    /* Change the Card State from Data to Tran */
    pHsd->State = SD_TRAN;

    SCB_InvalidateDCache_by_Addr(DestBuff, BlkCnt * SD_BLK_SIZE_512_MASK);

    if(!timeout_cnt)
        return SD_TIMEOUT_ERR;

    return SD_OK;
}

/**
  \fn           sd_drv_status_t SD_write(uint32_t sec, uint32_t BlkCnt, volatile unsigned char * SrcBuff)
  \brief        Write sd sector
  \param[in]    sector - input sector number to write
  \param[in]    BlkCnt - number of block to write
  \param[in]    SrcBuff - Source buffer pointer
  \return       sd driver status
  */
sd_drv_status_t SD_write(uint32_t sector, uint32_t BlkCnt, volatile unsigned char *SrcBuff){

    uint32_t pstate = 0, dma_irq=0;
    SD_HandleTypeDef *pHsd =  &Hsd;
    int timeout_cnt = 1000 * BlkCnt;
    uint32_t status;
    if(SrcBuff == NULL)
        return SD_WR_ERR;

#ifdef PRINTF_DEBUG
    printf("SD WRITE Src Buff: 0x%p Sec: %d, Block Count: %d\n",SrcBuff,sector,BlkCnt);
#endif

    status = SD_status(pHsd);
    if(status != SD_TRAN && status != SD_DATA && status != SD_RCV && status != SD_PRG)
        return SD_WR_ERR;

    pHsd->sd_hc->HOST_CTRL1 |= SD_LED_ON; //led caution on

    /* Clean the DCache */
    SCB_CleanDCache_by_Addr(SrcBuff, BlkCnt * SD_BLK_SIZE_512_MASK);

    /* Configure DMA */
    pHsd->sd_hc->ADMA_SA_LOW = (uint32_t)LocalToGlobal((void *)SrcBuff);
    pHsd->sd_hc->BLOCKCOUNT = BlkCnt;

    if(BlkCnt == 1){
        /* write single block request */
        pHsd->sd_cmd.cmdidx       = CMD24;
        pHsd->sd_cmd.arg          = sector;
        pHsd->sd_cmd.xfer_mode    = XFER_MODE_DATA_XFER_WR_MASK |
                                    XFER_MODE_AUTO_CMD12 << XFER_MODE_AUTO_CMD_EN_Pos |
                                    XFER_MODE_DMA_EN_MASK;
        if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
            SDErrorHandler();
        }

    }else{
        /* write multiple block */
        pHsd->sd_cmd.cmdidx       = CMD25;
        pHsd->sd_cmd.arg          = sector;
        pHsd->sd_cmd.xfer_mode    = XFER_MODE_DATA_XFER_WR_MASK | XFER_MODE_MULTI_BLK_SEL_MASK |
                                    XFER_MODE_AUTO_CMD23 << XFER_MODE_AUTO_CMD_EN_Pos |
                                    XFER_MODE_DMA_EN_MASK;
        if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
            SDErrorHandler();
        }
    }

    /* check for transfer active state */
    while(timeout_cnt){
        dma_irq = pHsd->sd_hc->NORMAL_INT_STAT;
        dma_irq = (dma_irq & NORMAL_INT_STAT_XFER_COMPLETE_MASK) >> NORMAL_INT_STAT_XFER_COMPLETE_POS;

        if(dma_irq)
            break;

        PMU_delay_loop_us(1);
        timeout_cnt--;
    }

    pHsd->sd_hc->HOST_CTRL1 ^= SD_LED_ON; //led caution off

    if(!timeout_cnt)
        return SD_TIMEOUT_ERR;

    return SD_OK;
}
