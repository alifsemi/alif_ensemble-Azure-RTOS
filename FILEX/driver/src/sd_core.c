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
 * @file     sd_core.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    SD Host Controller Driver APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "system_utils.h"
#include "sd_core.h"
#include "sd.h"

/**
  \fn          uint8_t getCmdRspType(uint8_t Cmd)
  \brief       get expected response of given command.
  \param[in]   cmd - input cmd index
  \return      expected response type.
  */
uint8_t getCmdRspType(uint8_t Cmd){

    uint8_t RetVal=0;

    switch(Cmd)
    {
        case CMD0:
            RetVal = RESP_NONE;
            break;
        case CMD1:
            RetVal = RESP_R48;
            break;
        case CMD2:
            RetVal = RESP_R136;
            break;
        case CMD3:
            RetVal = RESP_R48;
            break;
        case CMD5:
            RetVal = RESP_R48;
            break;
        case CMD6:
            RetVal = RESP_R48;
            break;
        case ACMD6:
            RetVal = RESP_R48;
            break;
        case CMD7:
            RetVal = RESP_R48;
            break;
        case CMD8:
            RetVal = RESP_R48;
            break;
        case CMD9:
            RetVal = RESP_R136;
            break;
        case CMD13:
            RetVal = RESP_R48;;
            break;
        case CMD16:
            RetVal = RESP_R48;
            break;
        case CMD17:
            RetVal = 2<<4|RESP_R48;
            break;
        case CMD24:
            RetVal = 2<<4|RESP_R48;;
            break;
        case CMD25:
            RetVal = 2<<4|RESP_R48;;
            break;
        case CMD41:
            RetVal = RESP_R48;
            break;
        case CMD52:
            RetVal = RESP_R48;
            break;
        case CMD53:
            RetVal = RESP_R48;
            break;
        case CMD55:
            RetVal = RESP_R48;
            break;
        default:
            RetVal = RESP_NONE;
            break;
    }

    return RetVal;
}

/**
  \fn          hc_status_t HC_SendCMD(SD_HandleTypeDef *pHsd, SD_CmdTypeDef *pCmd)
  \brief       Configure Host controller to send SD command
  \param[in]   pHsd - Global SD Handle pointer
  \param[in]   pcmd - command info pointer
  \return      Host controller driver API status.
  */
hc_status_t HC_SendCMD(SD_HandleTypeDef *pHsd, SD_CmdTypeDef *pCmd){

    uint16_t cmd;
    uint8_t rsp_type;

    rsp_type = getCmdRspType(pCmd->cmdidx);

    cmd = (pCmd->cmdidx <<  CMD_IDX_POS) | ( rsp_type );

    pHsd->sd_hc->ARG        = pCmd->arg;
    pHsd->sd_hc->XFER_MODE  = pCmd->xfer_mode;
    pHsd->sd_hc->CMD        = cmd;

#ifdef PRINTF_DEBUG
    printf("CMD: 0x%04x, ARG: 0x%08x, XFER: 0x%04x Resp01 %08x, Resp23 %08x, Resp45 %08x, Resp67 %08x\n",
            cmd,pCmd->arg,pCmd->xfer_mode,pHsd->sd_hc->RESP01,pHsd->sd_hc->RESP23,pHsd->sd_hc->RESP45,pHsd->sd_hc->RESP67);
#else
    PMU_delay_loop_us(2000);
#endif

    return HC_OK;
}

/**
  \fn           hc_status_t HC_reset(SD_HandleTypeDef *pHsd, uint8_t resetVal)
  \brief        software reset of the controller
  \param[in]    Global SD Handle pointer
  \param[in]    Reset Value
  \return       Host controller driver status
  */
hc_status_t HC_reset(SD_HandleTypeDef *pHsd, uint8_t resetVal){

    uint8_t currResetVal=0;

    /* Reset the HC lines */
    pHsd->sd_hc->SW_RST = resetVal;

    do{
        currResetVal = pHsd->sd_hc->SW_RST;
        PMU_delay_loop_us(100);
    }while(currResetVal != resetVal);

    return HC_OK;
}

/**
  \fn           hc_status_t HC_SetBusPower(SD_HandleTypeDef *pHsd, uint8_t PwrVal){
  \brief        Set required sd bus power supply
  \param[in]    Global SD Handle pointer
  \param[in]    required bus voltage equivalent register value as per host controller data sheet
  \return       Host controller driver status
  */
hc_status_t HC_SetBusPower(SD_HandleTypeDef *pHsd, uint8_t PwrVal){

    /* Disable clock */
    pHsd->sd_hc->PWR_CTRL = 0U;

    /* If selected power is zero, return from here */
    if (PwrVal == 0U) {
        return HC_OK;
    }

    /* set the required voltage level */
    pHsd->sd_hc->PWR_CTRL = PwrVal;

    /* .2ms delay after power on/off */
    PMU_delay_loop_us(200);

    return HC_OK;
}

/**
  \fn           hc_status_t HC_SetClkFreq(SD_HandleTypeDef *pHsd, uint32_t ClkFreq)
  \brief        Sets required clock
  \param[in]    Global SD Handle pointer
  \param[in]    required bus clock equivalent register value as per host controller data sheet
  \return       Host controller driver status
  */
hc_status_t HC_SetClkFreq(SD_HandleTypeDef *pHsd, uint16_t ClkFreq){

    uint16_t reg;

    /* Disable clock */
    pHsd->sd_hc->CLK_CTRL = 0;

    /* If selected frequency is zero, return from here */
    if (ClkFreq == 0U) {
        return HC_OK;
    }

    pHsd->sd_hc->CLK_CTRL = ClkFreq;

    PMU_delay_loop_us(1000);

    /* SD clk stability */
    do{
        reg = pHsd->sd_hc->CLK_CTRL;
        PMU_delay_loop_us(1);
    }while((reg & SD_INTERNAL_CLK_STABLE_MASK) == 0);

    return HC_OK;
}

/**
  \fn           hc_status_t HC_ConfigDMA(SD_HandleTypeDef *pHsd,uint8_t dmaMask)
  \brief        Host Controller DMA configuration
  \param[in]    Global SD Handle pointer
  \param[in]    dmaMask - Host ctrl 1 register value
  \return       Host controller driver status
  */
hc_status_t HC_ConfigDMA(SD_HandleTypeDef *pHsd, uint8_t dmaMask){

    /* Host Version 4 Param */
    pHsd->sd_hc->HOST_CTRL2 = 0x5801; //SDR25 Mode selected
    pHsd->sd_hc->HOST_CTRL1 = dmaMask;

    return HC_OK;
}

#ifdef SD_4BIT_MODE
/**
  \fn           hc_status_t HC_SetBus_Width(SD_HandleTypeDef *pHsd, uint8_t buswidth)
  \brief        Configure required bit for communication for Host and Card
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    buswidth - number of Data lines for data transfer
  \return       Host controller driver status
  */
hc_status_t HC_SetBus_Width(SD_HandleTypeDef *pHsd, uint8_t buswidth){

    uint8_t temp;

    if(SD_status(pHsd) != SD_TRAN)
        return HC_SD_INV_STATE; /* SD Must be TRAN state to change Bus width */

    temp = pHsd->sd_hc->HOST_CTRL1;

    if(buswidth == SD_1_BIT_WIDTH)
        temp = temp | SD_1_BIT_WIDTH;
    else if(buswidth == SD_4_BIT_WIDTH)
        temp = temp | SD_4_BIT_WIDTH | SD_HIGH_SPEED_MODE_EN;
    else
        temp = temp | SD_8_BIT_WIDTH | SD_HIGH_SPEED_MODE_EN;

    pHsd->sd_hc->HOST_CTRL1 = temp;

    /* just to indicate SD Card that the next cmd is APP CMD */
    pHsd->sd_cmd.cmdidx       = CMD55;
    pHsd->sd_cmd.arg          = pHsd->SdCard.RelCardAdd;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        SDErrorHandler();
    }

    /* Send ACMD6 to change Bus width */
    pHsd->sd_cmd.cmdidx       = CMD6;
    pHsd->sd_cmd.arg          = 0x2;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        SDErrorHandler();
    }

    return HC_OK;
}

#endif

/**
  \fn           void HC_ConfigInterrupt(SD_HandleTypeDef *pHsd)
  \brief        Configure Error and interrupt in Host Controller
  \param[in]    pHsd - Global SD Handle pointer
  \return       none
  */
void HC_ConfigInterrupt(SD_HandleTypeDef *pHsd){

    /* Enable all interrupt status except card interrupt initially */
    pHsd->sd_hc->NORMAL_INT_STAT_EN = SD_NORM_INTR_ALL_MASK & (~SD_INTR_CARD_MASK);

    pHsd->sd_hc->ERROR_INT_STAT_EN = SD_ERROR_INTR_ALL_MASK;

    /* Disable all interrupt signals by default. */
    pHsd->sd_hc->NORMAL_INT_SIGNAL_EN = 0x0U;

    pHsd->sd_hc->ERROR_INT_SIGNAL_EN = 0x0U;

}

/**
  \fn           hc_status_t HC_IdentifyCard(SD_HandleTypeDef *pHsd)
  \brief        Indentify the inserted cards
  \param[in]    Global SD Handle pointer
  \return       Host controller driver status
  */
hc_status_t HC_IdentifyCard(SD_HandleTypeDef *pHsd){

    uint32_t isCardPresent = 0, timeout = 0xFFFFU;
    hc_status_t status;

    HC_reset(pHsd, 0x06); // reset CMD and DATA Line

#ifdef PRINTF_DEBUG
    printf("Waiting for SD Card to be inserted...\n");
#endif

    /* Card Insertion and Removal State and Signal Enable */
    pHsd->sd_hc->NORMAL_INT_SIGNAL_EN = SD_INTR_CC_MASK | SD_INTR_TC_MASK | SD_INTR_DMA_MASK |\
                                        SD_INTR_BWR_MASK | SD_INTR_BRR_MASK | SD_INTR_CARD_INSRT_MASK |\
                                        SD_INTR_CARD_REM_MASK;

    pHsd->sd_hc->NORMAL_INT_STAT_EN = SD_INTR_CC_MASK | SD_INTR_TC_MASK | SD_INTR_DMA_MASK |\
                                      SD_INTR_BWR_MASK | SD_INTR_BRR_MASK | SD_INTR_CARD_INSRT_MASK |\
                                      SD_INTR_CARD_REM_MASK;

    while(1){
        isCardPresent = pHsd->sd_hc->PSTATE;
        isCardPresent = (isCardPresent & SD_CARD_INSRT_MASK)>>SD_CARD_INSRT_POS;

        if(isCardPresent){
            pHsd->SdCard.isCardPresent = isCardPresent;
#ifdef PRINTF_DEBUG
            printf("SD Card Detected...\n");
#endif
            break;
        }

        if(!timeout--){
            pHsd->ErrorCode = SD_TIMEOUT_ERR;
            status = HC_ERR;
            goto exit;
        }

        PMU_delay_loop_us(1);
    }

    status = HC_OK;

exit:
    return status;
}

/**
  \fn           hc_status_t HC_GetCardIFCond(SD_HandleTypeDef *pHsd)
  \brief        Get Card Interface condition
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
hc_status_t HC_GetCardIFCond(SD_HandleTypeDef *pHsd){

    uint32_t resp_ifcond;

    /* Change the Card State from Idle to Identification */
    pHsd->State = SD_IDENT;

    pHsd->sd_cmd.cmdidx       = CMD8;
    pHsd->sd_cmd.arg          = SD_CMD8_VOL_PATTERN;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        return HC_ERR;
    }

    resp_ifcond = pHsd->sd_hc->RESP01;

    if(resp_ifcond != SD_CMD8_VOL_PATTERN){
        /* SD Version 1 Card */
        pHsd->SdCard.CardVersion = SD_CARD_SDSC;
        pHsd->SdCard.CardType = SD_CARD_SDSC;
    }else{
        /* SD Version 2 or Later Card */
        pHsd->SdCard.CardVersion = SD_CARD_SDHC;
        pHsd->SdCard.CardType = SD_CARD_SDHC;
    }

    return HC_OK;
}

/**
  \fn           hc_status_t HC_GetCardOPCond(SD_HandleTypeDef *pHsd)
  \brief        Get Card operating voltage condition
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
hc_status_t HC_GetCardOPCond(SD_HandleTypeDef *pHsd){

    uint32_t resp_OPcond;
    uint32_t timeout = 0xFFFFU;

    /* Get the card operating condition */
    do{
        /* just to indicate SD Card that the next cmd is APP CMD */
        pHsd->sd_cmd.cmdidx       = CMD55;
        pHsd->sd_cmd.arg          = 0x0;
        if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
            SDErrorHandler();
        }

        pHsd->sd_cmd.cmdidx       = CMD41;
        pHsd->sd_cmd.arg          = SD_CMD41_HCS | SD_CMD41_3V3;
        if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
            SDErrorHandler();
        }

        resp_OPcond = pHsd->sd_hc->RESP01;
        resp_OPcond = resp_OPcond & SD_OCR_READY;
    }while( (!resp_OPcond) && (timeout--));

    if(timeout == MAX_TIMEOUT_32)
        return HC_ERR;

    return HC_OK;
}

/**
  \fn           hc_status_t HC_GetCardCID(SD_HandleTypeDef *pHsd)
  \brief        Get Card Identification information
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
hc_status_t HC_GetCardCID(SD_HandleTypeDef *pHsd){

    pHsd->sd_cmd.cmdidx       = CMD2;
    pHsd->sd_cmd.arg          = 0x0;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        SDErrorHandler();
    }

    pHsd->CID[0] = pHsd->sd_hc->RESP01;
    pHsd->CID[1] = pHsd->sd_hc->RESP23;
    pHsd->CID[2] = pHsd->sd_hc->RESP45;
    pHsd->CID[3] = pHsd->sd_hc->RESP67;

    /* Get the card Relative Addr */
    pHsd->sd_cmd.cmdidx       = CMD3;
    pHsd->sd_cmd.arg          = 0x0;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        SDErrorHandler();
    }

    pHsd->SdCard.RelCardAdd = pHsd->sd_hc->RESP01 & SD_RCA_MASK;

    return HC_OK;
}

/**
  \fn           hc_status_t HC_GetCardCSD(SD_HandleTypeDef *pHsd)
  \brief        Get Card Specific Data
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
hc_status_t HC_GetCardCSD(SD_HandleTypeDef *pHsd){

    uint32_t CSD[4];
    uint32_t BlkLen;
    uint32_t DeviceSize;
    uint32_t Mult;

    pHsd->sd_cmd.cmdidx       = CMD9;
    pHsd->sd_cmd.arg          = pHsd->SdCard.RelCardAdd;
    if(HC_SendCMD(pHsd, &pHsd->sd_cmd) != HC_OK){
        SDErrorHandler();
    }

    CSD[0] = pHsd->sd_hc->RESP01;
    CSD[1] = pHsd->sd_hc->RESP23;
    CSD[2] = pHsd->sd_hc->RESP45;
    CSD[3] = pHsd->sd_hc->RESP67;

    /* update the global instance */
    pHsd->CSD[0] = CSD[0];
    pHsd->CSD[1] = CSD[1];
    pHsd->CSD[2] = CSD[2];
    pHsd->CSD[3] = CSD[3];

    if(((CSD[3] & CSD_STRUCT_MASK) >> 22U) == 0U){
        BlkLen = (uint32_t)1U << ((uint32_t)(CSD[2] & READ_BLK_LEN_MASK) >> 8U);
        Mult = (uint32_t)1U << ((uint32_t)((CSD[1] & C_SIZE_MULT_MASK) >> 7U) + (uint32_t)2U);
        DeviceSize = (CSD[1] & C_SIZE_LOWER_MASK) >> 22U;
        DeviceSize |= (CSD[2] & C_SIZE_UPPER_MASK) << 10U;
        DeviceSize = (DeviceSize + 1U) * Mult;
        DeviceSize =  DeviceSize * BlkLen;
        pHsd->SdCard.SectorCount = (DeviceSize/SD_BLK_SIZE_512_MASK);
    }else if(((CSD[3] & CSD_STRUCT_MASK) >> 22U) == 1U){
        pHsd->SdCard.SectorCount = (((CSD[1] & CSD_V2_C_SIZE_MASK) >> 8U) +
                1U) * 1024U;
        pHsd->SdCard.Class = (uint16_t)((CSD[2] & CSD_CCC_MASK) >> CSD_CCC_SHIFT);
        pHsd->SdCard.SectorSize = 512;
        pHsd->SdCard.LogBlockSize = 512;
    }else{
        return HC_ERR;
    }

    return HC_OK;
}
