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
 * @file     sd_core.h
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    SD Host Controller Register mapping.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef _SD_CORE_H_
#define _SD_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "stdint.h"

/**
 * @brief SD host Interface register mapping
 */
typedef struct
{
    __IO uint32_t SDMASA;               /* SDMASA_R,    Address offset: 0x00        */
    __IO uint16_t BLOCK_SIZE;           /* BLOCK_SIZE_R Address offset: 0x04        */
    __IO uint16_t BLOCKCOUNT;           /* BLOCKCOUNT_R Address offset: 0x06        */
    __IO uint32_t ARG;                  /* SDIO argument register,         Address offset: 0x08 */
    __IO uint16_t XFER_MODE;            /* XFER_MODE_R  */
    __IO uint16_t CMD;                  /* SDIO command register,          Address offset: 0x0C */
    __IO const uint32_t  RESP01;        /* SDIO response 01 register,       Address offset: 0x10 */
    __IO const uint32_t  RESP23;        /* SDIO response 23 register,       Address offset: 0x14 */
    __IO const uint32_t  RESP45;        /* SDIO response 45 register,       Address offset: 0x18 */
    __IO const uint32_t  RESP67;        /* SDIO response 67 register,       Address offset: 0x1C */
    __IO uint32_t BUF_DATA;
    __IO const uint32_t PSTATE;
    __IO uint8_t HOST_CTRL1;
    __IO uint8_t PWR_CTRL;              /* SDIO power control register,    Address offset: 0x00 */
    __IO uint8_t BGAP_CTRL;
    __IO uint8_t WUP_CTRL;
    __IO uint16_t CLK_CTRL;             /* SDI clock control register,     Address offset: 0x04 */
    __IO uint8_t TOUT_CTRL;
    __IO uint8_t SW_RST;
    __IO uint16_t NORMAL_INT_STAT;
    __IO uint16_t ERROR_INT_STAT;
    __IO uint16_t NORMAL_INT_STAT_EN;
    __IO uint16_t ERROR_INT_STAT_EN;
    __IO uint16_t NORMAL_INT_SIGNAL_EN;
    __IO uint16_t ERROR_INT_SIGNAL_EN;
    __IO uint16_t AUTO_CMD_STAT;
    __IO uint16_t HOST_CTRL2;
    __IO uint32_t CAPABILITIES1;
    __IO uint32_t CAPABILITIES2;
    __IO uint32_t CURR_CAPABILITIES1;
    __IO uint32_t CURR_CAPABILITIES2;
    __IO uint16_t FORCE_AUTO_CMD_STAT;
    __IO uint16_t FORCE_ERROR_INT_STAT;
    __IO uint32_t ADMA_ERR_STAT;
    __IO uint32_t ADMA_SA_LOW;
    __IO uint32_t ADMA_SA_HIGH;
    __IO uint16_t PRESET_INIT;
    __IO uint16_t PRESET_DS;
    __IO uint16_t PRESET_HS;
    __IO uint16_t PRESET_SDR12;
    __IO uint16_t PRESET_SDR25;
    __IO uint16_t PRESET_SDR50;
    __IO uint16_t PRESET_SDR104;
    __IO uint16_t PRESET_DDR50;
}SDIO_TypeDef;

#define SD_TypeDef          SDIO_TypeDef
#define SDIO                ((SDIO_TypeDef *) SDMMC_BASE)
#define HC_VERSION_REG      (SDMMC_BASE + 0xFE)
#define HC_VERSION_REG_MASK 0xFFFF

/**
 * @brief  Host controller driver status enum definition
 */
typedef enum{
    HC_OK,
    HC_ERR,
    HC_SD_INV_STATE
}hc_status_t;

/* SDMMC Device ID Constnat */
#define SD_DEV_ID           1

/* Host Controller Specific Constant */
#define HC_SPEC_V3          0x0002U /**< HC spec version 3 */
#define HC_SPEC_V2          0x0001U /**< HC spec version 2 */
#define HC_SPEC_V1          0x0000U /**< HC spec version 1 */
#define HC_SPEC_VER_MASK    0x00FFU /**< Host Specification version mask */

/* Software Reset Register */
#define SD_SW_RST_ALL_Pos   0
#define SD_SW_RST_ALL_MASK  (0<<SD_SW_RST_ALL_Pos)
#define SD_SW_RST_CMD_Pos   1
#define SD_SW_RST_CMD_MASK  (1<<SD_SW_RST_CMD_Pos)
#define SD_SW_RST_DAT_Pos   2
#define SD_SW_RST_DAT_MASK  (1<<SD_SW_RST_DAT_Pos)

/* CMD and Response register mapping */
#define CMD_IDX_POS         8
#define CMD_TYPE_POS        6
#define CMD_RSP_SEL_POS     0

/* SD Clk freq */
#define SD_CLK_400_KHZ      0x400000U   /*!< 400KHz */
#define SD_CLK_25_MHZ       0x17D7840U  /*!< 25MHz  */
#define SD_CLK_50_MHz       0x2FAF080   /*!< 50MHz  */
#define SD_CLK_100_MHZ      0x5F5E100   /*!< 100MHz */
#define SD_BASE_CLK         SD_CLK_100_MHZ

/* Supported Data Bus */
#define SD_1_BIT_WIDTH      0x1U    /*!< Bus Width 1 */
#define SD_4_BIT_WIDTH      0x2U    /*!< Bus Width 4 */
#define SD_8_BIT_WIDTH      0x20U   /*!< Bus Width 8 */

/* Card Types */
#define SD_CARD_SDSC        1U      /*!< SDSC, Ver 1 Cards  */
#define SD_CARD_SDHC        2U      /*!< SDHC, SDXC Ver 2	*/
#define SD_CARD_MMC		    3U		/*!< MMC Cards          */
#define SD_CARD_SDIO        4U      /*!< SDIO Cards         */
#define SD_CARD_COMBO       5U      /*!< SD Combo Cards     */

/* Command index */
#define APP_CMD_PREFIX      0x80    /* App CMD prefix       */
#define CMD0                0x00U
#define CMD1                0x01U
#define CMD2                0x02U
#define CMD3                0x03U
#define CMD4                0x04U
#define CMD5                0x05U
#define CMD6                0x06U
#define ACMD6               (APP_CMD_PREFIX + 0x06U)
#define CMD7                0x07U
#define CMD8                0x08U
#define CMD9                0x09U
#define CMD10               0x0AU
#define CMD11               0x0BU
#define CMD12               0x0CU
#define CMD13               0x0DU
#define ACMD13              (APP_CMD_PREFIX + 0x0DU)
#define CMD16               0x10U
#define CMD17               0x11U
#define CMD18               0x12U
#define CMD19               0x13U
#define CMD21               0x15U
#define CMD23               0x17U
#define ACMD23              (APP_CMD_PREFIX + 0x17U)
#define CMD24               0x18U
#define CMD25               0x19U
#define CMD32               0x20U
#define CMD33               0x21U
#define CMD35               0x23U
#define CMD36               0x24U
#define CMD38               0x26U
#define CMD41               0x29U
#define ACMD41              (APP_CMD_PREFIX + 0x29U)
#define ACMD42              (APP_CMD_PREFIX + 0x2AU)
#define ACMD51              (APP_CMD_PREFIX + 0x33U)
#define CMD52               0x34U
#define CMD53               0x35U
#define CMD55               0x37U
#define CMD58               0x3AU

/* Response type */
#define RESP_NONE           0   /*!< No response expected     */
#define RESP_R136           1   /*!< 128Bit response expected */
#define RESP_R48            2   /*!< Single response expected */
#define RESP_R48B           3   /*!< check busy after resp    */

/* Present state */
#define SD_CARD_INSRT_MASK  0x00010000U
#define SD_CARD_INSRT_POS   0x10
#define RD_XFER_ACTIVE_MASK 0x200
#define RD_XFER_ACTIVE_POS  0x9
#define WR_XFER_ACTIVE_POS  0x8
#define DMA_IRQ_POS         0x3
#define DMA_IRQ_MASK        0x8
#define NORMAL_INT_STAT_XFER_COMPLETE_MASK 0x2
#define NORMAL_INT_STAT_XFER_COMPLETE_POS 1

/* Card CSD */
#define CSD_SPEC_VER_MASK       0x3C0000U
#define READ_BLK_LEN_MASK       0x00000F00U
#define C_SIZE_MULT_MASK        0x00000380U
#define C_SIZE_LOWER_MASK       0xFFC00000U
#define C_SIZE_UPPER_MASK       0x00000003U
#define CSD_STRUCT_MASK         0x00C00000U
#define CSD_V2_C_SIZE_MASK      0x3FFFFF00U
#define CSD_CCC_MASK            0xFFF00000U
#define CSD_CCC_SHIFT           20U
#define CSD_CCC_CLASS5_MASK     0x20U

/* Blk Size */
#define SD_BLK_SIZE_512_MASK    0x0200U     /*!< Blk Size 512             */
#define NORM_INTR_ALL_MASK      0x0000FFFFU /*!< Mask for normal irq bits */
#define ERROR_INTR_ALL_MASK     0x0000F3FFU /*!< Mask for error irq bits  */

/* Power Control */
#define SD_PC_BUS_PWR_VDD1_MASK 0x00000001U /**< Bus Power Control  */
#define SD_PC_BUS_VSEL_MASK     0x0000000EU /**< Bus Voltage Select */
#define SD_PC_BUS_VSEL_3V3_MASK 0x0000000EU /**< Bus Voltage 3.3V   */
#define SD_PC_BUS_VSEL_3V0_MASK 0x0000000CU /**< Bus Voltage 3.0V   */
#define SD_PC_BUS_VSEL_1V8_MASK 0x0000000AU /**< Bus Voltage 1.8V   */
#define SD_PC_EMMC_HW_RST_MASK  0x00000010U /**< HW reset for eMMC  */

/* Clock Control */
#define SD_INTERNAL_CLK_EN_MASK     0x1
#define SD_INTERNAL_CLK_STABLE_MASK 0x2
#define SD_CLK_EN_MASK              0x4
#define SD_PLL_EN_MASK              0x8
#define SD_CLK_GEN_SEL_Pos          0x5
#define SD_DIV_CLK_MODE             0x0
#define SD_PROG_CLK_MODE            0x1
#define SD_CLK_GEN_SEL_MASK         (SD_DIV_CLK_MODE << SD_CLK_GEN_SEL_Pos)
#define SD_UPPER_FREQ_SEL_Pos       6
#define SD_UPPER_FREQ_SEL           0x1 << SD_UPPER_FREQ_SEL_Pos
#define SD_FREQ_SEL_Pos             8
#define SD_CLK_DIVSOR               0x2
#define SD_FREQ_SEL                 SD_CLK_DIVSOR << SD_FREQ_SEL_Pos

/* Host Capabilities */
#define HOST_SD_CAP_VOLT_3V3_MASK     0x01000000U /*!< 3.3V support */
#define HOST_SD_CAP_VOLT_3V0_MASK     0x02000000U /*!< 3.0V support */
#define HOST_SD_CAP_VOLT_1V8_MASK     0x04000000U /*!< 1.8V support */

/* Xfer Mode Control */
#define XFER_MODE_DMA_EN_Pos            0
#define XFER_MODE_DMA_EN_MASK           1<<XFER_MODE_DMA_EN_Pos
#define XFER_MODE_BLK_CNT_EN_Pos        1
#define XFER_MODE_BLK_CNT_MASK          1<<XFER_MODE_BLK_CNT_EN_Pos
#define XFER_MODE_AUTO_CMD_EN_Pos       2
#define XFER_MODE_AUTO_CMD_DISABLE      0
#define XFER_MODE_AUTO_CMD12            1
#define XFER_MODE_AUTO_CMD23            2
#define XFER_MODE_AUTO_CMD_AUTO_SEL     3
#define XFER_MODE_AUTO_CMD_EN_MASK      AUTO_CMD12_EN<<XFER_MODE_AUTO_CMD_EN_Pos
#define XFER_MODE_DATA_XFER_DIR_Pos     4
#define XFER_MODE_DATA_XFER_RD_MASK     1<<XFER_MODE_DATA_XFER_DIR_Pos
#define XFER_MODE_DATA_XFER_WR_MASK     0<<XFER_MODE_DATA_XFER_DIR_Pos
#define XFER_MODE_MULTI_BLK_SEL_Pos     5
#define XFER_MODE_MULTI_BLK_SEL_MASK    1<<XFER_MODE_MULTI_BLK_SEL_Pos

/* DMA */
#define SD_HC_DMA_ADMA2_32_MASK  0x00000010U /**< ADMA2 Mode - 32 bit */

/* Normal IRQs Mask */
#define SD_INTR_CC_MASK             0x00000001U /*!< Command Complete         */
#define SD_INTR_TC_MASK             0x00000002U /*!< Transfer Complete        */
#define SD_INTR_BGE_MASK            0x00000004U /*!< Block Gap Event          */
#define SD_INTR_DMA_MASK            0x00000008U /*!< DMA Interrupt            */
#define SD_INTR_BWR_MASK            0x00000010U /*!< Buffer Write Ready       */
#define SD_INTR_BRR_MASK            0x00000020U /*!< Buffer Read Ready        */
#define SD_INTR_CARD_INSRT_MASK     0x00000040U /*!< Card Insert              */
#define SD_INTR_CARD_REM_MASK       0x00000080U /*!< Card Remove              */
#define SD_INTR_CARD_MASK           0x00000100U /*!< Card Interrupt           */
#define SD_INTR_INT_A_MASK          0x00000200U /*!< INT A Interrupt          */
#define SD_INTR_INT_B_MASK          0x00000400U /*!< INT B Interrupt          */
#define SD_INTR_INT_C_MASK          0x00000800U /*!< INT C Interrupt          */
#define SD_INTR_RE_TUNING_MASK      0x00001000U /*!< Re-Tuning Interrupt      */
#define SD_INTR_BOOT_ACK_RECV_MASK  0x00002000U /*!< Boot Ack Recv Irq        */
#define SD_INTR_BOOT_TERM_MASK      0x00004000U /*!< Boot Terminate Interrupt */
#define SD_INTR_ERR_MASK            0x00008000U /*!< Error Interrupt          */
#define SD_NORM_INTR_ALL_MASK       0x0000FFFFU

/* Error IRQs Mask */
#define SD_INTR_ERR_CT_MASK         0x00000001U /*!< Command Timeout Error    */
#define SD_INTR_ERR_CCRC_MASK       0x00000002U /*!< Command CRC Error        */
#define SD_INTR_ERR_CEB_MASK        0x00000004U /*!< Command End Bit Error    */
#define SD_INTR_ERR_CI_MASK         0x00000008U /*!< Command Index Error      */
#define SD_INTR_ERR_DT_MASK         0x00000010U /*!< Data Timeout Error       */
#define SD_INTR_ERR_DCRC_MASK       0x00000020U /*!< Data CRC Error           */
#define SD_INTR_ERR_DEB_MASK        0x00000040U /*!< Data End Bit Error       */
#define SD_INTR_ERR_CUR_LMT_MASK    0x00000080U /*!< Current Limit Error      */
#define SD_INTR_ERR_AUTO_CMD12_MASK 0x00000100U /*!< Auto CMD12 Error         */
#define SD_INTR_ERR_ADMA_MASK       0x00000200U /*!< ADMA Error               */
#define SD_INTR_ERR_TR_MASK         0x00001000U /*!< Tuning Error             */
#define SD_INTR_VEND_SPF_ERR_MASK   0x0000E000U /*!< Vendor Specific Error    */
#define SD_ERROR_INTR_ALL_MASK      0x0000F3FFU /*!< Mask for error bits      */

/* SD Status */
#define SD_STATUS_MASK              0x00001E00
#define SD_STATUS_POS               9

/* Host Control 1 */
#define SD_LED_ON                   0x1U
#define SD_4_BIT_WIDTH              0x2U /*!< Host control 1 4bit mode */
#define SD_HIGH_SPEED_MODE_EN       0x4U /*!< Host control 1 High speed enable */
#define SD_SEL_SDMA                 0x0U
#define SD_SEL_ADMA                 0x2U
#define SD_SEL_ADMA64               0x3U
#define SD_DMA_SEL                  SD_SEL_SDMA
#define SD_DMA_SEL_1BIT_MODE        0x0U

/* Card Interface Conditions constants */
#define SD_CMD8_VOL_PATTERN         0x1AA   /*!< CMD8 Voltage Pattern */

/* Card Operating Conditions constants */
#define SD_OCR_READY               0x80000000  /*!< OCR Ready */
#define SD_CMD41_HCS               0x40000000  /*!< ACMD41 High Capacity support */
#define SD_CMD41_3V3               0x00300000  /*!< ACMD41 3.3v support */

/* Time out constant */
#define MAX_TIMEOUT_32              0xFFFFFFFFU
#define MAX_TIMEOUT_16              0xFFFFU

/* SD Relative Card Address Constnat */
#define SD_RCA_Pos                 0x10
#define SD_RCA_MASK                0xFFFF0000U

#ifdef __cplusplus
}

#endif

#endif
