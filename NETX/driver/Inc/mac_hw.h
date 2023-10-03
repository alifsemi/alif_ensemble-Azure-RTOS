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
 * @file     mac_hw.h
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     2-July-2021
 * @brief    Header file for the NetXDuo Ethernet Driver.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef MAC_HW_H
#define MAC_HW_H

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#ifdef  __cplusplus
extern "C"
{
#endif

#define RX_DESC_COUNT                   12    /**< Rx DMA descriptor count */
#define TX_DESC_COUNT                   12    /**< Tx DMA descriptor count */

/** \brief Rx/Tx Dma Descriptor. */
typedef struct {
    uint32_t des0;
    uint32_t des1;
    uint32_t des2;
    uint32_t des3;
} DMA_DESC;

/** \brief Ethernet Configuration. */
typedef struct {
    uint8_t auto_neg_ctrl;      /**< Enable/Disable autonegotiation. Can be set to AN_DISABLE or AN_ENABLE */
    uint8_t speed;              /**< Speed configuration. Legal values are ETH_SPEED_10M and ETH_SPEED_100M */
    uint8_t duplex;             /**< Duplex configuration. Legal values are ETH_DUPLEX_HALF and ETH_DUPLEX_FULL */
} eth_config;

#define AN_DISABLE                      0x0 /**< Disable autonegotiation. */
#define AN_ENABLE                       0x1 /**< Enable autonegotiation. */

#define ETH_SPEED_10M		            0x0 /**< 10Mbps Speed. */
#define ETH_SPEED_100M		            0x1 /**< 100mbps Speed. */

#define ETH_DUPLEX_HALF		            0x0 /**< Half Duplex. */
#define ETH_DUPLEX_FULL		            0x1 /**< Full Duplex. */

/** \brief MAC device */
typedef struct {
    uint32_t base_addr;             /**< IOMEM base address of the Ethernet MAC instance. */
    IRQn_Type irq;                  /**< IRQ number of the Ethernet MAC instance */
    DMA_DESC *rx_descs;             /**< Array of Rx DMA descriptors. */
    DMA_DESC *tx_descs;             /**< Array of Tx DMA descriptors. */
    NX_PACKET **rx_nx_packets;      /**< Array of pointers to rx NX_PACKETs. */
    NX_PACKET **tx_nx_packets;      /**< Array of pointers to tx NX_PACKETs. */
    uint32_t cur_rx_desc_id;        /**< Index of the current Rx DMA descriptor. */
    uint32_t cur_tx_desc_id;        /**< Index of the current Tx DMA descriptor. */
    uint32_t tx_release_desc_id;    /**< Index of the current Tx DMA descriptor to be released */
    uint32_t tx_descs_in_use;       /**< Number of tx descriptors in use by the hardware */
    eth_config config;              /**< Configuration information (Speed/Duplex/Autonegotiation). */
    uint32_t rx_buf_size;           /**< DMA rx buf size to be used */
} MAC_DEV;

/**
  \brief Utility function to Write an iomem register in the MAC.

  \param[in] dev: Pointer to the MAC device structure.
  \param[in] offset: offset from the base address of the MAC instance.
  \param[in] val: value to be written into the register.
 */
static inline void mac_write_reg(MAC_DEV *dev, uint32_t offset, uint32_t val)
{
    *((volatile uint32_t *) (dev->base_addr + offset)) = val;
}

/**
  \brief Utility function to read an iomem register in the MAC.

  \param[in] dev: Pointer to the MAC device structure.
  \param[in] offset: offset from the base address of the MAC instance.
  \return  register value.
 */

static inline uint32_t mac_read_reg(MAC_DEV *dev, uint32_t offset)
{
    return (*((volatile uint32_t *) (dev -> base_addr + offset)));
}


/* common helpers */
#define BIT(nr)				        (1UL << (nr))
#define GENMASK(h, l) \
                                        (((~(0UL)) - ((1UL) << (l)) + 1) & \
                                                    (~(0UL) >> (32 - 1 - (h))))

/*  MAC registers and fields */
#define MAC_CONFIG			                    0x00000000
#define MAC_EXT_CONFIG		                    0x00000004
#define MAC_PACKET_FILTER		                0x00000008
#define MAC_HASH_TAB(x)		                    (0x10 + (x) * 4)
#define MAC_VLAN_TAG		                    0x00000050
#define MAC_VLAN_TAG_DATA		                0x00000054
#define MAC_VLAN_HASH_TABLE	                    0x00000058
#define MAC_RX_FLOW_CTRL		                0x00000090
#define MAC_VLAN_INCL		                    0x00000060
#define MAC_QX_TX_FLOW_CTRL(x)	                (0x70 + x * 4)
#define MAC_TXQ_PRTY_MAP0		                0x98
#define MAC_TXQ_PRTY_MAP1		                0x9C
#define MAC_RXQ_CTRL0		                    0x000000a0
#define MAC_RXQ_CTRL1		                    0x000000a4
#define MAC_RXQ_CTRL2		                    0x000000a8
#define MAC_RXQ_CTRL3		                    0x000000ac
#define MAC_INT_STATUS		                    0x000000b0
#define MAC_INT_EN			                    0x000000b4
#define MAC_PMT_CTRL_STS		                0x000000c0
#define MAC_1US_TIC_COUNTER	                    0x000000dc
#define MAC_PCS_BASE		                    0x000000e0
#define MAC_PHYIF_CONTROL_STATUS	            0x000000f8
#define MAC_PMT			                        0x000000c0
#define MAC_DEBUG			                    0x00000114
#define MAC_HW_FEATURE0		                    0x0000011c
#define MAC_HW_FEATURE1		                    0x00000120
#define MAC_HW_FEATURE2		                    0x00000124
#define MAC_HW_FEATURE3		                    0x00000128
#define MAC_MDIO_ADDR		                    0x00000200
#define MAC_MDIO_DATA		                    0x00000204
#define MAC_ARP_ADDR		                    0x00000210

#define MAC_ADDR_HIGH(reg)		                (0x300 + (reg) * 8)
#define MAC_ADDR_LOW(reg)		                (0x304 + (reg) * 8)
#define MAC_L3L4_CTRL(reg)		                (0x900 + (reg) * 0x30)
#define MAC_L4_ADDR(reg)		                (0x904 + (reg) * 0x30)
#define MAC_L3_ADDR0(reg)		                (0x910 + (reg) * 0x30)
#define MAC_L3_ADDR1(reg)		                (0x914 + (reg) * 0x30)

#define MAC_TIMESTAMP_CONTROL	                0x00000b00
#define MAC_SYS_TIME_SEC		                0x00000b08
#define MAC_SYS_TIME_NANOSEC	                0x00000b0c
#define MAC_SYS_TIME_SEC_UPD	                0x00000b10
#define MAC_SYS_TIME_NANOSEC_UPD	            0x0000b14
#define MAC_TIMESTAMP_ADDEND	                0x00000b18
#define MAC_PPS_TARGET_SEC		                0x00000b80
#define MAC_PPS_TARGET_NANOSEC	                0x00000b84

/* RX Queues Routing */
#define MAC_RXQCTRL_AVCPQ_MASK	                GENMASK(2, 0)
#define MAC_RXQCTRL_AVCPQ_SHIFT	                0
#define MAC_RXQCTRL_PTPQ_MASK	                GENMASK(6, 4)
#define MAC_RXQCTRL_PTPQ_SHIFT	                4
#define MAC_RXQCTRL_DCBCPQ_MASK	                GENMASK(10, 8)
#define MAC_RXQCTRL_DCBCPQ_SHIFT	            8
#define MAC_RXQCTRL_UPQ_MASK	                GENMASK(14, 12)
#define MAC_RXQCTRL_UPQ_SHIFT	                12
#define MAC_RXQCTRL_MCBCQ_MASK	                GENMASK(18, 16)
#define MAC_RXQCTRL_MCBCQ_SHIFT	                16
#define MAC_RXQCTRL_MCBCQEN	                    BIT(20)
#define MAC_RXQCTRL_MCBCQEN_SHIFT	            20
#define MAC_RXQCTRL_TACPQE		                BIT(21)
#define MAC_RXQCTRL_TACPQE_SHIFT	            21
#define MAC_RXQCTRL_FPRQ		                GENMASK(26, 24)
#define MAC_RXQCTRL_FPRQ_SHIFT	                24

#define MAC_RXQ_CTRL0_RXQ0EN_SHIFT              0
#define MAC_RXQ_CTRL0_RXQ0EN_MASK               3
#define MAC_RXQ_CTRL0_RXQ0EN_NOT_ENABLED        0
#define MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB        2
#define MAC_RXQ_CTRL0_RXQ0EN_ENABLED_AV         1

#define MAC_Q0_TX_FLOW_CTRL_PT_SHIFT            16
#define MAC_Q0_TX_FLOW_CTRL_PT_MASK             0xffff
#define MAC_Q0_TX_FLOW_CTRL_TFE                 BIT(1)

#define DMA_CH0_TX_CONTROL_TXPBL_SHIFT          16
#define DMA_CH0_TX_CONTROL_TXPBL_MASK           0x3f
#define DMA_CH0_TX_CONTROL_OSP                  BIT(4)
#define DMA_CH0_TX_CONTROL_ST                   BIT(0)

#define DMA_CH0_RX_CONTROL_RXPBL_SHIFT          16
#define DMA_CH0_RX_CONTROL_RXPBL_MASK           0x3f
#define DMA_CH0_RX_CONTROL_RBSZ_SHIFT           1
#define DMA_CH0_RX_CONTROL_RBSZ_MASK            0x3fff
#define DMA_CH0_RX_CONTROL_SR                   BIT(0)
#define DMA_CH0_RX_CONTROL_RPF                  BIT(31)

#define DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT        16
#define DMA_SYSBUS_MODE_RD_OSR_LMT_MASK         0xf
#define DMA_SYSBUS_MODE_WR_OSR_LMT_SHIFT        24
#define DMA_SYSBUS_MODE_ONEKBBE                 BIT(13)
#define DMA_SYSBUS_MODE_EAME                    BIT(11)
#define DMA_SYSBUS_MODE_BLEN16                  BIT(3)
#define DMA_SYSBUS_MODE_BLEN8                   BIT(2)
#define DMA_SYSBUS_MODE_BLEN4                   BIT(1)

/* MAC Packet Filtering */
#define MAC_PACKET_FILTER_PR                    BIT(0)
#define MAC_PACKET_FILTER_HMC                   BIT(2)
#define MAC_PACKET_FILTER_PM                    BIT(4)
#define MAC_PACKET_FILTER_DBF                   BIT(5)
#define MAC_PACKET_FILTER_PCF                   BIT(7)
#define MAC_PACKET_FILTER_HPF                   BIT(10)
#define MAC_PACKET_FILTER_VTFE                  BIT(16)
#define MAC_PACKET_FILTER_IPFE                  BIT(20)
#define MAC_PACKET_FILTER_RA                    BIT(31)

#define MAC_MAX_PERFECT_ADDRESSES               128

/* MAC VLAN */
#define MAC_VLAN_EDVLP                          BIT(26)
#define MAC_VLAN_VTHM                           BIT(25)
#define MAC_VLAN_DOVLTC                         BIT(20)
#define MAC_VLAN_ESVL                           BIT(18)
#define MAC_VLAN_ETV                            BIT(16)
#define MAC_VLAN_VID                            GENMASK(15, 0)
#define MAC_VLAN_VLTI                           BIT(20)
#define MAC_VLAN_CSVL                           BIT(19)
#define MAC_VLAN_VLC                            GENMASK(17, 16)
#define MAC_VLAN_VLC_SHIFT                      16

/* MAC VLAN Tag */
#define MAC_VLAN_TAG_VID                        GENMASK(15, 0)
#define MAC_VLAN_TAG_ETV                        BIT(16)

/* MAC VLAN Tag Control */
#define MAC_VLAN_TAG_CTRL_OB                    BIT(0)
#define MAC_VLAN_TAG_CTRL_CT                    BIT(1)
#define MAC_VLAN_TAG_CTRL_OFS_MASK              GENMASK(6, 2)
#define MAC_VLAN_TAG_CTRL_OFS_SHIFT             2
#define MAC_VLAN_TAG_CTRL_EVLS_MASK             GENMASK(22, 21)
#define MAC_VLAN_TAG_CTRL_EVLS_SHIFT            21
#define MAC_VLAN_TAG_CTRL_EVLRXS                BIT(24)

#define MAC_VLAN_TAG_STRIP_NONE                 (0x0 << MAC_VLAN_TAG_CTRL_EVLS_SHIFT)
#define MAC_VLAN_TAG_STRIP_PASS                 (0x1 << MAC_VLAN_TAG_CTRL_EVLS_SHIFT)
#define MAC_VLAN_TAG_STRIP_FAIL                 (0x2 << MAC_VLAN_TAG_CTRL_EVLS_SHIFT)
#define MAC_VLAN_TAG_STRIP_ALL                  (0x3 << MAC_VLAN_TAG_CTRL_EVLS_SHIFT)

/* MAC VLAN Tag Data/Filter */
#define MAC_VLAN_TAG_DATA_VID                   GENMASK(15, 0)
#define MAC_VLAN_TAG_DATA_VEN                   BIT(16)
#define MAC_VLAN_TAG_DATA_ETV                   BIT(17)

/* MAC PMT Control Status */
#define MAC_PMT_CTRL_STS_MGKPKTEN	            BIT(1)
#define MAC_PMT_CTRL_STS_PWRDWN	                BIT(0)

/* MAC Flow Control RX */
#define MAC_RX_FLOW_CTRL_RFE                    BIT(0)

/* MAC Flow Control TX */
#define MAC_TX_FLOW_CTRL_TFE                    BIT(1)
#define MAC_TX_FLOW_CTRL_PT_SHIFT               16

/* MAC config */
#define MAC_CONFIG_ARPEN                        BIT(31)
#define MAC_CONFIG_SARC                         GENMASK(30, 28)
#define MAC_CONFIG_SARC_SHIFT                   28
#define MAC_CONFIG_IPC                          BIT(27)
#define MAC_CONFIG_IPG                          GENMASK(26, 24)
#define MAC_CONFIG_IPG_SHIFT                    24
#define MAC_CONFIG_2K                           BIT(22)
#define MAC_CONFIG_CST                          BIT(21)
#define MAC_CONFIG_ACS                          BIT(20)
#define MAC_CONFIG_WD                           BIT(19)
#define MAC_CONFIG_BE                           BIT(18)
#define MAC_CONFIG_JD                           BIT(17)
#define MAC_CONFIG_JE                           BIT(16)
#define MAC_CONFIG_PS                           BIT(15)
#define MAC_CONFIG_FES                          BIT(14)
#define MAC_CONFIG_FES_SHIFT                    14
#define MAC_CONFIG_DM                           BIT(13)
#define MAC_CONFIG_LM                           BIT(12)
#define MAC_CONFIG_DCRS                         BIT(9)
#define MAC_CONFIG_TE                           BIT(1)
#define MAC_CONFIG_RE                           BIT(0)

#define MAC_MDIO_ADDR_PA_SHIFT	                21
#define MAC_MDIO_ADDR_RDA_SHIFT	                16
#define MAC_MDIO_ADDR_CR_SHIFT	                8
/* MDC clock divisors for different CSR clk values */
#define MAC_MDIO_ADDR_CR_60_100	                0
#define MAC_MDIO_ADDR_CR_100_150	            1
#define MAC_MDIO_ADDR_CR_20_35	                2
#define MAC_MDIO_ADDR_CR_35_60	                3
#define MAC_MDIO_ADDR_CR_150_250	            4
#define MAC_MDIO_ADDR_CR_250_300	            5
#define MAC_MDIO_ADDR_CR_300_500	            6
#define MAC_MDIO_ADDR_CR_500_800	            7
#define MAC_MDIO_ADDR_SKAP		                BIT(4)
#define MAC_MDIO_ADDR_GOC_SHIFT	                2
#define MAC_MDIO_ADDR_GOC_READ	                3
#define MAC_MDIO_ADDR_GOC_WRITE	                1
#define MAC_MDIO_ADDR_C45E		                BIT(1)
#define MAC_MDIO_ADDR_GB		                BIT(0)

#define MAC_ADDR_HIGH_AE                        BIT(31)

#define MAC_MDIO_DATA_GD_MASK                   0xffff

#define MAC_INT_EN_TSIE                         BIT(12)
#define MAC_INT_EN_PMTIE                        BIT(4)

/* PTP Timestamp control register defines */
#define PTP_TCR_TSENA                           BIT(0)  /* Timestamp Enable */
#define PTP_TCR_TSCFUPDT                        BIT(1)  /* Timestamp Fine/Coarse Update */
#define PTP_TCR_TSINIT                          BIT(2)  /* Timestamp Initialize */
#define PTP_TCR_TSUPDT                          BIT(3)  /* Timestamp Update */
#define PTP_TCR_TSTRIG                          BIT(4)  /* Timestamp Interrupt Trigger Enable */
#define PTP_TCR_TSADDREG                        BIT(5)  /* Addend Reg Update */
#define PTP_TCR_TSENALL                         BIT(8)  /* Enable Timestamp for All Frames */
#define PTP_TCR_TSCTRLSSR                       BIT(9)  /* Digital or Binary Rollover Control */
#define PTP_TCR_TSVER2ENA                       BIT(10)
#define PTP_TCR_TSIPENA                         BIT(11)
#define PTP_TCR_TSIPV6ENA                       BIT(12)
#define PTP_TCR_TSIPV4ENA                       BIT(13)
#define PTP_TCR_TSEVNTENA                       BIT(14)
#define PTP_TCR_TSMSTRENA                       BIT(15)
#define PTP_TCR_SNAPTYPSEL_1                    BIT(16)
#define PTP_TCR_TSENMACADDR                     BIT(18)

#define MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT        6
#define MAC_HW_FEATURE1_TXFIFOSIZE_MASK         0x1f
#define MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT        0
#define MAC_HW_FEATURE1_RXFIFOSIZE_MASK         0x1f

/*  MTL registers */
#define MTL_OPERATION_MODE                      0x00000c00
#define MTL_FRPE                                BIT(15)
#define MTL_OPERATION_SCHALG_MASK               GENMASK(6, 5)
#define MTL_OPERATION_SCHALG_WRR                (0x0 << 5)
#define MTL_OPERATION_SCHALG_WFQ                (0x1 << 5)
#define MTL_OPERATION_SCHALG_DWRR               (0x2 << 5)
#define MTL_OPERATION_SCHALG_SP                 (0x3 << 5)
#define MTL_OPERATION_RAA                       BIT(2)
#define MTL_OPERATION_RAA_SP                    (0x0 << 2)
#define MTL_OPERATION_RAA_WSP                   (0x1 << 2)

#define MTL_INT_STATUS                          0x00000c20
#define MTL_INT_QX(x)                           BIT(x)

#define MTL_CHAN_BASE_ADDR                      0x00000d00
#define MTL_CHAN_BASE_OFFSET                    0x40
#define MTL_CHANX_BASE_ADDR(x)                  (MTL_CHAN_BASE_ADDR + \
                                                    (x * MTL_CHAN_BASE_OFFSET))

#define MTL_CHAN_TX_OP_MODE(x)                  MTL_CHANX_BASE_ADDR(x)
#define MTL_CHAN_TX_DEBUG(x)                    (MTL_CHANX_BASE_ADDR(x) + 0x8)
#define MTL_CHAN_INT_CTRL(x)                    (MTL_CHANX_BASE_ADDR(x) + 0x2c)
#define MTL_CHAN_RX_OP_MODE(x)                  (MTL_CHANX_BASE_ADDR(x) + 0x30)
#define MTL_CHAN_RX_DEBUG(x)                    (MTL_CHANX_BASE_ADDR(x) + 0x38)

#define MTL_OP_MODE_RSF                         BIT(5)
#define MTL_OP_MODE_FEP			                BIT(4)
#define MTL_OP_MODE_FUP			                BIT(3)
#define MTL_OP_MODE_TXQEN_MASK                  GENMASK(3, 2)
#define MTL_OP_MODE_TXQEN_AV                    BIT(2)
#define MTL_OP_MODE_TXQEN                       BIT(3)
#define MTL_OP_MODE_TSF                         BIT(1)
#define MTL_OP_MODE_TTC_64B				        (1 << 4)

#define MTL_TXQ0_OPERATION_MODE_TQS_SHIFT       16
#define MTL_TXQ0_OPERATION_MODE_TQS_MASK        0x1ff

#define MTL_RXQ0_OPERATION_MODE_RQS_SHIFT       20
#define MTL_RXQ0_OPERATION_MODE_RQS_MASK        0x3ff

/* DMA regs and fields */
#define DMA_BUS_MODE			                0x00001000
#define DMA_SYS_BUS_MODE		                0x00001004
#define DMA_STATUS			                    0x00001008
#define DMA_DEBUG_STATUS_0		                0x0000100c
#define DMA_DEBUG_STATUS_1		                0x00001010
#define DMA_DEBUG_STATUS_2		                0x00001014
#define DMA_TBS_CTRL			                0x00001050

/* DMA Bus Mode bitmap */
#define DMA_BUS_MODE_SFT_RESET		            BIT(0)

/* DMA SYS Bus Mode bitmap */
#define DMA_BUS_MODE_SPH		                BIT(24)
#define DMA_BUS_MODE_PBL		                BIT(16)
#define DMA_BUS_MODE_PBL_SHIFT		            16
#define DMA_BUS_MODE_RPBL_SHIFT                 16
#define DMA_BUS_MODE_MB			                BIT(14)
#define DMA_BUS_MODE_FB			                BIT(0)

/* DMA Interrupt top status */
#define DMA_STATUS_MAC			                BIT(17)
#define DMA_STATUS_MTL			                BIT(16)
#define DMA_STATUS_CHAN7		                BIT(7)
#define DMA_STATUS_CHAN6		                BIT(6)
#define DMA_STATUS_CHAN5		                BIT(5)
#define DMA_STATUS_CHAN4		                BIT(4)
#define DMA_STATUS_CHAN3		                BIT(3)
#define DMA_STATUS_CHAN2		                BIT(2)
#define DMA_STATUS_CHAN1		                BIT(1)
#define DMA_STATUS_CHAN0		                BIT(0)

/* definitions for DMA channels */
#define DMA_CHAN_BASE_ADDR		                0x00001100
#define DMA_CHAN_BASE_OFFSET		            0x80
#define DMA_CHANX_BASE_ADDR(x)                  (DMA_CHAN_BASE_ADDR + \
                                                        (x * DMA_CHAN_BASE_OFFSET))
#define DMA_CHAN_REG_NUMBER		                17

#define DMA_CHAN_CONTROL(x)		                DMA_CHANX_BASE_ADDR(x)
#define DMA_CHAN_TX_CONTROL(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x4)
#define DMA_CHAN_RX_CONTROL(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x8)
#define DMA_CHAN_TX_BASE_ADDR_HI(x)	            (DMA_CHANX_BASE_ADDR(x) + 0x10)
#define DMA_CHAN_TX_BASE_ADDR(x)	            (DMA_CHANX_BASE_ADDR(x) + 0x14)
#define DMA_CHAN_RX_BASE_ADDR_HI(x)	            (DMA_CHANX_BASE_ADDR(x) + 0x18)
#define DMA_CHAN_RX_BASE_ADDR(x)	            (DMA_CHANX_BASE_ADDR(x) + 0x1c)
#define DMA_CHAN_TX_END_ADDR(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x20)
#define DMA_CHAN_RX_END_ADDR(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x28)
#define DMA_CHAN_TX_RING_LEN(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x2c)
#define DMA_CHAN_RX_RING_LEN(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x30)
#define DMA_CHAN_INTR_ENA(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x34)
#define DMA_CHAN_RX_WATCHDOG(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x38)
#define DMA_CHAN_SLOT_CTRL_STATUS(x)	        (DMA_CHANX_BASE_ADDR(x) + 0x3c)
#define DMA_CHAN_CUR_TX_DESC(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x44)
#define DMA_CHAN_CUR_RX_DESC(x)		            (DMA_CHANX_BASE_ADDR(x) + 0x4c)
#define DMA_CHAN_CUR_TX_BUF_ADDR(x)	            (DMA_CHANX_BASE_ADDR(x) + 0x54)
#define DMA_CHAN_CUR_RX_BUF_ADDR(x)	            (DMA_CHANX_BASE_ADDR(x) + 0x5c)
#define DMA_CHAN_STATUS(x)		                (DMA_CHANX_BASE_ADDR(x) + 0x60)

/* DMA Control X */
#define DMA_CONTROL_SPH                         BIT(24)
#define DMA_CONTROL_MSS_MASK                    GENMASK(13, 0)

#define DMA_CHAN_CONTROL_PBLX8		            BIT(16)

/* DMA Tx Channel X Control register defines */
#define DMA_CONTROL_EDSE                        BIT(28)
#define DMA_CONTROL_TSE                         BIT(12)
#define DMA_CONTROL_OSP                         BIT(4)
#define DMA_CONTROL_ST                          BIT(0)

/* DMA Rx Channel X Control register defines */
#define DMA_CONTROL_SR                          BIT(0)
#define DMA_RBSZ_MASK                           GENMASK(14, 1)
#define DMA_RBSZ_SHIFT                          1

/* Interrupt status per channel */
#define DMA_CHAN_STATUS_REB                     GENMASK(21, 19)
#define DMA_CHAN_STATUS_REB_SHIFT               19
#define DMA_CHAN_STATUS_TEB                     GENMASK(18, 16)
#define DMA_CHAN_STATUS_TEB_SHIFT               16
#define DMA_CHAN_STATUS_NIS                     BIT(15)
#define DMA_CHAN_STATUS_AIS                     BIT(14)
#define DMA_CHAN_STATUS_CDE                     BIT(13)
#define DMA_CHAN_STATUS_FBE                     BIT(12)
#define DMA_CHAN_STATUS_ERI                     BIT(11)
#define DMA_CHAN_STATUS_ETI                     BIT(10)
#define DMA_CHAN_STATUS_RWT                     BIT(9)
#define DMA_CHAN_STATUS_RPS                     BIT(8)
#define DMA_CHAN_STATUS_RBU                     BIT(7)
#define DMA_CHAN_STATUS_RI                      BIT(6)
#define DMA_CHAN_STATUS_TBU                     BIT(2)
#define DMA_CHAN_STATUS_TPS             i       BIT(1)
#define DMA_CHAN_STATUS_TI                      BIT(0)

/* Interrupt enable bits per channel */
#define DMA_CHAN_INTR_ENA_NIE                   BIT(15)
#define DMA_CHAN_INTR_ENA_AIE                   BIT(14)
#define DMA_CHAN_INTR_ENA_CDE                   BIT(13)
#define DMA_CHAN_INTR_ENA_FBE                   BIT(12)
#define DMA_CHAN_INTR_ENA_ERE                   BIT(11)
#define DMA_CHAN_INTR_ENA_ETE                   BIT(10)
#define DMA_CHAN_INTR_ENA_RWE                   BIT(9)
#define DMA_CHAN_INTR_ENA_RSE                   BIT(8)
#define DMA_CHAN_INTR_ENA_RBUE                  BIT(7)
#define DMA_CHAN_INTR_ENA_RIE                   BIT(6)
#define DMA_CHAN_INTR_ENA_TBUE                  BIT(2)
#define DMA_CHAN_INTR_ENA_TSE                   BIT(1)
#define DMA_CHAN_INTR_ENA_TIE                   BIT(0)

/* Descriptor definitions */
/* TDES2 (read format) */
#define TDES2_BUFFER1_SIZE_MASK		            GENMASK(13, 0)
#define TDES2_VLAN_TAG_MASK		                GENMASK(15, 14)
#define TDES2_VLAN_TAG_SHIFT		            14
#define TDES2_BUFFER2_SIZE_MASK		            GENMASK(29, 16)
#define TDES2_BUFFER2_SIZE_MASK_SHIFT	        16
#define TDES3_IVTIR_MASK		                GENMASK(19, 18)
#define TDES3_IVTIR_SHIFT		                18
#define TDES3_IVLTV			                    BIT(17)
#define TDES2_TIMESTAMP_ENABLE		            BIT(30)
#define TDES2_IVT_MASK			                GENMASK(31, 16)
#define TDES2_IVT_SHIFT			                16
#define TDES2_INTERRUPT_ON_COMPLETION	        BIT(31)

/* TDES3 (read format) */
#define TDES3_PACKET_SIZE_MASK		            GENMASK(14, 0)
#define TDES3_VLAN_TAG			                GENMASK(15, 0)
#define TDES3_VLTV			                    BIT(16)
#define TDES3_CHECKSUM_INSERTION_MASK	        GENMASK(17, 16)
#define TDES3_CHECKSUM_INSERTION_SHIFT	        16
#define TDES3_TCP_PKT_PAYLOAD_MASK	            GENMASK(17, 0)
#define TDES3_TCP_SEGMENTATION_ENABLE	        BIT(18)
#define TDES3_HDR_LEN_SHIFT		                19
#define TDES3_SLOT_NUMBER_MASK		            GENMASK(22, 19)
#define TDES3_SA_INSERT_CTRL_MASK	            GENMASK(25, 23)
#define TDES3_SA_INSERT_CTRL_SHIFT	            23
#define TDES3_CRC_PAD_CTRL_MASK		            GENMASK(27, 26)

/* Transmit checksum insertion control */
#define TX_CIC_FULL                             3 /* Include IP header and pseudoheader */

/* TDES3 (write back format) */
#define TDES3_IP_HDR_ERROR		                BIT(0)
#define TDES3_DEFERRED			                BIT(1)
#define TDES3_UNDERFLOW_ERROR		            BIT(2)
#define TDES3_EXCESSIVE_DEFERRAL	            BIT(3)
#define TDES3_COLLISION_COUNT_MASK	            GENMASK(7, 4)
#define TDES3_COLLISION_COUNT_SHIFT	            4
#define TDES3_EXCESSIVE_COLLISION	            BIT(8)
#define TDES3_LATE_COLLISION		            BIT(9)
#define TDES3_NO_CARRIER		                BIT(10)
#define TDES3_LOSS_CARRIER		                BIT(11)
#define TDES3_PAYLOAD_ERROR		                BIT(12)
#define TDES3_PACKET_FLUSHED		            BIT(13)
#define TDES3_JABBER_TIMEOUT		            BIT(14)
#define TDES3_ERROR_SUMMARY		                BIT(15)
#define TDES3_TIMESTAMP_STATUS		            BIT(17)
#define TDES3_TIMESTAMP_STATUS_SHIFT	        17

/* TDES3 context */
#define TDES3_CTXT_TCMSSV		                BIT(26)

/* TDES3 Common */
#define TDES3_RS1V                              BIT(26)
#define TDES3_RS1V_SHIFT                        26
#define TDES3_LAST_DESCRIPTOR                   BIT(28)
#define TDES3_LAST_DESCRIPTOR_SHIFT             28
#define TDES3_FIRST_DESCRIPTOR                  BIT(29)
#define TDES3_CONTEXT_TYPE                      BIT(30)
#define TDES3_CONTEXT_TYPE_SHIFT                30

/* TDES4 */
#define TDES4_LTV                               BIT(31)
#define TDES4_LT                                GENMASK(7, 0)

/* TDS3 use for both format (read and write back) */
#define TDES3_OWN                               BIT(31)
#define TDES3_OWN_SHIFT                         31

/* RDES0 (write back format) */
#define RDES0_VLAN_TAG_MASK                     GENMASK(15, 0)

/* RDES1 (write back format) */
#define RDES1_IP_PAYLOAD_TYPE_MASK              GENMASK(2, 0)
#define RDES1_IP_HDR_ERROR                      BIT(3)
#define RDES1_IPV4_HEADER                       BIT(4)
#define RDES1_IPV6_HEADER                       BIT(5)
#define RDES1_IP_CSUM_BYPASSED                  BIT(6)
#define RDES1_IP_CSUM_ERROR                     BIT(7)
#define RDES1_PTP_MSG_TYPE_MASK                 GENMASK(11, 8)
#define RDES1_PTP_PACKET_TYPE                   BIT(12)
#define RDES1_PTP_VER                           BIT(13)
#define RDES1_TIMESTAMP_AVAILABLE               BIT(14)
#define RDES1_TIMESTAMP_AVAILABLE_SHIFT         14
#define RDES1_TIMESTAMP_DROPPED                 BIT(15)
#define RDES1_IP_TYPE1_CSUM_MASK                GENMASK(31, 16)

/* RDES2 (write back format) */
#define RDES2_L3_L4_HEADER_SIZE_MASK            GENMASK(9, 0)
#define RDES2_VLAN_FILTER_STATUS                BIT(15)
#define RDES2_SA_FILTER_FAIL                    BIT(16)
#define RDES2_DA_FILTER_FAIL                    BIT(17)
#define RDES2_HASH_FILTER_STATUS                BIT(18)
#define RDES2_MAC_ADDR_MATCH_MASK               GENMASK(26, 19)
#define RDES2_HASH_VALUE_MATCH_MASK             GENMASK(26, 19)
#define RDES2_L3_FILTER_MATCH                   BIT(27)
#define RDES2_L4_FILTER_MATCH                   BIT(28)
#define RDES2_L3_L4_FILT_NB_MATCH_MASK          GENMASK(27, 26)
#define RDES2_L3_L4_FILT_NB_MATCH_SHIFT         26
#define RDES2_HL                                GENMASK(9, 0)

/* RDES3 (write back format) */
#define RDES3_PACKET_SIZE_MASK                  GENMASK(14, 0)
#define RDES3_ERROR_SUMMARY                     BIT(15)
#define RDES3_PACKET_LEN_TYPE_MASK              GENMASK(18, 16)
#define RDES3_DRIBBLE_ERROR                     BIT(19)
#define RDES3_RECEIVE_ERROR                     BIT(20)
#define RDES3_OVERFLOW_ERROR                    BIT(21)
#define RDES3_RECEIVE_WATCHDOG                  BIT(22)
#define RDES3_GIANT_PACKET                      BIT(23)
#define RDES3_CRC_ERROR                         BIT(24)
#define RDES3_RDES0_VALID                       BIT(25)
#define RDES3_RDES1_VALID                       BIT(26)
#define RDES3_RDES2_VALID                       BIT(27)
#define RDES3_LAST_DESCRIPTOR                   BIT(28)
#define RDES3_FIRST_DESCRIPTOR                  BIT(29)
#define RDES3_CONTEXT_DESCRIPTOR                BIT(30)
#define RDES3_CONTEXT_DESCRIPTOR_SHIFT          30

/* RDES3 (read format) */
#define RDES3_BUFFER1_VALID_ADDR                BIT(24)
#define RDES3_BUFFER2_VALID_ADDR                BIT(25)
#define RDES3_INT_ON_COMPLETION_EN              BIT(30)
#define RDES3_OWN                               BIT(31)

#ifdef  __cplusplus
}
#endif

#endif /* MAC_HW_H */
