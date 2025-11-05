/**************************************************************************/
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
 * @file     usbd_spec.h
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    Header file for the usbx driver for dwc.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef USBD_SPEC_H
#define USBD_SPEC_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include "sys_utils.h"

#ifdef __cplusplus
/* Yes, C++ compiler is present.  Use standard C.  */
extern "C" {

#endif
//#define DEBUG
/* Define the macros    */

#define USB_SUCCESS                        0
#define USB_INIT_ERROR                     -1
#define USB_CORE_INVALID                   -2
#define USB_MODE_MISMATCH                  -3
#define USB_CORE_SFTRST_TIMEOUT_ERROR      -4
#define USB_CONTROLLER_INIT_FAILED         -5
#define USB_EP_DIRECTION_WRONG             -6
#define USB_EP_BUSY_ERROR                  -7
#define USB_EP_BUFF_LENGTH_INVALID         -8
#define USB_EP_CMD_CMPLT_ERROR             -9
#define USB_EP_CMD_CMPLT_BUS_EXPIRY_ERROR  -10
#define USB_EP_CMD_CMPLT_NO_RESOURCE_ERROR -11
#define USB_EP_CMD_CMPLT_STATUS_UNKNOWN    -12
#define USB_EP_CMD_CMPLT_TIMEOUT_ERROR     -13
#define USB_EP_INVALID                     -14
#define USB_LINKSTATE_INVALID              -15
#define USB_LINKSTATE_RECOVERY_FAILED      -16
#define USB_REMOTE_WAKEUP_FAILED           -17
#define USB_LINKSTATE_TIMEOUT_ERROR        -18
#define USB_LINKSTATE_SET_FAILED           -19
#define USB_MODE_UNSUPPORTED               -20

#define USB_DEVICE_DEFAULT_ADDRESS         0
#define USB_EVNT_BUFF_ALLOC_ERROR          100
#define USB_DGCMD_CMPLT_ERROR              101
#define USB_DGCMD_TIMEOUT_ERROR            102
#define USB_EP_ENABLE_ERROR                103
#define USB_EP_RESOURCE_INDEX_INVALID      105
#define USB_DEVICE_SET_ADDRESS_INVALID     106
#define USB_DEVICE_ALREADY_CONFIGURED      107

#define ATTR                               __attribute__
#define ATTR_ALIGN(CACHELINE)              ATTR((aligned(CACHELINE)))
#define ATTR_SECTION(sec)                  ATTR((section(sec)))
#define IS_ALIGNED(x, a)                   (((x) & ((uint32_t) (a) - 1U)) == 0U)

#define UPPER_32_BITS(n)                   ((uint32_t) (((uint32_t) (n) >> 16) >> 16))
#define LOWER_32_BITS(n)                   ((uint32_t) (n))
#define ROUND_UP(x, align)                                                                         \
    ((((unsigned long) (x) + ((unsigned long) (align) - 1)) / (unsigned long) (align)) *           \
     (unsigned long) (align))
/* USB devices address can be assigned addresses from 1 to 127 */
#define USB_DEVICE_MAX_ADDRESS                    127

#define USB_EP_ENABLED                            (0x00000001U << 0U)
#define USB_EP_STALL                              (0x00000001U << 1U)
#define USB_EP_BUSY                               (0x00000001U << 4U)
#define USB_EP_PENDING_REQUEST                    (0x00000001U << 5U)
#define USB_EP_MISSED_ISOC                        (0x00000001U << 6U)

#define USB_DCTL_START_TIMEOUT                    500
#define USB_GENERIC_CMD_TIMEOUT                   500
#define USB_DEPCMD_TIMEOUT                        1000
#define USB_DCTL_CSFTRST_TIMEOUT                  1000
#define USB_TRANSFER_WAKEUP_RETRY                 20000
#define USB_LINK_STATE_RETRY                      10000

#define USB_BULK_EP_MAX_PKT                       512
#define USB_CONTROL_EP_MAX_PKT                    64
#define USB_ISOC_EP_MAX_PKT                       1024
#define USB_SETUP_PKT_SIZE                        8

#define USB_CONTROL_EP                            0
#define USB_ISOCRONOUS_EP                         1
#define USB_BULK_EP                               2
#define USB_INTERRUPT_EP                          3

#define USB_REQUEST_IN                            0x80U
#define USB_HIGH_SPEED_DEVICE                     2
#define USB_DIR_IN                                1U
#define USB_DIR_OUT                               0U

#define USB_CTRL_PHY_EP0                          0
#define USB_CTRL_PHY_EP1                          1

#define USB_NUM_TRBS                              8U
#define NO_OF_TRB_PER_EP                          8U

#define USB_GET_PHYSICAL_EP(epnum, direction)     (((epnum) << 1U) | (direction))
#define USB_DEPCMD_MSK                            0xF
#define USB_DEPCMD_CMD(x)                         ((x) & USB_DEPCMD_MSK)

#define USB_EVENT_BUFFER_SIZE                     4096
#define USB_EVENT_CNT_SIZE                        4
#define USB_SCRATCHPAD_BUF_SIZE                   4096

/* Define USB endpoint transfer status definition.  */
#define USB_EP_TRANSFER_IDLE                      0
#define USB_EP_TRANSFER_SETUP                     1
#define USB_EP_TRANSFER_DATA_COMPLETION           2
#define USB_EP_TRANSFER_STATUS_COMPLETION         3

#define USB_DEPEVT_XFERCOMPLETE                   0x01
#define USB_DEPEVT_XFERINPROGRESS                 0x02
#define USB_DEPEVT_XFERNOTREADY                   0x03
#define USB_DEPEVT_RXTXFIFOEVT                    0x04
#define USB_DEPEVT_STREAMEVT                      0x06
#define USB_DEPEVT_EPCMDCMPLT                     0x07

/* In response to Start Transfer */
#define USB_DEPEVT_TRANSFER_NO_RESOURCE           1
#define USB_DEPEVT_TRANSFER_BUS_EXPIRY            2
#define USB_DEPEVT_CMD_SUCCESS                    0

/* Control-only Status */
#define USB_DEPEVT_STATUS_CONTROL_DATA            1U
#define USB_DEPEVT_STATUS_CONTROL_STATUS          2U
#define USB_DEPEVT_STATUS_CONTROL_DATA_INVALTRB   9U
#define USB_DEPEVT_STATUS_CONTROL_STATUS_INVALTRB 0xAU

/* Device endpoint specific events */
#define USB_DEPEVT_CMD_CMPLT_MSK                  0x3C0
#define USB_DEPEVT_CMD_CMPLT_POS                  6
#define USB_GET_DEPEVT_TYPE(reg)                                                                   \
    ((reg & USB_DEPEVT_CMD_CMPLT_MSK) >> USB_DEPEVT_CMD_CMPLT_POS)
/* Device endpoint num  */
#define USB_DEPEVT_EP_NUM_MSK                     0x3E
#define USB_DEPEVT_EP_NUM_POS                     1
#define USB_GET_DEPEVT_EP_NUM(reg)                                                                 \
   ((reg & USB_DEPEVT_EP_NUM_MSK) >> USB_DEPEVT_EP_NUM_POS)

/* Device specific events */
#define USB_DEVT_MSK                              0x1F00
#define USB_DEVT_POS                              8
#define USB_DEVT_TYPE(reg)                        ((reg & USB_DEVT_MSK) >> USB_DEVT_POS)

/* device Link state change event info  */
#define USB_DEVT_LINK_STATE_MSK                   0xF0000
#define USB_DEVT_LINK_STATE_POS                   16
#define USB_DEVT_LINK_STATE_INFO(reg)                                                              \
   ((reg & USB_DEVT_LINK_STATE_MSK) >> USB_DEVT_LINK_STATE_POS)
/* Device EP event status   */
#define USB_EVENT_EP_STATUS_POS                      12U
#define USB_EVENT_CMD_PARAM_POS                      16U
#define USB_EVENT_EP_STATUS_MSK                      0xF000
#define USB_EVENT_CMD_PARAM_MASK                     0xFFFF
#define USB_GET_EP_EVENT_STATUS(reg)                                                               \
   ((reg & USB_EVENT_EP_STATUS_MSK) >> USB_EVENT_EP_STATUS_POS)
#define USB_GET_EVENT_CMD_PARAM(reg)                                                               \
   ((reg & USB_EVENT_CMD_PARAM_MASK) >> USB_EVENT_CMD_PARAM_POS)
/* Global HWPARAMS0 Register */
#define USB_GHWPARAMS0_MODE_MSK                   0x3
#define USB_GHWPARAMS0_MODE(n)                    ((n) & USB_GHWPARAMS0_MODE_MSK)
#define USB_GHWPARAMS0_MODE_DEVICE                0
#define USB_GHWPARAMS0_MODE_HOST                  1
#define USB_GHWPARAMS0_MODE_DRD                   2
#define USB_GHWPARAMS0_MBUS_TYPE_MSK              0x38
#define USB_GHWPARAMS0_MBUS_TYPE_POS              3
#define USB_GHWPARAMS0_MBUS_TYPE(n)                                                                \
    (((n) & USB_GHWPARAMS0_MBUS_TYPE_MSK) >> USB_GHWPARAMS0_MBUS_TYPE_POS)
#define USB_GHWPARAMS0_SBUS_TYPE_MSK 0xC0
#define USB_GHWPARAMS0_SBUS_TYPE_POS 0x6
#define USB_GHWPARAMS0_SBUS_TYPE(n)                                                                \
    (((n) & USB_GHWPARAMS0_SBUS_TYPE_MSK) >> USB_GHWPARAMS0_SBUS_TYPE_POS)
#define USB_GHWPARAMS0_MDWIDTH_MSK   0xFF00
#define USB_GHWPARAMS0_MDWIDTH_POS   8
#define USB_GHWPARAMS0_MDWIDTH(n)                                                                  \
   (((n) & USB_GHWPARAMS0_MDWIDTH_MSK) >> USB_GHWPARAMS0_MDWIDTH_POS)
#define USB_GHWPARAMS0_SDWIDTH_MSK   0xFF0000
#define USB_GHWPARAMS0_SDWIDTH_POS   16
#define USB_GHWPARAMS0_SDWIDTH(n)                                                                  \
   (((n) & USB_GHWPARAMS0_SDWIDTH_MSK) >> USB_GHWPARAMS0_SDWIDTH_POS)
#define USB_GHWPARAMS0_AWIDTH_MSK    0xFF000000
#define USB_GHWPARAMS0_AWIDTH_POS    24
#define USB_GHWPARAMS0_AWIDTH(n)                                                                   \
   (((n) & USB_GHWPARAMS0_AWIDTH_MSK) >> USB_GHWPARAMS0_AWIDTH_POS)

/* Global HWPARAMS3 Register */
#define USB_GHWPARAMS3_HSPHY_IFC_MSK 0xC0
#define USB_GHWPARAMS3_HSPHY_IFC_POS 2
#define USB_GHWPARAMS3_HSPHY_IFC(n)                                                                \
    (((n) & USB_GHWPARAMS3_HSPHY_IFC_MSK) >> USB_GHWPARAMS3_HSPHY_IFC_POS)
#define USB_GHWPARAMS3_HSPHY_IFC_DIS       0
#define USB_GHWPARAMS3_HSPHY_IFC_UTMI      1
#define USB_GHWPARAMS3_HSPHY_IFC_ULPI      2
#define USB_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI 3
#define USB_GHWPARAMS3_FSPHY_IFC_MSK       0x30
#define USB_GHWPARAMS3_FSPHY_IFC_POS       4
#define USB_GHWPARAMS3_FSPHY_IFC(n)                                                                \
    (((n) & USB_GHWPARAMS3_FSPHY_IFC_MSK) >> USB_GHWPARAMS3_FSPHY_IFC_POS)
#define USB_GHWPARAMS3_FSPHY_IFC_DIS      0
#define USB_GHWPARAMS3_FSPHY_IFC_ENA      1
#define USB_GHWPARAMS3_NUM_IN_EPS_MSK     0x7C0000
#define USB_GHWPARAMS3_NUM_IN_EPS_POS     18
#define USB_GHWPARAMS3_NUM_EPS_MSK        0x3F000
#define USB_GHWPARAMS3_NUM_EPS_POS        12

#define USB_NUM_EPS(ep_num)                                                                        \
   ((ep_num & USB_GHWPARAMS3_NUM_EPS_MSK) >> USB_GHWPARAMS3_NUM_EPS_POS)
#define USB_IN_EPS(p)                                                                              \
   ((p & USB_GHWPARAMS3_NUM_IN_EPS_MSK) >> USB_GHWPARAMS3_NUM_IN_EPS_POS)

/* Device Generic Command Register */
#define USB_DGCMD_SET_LMP                 0x01
#define USB_DGCMD_SET_PERIODIC_PAR        0x02
#define USB_DGCMD_XMIT_FUNCTION           0x03
#define USB_DGCMD_CMDACT                  BIT(10)
#define USB_DGCMD_STATUS_MSK              0xF000
#define USB_DGCMD_STATUS_POS              12
#define USB_DGCMD_STATUS(n)               (((n) & USB_DGCMD_STATUS_MSK) >> USB_DGCMD_STATUS_POS)
#define USB_DGCMD_CMDIOC                  BIT(8)
#define USB_DGCMD_SET_SCRATCHPAD_ADDR_LO  0x04
#define USB_DGCMD_SET_SCRATCHPAD_ADDR_HI  0x05

/* Device Configuration Register */
#define USB_DCFG_DEVADDR_POS              3
#define USB_DCFG_DEVADDR(addr)            ((addr) << USB_DCFG_DEVADDR_POS)
#define USB_DCFG_SET_ADDR_MSK             0x7F
#define USB_DCFG_DEVADDR_MASK             USB_DCFG_DEVADDR(USB_DCFG_SET_ADDR_MSK)
#define USB_DCFG_SPEED_MASK               7
#define USB_DCFG_HIGHSPEED                0
#define USB_DCFG_LOWSPEED                 1U
#define USB_DCFG_NUMP_POS                 17
#define USB_DCFG_NUMP_MSK                 0x3E0000
#define USB_DCFG_NUMP(n)                  (((n) & USB_DCFG_NUMP_MSK) >> USB_DCFG_NUMP_POS)
#define USB_DCFG_LPM_CAP                  BIT(22)

/* Global USB2 PHY Configuration Register */
#define USB_GUSB2PHYCFG_PHYSOFTRST        BIT(31)
#define USB_GUSB2PHYCFG_U2_FREECLK_EXISTS BIT(30)
#define USB_GUSB2PHYCFG_SUSPHY            BIT(6)
#define USB_GUSB2PHYCFG_ULPI_UTMI         BIT(4)
#define USB_GUSB2PHYCFG_ENBLSLPM          BIT(8)
#define USB_GUSB2PHYCFG_PHYIF_POS         3
#define USB_GUSB2PHYCFG_PHYIF(n)          ((n) << USB_GUSB2PHYCFG_PHYIF_POS)
#define USB_GUSB2PHYCFG_PHYIF_MASK        USB_GUSB2PHYCFG_PHYIF(1)
#define USB_GUSB2PHYCFG_USBTRDTIM_POS     10
#define USB_GUSB2PHYCFG_USBTRDTIM(n)      ((n) << USB_GUSB2PHYCFG_USBTRDTIM_POS)
#define USB_GUSB2PHYCFG_USBTRDTIM_MASK    USB_GUSB2PHYCFG_USBTRDTIM(0xF)
#define USBTRDTIM_UTMI_8_BIT              9
#define USBTRDTIM_UTMI_16_BIT             5
#define UTMI_PHYIF_16_BIT                 1
#define UTMI_PHYIF_8_BIT                  0
#define USB_GUSB2PHYCFG_ULPIAUTORES_POS   15
#define USB_GUSB2PHYCFG_ULPIAUTORES       (1U << USB_GUSB2PHYCFG_ULPIAUTORES_POS)

/* TRB Control */
#define USB_TRB_CTRL_HWO                  BIT(0)
#define USB_TRB_CTRL_LST                  BIT(1)
#define USB_TRB_CTRL_CHN                  BIT(2)
#define USB_TRB_CTRL_CSP                  BIT(3)
#define USB_TRB_CTRL_TRBCTL_MSK           0x3F
#define USB_TRB_CTRL_TRBCTL_POS           4
#define USB_TRB_CTRL_TRBCTL(n)                                                                     \
   (((n) & USB_TRB_CTRL_TRBCTL_MSK) << USB_TRB_CTRL_TRBCTL_POS)
#define USB_TRB_CTRL_ISP_IMI              BIT(10)
#define USB_TRB_CTRL_IOC                  BIT(11)
#define USB_TRB_CTRL_SID_SOFN_MSK         0xFFFFU
#define USB_TRB_CTRL_SID_SOFN_POS         14
#define USB_TRB_CTRL_SID_SOFN(n)                                                                   \
   (((n) & USB_TRB_CTRL_SID_SOFN_MSK) << USB_TRB_CTRL_SID_SOFN_POS)

#define USB_TRBCTL_TYPE_MSK               0x3F0
#define USB_TRBCTL_TYPE(n)                ((n) & USB_TRBCTL_TYPE_MSK)
#define USB_TRBCTL_NORMAL                 USB_TRB_CTRL_TRBCTL(1)
#define USB_TRBCTL_CONTROL_SETUP          USB_TRB_CTRL_TRBCTL(2)
#define USB_TRBCTL_CONTROL_STATUS2        USB_TRB_CTRL_TRBCTL(3)
#define USB_TRBCTL_CONTROL_STATUS3        USB_TRB_CTRL_TRBCTL(4)
#define USB_TRBCTL_CONTROL_DATA           USB_TRB_CTRL_TRBCTL(5)
#define USB_TRBCTL_ISOCHRONOUS_FIRST      USB_TRB_CTRL_TRBCTL(6)
#define USB_TRBCTL_ISOCHRONOUS            USB_TRB_CTRL_TRBCTL(7)
#define USB_TRBCTL_LINK_TRB               USB_TRB_CTRL_TRBCTL(8)
#define USB_TRBCTL_NORMAL_ZLP             USB_TRB_CTRL_TRBCTL(9)

/* TRB Length, PCM and Status */
#define USB_TRB_SIZE_MASK                 (0x00FFFFFF)
#define USB_TRB_SIZE_LENGTH(n)            ((n) & USB_TRB_SIZE_MASK)
#define USB_TRB_PCM1_MSK                  0x3
#define USB_TRB_PCM1_POS                  24
#define USB_TRB_PCM1(n)                   (((n) & USB_TRB_PCM1_MSK) << USB_TRB_PCM1_POS)
#define USB_TRB_SIZE_TRBSTS_MSK           0xF0000000
#define USB_TRB_SIZE_TRBSTS_POS           28
#define USB_TRB_SIZE_TRBSTS(n)                                                                     \
   (((n) & USB_TRB_SIZE_TRBSTS_MSK) >> USB_TRB_SIZE_TRBSTS_POS)

#define USB_TRBSTS_OK                     0
#define USB_TRBSTS_MISSED_ISOC            1
#define USB_TRBSTS_SETUP_PENDING          2
#define USB_TRB_STS_XFER_IN_PROG          4
#define USB_ISOC_MAX_MULT_VALUE           2

/* Device Endpoint Command Register */
#define USB_DEPCMD_DEPSTARTCFG            (0x09 << 0)
#define USB_DEPCMD_ENDTRANSFER            (0x08 << 0)
#define USB_DEPCMD_UPDATETRANSFER         (0x07 << 0)
#define USB_DEPCMD_STARTTRANSFER          (0x06 << 0)
#define USB_DEPCMD_CLEARSTALL             (0x05 << 0)
#define USB_DEPCMD_SETSTALL               (0x04 << 0)
#define USB_DEPCMD_GETEPSTATE             (0x03 << 0)
#define USB_DEPCMD_SETTRANSFRESOURCE      (0x02 << 0)
#define USB_DEPCMD_SETEPCONFIG            (0x01 << 0)

#define USB_DEPCMD_CLEARPENDIN            BIT(11)
#define USB_DEPCMD_CMDACT                 BIT(10)
#define USB_DEPCMD_CMDIOC                 BIT(8)

#define USB_DEPCMD_PARAM_MSK              0x7F0000
#define USB_DEPCMD_PARAM_POS              16
#define USB_DEPCMD_PARAM(x)               ((x) << USB_DEPCMD_PARAM_POS)
#define USB_DEPCMD_GET_RSC_IDX(x)         (((x) & USB_DEPCMD_PARAM_MSK) >> USB_DEPCMD_PARAM_POS)
#define USB_DEPCMD_STATUS_MSK             0xF000
#define USB_DEPCMD_STATUS_POS             12
#define USB_DEPCMD_STATUS(x)              (((x) & USB_DEPCMD_STATUS_MSK) >> USB_DEPCMD_STATUS_POS)
#define USB_DEPCMD_HIPRI_FORCERM          BIT(11)
#define USB_DEPCMD_FORCERM                1

/* Define USB device control endpoint ep0-out command configuration config values */
#define USB_DEPCMD_EP0_OUT_PAR1           0x00000500
#define USB_DEPCMD_EP0_OUT_PAR0           0x00000200
/* Define USB device control endpoint ep1-in command configuration config values */
#define USB_DEPCMD_EP1_IN_PAR1            0x06000500
#define USB_DEPCMD_EP1_IN_PAR0            0x00000200
/* Define USB device endpoint command config for ep/0/1/2/3/in/out */
#define USB_DEPCMD_EP_XFERCFG_PAR0        0x00000001
#define USB_DEPCMD_EP_XFERCFG             0x00000402

#define USB_DEPCMD_EP0_XFERCFG_PAR1       0x02041000
#define USB_DEPCMD_EP0_XFERCFG_PAR0       0x00000000
#define USB_DEPCMD_XFERCFG_EP0            0x00000506

/* DEPXFERCFG parameter 0 */
#define USB_DEPXFERCFG_MSK                0xFFFFU
#define USB_DEPXFERCFG_NUM_XFER_RES(n)    (n & USB_DEPXFERCFG_MSK)
/* The EP number goes 0..31 so ep0 is always out and ep1 is always in */
#define USB_DALEPENA_EP(n)                (0x00000001U << (n))

/* DEPCFG parameter 0 */
#define USB_DEPCFG_EP_TYPE_POS            1
#define USB_DEPCFG_EP_TYPE(n)             ((n) << USB_DEPCFG_EP_TYPE_POS)
#define USB_DEPCFG_MAX_PACKET_SIZE_POS    3
#define USB_DEPCFG_MAX_PACKET_SIZE(n)     ((n) << USB_DEPCFG_MAX_PACKET_SIZE_POS)
#define USB_DEPCFG_FIFO_NUMBER_MSK        0x1F
#define USB_DEPCFG_FIFO_NUMBER_POS        17
#define USB_DEPCFG_FIFO_NUMBER(n)                                                                  \
   (((n) & USB_DEPCFG_FIFO_NUMBER_MSK) << USB_DEPCFG_FIFO_NUMBER_POS)
#define USB_DEPCFG_BURST_SIZE_POS         22
#define USB_DEPCFG_BURST_SIZE(n)          ((n) << USB_DEPCFG_BURST_SIZE_POS)
#define USB_DEPCFG_DATA_SEQ_NUM_POS       26
#define USB_DEPCFG_DATA_SEQ_NUM(n)        ((n) << USB_DEPCFG_DATA_SEQ_NUM_POS)

#define USB_DEPCFG_ACTION_POS             30
#define USB_DEPCFG_ACTION_INIT            (0 << USB_DEPCFG_ACTION_POS)
#define USB_DEPCFG_ACTION_RESTORE         (1 << USB_DEPCFG_ACTION_POS)
#define USB_DEPCFG_ACTION_MODIFY          (2 << USB_DEPCFG_ACTION_POS)

#define USB_DEPCFG_INT_NUM(n)             ((n) << 0U)
#define USB_DEPCFG_XFER_COMPLETE_EN       BIT(8)
#define USB_DEPCFG_XFER_IN_PROGRESS_EN    BIT(9)
#define USB_DEPCFG_XFER_NOT_READY_EN      BIT(10)
#define USB_DEPCFG_FIFO_ERROR_EN          BIT(11)
#define USB_DEPCFG_STREAM_EVENT_EN        BIT(13)
#define USB_DEPCFG_BINTERVAL_M1_MSK       0xFF
#define USB_DEPCFG_BINTERVAL_M1_POS       16
#define USB_DEPCFG_BINTERVAL_M1(n)                                                                 \
    (((n) & USB_DEPCFG_BINTERVAL_M1_MSK) << USB_DEPCFG_BINTERVAL_M1_POS)
#define USB_DEPCFG_STREAM_CAPABLE    BIT(24)
#define USB_DEPCFG_EP_NUMBER_POS     25
#define USB_DEPCFG_EP_NUMBER(n)      ((n) << USB_DEPCFG_EP_NUMBER_POS)
#define USB_DEPCFG_BULK_BASED        BIT(30)
#define USB_DEPCFG_FIFO_BASED        BIT(31)

/* USB Device status register      */
#define USB_DSTS_USBLNKST_POS        18
#define USB_DSTS_USBLNKST_MASK       (0x0F << USB_DSTS_USBLNKST_POS)
#define USB_DSTS_USBLNKST(n)         ((n & USB_DSTS_USBLNKST_MASK) >> USB_DSTS_USBLNKST_POS)
#define USB_DSTS_DEVCTRLHLT          BIT(22)
#define USB_DSTS_DCNRD               BIT(29)
#define USB_DSTS_MICRO_FRAME_MASK    0x38
#define USB_DSTS_MICRO_FRAME_POS     3
#define USB_DSTS_SOFFN(reg)          ((reg & USB_DSTS_MICRO_FRAME_MASK) >> USB_DSTS_MICRO_FRAME_POS)
#define USB_DSTS_CONNECTSPD          (7 << 0)
#define USB_DSTS_HIGHSPEED           (0 << 0)

/* Device control register */
#define USB_DCTL_CSFTRST             BIT(30)
#define USB_DCTL_START               BIT(31)
#define USB_DCTL_KEEP_CONNECT        BIT(19)
#define USB_DCTL_INITU2ENA           BIT(12)
#define USB_DCTL_ACCEPTU2ENA         BIT(11)
#define USB_DCTL_INITU1ENA           BIT(10)
#define USB_DCTL_ACCEPTU1ENA         BIT(9)
#define USB_DCTL_TSTCTRL_POS         1
#define USB_DCTL_TSTCTRL_MASK        (0xF << USB_DCTL_TSTCTRL_POS)
#define USB_DCTL_ULSTCHNGREQ_POS     5
#define USB_DCTL_ULSTCHNGREQ_MASK    0x1E0
#define USB_DCTL_ULSTCHNGREQ(n)      (((n) << USB_DCTL_ULSTCHNGREQ_POS) & USB_DCTL_ULSTCHNGREQ_MASK)
#define USB_DCTL_L1_HIBER_EN         BIT(18)
#define USB_DCTL_HIRD_THRES_POS      24
#define USB_DCTL_HIRD_THRES_MASK     (0x1F << USB_DCTL_HIRD_THRES_POS)
#define USB_DCTL_HIRD_THRES(n)       ((n) << USB_DCTL_HIRD_THRES_POS)

/* Global Event Size Registers */
#define USB_GEVNTSIZ_INTMASK         BIT(31)
#define USB_GEVNTSIZ_SIZE_MSK        0xFFFF
#define USB_GEVNTSIZ_SIZE(n)         ((n) & USB_GEVNTSIZ_SIZE_MSK)
#define USB_GEVNTCOUNT_MASK          0xFFFC

#define USB_EP_EVENT_TYPE            0
#define USB_DEV_EVENT_TYPE           0x1
#define USB_EVENT_TYPE_CHECK         0x1

/* USB device Disconnect event   */
#define USB_DEV_DISSCONNEVTEN        BIT(0)
/* USB device reset event   */
#define USB_DEV_USBRSTEVTEN          BIT(1)
/* USB device connection done event */
#define USB_DEV_CONNECTDONEEVTEN     BIT(2)
/* USB device link state change event  */
#define USB_DEV_EVENT_ULSTCNGEN      BIT(3)

/* Device specific events */
#define USB_EVENT_DISCONNECT         0
#define USB_EVENT_RESET              1
#define USB_EVENT_CONNECT_DONE       2
#define USB_EVENT_LINK_STATUS_CHANGE 3
#define USB_EVENT_WAKEUP             4
#define USB_EVENT_HIBER_REQ          5
#define USB_EVENT_EOPF               6
#define USB_EVENT_SOF                7
#define USB_EVENT_ERRATIC_ERROR      9
#define USB_EVENT_CMD_CMPL           10
#define USB_EVENT_OVERFLOW           11

/* USB global control register bits   */
#define USB_GUCTL_HSTINAUTORETRY     BIT(14)
#define USB_GCTL_CORESOFTRESET       BIT(11)
#define USB_GCTL_SOFITPSYNC          BIT(10)
#define USB_GCTL_SCALEDOWN_POS       4
#define USB_GCTL_SCALEDOWN(n)        ((n) << USB_GCTL_SCALEDOWN_POS)
#define USB_GCTL_SCALEDOWN_MASK      USB_GCTL_SCALEDOWN(3U)
#define USB_GCTL_DISSCRAMBLE         BIT(3)
#define USB_GCTL_U2EXIT_LFPS         BIT(2)
#define USB_GCTL_GBLHIBERNATIONEN    BIT(1)
#define USB_GCTL_DSBLCLKGTNG         BIT(0)
#define USB_GCTL_PRTCAP_MSK          0x3000
#define USB_GCTL_PRTCAP_POS          12
#define USB_GCTL_PRTCAP(n)           (((n) & USB_GCTL_PRTCAP_MSK) >> USB_GCTL_PRTCAP_POS)
#define USB_GCTL_PRTCAPDIR(n)        ((n) << USB_GCTL_PRTCAP_POS)
#define USB_GCTL_PRTCAP_HOST         1
#define USB_GCTL_PRTCAP_DEVICE       2
#define USB_GCTL_PRTCAP_OTG          3

/* Define USB device controller global soc bus config values */

#define USB_GSBUSCFG0_INCR256BRSTENA BIT(7) /* INCR256 burst */
#define USB_GSBUSCFG0_INCR128BRSTENA BIT(6) /* INCR128 burst */
#define USB_GSBUSCFG0_INCR64BRSTENA  BIT(5) /* INCR64 burst */
#define USB_GSBUSCFG0_INCR32BRSTENA  BIT(4) /* INCR32 burst */
#define USB_GSBUSCFG0_INCR16BRSTENA  BIT(3) /* INCR16 burst */
#define USB_GSBUSCFG0_INCR8BRSTENA   BIT(2) /* INCR8 burst */
#define USB_GSBUSCFG0_INCR4BRSTENA   BIT(1) /* INCR4 burst */
#define USB_GSBUSCFG0_INCRBRSTENA    BIT(0) /* undefined length enable */
#define USB_GSBUSCFG0_INCRBRST_MASK  0xFF

/* Global controller ID Register  */
/* bit [31:16] for Core Identification Number */
#define USB_GSNPSID_MASK             0xFFFF0000
/* bit [15:0] for Core release number */
#define USB_GSNPSREV_MASK            0xFFFF

/* Global Frame Length Adjustment Register */
#define USB_GFLADJ_30MHZ_SDBND_SEL   BIT(7)
#define USB_GFLADJ_30MHZ_MASK        0x3F
#define USB_GFLADJ_DEFAULT_VALUE     0x20

typedef enum _USB_DEVICE_STATE {
    USBD_STATE_NOTATTACHED,
    USBD_STATE_ATTACHED,
    USBD_STATE_POWERED,
    USBD_STATE_RECONNECTING,
    USBD_STATE_UNAUTHENTICATED,
    USBD_STATE_DEFAULT,
    USBD_STATE_ADDRESS,
    USBD_STATE_CONFIGURED,
} USB_DEVICE_STATE;

/* Control Endpoint State */
typedef enum _USBD_EP0_STATE {
    EP0_UNCONNECTED,
    EP0_SETUP_PHASE,
    EP0_DATA_PHASE,
    EP0_STATUS_PHASE,
} USBD_EP0_STATE;

/* Controller Role */
typedef enum _USB_DR_MODE {
    USB_DR_MODE_UNKNOWN,
    USB_DR_MODE_HOST,
    USB_DR_MODE_PERIPHERAL,
    USB_DR_MODE_OTG,
} USB_DR_MODE;

/* PHY Interface */
typedef enum _USB_PHY_INTERFACE {
    USB_PHY_INTERFACE_MODE_UNKNOWN,
    USB_PHY_INTERFACE_MODE_UTMI,
    USB_PHY_INTERFACE_MODE_UTMIW,
    USB_PHY_INTERFACE_MODE_ULPI,
    USB_PHY_INTERFACE_MODE_SERIAL,
    USB_PHY_INTERFACE_MODE_HSIC,
} USB_PHY_INTERFACE;

/* Endpoint Parameters */
typedef struct _USBD_EP_PARAMS {
    uint32_t param2;
    uint32_t param1;
    uint32_t param0;
} USBD_EP_PARAMS;
/* Transfer Request Block */
typedef struct _USBD_TRB {
    uint32_t buf_ptr_low;
    uint32_t buf_ptr_high;
    uint32_t size;
    uint32_t ctrl;
} USBD_TRB;

/* Define USB controller physical endpoint structure.  */
typedef struct _USBD_EP {
    struct UX_SLAVE_ENDPOINT_STRUCT *endpoint;
    USBD_TRB                         ep_trb[NO_OF_TRB_PER_EP + 1] ATTR_ALIGN(32);
    uint32_t                         ep_status;
    uint32_t                         ep_state;
    uint8_t                          ep_index;
    uint32_t                         ep_transfer_status;
    uint8_t                          ep_dir;
    uint32_t                         ep_maxpacket;
    uint32_t                         ep_resource_index;
    uint32_t                         ep_requested_bytes;
    uint32_t                         trb_enqueue;
    uint32_t                         trb_dequeue;
    uint8_t                          phy_ep;
    uint32_t                         bytes_txed;
    uint32_t                         unaligned_txed;
} USBD_EP;

/* USB setup packet structure */
typedef struct _USB_CTRL_REQUEST {
    uint8_t  bRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USBD_CTRL_REQUEST;

/* USB event buffer structure */
typedef struct _USBD_EVENT_BUFFER {
    void    *buf;
    uint32_t length;
    uint32_t lpos;
    uint32_t count;
} USBD_EVENT_BUFFER;

/* HWPARAMS registers */
typedef struct _USB_HWPARAMS {
    uint32_t hwparams0;
    uint32_t hwparams1;
    uint32_t hwparams2;
    uint32_t hwparams3;
    uint32_t hwparams4;
    uint32_t hwparams5;
    uint32_t hwparams6;
    uint32_t hwparams7;
    uint32_t hwparams8;
} USB_HWPARAMS;

/* USB driver structure */
typedef struct _USB_DRIVER {
    USB_Type *regs;
    void (*ux_dcd_reset_cb)(struct _USB_DRIVER *drv);
    void (*ux_dcd_connect_cb)(void);
    void (*ux_dcd_disconnect_cb)(void);
    void (*ux_dcd_setup_stage_cb)(struct _USB_DRIVER *drv, uint8_t phy_ep);
    void (*ux_dcd_ep0_data_stage_cb)(struct _USB_DRIVER *drv, uint8_t phy_ep);
    void (*ux_dcd_data_stage_cb)(struct _USB_DRIVER *drv, uint8_t phy_ep);
    /* for isoc only */
    void (*ux_dcd_xfernotready_data)(struct _USB_DRIVER *drv, uint8_t phy_ep);

    USBD_CTRL_REQUEST            setup_data ATTR_ALIGN(32);
    USBD_TRB                     ep0_trb ATTR_ALIGN(32);
    USBD_EP                      eps[SOC_FEAT_USB_EP_TOTAL];
    USBD_EVENT_BUFFER            *event_buf;
    USBD_EP0_STATE               ep0_state;
    USB_DEVICE_STATE             config_state;
    USB_HWPARAMS                 hwparams;
    uint8_t                      in_eps;
    uint8_t                      out_eps;
    uint32_t                     num_bytes;
    uint32_t                     usb2_phy_config;
    bool                         setup_packet_pending;
    bool                         three_stage_setup;
    bool                         ep0_expect_in;
    uint32_t                     revision;
    uint8_t                      dr_role;
    uint8_t                      speed;
    uint8_t                      endp_number;
    uint8_t                      dr_mode;
    uint8_t                      hsphy_mode;
    uint8_t                      fladj;
    uint32_t                     event_type;
    uint32_t                     micro_frame_number;
} USB_DRIVER;

void     usbd_devt_handler(USB_DRIVER *drv, uint32_t reg);
void     usbd_depevt_handler(USB_DRIVER *drv, uint32_t reg);
void     usbd_prepare_setup(USB_DRIVER *drv);
int32_t  usbd_ep0_stall_restart(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir);
uint32_t usbd_ep_enable(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t ep_type,
                        uint16_t ep_max_pkt_size, uint8_t ep_interval);
int32_t  usbd_ep_disable(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir);

uint32_t usbd_configure_endpoint_parameters(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir,
       uint8_t ep_type, uint16_t ep_max_pkt_size, uint8_t ep_interval);
uint32_t usbd_get_ep_transfer_resource_index(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir);
uint32_t usbd_set_xfer_resource(USB_DRIVER *drv, uint8_t phy_ep);
uint32_t usbd_start_endpoint_config(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir);

uint32_t usbd_set_device_address(USB_DRIVER *drv, uint8_t addr);
uint32_t usbd_state_change(USB_DRIVER *drv, uint32_t state);
int32_t  usbd_stop_transfer(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint32_t force_rm);
int32_t  usbd_ep_stall(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir);
int32_t usbd_ep_transfer_abort(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir);
int32_t  usbd_ep_clear_stall(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir);
void     usbd_interrupt_handler(USB_DRIVER *drv);
void     usbd_clear_trb(USB_DRIVER *drv, uint8_t endp_number);

void    usbd_usb2phy_config_check(USB_DRIVER *drv);
void    usbd_usb2phy_config_reset(USB_DRIVER *drv);
int32_t usbd_send_ep_cmd(USB_DRIVER *drv, uint8_t phy_ep, uint32_t cmd, USBD_EP_PARAMS params);
int32_t usbd_bulk_recv(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t *bufferptr,
                       uint32_t buf_len);
int32_t usbd_bulk_send(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t *bufferptr,
                       uint32_t buf_len);
int32_t usbd_ep0_send(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t *bufferptr,
                      uint32_t buf_len);
int32_t usbd_ep0_recv(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t *bufferptr,
                      uint32_t buf_len);
int32_t usbd_isoc_send(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t *bufferptr,
                       uint32_t buf_len);
void    usbd_reset_event(USB_DRIVER *drv);
void    usbd_connectiondone_event(USB_DRIVER *drv);

bool     usbd_verify_ip_core(USB_DRIVER *drv);
void     usbd_set_port_capability(USB_DRIVER *drv, uint32_t mode);
void     usbd_set_default_config(USB_DRIVER *drv);
void     usbd_read_hw_params(USB_DRIVER *drv);
int32_t  usbd_validate_mode(USB_DRIVER *drv);
int32_t  usbd_device_soft_reset(USB_DRIVER *drv);
void     usbd_reset_phy_and_core(USB_DRIVER *drv);
void     usbd_configure_usb2_phy(USB_DRIVER *drv);
void     usbd_configure_global_control_reg(USB_DRIVER *drv);
void     usbd_configure_fladj_register(USB_DRIVER *drv);
void     usbd_configure_burst_transfer(USB_DRIVER *drv);
void     usbd_cleanup_event_buffer(USB_DRIVER *drv);
void     usbd_set_speed(USB_DRIVER *drv);
void     ux_dcd_setup_event_buffer(USB_DRIVER *drv);
void     usbd_configure_event_buffer_registers(USB_DRIVER *drv);
void     usbd_get_eps(USB_DRIVER *drv);
void     usbd_initialize_eps(USB_DRIVER *drv);
void     usbd_configure_initial_ep_commands(USB_DRIVER *drv);
uint32_t usbd_start(USB_DRIVER *drv);
void     usbd_enable_device_events(USB_DRIVER *drv);

#ifdef __cplusplus
}

#endif

#endif
