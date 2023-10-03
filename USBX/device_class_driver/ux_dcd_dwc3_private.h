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
 * @file     ux_dcd_dwc3.h
 * @author   anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    Header file for the usbx driver for dwc.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef UX_DCD_DWC3_PRIVATE_H
#define UX_DCD_DWC3_PRIVATE_H

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

/* Define DWC3 generic equivalences.  */

#define UX_DCD_DWC3_SLAVE_CONTROLLER                   0x90
#define UX_DCD_DWC3_MAX_ED                             0x04
#define UX_DCD_DWC3_MAX_ENDP                           16
#define UX_DCD_DWC3_IN_FIFO                            3
#define UX_DCD_DWC3_DATA_FIFO_OFFSET                   0x00001000
#define UX_DCD_DWC3_DATA_FIFO_SIZE                     0x00001000
#define UX_DCD_DWC3_RX_FIFO_SIZE                       128
#define UX_DCD_DWC3_TX_FIFO_SIZE                       128
#define UX_DCD_DWC3_NP_TX_FIFO_SIZE                    96
#define UX_DCD_DWC3_ENDPOINT_TX_FIFO_SIZE              96
#define UX_DCD_DWC3_FLUSH_RX_FIFO                      0x00000010
#define UX_DCD_DWC3_FLUSH_TX_FIFO                      0x00000020
#define UX_DCD_DWC3_FLUSH_FIFO_ALL                     0x00000010
#define UX_DCD_DWC3_ENDPOINT_SPACE_SIZE                0x00000020
#define UX_DCD_DWC3_ENDPOINT_CHANNEL_SIZE              0x00000020

#define UX_DCD_DWC3_CONTROLLER_DELAY                   72
#define UX_DCD_DWC3_EVENT_BUFFER_SIZE                  4096

/* Define DWC3 USB device controller registers equivalences.  */
/* Global Registers */
#define UX_DCD_DWC3_GSBUSCFG0          0xc100
#define UX_DCD_DWC3_GSBUSCFG1          0xc104
#define UX_DCD_DWC3_GTXTHRCFG          0xc108
#define UX_DCD_DWC3_GRXTHRCFG          0xc10c
#define UX_DCD_DWC3_GCTL               0xc110
#define UX_DCD_DWC3_GEVTEN             0xc114
#define UX_DCD_DWC3_GSTS               0xc118
#define UX_DCD_DWC3_GUCTL1             0xc11c
#define UX_DCD_DWC3_GSNPSID            0xc120
#define UX_DCD_DWC3_GGPIO              0xc124
#define UX_DCD_DWC3_GUID               0xc128
#define UX_DCD_DWC3_GUCTL              0xc12c
#define UX_DCD_DWC3_GBUSERRADDR0       0xc130
#define UX_DCD_DWC3_GBUSERRADDR1       0xc134
#define UX_DCD_DWC3_GPRTBIMAP0         0xc138
#define UX_DCD_DWC3_GPRTBIMAP1         0xc13c
#define UX_DCD_DWC3_GHWPARAMS0         0xc140
#define UX_DCD_DWC3_GHWPARAMS1         0xc144
#define UX_DCD_DWC3_GHWPARAMS2         0xc148
#define UX_DCD_DWC3_GHWPARAMS3         0xc14c
#define UX_DCD_DWC3_GHWPARAMS4         0xc150
#define UX_DCD_DWC3_GHWPARAMS5         0xc154
#define UX_DCD_DWC3_GHWPARAMS6         0xc158
#define UX_DCD_DWC3_GHWPARAMS7         0xc15c
#define UX_DCD_DWC3_GDBGFIFOSPACE      0xc160
#define UX_DCD_DWC3_GDBGLTSSM          0xc164
#define UX_DCD_DWC3_GDBGBMU            0xc16c
#define UX_DCD_DWC3_GDBGLSPMUX         0xc170
#define UX_DCD_DWC3_GDBGLSP            0xc174
#define UX_DCD_DWC3_GDBGEPINFO0        0xc178
#define UX_DCD_DWC3_GDBGEPINFO1        0xc17c
#define UX_DCD_DWC3_GPRTBIMAP_HS0      0xc180
#define UX_DCD_DWC3_GPRTBIMAP_HS1      0xc184
#define UX_DCD_DWC3_GPRTBIMAP_FS0      0xc188
#define UX_DCD_DWC3_GPRTBIMAP_FS1      0xc18c
#define UX_DCD_DWC3_GUCTL2             0xc19c

#define UX_DCD_DWC3_VER_NUMBER         0xc1a0
#define UX_DCD_DWC3_VER_TYPE           0xc1a4

#define UX_DCD_DWC3_GUSB2PHYCFG(n)     (0xc200 + ((n) * 0x04))
#define UX_DCD_DWC3_GUSB2I2CCTL(n)     (0xc240 + ((n) * 0x04))

#define UX_DCD_DWC3_GUSB2PHYACC(n)     (0xc280 + ((n) * 0x04))

#define UX_DCD_DWC3_GUSB3PIPECTL(n)    (0xc2c0 + ((n) * 0x04))

#define UX_DCD_DWC3_GTXFIFOSIZ(n)      (0xc300 + ((n) * 0x04))
#define UX_DCD_DWC3_GRXFIFOSIZ(n)      (0xc380 + ((n) * 0x04))

#define UX_DCD_DWC3_GEVNTADRLO(n)      (0xc400 + ((n) * 0x10))
#define UX_DCD_DWC3_GEVNTADRHI(n)      (0xc404 + ((n) * 0x10))
#define UX_DCD_DWC3_GEVNTSIZ(n)        (0xc408 + ((n) * 0x10))
#define UX_DCD_DWC3_GEVNTCOUNT(n)      (0xc40c + ((n) * 0x10))

#define UX_DCD_DWC3_GHWPARAMS8         0xc600
#define UX_DCD_DWC3_GFLADJ             0xc630

/* Device Registers */
#define UX_DCD_DWC3_DCFG               0xc700
#define UX_DCD_DWC3_DCTL               0xc704
#define UX_DCD_DWC3_DEVTEN             0xc708
#define UX_DCD_DWC3_DSTS               0xc70c
#define UX_DCD_DWC3_DGCMDPAR           0xc710
#define UX_DCD_DWC3_DGCMD              0xc714
#define UX_DCD_DWC3_DALEPENA           0xc720

#define UX_DCD_DWC3_DEP_BASE(n)        (0xc800 + ((n) * 0x10))
#define UX_DCD_DWC3_DEPCMDPAR2         0x00
#define UX_DCD_DWC3_DEPCMDPAR1         0x04
#define UX_DCD_DWC3_DEPCMDPAR0         0x08
#define UX_DCD_DWC3_DEPCMD             0x0c

#define UX_DCD_DWC3_DEV_IMOD(n)        (0xca00 + ((n) * 0x4))



#define UX_DCD_GTXFIFOSIZ_TXFDEF(n)       ((n) & 0xffff)

/* Define DWC3 USB device controller core reset values.  */
#define UX_DCD_DWC3_FS_DCTL_CSRST                   0x40000000
#define UX_DCD_DWC3_FS_DCTL_EPENABLE                0x80F00000
#define UX_DCD_DWC3_FS_DTCL_EPCONFIG                0x40F00A00

#define UX_DCD_DWC3_FS_DCTL_CSRST_STS               0x40000000

#define UX_DCD_DWC3_DCTL_LINK_STATUS                0x80F00A00

/* Define DWC3 USB device controller PHY config values.  */
#define UX_DCD_DWC3_GEVENTADDR_LO                   0x02040000
#define UX_DCD_DWC3_GEVENTADDR_HI                   0x00000000
#define UX_DCD_DWC3_GEVENT_SIZE                     0x00000030
#define UX_DCD_DWC3_GEVENT_COUNT                    0x00000000

/* Define DWC3 USB device controller global event values */
#define UX_DCD_DWC3_GUSB2PHYCFG_CFG                 0x00001508
#define UX_DCD_DWC3_GUSB2PHYCFG_EPCMD_CFG           0x40002547
#define UX_DCD_DWC3_FS_DCTL_EPENABLE                0x80F00000
#define UX_DCD_DWC3_GUSB2PHYCFG_SUSPHY              BIT(6)

/* Define DWC3 USB device controller global core control values */
#define UX_DCD_DWC3_GCTL_CFG                       0x30c12234


/* Define DWC3 USB device controller global soc bus config values */
#define UX_DCD_DWC3_GSBUSCFG                       0x00000009

/* Define DWC3 USB device configration with default address config values */
#define UX_DCD_DWC3_DCFG_ADDR                      0x00480800
#define UX_DCD_DWC3_DCFG_ADDR_MASK                 0x000001f8

/* Define DWC3 USB device event configration config values */
#define UX_DCD_DWC3_DEV_INT_EN                     0x00000007

/* Define DWC3 USB global user control configration config values */
#define UX_DCD_DWC3_GUCTL_CFG                      0x02000010 //Changed value

/* Define common config for devie endpoints ep/in/out/bulk/control/isoc/interrupt */
#define UX_DCD_DWC3_DEPCMD_SCFG                    0x00000409
#define UX_DCD_DWC3_DEPCMD_COMMON_SCFG             0x00000401

/* Define DWC3 USB device control endpoint ep0-out/ep0-in command configration config values */
#define UX_DCD_DWC3_DEPCMD_EP0_OUT_PAR1            0x00000700
#define UX_DCD_DWC3_DEPCMD_EP0_IN_PAR1             0x02000700
#define UX_DCD_DWC3_DEPCMD_EP0_PAR0                0x00000200
#define UX_DCD_DWC3_DEPCMD_EP0_PAR2                0x00000000

/* Define DWC3 USB device interrupt endpoint ep1-out/ep1-in command configration config values */
#define UX_DCD_DWC3_DEPCMD_EP1_OUT_PAR1             0x04000700
#define UX_DCD_DWC3_DEPCMD_EP1_IN_PAR1              0x06000700
#define UX_DCD_DWC3_DEPCMD_EP1_OUT_PAR0             0x00002006
#define UX_DCD_DWC3_DEPCMD_EP1_IN_PAR0              0x00022006
#define UX_DCD_DWC3_DEPCMD_EP1_PAR2                 0x00000000

/* Define DWC3 USB device bulk endpoint ep2-out/ep2-in command configration config values */
#define UX_DCD_DWC3_DEPCMD_EP2_OUT_PAR1             0x08000700
#define UX_DCD_DWC3_DEPCMD_EP2_IN_PAR1              0x0A000700
#define UX_DCD_DWC3_DEPCMD_EP2_OUT_PAR0             0x00001004
#define UX_DCD_DWC3_DEPCMD_EP2_IN_PAR0              0x00041004
#define UX_DCD_DWC3_DEPCMD_EP2_PAR2                 0x00000000

/* Define DWC3 USB device isochronous endpoint ep3-out/ep3-in command configration config values */
#define UX_DCD_DWC3_DEPCMD_EP3_OUT_PAR1             0x0C000700
#define UX_DCD_DWC3_DEPCMD_EP3_IN_PAR1              0x0E000700
#define UX_DCD_DWC3_DEPCMD_EP3_OUT_PAR0             0x00002002
#define UX_DCD_DWC3_DEPCMD_EP3_IN_PAR0              0x00062002
#define UX_DCD_DWC3_DEPCMD_EP3_PAR2                 0x00000000

/* Define DWC3 USB device endpoint command config for ep/0/1/2/3/in/out */
#define UX_DCD_DWC3_DEPCMD_EP_XFERCFG_PAR0          0x00000001
#define UX_DCD_DWC3_DEPCMD_EP_XFERCFG               0x00000402

#define UX_DCD_DWC3_DEPCMD_EP0_XFERCFG_PAR1         0x02041000
#define UX_DCD_DWC3_DEPCMD_EP0_XFERCFG_PAR0         0x00000000
#define UX_DCD_DWC3_DEPCMD_XFERCFG_EP0              0x00000506


#define UX_DCD_DWC3_DEPCMD_EP0_OUT_CFG1             0x80000200
#define UX_DCD_DWC3_DEPCMD_EP0_OUT_CFG0             0x00000500
#define UX_DCD_DWC3_DEPCMD_EP0_IN_CFG1              0x80000200
#define UX_DCD_DWC3_DEPCMD_EP0_IN_CFG0              0x02000500
#define UX_DCD_DWC3_DEPCMD_EP0_CFG_CMD              0x00000401

/* Define DWC3 USB device endpoint activate command config for ep/0/1/2/3/in/out */
#define UX_DCD_DWC3_DALEP_CMD                   0x00000003
#define UX_DCD_DWC3_DALEPENA_CMD                0x00000003
#define UX_DCD_DWC3_DALEP_ALL_CMD               0x000000FF

/*Issuing DEPSTRTXFER Command to ep_resource[0] (for EP 0/CONTROL/)*/
#define UX_DCD_DWC3_DEPCMD_CONN_OUT_PARAM1          0x00041000
#define UX_DCD_DWC3_DEPCMD_CONN_OUT_PARAM0          0x00000000
#define UX_DCD_DWC3_DEPCMD_CONN_IN_PARAM1           0x02041000
#define UX_DCD_DWC3_DEPCMD_CONN_IN_PARAM0           0x00000000
#define UX_DCD_DWC3_DEPCMD_CONN_PARAM               0x00000506

/* The EP number goes 0..31 so ep0 is always out and ep1 is always in */
#define UX_DCD_DALEPENA_EP(n)                  (0x00000001U << (n))

#define UX_DCD_GCTL_CORESOFTRESET              0x00000800U /* bit 11 */
#define UX_DCD_GCTL_SOFITPSYNC                 0x00000400U /* bit 10 */
#define UX_DCD_GCTL_SCALEDOWN(n)               ((n) << 4U)
#define UX_DCD_GCTL_SCALEDOWN_MASK             UX_DCD_GCTL_SCALEDOWN(3U)
#define UX_DCD_GCTL_DISSCRAMBLE                (0x00000001U << 3U)
#define UX_DCD_GCTL_U2EXIT_LFPS                (0x00000001U << 2U)
#define UX_DCD_GCTL_GBLHIBERNATIONEN           (0x00000001U << 1U)
#define UX_DCD_GCTL_DSBLCLKGTNG                (0x00000001U << 0U)


/* DEPXFERCFG parameter 0 */
#define UX_DCD_DEPXFERCFG_NUM_XFER_RES(n) (n & 0xFFFFU)

#define ALIGNMENT_CACHELINE             __attribute__ ((aligned(32)))

/**
 * upper_32_bits - return bits 32-63 of a number
 * @n: the number we're accessing
 *
 * A basic shift-right of a 64- or 32-bit quantity.  Use this to suppress
 * the "right shift count >= width of type" warning when that quantity is
 * 32-bits.
 */
#define upper_32_bits(n) ((ULONG)(((ULONG)(n) >> 16) >> 16))

/**
 * lower_32_bits - return bits 0-31 of a number
 * @n: the number we're accessing
 */
#define lower_32_bits(n) ((ULONG)(n))

#define UX_DCD_DWC3_GUSB3PIPECTL_EXIT_PX          BIT(27)
#define UX_DCD_DWC3_GUSB3PIPECTL_LFPSFILT         BIT(9)

/* Configure GUSB2PHYCFG after sending DEPCMF for connection done */
#define UX_DCD_DWC3_GUSB2PHYCFG_CONN            0x40001508
#define UX_DCD_DWC3_SCRATCH_BUFF_SIZE           4096

#define BIT(nr)                                 (1 << (nr))

#define UX_DCD_DWC3_EVENT_PENDING               BIT(0)

#define UX_DCD_BOLT_EVENT_BUFFER_SIZE           4096

#define UX_DCD_CLK_BASE_ADDRESS                0x1A010000
#define ACLK_CTRL                              0x820
#define ACLK_CTRL_VAL                          0x2
#define ACLK_DIV                               0x824
#define ACLK_DIV_VAL                           0x0

/* Start USB controller */
#define UX_DCD_DWC3_DCTL_START                  0x80000000 //24-28 is C, before this 0
#define UX_DCD_DWC3_DCTL_START_INIT             0x40F00000
#define UX_DCD_DWC3_DCTL_CONN_DONE              0x80F00A00

#define UX_DCD_DWC3_NULL_VAL                    0x00000000

/* Global Event Size Registers */
#define UX_DCD_GEVNTSIZ_INTMASK                 BIT(31)
#define UX_DCD_GEVNTSIZ_SIZE(n)                 ((n) & 0xffff)
#define UX_DCD_GEVNTCOUNT_MASK                  0xfffc

/* Device specific events */
#define UX_DCD_EVENT_DISCONNECT                 0
#define UX_DCD_EVENT_RESET                      1
#define UX_DCD_EVENT_CONNECT_DONE               2
#define UX_DCD_EVENT_LINK_STATUS_CHANGE         3
#define UX_DCD_EVENT_WAKEUP                     4
#define UX_DCD_EVENT_HIBER_REQ                  5
#define UX_DCD_EVENT_EOPF                       6
#define UX_DCD_EVENT_SOF                        7
#define UX_DCD_EVENT_ERRATIC_ERROR              9
#define UX_DCD_EVENT_CMD_CMPL                   10
#define UX_DCD_EVENT_OVERFLOW                   11


#define UX_DCD_DCTL_KEEP_CONNECT               0x00080000U /* bit 19 */

#define UX_DCD_DCTL_INITU2ENA             BIT(12)
#define UX_DCD_DCTL_ACCEPTU2ENA           BIT(11)
#define UX_DCD_DCTL_INITU1ENA             BIT(10)
#define UX_DCD_DCTL_ACCEPTU1ENA           BIT(9)
#define UX_DCD_DCTL_TSTCTRL_MASK          (0xf << 1)


#define UX_DCD_EP_TRANSFER_STARTED             BIT(3)
#define UX_DCD_EP_END_TRANSFER_PENDING         BIT(4)
#define UX_DCD_EP_DELAY_START                  BIT(6)

#define UX_DCD_EP_ENABLED                (0x00000001U << 0U)
#define UX_DCD_EP_STALL                  (0x00000001U << 1U)
#define UX_DCD_EP_WEDGE                  (0x00000001U << 2U)
#define UX_DCD_EP_BUSY                   (0x00000001U << 4U)
#define UX_DCD_EP_PENDING_REQUEST        (0x00000001U << 5U)
#define UX_DCD_EP_MISSED_ISOC            (0x00000001U << 6U)


        /* This last one is specific to EP0 */
#define UX_DCD_EP0_DIR_IN         BIT(31)

/* Device specific commmands, DEPCMD register is used */
#define UX_DCD_DEPCMD_DEPSTARTCFG         (0x09 << 0)
#define UX_DCD_DEPCMD_ENDTRANSFER         (0x08 << 0)
#define UX_DCD_DEPCMD_UPDATETRANSFER      (0x07 << 0)
#define UX_DCD_DEPCMD_STARTTRANSFER       (0x06 << 0)
#define UX_DCD_DEPCMD_CLEARSTALL          (0x05 << 0)
#define UX_DCD_DEPCMD_SETSTALL            (0x04 << 0)
/* This applies for core versions 1.90a and earlier */
#define UX_DCD_DEPCMD_GETSEQNUMBER        (0x03 << 0)
/* This applies for core versions 1.94a and later */
#define UX_DCD_DEPCMD_GETEPSTATE          (0x03 << 0)
#define UX_DCD_DEPCMD_SETTRANSFRESOURCE   (0x02 << 0)
#define UX_DCD_DEPCMD_SETEPCONFIG         (0x01 << 0)

#define UX_DCD_DEPCMD_CLEARPENDIN         BIT(11)
#define UX_DCD_DEPCMD_CMDACT              BIT(10)
#define UX_DCD_DEPCMD_CMDIOC              BIT(8)

#define UX_DCD_DEPCMD_STATUS(x)           (((x) >> 12) & 0x0F)

/* Device Generic Command Register */
#define UX_DCD_DGCMD_SET_LMP              0x01
#define UX_DCD_DGCMD_SET_PERIODIC_PAR     0x02
#define UX_DCD_DGCMD_XMIT_FUNCTION        0x03
#define UX_DCD_DGCMD_CMDACT               BIT(10)

#define UX_DCD_DGCMD_STATUS(n)                 (((ULONG)(n) >> 12U) & 0x0fU)
#define UX_DCD_DGCMD_CMDIOC                    0x00000100U /* bit 8 */


/* Device Endpoint Command Register */
#define UX_DCD_DEPCMD_PARAM_SHIFT         16
#define UX_DCD_DEPCMD_PARAM(x)            ((x) << UX_DCD_DEPCMD_PARAM_SHIFT)
#define UX_DCD_DEPCMD_GET_RSC_IDX(x)      (((x) >> UX_DCD_DEPCMD_PARAM_SHIFT) & 0x7f)
#define UX_DCD_DEPCMD_STATUS(x)           (((x) >> 12) & 0x0F)
#define UX_DCD_DEPCMD_HIPRI_FORCERM       BIT(11)


/* In response to Start Transfer */
#define DEPEVT_TRANSFER_NO_RESOURCE     1
#define DEPEVT_TRANSFER_BUS_EXPIRY      2

/* Global USB3 PIPE Control Register */
#define UX_DCD_GUSB3PIPECTL_PHYSOFTRST         0x80000000U /* bit 31 */
#define UX_DCD_GUSB3PIPECTL_SUSPHY             0x00020000U /* bit 17 */
#define UX_DCD_GUSB3PIPECTL_UX_EXIT_PX         BIT(27)



/* Global USB2 PHY Configuration Register */
#define UX_DCD_GUSB2PHYCFG_PHYSOFTRST             BIT(31)
#define UX_DCD_GUSB2PHYCFG_U2_FREECLK_EXISTS      BIT(30)
#define UX_DCD_GUSB2PHYCFG_SUSPHY                 BIT(6)
#define UX_DCD_GUSB2PHYCFG_ULPI_UTMI              BIT(4)
#define UX_DCD_GUSB2PHYCFG_ENBLSLPM               BIT(8)
#define UX_DCD_GUSB2PHYCFG_PHYIF(n)               (n << 3)
#define UX_DCD_GUSB2PHYCFG_PHYIF_MASK             UX_DCD_GUSB2PHYCFG_PHYIF(1)
#define UX_DCD_GUSB2PHYCFG_USBTRDTIM(n)           (n << 10)
#define UX_DCD_GUSB2PHYCFG_USBTRDTIM_MASK         UX_DCD_GUSB2PHYCFG_USBTRDTIM(0xf)
#define USBTRDTIM_UTMI_8_BIT            9
#define USBTRDTIM_UTMI_16_BIT           5
#define UTMI_PHYIF_16_BIT               1
#define UTMI_PHYIF_8_BIT                0

/* Device Configuration Register */
#define UX_DCD_DWC3_DCFG_DEVADDR                  0x00480800
/* Device Configuration Register */
#define UX_DCD_DWC3_DCFG_DEVADDR_RESET(addr)      ((ULONG)(addr) << 3)
#define UX_DCD_DWC3_DCFG_DEVADDR_MASK             UX_DCD_DWC3_DCFG_DEVADDR_RESET(0x7f)
#define UX_DCD_DWC3_DCFG_DEVADDR_SET(addr)        ((ULONG)(addr) << 3)


/* Device control register configuration for connection change. */
#define UX_DCD_DWC3_DCTL_CONNECT                  0x80F00A00

/* TRB Length, PCM and Status */
#define UX_DCD_DWC3_TRB_SIZE_MASK      (0x00ffffff)
#define UX_DCD_DWC3_TRB_SIZE_LENGTH(n) ((n) & UX_DCD_DWC3_TRB_SIZE_MASK)
#define UX_DCD_DWC3_TRB_SIZE_PCM1(n)   (((n) & 0x03) << 24)
#define UX_DCD_DWC3_TRB_SIZE_TRBSTS(n) (((n) & (0x0f << 28)) >> 28)

#define UX_DCD_DWC3_TRBSTS_OK                  0
#define UX_DCD_DWC3_TRBSTS_MISSED_ISOC         1
#define UX_DCD_DWC3_TRBSTS_SETUP_PENDING       2
#define UX_DCD_DWC3_TRB_STS_XFER_IN_PROG       4

/* TRB Control */
#define UX_DCD_DWC3_TRB_CTRL_HWO            (0x00000001U << 0U)
#define UX_DCD_DWC3_TRB_CTRL_LST            (0x00000001U << 1U)
#define UX_DCD_DWC3_TRB_CTRL_CHN            (0x00000001U << 2U)
#define UX_DCD_DWC3_TRB_CTRL_CSP            (0x00000001U << 3U)
#define UX_DCD_DWC3_TRB_CTRL_TRBCTL(n)      (((n) & 0x3FU) << 4U)
#define UX_DCD_DWC3_TRB_CTRL_ISP_IMI        0x00000400U /* bit 10 */
#define UX_DCD_DWC3_TRB_CTRL_IOC            0x00000800U /* bit 11 */
#define UX_DCD_DWC3_TRB_CTRL_SID_SOFN(n)    (((n) & (u32)0xFFFFU) << 14U)

#define UX_DCD_DWC3_TRBCTL_TYPE(n)             ((n) & (0x3f << 4))
#define UX_DCD_DWC3_TRBCTL_NORMAL              UX_DCD_DWC3_TRB_CTRL_TRBCTL(1)
#define UX_DCD_DWC3_TRBCTL_CONTROL_SETUP       UX_DCD_DWC3_TRB_CTRL_TRBCTL(2)
#define UX_DCD_DWC3_TRBCTL_CONTROL_STATUS2     UX_DCD_DWC3_TRB_CTRL_TRBCTL(3)
#define UX_DCD_DWC3_TRBCTL_CONTROL_STATUS3     UX_DCD_DWC3_TRB_CTRL_TRBCTL(4)
#define UX_DCD_DWC3_TRBCTL_CONTROL_DATA        UX_DCD_DWC3_TRB_CTRL_TRBCTL(5)
#define UX_DCD_DWC3_TRBCTL_ISOCHRONOUS_FIRST   UX_DCD_DWC3_TRB_CTRL_TRBCTL(6)
#define UX_DCD_DWC3_TRBCTL_ISOCHRONOUS         UX_DCD_DWC3_TRB_CTRL_TRBCTL(7)
#define UX_DCD_DWC3_TRBCTL_LINK_TRB            UX_DCD_DWC3_TRB_CTRL_TRBCTL(8)

/* Define USB DWC3 physical endpoint status definition.  */

#define UX_DCD_DWC3_ED_STATUS_UNUSED                            0
#define UX_DCD_DWC3_ED_STATUS_USED                              1
#define UX_DCD_DWC3_ED_STATUS_TRANSFER                          2
#define UX_DCD_DWC3_ED_STATUS_STALLED                           4

/* USB DCD HWPARAMS3 */
#define UX_DCD_DWC3_NUM_IN_EPS_MASK    (0x1f << 18)
#define UX_DCD_DWC3_NUM_EPS_MASK       (0x3f << 12)
#define UX_DCD_DWC3_NUM_EPS(ep_num)         ((ep_num &              \
                        (UX_DCD_DWC3_NUM_EPS_MASK)) >> 12)
#define UX_DCD_DWC3_IN_EPS(p)      ((p &              \
                        (UX_DCD_DWC3_NUM_IN_EPS_MASK)) >> 18)



/* Define USB DWC3 endpoint transfer status definition.  */

#define UX_DCD_DWC3_ED_TRANSFER_STATUS_IDLE                     0
#define UX_DCD_DWC3_ED_TRANSFER_STATUS_SETUP                    1
#define UX_DCD_DWC3_ED_TRANSFER_STATUS_IN_COMPLETION            2
#define UX_DCD_DWC3_ED_TRANSFER_STATUS_OUT_COMPLETION           3

#define UX_DCD_DWC3_ED_NUM_MASK                0x3E

#define UX_DCD_DWC3_DEPEVT_XFERCOMPLETE        0x01
#define UX_DCD_DWC3_DEPEVT_XFERINPROGRESS      0x02
#define UX_DCD_DWC3_DEPEVT_XFERNOTREADY        0x03
#define UX_DCD_DWC3_DEPEVT_RXTXFIFOEVT         0x04
#define UX_DCD_DWC3_DEPEVT_STREAMEVT           0x06
#define UX_DCD_DWC3_DEPEVT_EPCMDCMPLT          0x07

/* For Command Complete Events */
#define DEPEVT_PARAMETER_CMD(n) (((n) & (0xf << 8)) >> 8)

#define UX_DCD_DWC3_ED_STATE_IDLE                              0
#define UX_DCD_DWC3_ED_STATE_DATA_TX                           1
#define UX_DCD_DWC3_ED_STATE_DATA_RX                           2
#define UX_DCD_DWC3_ED_STATE_STATUS_TX                         3
#define UX_DCD_DWC3_ED_STATE_STATUS_RX                         4

/* Global RX Threshold Configuration Register */
#define UX_DCD_DWC3_GRXTHRCFG_MAXRXBURSTSIZE(n) (((n) & 0x1f) << 19)
#define UX_DCD_DWC3_GRXTHRCFG_RXPKTCNT(n) (((n) & 0xf) << 24)
#define UX_DCD_DWC3_GRXTHRCFG_PKTCNTSEL BIT(29)


/* Global HWPARAMS7 Register */
#define UX_DCD_DWC3_GHWPARAMS7_RAM1_DEPTH(n)   ((n) & 0xffff)
#define UX_DCD_DWC3_GHWPARAMS7_RAM2_DEPTH(n)   (((n) >> 16) & 0xffff)
#define DWC3_GHWPARAMS0_MDWIDTH(n)      (((n) >> 8) & 0xff)

#define UX_DCD_DWC3_DCFG_NUMP_SHIFT    17
#define UX_DCD_DWC3_DCFG_NUMP(n)       (((n) >> UX_DCD_DWC3_DCFG_NUMP_SHIFT) & 0x1f)
#define UX_DCD_DWC3_DCFG_NUMP_MASK     (0x1f << UX_DCD_DWC3_DCFG_NUMP_SHIFT)
#define UX_DCD_DWC3_DCFG_LPM_CAP       BIT(22)


#define min_t(type, a, b) MIN(((type) a), ((type) b))

#define IS_ALIGNED(x, a)        (((x) & ((ULONG)(a) - 1U)) == 0U)

#define roundup(x, y) (((((x) + (ULONG)(y - 1U)) / (ULONG)y) * (ULONG)y))

/* DEPCFG parameter 0 */
#define UX_DCD_DEPCFG_EP_TYPE(n)               ((n) << 1U)
#define UX_DCD_DEPCFG_MAX_PACKET_SIZE(n)       ((n) << 3U)
#define UX_DCD_DEPCFG_FIFO_NUMBER(n)           (((n) & 0x1f) << 17)
#define UX_DCD_DEPCFG_BURST_SIZE(n)            ((n) << 22U)
#define UX_DCD_DEPCFG_DATA_SEQ_NUM(n)          ((n) << 26U)

#define UX_DCD_DEPCFG_INT_NUM(n)               ((n) << 0U)
#define UX_DCD_DEPCFG_XFER_COMPLETE_EN         0x00000100U /* bit 8 */
#define UX_DCD_DEPCFG_XFER_IN_PROGRESS_EN      0x00000200U /* bit 9 */
#define UX_DCD_DEPCFG_XFER_NOT_READY_EN        0x00000400U /* bit 10 */
#define UX_DCD_DEPCFG_FIFO_ERROR_EN            0x00000800U /* bit 11 */
#define UX_DCD_DEPCFG_STREAM_EVENT_EN          0x00002000U /* bit 13 */
#define UX_DCD_DEPCFG_BINTERVAL_M1(n)          ((n) << 16U)
#define UX_DCD_DEPCFG_STREAM_CAPABLE           0x01000000U /* bit 24 */
#define UX_DCD_DEPCFG_EP_NUMBER(n)             ((n) << 25U)
#define UX_DCD_DEPCFG_BULK_BASED               0x40000000U /* bit 30 */
#define UX_DCD_DEPCFG_FIFO_BASED               0x80000000U /* bit 31 */

#define UX_DCD_EP_DIR_IN               1U
#define UX_DCD_EP_DIR_OUT              0U

#define UX_DCD_NUM_TRBS                8U

#define UX_DCD_EVENT_PENDING           (0x00000001U << 0U)


#define UX_DCD_EP_BUSY                 (0x00000001U << 4U)

#define UX_DCD_EP_MISSED_ISOC          (0x00000001U << 6U)

#define UX_DCD_ENDPOINTS_NUM                   8U
#define UX_DCD_EVT_EPSTATUS_MASK               12U

#define UX_DCD_ENDPOINT_XFERTYPE_MASK      0x03    /* in bmAttributes */
#define UX_DCD_ENDPOINT_XFER_CONTROL       0U
#define UX_DCD_ENDPOINT_XFER_ISOC          1U
#define UX_DCD_ENDPOINT_XFER_BULK          2U
#define UX_DCD_ENDPOINT_XFER_INT           3U
#define UX_DCD_ENDPOINT_MAX_ADJUSTABLE     0x80

/* Control-only Status */
#define DEPEVT_STATUS_CONTROL_DATA              1U
#define DEPEVT_STATUS_CONTROL_STATUS            2U
#define DEPEVT_STATUS_CONTROL_DATA_INVALTRB     9U
#define DEPEVT_STATUS_CONTROL_STATUS_INVALTRB   0xAU


#define UX_DCD_DSTS_USBLNKST_MASK         (0x0f << 18)
#define UX_DCD_DSTS_USBLNKST(n)           (((n) & UX_DCD_DSTS_USBLNKST_MASK) >> 18)
#define UX_DCD_DSTS_DEVCTRLHLT              BIT(22)


/*
 * return Physical EP number as dwc3 mapping
 */
#define UX_DCD_PhysicalEp(epnum, direction)    (((epnum) << 1U ) | (direction))

#define UX_DCD_DEPCMD_CMD(x)              ((x) & 0xf)

#define UX_DCD_DCTL_ULSTCHNGREQ_MASK      (0x0f << 5)
#define UX_DCD_DCTL_ULSTCHNGREQ(n) (((n) << 5) & UX_DCD_DCTL_ULSTCHNGREQ_MASK)

#define UX_DCD_DCTL_L1_HIBER_EN                0x00040000U /* bit 18 */
#define UX_DCD_DCTL_HIRD_THRES_MASK       (0x1f << 24)
#define UX_DCD_DCTL_HIRD_THRES(n) ((n) << 24)

#define UX_DCD_DCFG_SPEED_MASK                                 7U
#define UX_DCD_DCFG_HIGHSPEED                                  0U
#define UX_DCD_DCFG_LOWSPEED                                   1U

/* Global HWPARAMS1 Register */
#define UX_DCD_GHWPARAMS1_EN_PWROPT(n)         (((ULONG)(n) & ((ULONG)3 << 24U)) >> 24U)
#define UX_DCD_GHWPARAMS1_EN_PWROPT_NO         0U
#define UX_DCD_GHWPARAMS1_EN_PWROPT_CLK        1U
#define UX_DCD_GHWPARAMS1_EN_PWROPT_HIB        2U
#define UX_DCD_GHWPARAMS1_PWROPT(n)            ((ULONG)(n) << 24U)
#define UX_DCD_GHWPARAMS1_PWROPT_MASK          UX_DCD_GHWPARAMS1_PWROPT(3U)

/* These apply for core versions 1.94a and later */
#define UX_DCD_DGCMD_SET_SCRATCHPAD_ADDR_LO       0x04
#define UX_DCD_DGCMD_SET_SCRATCHPAD_ADDR_HI       0x05
#define DCD_DWC3_EVT_BUFF                         4096
enum ux_dcd_link_state {
        /* In SuperSpeed */
        UX_DCD_LINK_STATE_U0              = 0x00, /* in HS, means ON */
        UX_DCD_LINK_STATE_U1              = 0x01,
        UX_DCD_LINK_STATE_U2              = 0x02, /* in HS, means SLEEP */
        UX_DCD_LINK_STATE_U3              = 0x03, /* in HS, means SUSPEND */
        UX_DCD_LINK_STATE_SS_DIS          = 0x04,
        UX_DCD_LINK_STATE_RX_DET          = 0x05, /* in HS, means Early Suspend */
        UX_DCD_LINK_STATE_SS_INACT        = 0x06,
        UX_DCD_LINK_STATE_POLL            = 0x07,
        UX_DCD_LINK_STATE_RECOV           = 0x08,
        UX_DCD_LINK_STATE_HRESET          = 0x09,
        UX_DCD_LINK_STATE_CMPLY           = 0x0a,
        UX_DCD_LINK_STATE_LPBK            = 0x0b,
        UX_DCD_LINK_STATE_RESET           = 0x0e,
        UX_DCD_LINK_STATE_RESUME          = 0x0f,
        UX_DCD_LINK_STATE_MASK            = 0x0f,
};
/* USB Bulk IN  and BULK OUT endpoint, Disconnect event  */
typedef enum
{
    UX_BULKIN_EVENT       = (1 << 0),
    UX_BULKOUT_EVENT      = (1 << 1),
    UX_DISCONNECT_EVENT   = (1 << 2)

}UX_USB_BULK_IN_BULK_OUT_EVENTS;

/*
 * Endpoint Parameters
 */
typedef struct UX_DCD_EP_PARAMS_STRUCT {
        ULONG     Param2;         /**< Parameter 2 */
        ULONG     Param1;         /**< Parameter 1 */
        ULONG     Param0;         /**< Parameter 0 */
} UX_DCD_EP_PARAMS;

#define NO_OF_TRB_PER_EP		8

typedef struct UX_DCD_EP_TRB_STRUCT {
	ULONG             BufferPtrLow;
        ULONG             BufferPtrHigh;
        ULONG             Size;
        ULONG             Ctrl;
} UX_DCD_EP_TRB;

typedef struct UX_DCD_BULK_EP_STRUCT {
	UCHAR array[512];
} UX_DCD_BULK_EP;

typedef struct UX_DCD_CTRL_EP_STRUCT {
	UCHAR array[64];
} UX_DCD_CTRL_EP;

/* Define USB DWC3 physical endpoint structure.  */
typedef struct UX_DCD_DWC3_ED_STRUCT
{

  void (*Handler)(void *, ULONG, ULONG, UCHAR *, ULONG);
                                                /** < User handler called
                                                 *   when data is sent for IN Ep
                                                 *   and received for OUT Ep
                                                 */
    struct UX_DCD_EP_TRB_STRUCT
			EpTrb[NO_OF_TRB_PER_EP + 1U] ALIGNMENT_CACHELINE;

    ULONG           ux_dcd_dwc3_ed_status;
    ULONG           ux_dcd_dwc3_ed_state;
    ULONG           ux_dcd_dwc3_ed_index;
    ULONG           ux_dcd_dwc3_ed_payload_length;
    ULONG           ux_dcd_dwc3_ed_transfer_status;
    ULONG           ux_dcd_dwc3_ed_ping_pong;
    ULONG           ux_dcd_dwc3_ed_type;
    ULONG           ux_dcd_dwc3_ed_direction;
    ULONG	    ux_dcd_dwc3_ed_address;
    ULONG	    ux_dcd_dwc3_ed_maxpacket;
    ULONG	    ux_dcd_dwc3_ed_resource_index;
    ULONG	    ux_dcd_dwc3_ed_requested_bytes;
    ULONG           trb_enqueue;
    ULONG           trb_dequeue;
    ULONG	    endp_flag;
    ULONG	    Phy_EPNum;
    ULONG	    CurUf;
    ULONG	    *BufferPtr;
    ULONG	    BytesTxed;
    ULONG	    UnalignedTx;
    struct UX_SLAVE_ENDPOINT_STRUCT
                    *ux_dcd_dwc3_ed_endpoint;
     /** Each pipe to have its own transfer request semaphore */
     TX_SEMAPHORE    ux_dcd_dwc3_ep_slave_transfer_request_semaphore;

} UX_DCD_DWC3_ED;


enum UX_DCD_DEVICE_STATE {
        /* NOTATTACHED isn't in the USB spec, and this state acts
         * the same as ATTACHED ... but it's clearer this way.
         */
        USB_STATE_NOTATTACHED = 0,

        /* chapter 9 and authentication (wireless) device states */
        USB_STATE_ATTACHED,
        USB_STATE_POWERED,                      /* wired */
        USB_STATE_RECONNECTING,                 /* auth */
        USB_STATE_UNAUTHENTICATED,              /* auth */
        USB_STATE_DEFAULT,                      /* limited function */
        USB_STATE_ADDRESS,
        USB_STATE_CONFIGURED,                   /* most functions */

        USB_STATE_SUSPENDED

        /* NOTE:  there are actually four different SUSPENDED
         * states, returning to POWERED, DEFAULT, ADDRESS, or
         * CONFIGURED respectively when SOF tokens flow again.
         * At this level there's no difference between L1 and L2
         * suspend states.  (L2 being original USB 1.1 suspend.)
         */
};



enum UX_DCD_EP0_STATE {
        EP0_UNCONNECTED         = 0,
        EP0_SETUP_PHASE,
        EP0_DATA_PHASE,
        EP0_STATUS_PHASE,
};

enum UX_DCD_EP0_NEXT {
        EP0_UNKNOWN = 0,
        EP0_COMPLETE,
        EP0_NRDY_DATA,
        EP0_NRDY_STATUS,
};

typedef struct UX_DCD_EP0_TRB_STRUCT {
        ULONG           bpl;
        ULONG           bph;
        ULONG           size;
        ULONG           ctrl;
} UX_DC_EP0_TRB;

typedef struct USB_CTRL_REQUEST {
        UCHAR bRequestType;
        UCHAR bRequest;
        USHORT wValue;
        USHORT wIndex;
        USHORT wLength;
} UX_CTRL_REQUEST;

typedef struct USB_LINE_CODING {
        ULONG BaudRate;
        UCHAR StopBit;
        UCHAR Parity;
        UCHAR DataBit;
} UX_LINE_CODING;


/* Define USB DWC3 DCD Event structure definition. */

typedef struct UX_DCD_DWC3_EVENT_BUFFER {
        VOID                    *buf;
        VOID                    *cache;
        ULONG                   length;
        ULONG                   lpos;
        ULONG                   count;
        ULONG                   flags;
        struct UX_DCD_DWC3_STRUCT   *dcd_dwc3;
} UX_DCD_EVENT;

typedef struct UX_DCD_DWC3_LINE_CODING {
      UCHAR line_set[7];
} UX_DCD_LINE;

#define MIN(a,b) (((a)<(b))?(a):(b))

#define QUEUE_SIZE              8
/* Define USB DWC3 DCD structure definition.  */

typedef struct UX_DCD_DWC3_STRUCT
{

    struct USB_CTRL_REQUEST         SetupData ALIGNMENT_CACHELINE;
    struct UX_DCD_EP_TRB_STRUCT     endp0_trb ALIGNMENT_CACHELINE;
    struct UX_DCD_EP_TRB_STRUCT     endp0_trb_unaligned[2];
    struct UX_DCD_DWC3_ED_STRUCT    eps[UX_DCD_ENDPOINTS_NUM]; /**< Endpoints */
    struct UX_DCD_BULK_EP_STRUCT    BulkData[NO_OF_TRB_PER_EP] ALIGNMENT_CACHELINE;

    struct UX_DCD_CTRL_EP_STRUCT    CtrlData ALIGNMENT_CACHELINE;
    struct UX_SLAVE_DCD_STRUCT      *ux_dcd_dwc3_dcd_owner;
    struct UX_DCD_DWC3_ED_STRUCT     ux_dcd_dwc3_ed[UX_DCD_ENDPOINTS_NUM];
    struct UX_DCD_DWC3_EVENT_BUFFER  *ux_dcd_dwc3_event;
    struct UX_DCD_EP0_TRB_STRUCT     *ep0_trb;
    struct UX_DCD_EP_PARAMS_STRUCT    EpParams;
    struct USB_LINE_CODING           LineCoding ALIGNMENT_CACHELINE;
    enum UX_DCD_EP0_STATE            ep0state;
    enum UX_DCD_DEVICE_STATE    config_state;
    enum ux_dcd_link_state      link_state;
    ULONG       NumInEps;
    ULONG       NumBytes;
    ULONG       ConnectionDone;
    ULONG       NumOutEps;
    ULONG       IsConfigDone;
    ULONG       total_trb;
    ULONG       endp_flag;
    ULONG       TrbNum;
    ULONG       transfer_status;
    ULONG           ux_dcd_dwc3_base;
    ULONG           *ux_dcd_dwc3_debug;
    ULONG           event_lpos;
    ULONG           event_count;
    ULONG           endp_config;
    ULONG           *BufferPtr;
    ULONG           application_read;
    TX_SEMAPHORE    ux_dcd_dwc3_ep_slave_transfer_request_semaphore;
    ULONG           setup_packet_pending:1;
    ULONG       connected:1;
    ULONG       test_mode:1;
    ULONG       event_flag:1;
    ULONG       three_stage_setup:1;
    ULONG       ep0_expect_in:1;
    ULONG       IsocTrans:1;
} UX_DCD_DWC3;



/* Define USB DWC3 DCD prototypes.  */

VOID    _ux_dcd_dwc3_controller_config_check(UX_DCD_DWC3 *dcd_dwc3);
VOID    _ux_dcd_dwc3_controller_config_reset(UX_DCD_DWC3 *dcd_dwc3);
UINT    _ux_dcd_dwc3_check_devt_event_type(ULONG dwc3_register);
ULONG   _ux_dcd_dwc3_check_depevt_event_type(ULONG dwc3_register);
UINT    _ux_dcd_dwc3_get_endpoint_num(ULONG dwc3_register);
UINT    _ux_dcd_dwc3_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter);
VOID    _ux_dcd_dwc3_register_clear(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register, ULONG value);
UINT    _ux_dcd_dwc3_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter);
UINT    _ux_dcd_dwc3_endpoint_create(UX_DCD_DWC3 *dcd_dwc3, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_dwc3_transfer_callback(UX_DCD_DWC3 *dcd_dwc3, UX_SLAVE_TRANSFER *transfer_request);

UINT    _ux_dcd_dwc3_bulk_transfer_callback(UX_DCD_DWC3 *dcd_dwc3, UX_SLAVE_TRANSFER *transfer_request);

UINT    _ux_dcd_Ep0DataDone(UX_DCD_DWC3 *dcd_dwc3, ULONG endpoint_number, UX_SLAVE_TRANSFER *transfer_request);
UINT    _ux_dcd_Ep0StartStatus(UX_DCD_DWC3 *dcd_dwc3, ULONG endpoint_number);
UINT    _ux_dcd_Ep0StatusDone(UX_DCD_DWC3 *dcd_dwc3);


UINT    _ux_dcd_dwc3_depcmd_check(UX_DCD_DWC3 *dcd_dwc3);

UINT  _ux_dcd_dwc3_fifo_read(UX_DCD_DWC3 *dcd_dwc3, UCHAR *data_pointer, ULONG fifo_length);
UINT  _ux_dcd_dwc3_fifo_fill_stack(UX_DCD_DWC3 *dcd_dwc3, UCHAR *data_pointer, ULONG fifo_length);

VOID    _ux_dcd_dwc3_register_set(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register, ULONG value);
VOID    _ux_dcd_dwc3_enable_control_endpoint(UX_DCD_DWC3 *dcd_dwc3, ULONG size);
UINT    _ux_dcd_RecvSetup(UX_DCD_DWC3 *dcd_dwc3);

VOID _ux_dcd_EventBufferHandler(UX_DCD_DWC3 *dcd_dwc3);

UINT    _ux_dcd_EpEnable(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir,
                        ULONG Maxsize, ULONG Type, ULONG Restore);
UINT    _ux_dcd_SetEpConfig(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir,
                                                ULONG Size, ULONG Type, ULONG Restore);
UINT    _ux_dcd_SendEpCmd(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir,
                                  ULONG Cmd, UX_DCD_EP_PARAMS Params);

UINT _ux_dcd_EpGetTransferIndex(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum,
                                                                ULONG Dir);
UINT _ux_dcd_SetXferResource(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir);

UINT   _ux_dcd_StartEpConfig(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir);

UINT EpBufferSend(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEp,
                        ULONG *BufferPtr, ULONG BufferLen);


UINT _ux_dcd_RecvBulkData(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEp,
                        ULONG *BufferPtr, ULONG BufferLen);


UINT _ux_dcd_EpBufferSend(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEp,
                        ULONG *BufferPtr, ULONG BufferLen);

VOID _ux_dcd_SetEpHandler(UX_DCD_DWC3 *dcd_dwc3, ULONG Epnum,
                        ULONG Dir, VOID (*Handler)(VOID *, ULONG, ULONG, UCHAR *,
                        ULONG));

VOID BulkOutHandler(VOID *CallBackRef, ULONG RequestedBytes, ULONG BytesTxed,
                     UCHAR *transfer_request, ULONG TrbNum);

VOID BulkInHandler(VOID *CallBackRef, ULONG RequestedBytes, ULONG BytesTxed,
             UCHAR *transfer_request, ULONG TrbNum);

UINT _ux_dcd_SetDeviceAddress(UX_DCD_DWC3 *dcd_dwc3, ULONG  Addr);


UINT _ux_dcd_dwc3_SetConfiguration(UX_DCD_DWC3 *dcd_dwc3, ULONG Parameter);

UINT _ux_dcd_Ep0Send(UX_DCD_DWC3 *dcd_dwc3, ULONG *BufferPtr, ULONG BufferLen);

UINT _ux_dcd_transfer_wakeup(UX_DCD_DWC3 * dcd_dwc3);

UINT ux_dcd_gadget_set_link_state(UX_DCD_DWC3 *dcd_dwc3, enum ux_dcd_link_state state);

VOID ux_dcd_StopActiveTransfers(UX_DCD_DWC3 *dcd_dwc3);
VOID ux_dcd_ClearStallAllEp(UX_DCD_DWC3 *dcd_dwc3);
VOID ux_dcd_StopTransfer(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum,
                        ULONG Dir, ULONG Force);
VOID ux_dcd_EpClearStall(UX_DCD_DWC3 *dcd_dwc3, ULONG Epnum, ULONG Dir);

VOID _ux_dcd_Ep0_EndControlData(UX_DCD_DWC3 *dcd_dwc3, UX_DCD_DWC3_ED  *Ept);

VOID _ux_dcd_Ep0StallRestart(UX_DCD_DWC3 *dcd_dwc3);

VOID _ux_dcd_EpSetStall(UX_DCD_DWC3 *dcd_dwc3, ULONG Epnum, ULONG Dir);

VOID _ux_dcd_set_speed(UX_DCD_DWC3 *dcd_dwc3);

UINT _ux_dcd_SetupScratchpad(UX_DCD_DWC3 *dcd_dwc3, ULONG ScratchBufF);

UINT _ux_dcd_SendGadgetGenericCmd(UX_DCD_DWC3 *dcd_dwc3, ULONG cmd,
                        ULONG param);

UINT  _ux_dcd_dwc3_state_change(UX_DCD_DWC3 *dcd_dwc3, ULONG state);

UINT EpBufferRecv(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEp,
                                UCHAR *BufferPtr, ULONG Length);
UINT _ux_dcd_EpBufferRecv(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEp,
                                UCHAR *BufferPtr, ULONG Length);

VOID _ux_dcd_EpXferComplete(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register,
                  ULONG endp_number, UCHAR *transfer_request);

VOID _ux_dcd_EpXferNotReady(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register,
                   ULONG endp_number);


UINT _ux_dcd_Ep0Recv(UX_DCD_DWC3 *dcd_dwc3, ULONG *BufferPtr, ULONG BufferLen);

UINT _ux_dcd_RecvLineCoding(UX_DCD_DWC3 *dcd_dwc3);

UINT _ux_dcd_SendLineCoding(UX_DCD_DWC3 *dcd_dwc3);

VOID  _ux_dcd_dwc3_register_write(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register, ULONG value);
ULONG  _ux_dcd_dwc3_register_read(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register);
VOID    _ux_dcd_dwc3_clock_set(ULONG clock_base, ULONG clock_offset, ULONG value);
UINT  _ux_dcd_dwc3_transfer_request(UX_DCD_DWC3 *dcd_dwc3, UX_SLAVE_TRANSFER *transfer_request);
VOID    _ux_dcd_dwc3_delay(ULONG usec);
UINT  _ux_dcd_dwc3_address_set(UX_DCD_DWC3 *dcd_dwc3, ULONG address);
UINT  _ux_dcd_dwc3_initialize_complete(VOID);
VOID  _ux_dcd_dwc3_gadget_setup_nump(UX_DCD_DWC3 *dcd_dwc3);
VOID _ux_dcd_InitializeEps(UX_DCD_DWC3 *dcd_dwc3);
VOID _ux_dcd_phy_reset(UX_DCD_DWC3 *dcd_dwc3);
VOID  _ux_dcd_dwc3_interrupt_handler(VOID);
VOID _ux_dcd_dwc3_clear_trb(UX_DCD_DWC3 *dcd_dwc3,ULONG endpoint_number);

#ifdef  __cplusplus

}

#endif

#endif

