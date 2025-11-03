/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     demo_mipi_Interface_video_isp_threadx.c
 * @author   Yogender Kumar Arya  and Shivakumar Malke
 * @email    yogender.kumar@alifsemi.com and shivakumar.malke@alifsemi.com
 * @version  V1.0.0
 * @date     15-September-2025
 * @brief    ISP verification application for ARX3A0 camera sensor and ILI9806E LCD panel
 *           using Azure RTOS ThreadX.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
/* System Includes */
#include <stdio.h>
#include "tx_api.h"

#include "board_config.h"

/* Project Includes */

/* Camera Controller Driver */
#include "Driver_CPI.h"

#include "RTE_Device.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif  /* RTE_CMSIS_Compiler_STDOUT */


#if RTE_ISP
#include "Driver_ISP.h"
#include "vsi_comm_video.h"
#endif

/* PINMUX Driver */
#include "pinconf.h"

/* CDC200 Driver*/
#include "Driver_CDC200.h"

/* SE Services */
#include "se_services_port.h"

/* AIPL */
#include "aipl_error.h"

/* Device Header files*/
#include CMSIS_device_header

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

/*CDC200 Driver instance*/
extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

void video_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE             1024
#define DEMO_BYTE_POOL_SIZE         9120

TX_THREAD                           Video_thread;
TX_EVENT_FLAGS_GROUP                event_flags;

#define USE_MT9M114    1
#define USE_ARX3A0     0

#if USE_MT9M114
/* @Note: MT9M114 Camera Sensor configurations
 *        are directly borrowed from MT9M114 Camera Sensor drivers,
 *        for detail refer MT9M114 driver.
 *
 * Selected MT9M114 Camera Sensor configurations:
 *   - Interface     : MIPI CSI2
 *   - Resolution    : 1280X720
 *   - Output Format : RAW Bayer8
 */
#define CAM_FRAME_WIDTH        (1280)
#define CAM_FRAME_HEIGHT       (720)
#elif USE_ARX3A0
/* @Note: ARX3A0 Camera Sensor configurations
 *        are directly borrowed from ARX3A0 Camera Sensor drivers,
 *        for detail refer ARX3A0 driver.
 *
 * Selected ARX3A0 Camera Sensor configurations:
 *   - Interface     : MIPI CSI2
 *   - Resolution    : 560X560
 *   - Output Format : RAW Bayer10
 */
#define CAM_FRAME_WIDTH        (560)
#define CAM_FRAME_HEIGHT       (560)
#else
#error "No camera sensor selected!"
#endif

/* Allocate Camera frame buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

/* pool size for Camera frame buffer:
 *  which will be camera frame width x frame height
 */
#define CAMERA_FRAMEBUFFER_POOL_SIZE   ((CAM_FRAME_WIDTH) * (CAM_FRAME_HEIGHT))

/* pool area for Camera frame buffer.
 *  Allocated in the "camera_frame_buf" section.
 */
uint8_t cam_framebuffer_pool[CAMERA_FRAMEBUFFER_POOL_SIZE] \
        __attribute__((section(".bss.camera_frame_buf")));

/* @Note: ILI9806E LCD Panel configurations
 *        are directly borrowed from ILI9806E LCD Panel drivers,
 *        for detail refer ILI9806E LCD Panel driver.
 *
 * Selected ILI9806E LCD Panel configurations:
 *   - Interface     : MIPI DSI
 *   - Resolution    : 480X800
 *   - Input Format  : RGB888
 */

/* ILI9806E LCD Panel Resolution. */
#define ILI9806E_Panel_RESOLUTION_480x800          0
#define ILI9806E_Panel_RESOLUTION                  ILI9806E_Panel_RESOLUTION_480x800

#if (ILI9806E_Panel_RESOLUTION == ILI9806E_Panel_RESOLUTION_480x800)
#define LCD_FRAME_WIDTH        (480)
#define LCD_FRAME_HEIGHT       (800)
#endif

/*Number of bytes per pixel in RGB888*/
#define RGB_BYTES_PER_PIXEL 3

/* Allocate LCD Panel buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

/* pool size for LCD Panel buffer:
 *  which will be LCD frame width x frame height
 */
#define LCD_FRAMEBUFFER_POOL_SIZE   ((LCD_FRAME_WIDTH) * (LCD_FRAME_HEIGHT) * RGB_BYTES_PER_PIXEL)

/* pool area for LCD panel frame buffer.
 *  Allocated in the "lcd_frame_buf" section.
 */
uint8_t lcd_framebuffer_pool[LCD_FRAMEBUFFER_POOL_SIZE] \
        __attribute__((section(".bss.lcd_frame_buf")));

/* Required to Crop and interpolate the captured image data format to
 * LCD Panel supported image format.
 *
 *  - for ARX3A0 Camera sensor,
 *     it gives frame resolution as 560x560
 *  - for ILI9806E LCD Panel,
 *     it supports resolution 480x800
 *  - in-order to display the camera image on LCD panel,
 *     we have scaled down the image to 480x480.
 */
#if (ILI9806E_Panel_RESOLUTION == ILI9806E_Panel_RESOLUTION_480x800)
#define CRP_FRAME_WIDTH        (480)
#define CRP_FRAME_HEIGHT       (480)
#endif

enum {
    ISP_PLANAR = 1,
    ISP_SEMIPLANAR,
    ISP_INTERLEAVED,
    ISP_NONE,
};

/* ISP Output image width */
#define ISP_OUTPUT_X    480

/* ISP output image height */
#define ISP_OUTPUT_Y    480

#if (RTE_ISP_OUTPUT_FORMAT == 20)   // RAW8 output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_AUX_BUFFER_TYPE  ISP_NONE
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y)

#elif (RTE_ISP_OUTPUT_FORMAT == 21) // RAW10 output from ISP
#define ISP_OUTPUT_SIZE_Y  ((ISP_OUTPUT_X * ISP_OUTPUT_Y * 5) >> 2)
#define ISP_AUX_BUFFER_TYPE  ISP_NONE
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y)

#elif (RTE_ISP_OUTPUT_FORMAT == 22) // RAW12 output from ISP
#define ISP_OUTPUT_SIZE_Y  ((ISP_OUTPUT_X * ISP_OUTPUT_Y * 12) >> 3)
#define ISP_AUX_BUFFER_TYPE  ISP_NONE
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y)

#elif (RTE_ISP_OUTPUT_FORMAT == 23) // NV12 output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_AUX_BUFFER_TYPE  ISP_SEMIPLANAR
#define ISP_OUTPUT_SIZE_CBCR  ((ISP_OUTPUT_X * ISP_OUTPUT_Y) >>1 )
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y + ISP_OUTPUT_SIZE_CBCR)

#elif (RTE_ISP_OUTPUT_FORMAT == 25) // NV16 output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_AUX_BUFFER_TYPE  ISP_SEMIPLANAR
#define ISP_OUTPUT_SIZE_CBCR  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y + ISP_OUTPUT_SIZE_CBCR)

#elif (RTE_ISP_OUTPUT_FORMAT == 30) // YUV422P output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_AUX_BUFFER_TYPE  ISP_PLANAR
#define ISP_OUTPUT_SIZE_CB  ((ISP_OUTPUT_X * ISP_OUTPUT_Y) >> 1)
#define ISP_OUTPUT_SIZE_CR  ((ISP_OUTPUT_X * ISP_OUTPUT_Y) >> 1)
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y + ISP_OUTPUT_SIZE_CB + ISP_OUTPUT_SIZE_CR)

#elif (RTE_ISP_OUTPUT_FORMAT == 31) // YUV420P output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_AUX_BUFFER_TYPE  ISP_PLANAR
#define ISP_OUTPUT_SIZE_CB  ((ISP_OUTPUT_X * ISP_OUTPUT_Y) >> 2)
#define ISP_OUTPUT_SIZE_CR  ((ISP_OUTPUT_X * ISP_OUTPUT_Y) >> 2)
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y + ISP_OUTPUT_SIZE_CB + ISP_OUTPUT_SIZE_CR)

#elif (RTE_ISP_OUTPUT_FORMAT == 32) // YUYV (YUV422 packed) output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y * 2)
#define ISP_PITCH          ISP_OUTPUT_X
#define ISP_AUX_BUFFER_TYPE  ISP_INTERLEAVED
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y)

#elif (RTE_ISP_OUTPUT_FORMAT == 37) // YUV400 (Grayscale) output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_AUX_BUFFER_TYPE  ISP_NONE
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y)

#elif (RTE_ISP_OUTPUT_FORMAT == 38) // RGB888 Interleaved output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y * 3)
#define ISP_AUX_BUFFER_TYPE  ISP_INTERLEAVED
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y)

#elif (RTE_ISP_OUTPUT_FORMAT == 39) // RGB888 planar output from ISP
#define ISP_OUTPUT_SIZE_Y  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_AUX_BUFFER_TYPE  ISP_PLANAR
#define ISP_OUTPUT_SIZE_CB  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_OUTPUT_SIZE_CR  (ISP_OUTPUT_X * ISP_OUTPUT_Y)
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y + ISP_OUTPUT_SIZE_CB + ISP_OUTPUT_SIZE_CR)
#endif

#if (ISP_OUTPUT_SIZE_Y)
uint8_t y_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_Y] \
    __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));
#endif

#if (ISP_OUTPUT_SIZE_CB)
uint8_t cb_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_CB] \
    __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));
#endif

#if (ISP_OUTPUT_SIZE_CR)
uint8_t cr_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_CR] \
    __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));
#endif

#if (ISP_OUTPUT_SIZE_CBCR)
uint8_t cbcr_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_CBCR] \
    __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));
#endif

VIDEO_BUF_S buffer_array[RTE_ISP_BUFFER_COUNT];

/* AIPL api to convert YUYV to RGB888 interleaved format */
extern aipl_error_t aipl_color_convert_yuy2_to_rgb888_helium(const void* input,
                                                      void* output,
                                                      uint32_t pitch,
                                                      uint32_t width,
                                                      uint32_t height);

/* Camera callback events */
typedef enum
{
    CAM_CB_EVENT_ERROR        = (1 << 0),
    DISP_CB_EVENT_ERROR       = (1 << 1),
    CAM_VSYNC_CB_EVENT        = (1 << 2),
    ISP_VSYNC_CB_EVENT        = (1 << 3),
    ISP_MI_FRAME_DUMP_EVENT   = (1 << 4),
    ISP_CB_EVENT_ERROR        = (1 << 5),
}CB_EVENTS;

volatile uint32_t softreset_interval_counter = 0;

/**
  \fn          void Camera_callback(uint32_t event)
  \brief       Camera isr callback
  \param[in]   event: Camera Event
  \return      none
  */
void Camera_callback(uint32_t event)
{
    if(event & ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED)
    {
        /* Transfer Success: Frame VSYNC detected, Wake-up Thread. */
        tx_event_flags_set(&event_flags, CAM_VSYNC_CB_EVENT, TX_OR);

        if(softreset_interval_counter % 2)
        {
            CAMERAdrv->Control(CPI_SOFTRESET, 0);
        }

        softreset_interval_counter++;
    }
#if (!RTE_ISP)
    if(event & ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN)
    {
        /* Transfer Error: Received input FIFO over-run, Wake-up Thread. */
        tx_event_flags_set(&event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    }

    if(event & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN)
    {
        /* Transfer Error: Received output FIFO over-run, Wake-up Thread. */
        tx_event_flags_set(&event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    }
#endif

    if(event & ARM_CPI_EVENT_MIPI_CSI2_ERROR)
    {
        /* Transfer Error: Frame VSYNC detected, Wake-up Thread. */
        tx_event_flags_set(&event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    }

    if (event & ARM_ISP_EVENT_FRAME_VSYNC_DETECTED)
    {
        /* Transfer Success: Received MIPI CSI2 error, Wake-up Thread. */
        tx_event_flags_set(&event_flags, ISP_VSYNC_CB_EVENT, TX_OR);
    }

    if (event & ARM_ISP_EVENT_DATALOSS_DETECTED)
    {
        /* Transfer Error: Received ISP error, Wake-up Thread. */
        tx_event_flags_set(&event_flags, ISP_CB_EVENT_ERROR, TX_OR);
    }

    if (event & ARM_ISP_MI_EVENT_MP_FRAME_END_DETECTED)
    {
        /* Transfer Success: ISP dumped buffer to memory, Wake-up Thread. */
        tx_event_flags_set(&event_flags, ISP_MI_FRAME_DUMP_EVENT, TX_OR);
    }
}

/**
  \fn          void Display_callback(uint32_t event)
  \brief       Display isr callback
  \param[in]   event: Display Event
  \return      none
  */
void Display_callback(uint32_t event)
{
    if(event & ARM_CDC_DSI_ERROR_EVENT)
    {
        /* Transfer Error: Received Hardware error, Wake-up Thread. */
        tx_event_flags_set(&event_flags, DISP_CB_EVENT_ERROR, TX_OR);
    }
}

/**
  \fn          int i3c_pinmux(void)
  \brief       i3c hardware pin initialization:
                  - PIN-MUX configuration
                  - PIN-PAD configuration
  \param[in]   none
  \return      0:success; -1:failure
  */
int i3c_pinmux(void)
{
    int ret;

    /* Configure GPIO Pin : P7_2 as i2c_sda_c
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_7, PIN_2, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE |
                                                                  PADCTRL_DRIVER_DISABLED_PULL_UP);

    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: i2c PINMUX and PINPAD failed.\r\n");
        return -1;
    }

    /* Configure GPIO Pin : P7_3 as i2c_scl_c
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP

     */
    ret = pinconf_set(PORT_7, PIN_3, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE |
                                                                  PADCTRL_DRIVER_DISABLED_PULL_UP);

    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: i2c PINMUX and PINPAD failed.\r\n");
        return -1;
    }

    return 0;
}

/**
  \fn          int camera_pinmux(void)
  \brief       Camera hardware pin initialization:
                    - PIN-MUX configuration
  \param[in]   none
  \return      0:success; -1:failure
  */
int camera_pinmux()
{
    int ret;

    /* Configure GPIO Pin : P0_3 as CAM_XVCLK_A */
    ret = pinconf_set(PORT_0, PIN_3, PINMUX_ALTERNATE_FUNCTION_6, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Camera Pin-Mux failed.\r\n");
        return -1;
    }

    return 0;
}

/**
  \fn          int hardware_init(void)
  \brief       - i3c hardware pin initialization:
                  - PIN-MUX configuration
                  - PIN-PAD configuration
               - Camera hardware pin initialization:
                  - PIN-MUX configuration
  \param[in]   none
  \return      0:success; -1:failure
  */
int hardware_init(void)
{
    int ret;

    /* i3c pinmux. */
    ret = i3c_pinmux();
    if(ret != 0)
    {
        printf("\r\n Error in i3c pinmux.\r\n");
        return -1;
    }

    /* Camera pinmux. */
    ret = camera_pinmux();
    if(ret != 0)
    {
        printf("\r\n Error in Camera pinmux.\r\n");
        return -1;
    }

    /*MPU setup*/
    MPU_Setup();

    return 0;
}


/**
  \fn          void video_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify ARX3A0 Camera Sensor and ILI9806E LCD Panel
               with Azure RTOS ThreadX as an Operating System.
               This demo thread does:
                   - initialize i3c and Camera hardware pins
                   - initialize CDC200 driver.
                   - initialize Camera driver with Camera Resolution.
                   - captured data will display on LCD Panel.
                   - convert captured image format
                     into RGB888 image format
                   - for ARX3A0 Camera sensor,
                     it gives image in frame resolution 560x560
                   - for ILI9806E LCD Panel,
                     it supports resolution 480x800
                   - in-order to display the camera image on LCD panel,
                     we are cropping and interpolate the captured image
                     to 480x480.
                   - Stream captured image on LCD Panel
  \param[in]   thread_input : thread input
  \return      none
  */
void video_demo_thread_entry(ULONG thread_input)
{
    INT   ret = 0;
    ULONG actual_events = 0;
    ULONG wait_timer_ticks = TX_TIMER_TICKS_PER_SECOND;
    UINT  service_error_code;
    UINT  error_code;

    run_profile_t runp = {0};

    ARM_DRIVER_VERSION version;

    printf("\r\n >>> ARX3A0 Camera Sensor and ILI9806E LCD Panel demo "
            "with Azure RTOS ThreadX is starting up!!! <<< \r\n");

    ret = board_gpios_config();
    if (ret != 0) {
        return;
    }

    for (int i = 0; i < RTE_ISP_BUFFER_COUNT; i++) {
        buffer_array[i].index = i;
        switch (ISP_AUX_BUFFER_TYPE) {
        case ISP_PLANAR:
            buffer_array[i].numPlanes = 3;
            break;
        case ISP_SEMIPLANAR:
            buffer_array[i].numPlanes = 2;
            break;
        case ISP_NONE:
        case ISP_INTERLEAVED:
        default:
            buffer_array[i].numPlanes = 1;
            break;
        }
        buffer_array[i].imageSize = ISP_OUTPUT_TOTAL_SIZE;
        buffer_array[i].planes[0].dmaPhyAddr = (vsi_dma_t)y_buffer[i];
#if ISP_OUTPUT_SIZE_CB
        buffer_array[i].planes[1].dmaPhyAddr = (vsi_dma_t)cb_buffer[i];
#endif
#if ISP_OUTPUT_SIZE_CR
        buffer_array[i].planes[2].dmaPhyAddr = (vsi_dma_t)cr_buffer[i];
#endif
#if ISP_OUTPUT_SIZE_CBCR
        buffer_array[i].planes[1].dmaPhyAddr = (vsi_dma_t)cbcr_buffer[i];
#endif
    }

    printf("\n \t camera frame buffer          pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            CAMERA_FRAMEBUFFER_POOL_SIZE, (uint32_t) cam_framebuffer_pool);

    printf("\n \t lcd frame  buffer           pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            LCD_FRAMEBUFFER_POOL_SIZE, (uint32_t) lcd_framebuffer_pool);


    /* Initialize i3c and Camera hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != 0)
    {
        printf("\r\n Error: CAMERA Hardware Initialize failed.\r\n");
        return;
    }

    /* Initialize the SE services */
    se_services_port_init();

    /* Enable MIPI Clocks */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M,
                                              true, &service_error_code);
    if(error_code != SERVICES_REQ_SUCCESS)
    {
        printf("SE: MIPI 100MHz clock enable = %d\n", error_code);
        return;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC,
                                              true, &service_error_code);
    if(error_code != SERVICES_REQ_SUCCESS)
    {
        printf("SE: MIPI 38.4Mhz(HFOSC) clock enable = %d\n", error_code);
        goto error_disable_100mhz_clk;
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle,
                                      &runp,
                                      &service_error_code);
    if(error_code)
    {
        printf("\r\nSE: get_run_cfg error = %d\n", error_code);
        goto error_disable_hfosc_clk;
    }

    /*
     * Note:
     * This demo uses a specific profile setting that only enables the
     * items it needs. For example, it only requests the RAM regions and
     * peripheral power that are relevant for this demo. If you want to adapt
     * this example for your own use case, you should adjust the profile setting
     * accordingly. You can either add any additional items that you need, or
     * remove the request altogether to use the default setting that turns on
     * almost everything.
     */

    runp.memory_blocks = MRAM_MASK | SRAM0_MASK;
    runp.phy_pwr_gating = MIPI_PLL_DPHY_MASK | MIPI_TX_DPHY_MASK | MIPI_RX_DPHY_MASK | LDO_PHY_MASK;

    /* Set the new run configuration */
    error_code = SERVICES_set_run_cfg(se_services_s_handle,
                                      &runp,
                                      &service_error_code);
    if(error_code)
    {
        printf("\r\nSE: set_run_cfg error = %d\n", error_code);
        goto error_disable_hfosc_clk;
    }

    version = CAMERAdrv->GetVersion();
    printf("\r\n Camera driver version api:0x%X driver:0x%X \r\n",version.api, version.drv);

    version = CDCdrv->GetVersion();
    printf("\r\n CDC driver version api:0x%X driver:0x%X \r\n",version.api, version.drv);

    /* Initialize Display Driver CDC200*/
    ret = CDCdrv->Initialize(Display_callback);
    if(ret != 0)
    {
        printf("\r\n Error: CDC200 Initialize failed.\r\n");
        return;
    }

    /* Initialize Camera Driver*/
    ret = CAMERAdrv->Initialize(Camera_callback);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA Initialize failed.\r\n");
        goto error_uninitialize_cdc;
    }

    /* Power on CDC200 peripheral*/
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != 0)
    {
        printf("\r\n Error: CDC200 Power Up failed.\r\n");
        goto error_uninitialize_camera;
    }

    /* Power up Camera peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA Power Up failed.\r\n");
        goto error_poweroff_cdc;
    }

    /* Configure the Display and set Frame buffer*/
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)(lcd_framebuffer_pool));

    if(ret != 0)
    {
        printf("\r\n Error: CDC200 Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Configure Camera */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        goto error_poweroff_camera;
    }


    /* Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED| \
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN | \
                             ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    for (int i = 0; i < RTE_ISP_BUFFER_COUNT; i++) {
        /* Control configuration for camera events */
        ret = CAMERAdrv->Control(ISP_CONTROL_QBUF, (uint32_t) &buffer_array[i]);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
            goto error_poweroff_camera;
        }
    }
    /* Start Display*/
    ret = CDCdrv->Start();
    if(ret != 0)
    {
        printf("\r\n Error: Starting CDC200 failed.\r\n");
        goto error_poweroff_camera;
    }

    printf("\r\n Let's Start Capturing Camera Frame...\r\n");

    /*Start Camera and capture image in given frame buffer*/
    ret = CAMERAdrv->CaptureVideo(cam_framebuffer_pool);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA Capturing Frame failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Convert the camera captured image into RGB888 and
     * copy it to Display Frame buffer
    */
    for(;;)
    {
        ret = tx_event_flags_get(&event_flags,                         \
                                 CAM_VSYNC_CB_EVENT                 |  \
                                 CAM_CB_EVENT_ERROR                 |  \
                                 DISP_CB_EVENT_ERROR                |  \
                                 ISP_VSYNC_CB_EVENT                 |  \
                                 ISP_MI_FRAME_DUMP_EVENT            |  \
                                 ISP_CB_EVENT_ERROR,                   \
                                 TX_OR_CLEAR,                          \
                                 &actual_events,                       \
                                 wait_timer_ticks);
        if (ret != TX_SUCCESS)
        {
            printf("Error: Camera or Display tx_event_flags_get failed.\n");
            goto error_poweroff_camera;
        }

        if((actual_events & CAM_CB_EVENT_ERROR) || (actual_events & DISP_CB_EVENT_ERROR) ||
           (actual_events & ISP_CB_EVENT_ERROR))
        {
            /* Error: Camera or Display failed. */
            printf("\r\n \t\t >> Error: Camera or Display failed. \r\n");
            goto error_poweroff_camera;
        }

        if (actual_events & ISP_MI_FRAME_DUMP_EVENT) {
            ret = CAMERAdrv->Control(ISP_PROCESS_FRAME_END, 0);
        }

        /* NOTE:
        * The current LCD display plan supports RGB interleaved data format.
        * It is not yet confirmed whether the ISP can output RGB888 in interleaved mode.
        * To ensure compatibility, the AIPL API is used to convert the ISP's YUYV output
        * into RGB888 interleaved format before passing it to the display.
        */
        aipl_color_convert_yuy2_to_rgb888_helium(y_buffer[0],           // ISP's YUYV buffer
                                                lcd_framebuffer_pool,  // LCD framebuffer
                                                ISP_PITCH,
                                                ISP_OUTPUT_X,
                                                ISP_OUTPUT_Y);


    }

error_poweroff_camera:
    /* Power off CAMERA peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA Power OFF failed.\r\n");
    }

error_poweroff_cdc:
    /* Power off CDC200 peripheral */
    ret = CDCdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CDC200 Power OFF failed.\r\n");
    }

error_uninitialize_camera:
    /* Un-initialize CAMERA driver */
    ret = CAMERAdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA Uninitialize failed.\r\n");
    }

error_uninitialize_cdc:
    /* Un-initialize CDC200 driver */
    ret = CDCdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CDC200 Uninitialize failed.\r\n");
    }

error_disable_hfosc_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, false, &service_error_code);
    if(error_code != SERVICES_REQ_SUCCESS)
    {
        printf("SE: MIPI 38.4Mhz(HFOSC)  clock disable = %d\n", error_code);
    }

error_disable_100mhz_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M, false, &service_error_code);
    if(error_code != SERVICES_REQ_SUCCESS)
    {
        printf("SE: MIPI 100MHz clock disable = %d\n", error_code);
    }

    printf("\r\n XXX Camera demo thread is exiting XXX...\r\n");

    /* wait forever */
    while(1);
}

/* Define main entry point.  */
int main()
{
    #if defined(RTE_CMSIS_Compiler_STDOUT)
    int32_t ret;
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
    #endif

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

/* Define what the initial system looks like.  */
void tx_application_define(void *first_unused_memory)
{
    CHAR    *pointer = TX_NULL;
    UINT     status  = 0;

    /* Create the event flags group used by video thread */
    status = tx_event_flags_create(&event_flags, "event flags Camera and display");
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create event flags\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&Video_thread, "Video_thread", video_demo_thread_entry, 0,
            first_unused_memory, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create thread \n");
        goto error_delete_event_flag;
    }

    return; //SUCCESS

error_delete_event_flag:
    status = tx_event_flags_delete(&event_flags);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not delete event flags\n");
    }

    while(1);
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
