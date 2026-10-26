/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     demo_mipi_Interface_video_threadx.c
 * @author   Prasanna Ravi and Chandra Bhushan Singh
 * @email    prasanna.ravi@alifsemi.com and chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     14-Dec-2023
 * @brief    TestApp to verify a MIPI CSI2 Camera Sensor and ILI9806E LCD Panel
 *           with Azure RTOS ThreadX as an Operating System.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
/* System Includes */
#include <stdio.h>
#include "tx_api.h"

#include "RTE_Components.h"
#include "RTE_Device.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

#include "board_config.h"
#include "pinconf.h"

#include "Driver_CPI.h"
#include "Driver_CDC200.h"

#include "se_services_port.h"

/* Device Header files*/
#include CMSIS_device_header

#if defined(RTE_CPI_AXI_PORT)
#if (RTE_CPI_AXI_PORT != 1)
#error "Enable the CPI AXI port in RTE_Device.h"
#endif
#endif

/* AIPL
 *
 * This application requires the AIPL pack. In the RTE configuration
 * (RTE -> Graphics), enable all AIPL-related components before building.
 */
#include "aipl_error.h"
#include "aipl_crop.h"
#include "aipl_demosaic.h"

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
TX_BYTE_POOL                        byte_pool_0;
UCHAR                               memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP                event_flags;

/* Camera Sensor Selection
 * Supports: ARX3A0, MT9M114, OV5675
 *
 * Selection is driven by the RTE components enabled in RTE_Components.h.
 * Enable the corresponding CAMERA_SENSOR driver in the RTE configuration.
 *
 *   - RTE_Drivers_CAMERA_SENSOR_ARX3A0  (560x560,  IPI-16 RAW8)
 *   - RTE_Drivers_CAMERA_SENSOR_MT9M114 (1280x720, IPI-16 RAW8)
 *   - RTE_Drivers_CAMERA_SENSOR_OV5675  (1296x972, IPI-16 RAW8)
 *
 * Note: Sensors output RAW10, but CPI interface default is IPI-16 RAW8.
 *       CPI color mode can be changed via RTE configuration if needed.
 */

#if defined(RTE_Drivers_CAMERA_SENSOR_ARX3A0)
  #define SELECTED_CAMERA_SENSOR     "ARX3A0"
  #define CAM_FRAME_WIDTH            RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH
  #define CAM_FRAME_HEIGHT           RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT

#elif defined(RTE_Drivers_CAMERA_SENSOR_MT9M114)
  #define SELECTED_CAMERA_SENSOR     "MT9M114"
  #define CAM_FRAME_WIDTH            RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_WIDTH
  #define CAM_FRAME_HEIGHT           RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_HEIGHT

#elif defined(RTE_Drivers_CAMERA_SENSOR_OV5675)
  #define SELECTED_CAMERA_SENSOR     "OV5675"
  #define CAM_FRAME_WIDTH            RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH
  #define CAM_FRAME_HEIGHT           RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT

#else
  #error "Enable one RTE_Drivers_CAMERA_SENSOR_* definition"
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

#define LCD_FRAMEBUFFER_ATTR  __attribute__((section(".bss.lcd_frame_buf")))

/* pool size for LCD Panel buffer:
 *  which will be LCD frame width x frame height
 */
#define LCD_FRAMEBUFFER_POOL_SIZE   ((LCD_FRAME_WIDTH) * (LCD_FRAME_HEIGHT) * RGB_BYTES_PER_PIXEL)

/* pool area for LCD panel frame buffer.
 *  Allocated in the "lcd_frame_buf" section.
 */
uint8_t lcd_framebuffer_pool[LCD_FRAMEBUFFER_POOL_SIZE] \
        LCD_FRAMEBUFFER_ATTR;

/* Crop the captured camera image to LCD panel resolution using AIPL crop. */
#if (ILI9806E_Panel_RESOLUTION == ILI9806E_Panel_RESOLUTION_480x800)
#define CRP_FRAME_WIDTH        (480)
#define CRP_FRAME_HEIGHT       (480)
#endif

/* Required convert captured image data format to RGB image format.
 *
 *  Bayer format is converted to RGB using AIPL demosaic.
 */

#define BAYER_TO_RGB_FRAMEBUFFER_ATTR   \
    __attribute__((section(".bss.camera_frame_bayer_to_rgb_buf")))

/* pool size for Camera frame buffer for Bayer to RGB conversion:
 *   frame width x frame height x RGB bytes per pixel.
 */
#define BAYER_TO_RGB_BUFFER_POOL_SIZE   \
    ((CAM_FRAME_WIDTH) * (CAM_FRAME_HEIGHT) * RGB_BYTES_PER_PIXEL)

/* pool area for Camera frame buffer for Bayer to RGB conversion.
 *  Allocated in the "camera_frame_bayer_to_rgb_buf" section.
 */
uint8_t bayer_to_rgb_buffer_pool[BAYER_TO_RGB_BUFFER_POOL_SIZE] \
        BAYER_TO_RGB_FRAMEBUFFER_ATTR;

/* Camera callback events */
typedef enum
{
    CAM_CB_EVENT_ERROR        = (1 << 0),
    DISP_CB_EVENT_ERROR       = (1 << 1),
    CAM_VSYNC_CB_EVENT        = (1 << 2)
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

    if(event & ARM_CPI_EVENT_MIPI_CSI2_ERROR)
    {
        /* Transfer Error: Received MIPI CSI2 error, Wake-up Thread. */
        tx_event_flags_set(&event_flags, CAM_CB_EVENT_ERROR, TX_OR);
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
  \brief       TestApp to verify a MIPI CSI2 Camera Sensor and ILI9806E LCD Panel
               with Azure RTOS ThreadX as an Operating System.
               This demo thread does:
                   - initialize i3c and Camera hardware pins
                   - initialize CDC200 driver.
                   - initialize Camera driver with Camera Resolution.
                   - captured data will display on LCD Panel.
                   - convert captured image format
                     into RGB888 image format using AIPL demosaic
                   - for ILI9806E LCD Panel,
                     it supports resolution 480x800
                   - crop the captured image to 480x480.
                   - Stream captured image on LCD Panel
  \param[in]   thread_input : thread input
  \return      none
  */
void video_demo_thread_entry(ULONG thread_input)
{
    INT   ret = 0;
    ULONG actual_events = 0;
    UINT  service_error_code;
    UINT  error_code;

    run_profile_t runp = {0};

    ARM_DRIVER_VERSION version;

#if BOARD_CONFIGURE_LVDS_MUX
    /* MIPI LVDS multiplexer pin */
    ret = board_gpios_config();
    if(ret != 0)
    {
        printf("Error in gpio pin configuration: %d\n", ret);
        return;
    }
#endif

    printf("\r\n >>> %s Camera Sensor and ILI9806E LCD Panel demo ",
            SELECTED_CAMERA_SENSOR);
    printf("with Azure RTOS ThreadX is starting up!!! <<< \r\n");

    /* Allocated memory address for
     *   - Camera frame buffer and
     *   - Camera frame buffer for Bayer to RGB Conversion.
     */
    printf("\n \t camera frame buffer          pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            CAMERA_FRAMEBUFFER_POOL_SIZE, (uint32_t) cam_framebuffer_pool);

    printf("\n \t bayer_to_rgb buffer          pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            BAYER_TO_RGB_BUFFER_POOL_SIZE, (uint32_t) bayer_to_rgb_buffer_pool);

    printf("\n \t lcd frame buffer             pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
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

    runp.memory_blocks = MRAM_MASK | SRAM0_MASK | SRAM1_MASK;
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

    /* Convert the camera captured image into RGB888,
     * crop the image and copy it to the Display Frame buffer.
     */
    for(;;)
    {
        ret = tx_event_flags_get(&event_flags,                         \
                                 CAM_VSYNC_CB_EVENT                 |  \
                                 CAM_CB_EVENT_ERROR                 |  \
                                 DISP_CB_EVENT_ERROR,                  \
                                 TX_OR_CLEAR,                          \
                                 &actual_events,                       \
                                 TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf("Error: Camera or Display tx_event_flags_get failed.\n");
            goto error_poweroff_camera;
        }

        if((actual_events & CAM_CB_EVENT_ERROR) || (actual_events & DISP_CB_EVENT_ERROR))
        {
            /* Error: Camera or Display failed. */
            printf("\r\n \t\t >> Error: Camera or Display failed. \r\n");
            goto error_poweroff_camera;
        }

        ret = aipl_demosaic_rgb888(cam_framebuffer_pool,
                                   bayer_to_rgb_buffer_pool,
                                   CAM_FRAME_WIDTH,
                                   CAM_FRAME_WIDTH,
                                   CAM_FRAME_HEIGHT,
                                   AIPL_BAYER_GBRG);
        if(ret != 0)
        {
            printf("\r\n Error: CAMERA image conversion failed.\r\n");
            goto error_poweroff_camera;
        }

        ret = aipl_crop(bayer_to_rgb_buffer_pool,
                        lcd_framebuffer_pool,
                        CAM_FRAME_WIDTH,
                        CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT,
                        AIPL_COLOR_RGB888,
                        (CAM_FRAME_WIDTH - CRP_FRAME_WIDTH) / 2,
                        (CAM_FRAME_HEIGHT - CRP_FRAME_HEIGHT) / 2,
                        (CAM_FRAME_WIDTH + CRP_FRAME_WIDTH) / 2,
                        (CAM_FRAME_HEIGHT + CRP_FRAME_HEIGHT) / 2);
        if(ret != 0)
        {
            printf("\r\n Error: CAMERA AIPL crop failed.\r\n");
            goto error_poweroff_camera;
        }
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
    #if defined(RTE_Compiler_IO_STDOUT_User)
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

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create byte pool\n");
        return;
    }

    /* Create the event flags group used by video thread */
    status = tx_event_flags_create(&event_flags, "event flags Camera and display");
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create event flags\n");
        goto error_delete_byte_pool;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create byte allocate\n");
        goto error_delete_event_flag;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&Video_thread, "Video_thread", video_demo_thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create thread \n");
        goto error_release_allocated_byte;
    }

    return; //SUCCESS

error_release_allocated_byte:
    status = tx_byte_release((VOID *) pointer);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not release byte allocate\n");
    }

error_delete_event_flag:
    status = tx_event_flags_delete(&event_flags);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not delete event flags\n");
    }

error_delete_byte_pool:
    status = tx_byte_pool_delete(&byte_pool_0);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not delete byte pool\n");
    }

    while(1);
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
