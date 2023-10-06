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
 * @file     MIPI_interface_Video_testApp.c
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     18-August-2022
 * @brief    TestApp to verify ARX3A0 Camera Sensor and ILI9806E LCD Panel
 *           with Azure RTOS ThreadX as an Operating System.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Project Includes */
/* Camera Controller Driver */
#include "Driver_Camera_Controller.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/* Camera Resolution */
#include "Camera_Common.h"

/* PINMUX Driver */
#include "Driver_PINMUX_AND_PINPAD.h"

/* CDC200 Driver*/
#include "Driver_CDC200.h"

/* Device Header files*/
#include CMSIS_device_header

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CAMERA_CONTROLLER Driver_CAMERA0;
static ARM_DRIVER_CAMERA_CONTROLLER *CAMERAdrv = &Driver_CAMERA0;

/*CDC200 Driver instance*/
extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

void video_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE             1024
#define DEMO_BYTE_POOL_SIZE         9120

TX_THREAD               Video_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags;

/* @Note: ARX3A0 Camera Sensor configurations
 *        are directly borrowed from ARX3A0 Camera Sensor drivers,
 *        for detail refer ARX3A0 driver.
 *
 * Selected ARX3A0 Camera Sensor configurations:
 *   - Interface     : MIPI CSI2
 *   - Resolution    : 560X560 and 480x480
 *   - Output Format : RAW Bayer10
 */

/* ARX3A0 Camera Sensor Resolution. */
#define ARX3A0_CAMERA_RESOLUTION_560x560           0
#define ARX3A0_CAMERA_RESOLUTION_480x480           1
#define ARX3A0_CAMERA_RESOLUTION                   ARX3A0_CAMERA_RESOLUTION_480x480

#if (ARX3A0_CAMERA_RESOLUTION == ARX3A0_CAMERA_RESOLUTION_560x560)
#define CAM_FRAME_WIDTH        (560)
#define CAM_FRAME_HEIGHT       (560)
#elif (ARX3A0_CAMERA_RESOLUTION == ARX3A0_CAMERA_RESOLUTION_480x480)
#define CAM_FRAME_WIDTH        (480)
#define CAM_FRAME_HEIGHT       (480)
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
        __attribute__((section("camera_frame_buf")));

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

/* Enable image Cropping and interpolate. */
#define IMAGE_CROP_AND_INTERPOLATE_EN      0

#if IMAGE_CROP_AND_INTERPOLATE_EN

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
        __attribute__((section("lcd_frame_buf")));

/* Required to Crop and interpolate the captured image data format to
 * LCD Panel supported image format.
 *
 *  - for ARX3A0 Camera sensor,
 *     it gives frame resolution as 560x560
 *  - for ILI9806E LCD Panel,
 *     it supports resolution 480x800
 *  - in-order to display the camera image on LCD panel,
 *     we have Crop and interpolate the captured image
 *     to 480x480.
 */

#if (ILI9806E_Panel_RESOLUTION == ILI9806E_Panel_RESOLUTION_480x800)
#define CRP_FRAME_WIDTH        (480)
#define CRP_FRAME_HEIGHT       (480)
#endif

/* pool size for crop the image:
 *  which will be crop frame width x frame height
 */
#define CRP_FRAMEBUFFER_POOL_SIZE   ((CAM_FRAME_WIDTH) * (CAM_FRAME_HEIGHT) * RGB_BYTES_PER_PIXEL)

/**
  Image processing:

  \fn         int crop_and_interpolate( uint8_t const *srcImage, \
                      uint32_t srcWidth, uint32_t srcHeight, \
                      uint8_t *dstImage, uint32_t dstWidth, \
                      uint32_t dstHeight, uint32_t bpp);
  \brief      Crop and interpolate the image to the given resolution.
  \param[in]  srcImage source image buffer address.
  \param[in]  srcWidth source image width.
  \param[in]  srcHeight source image height.
  \param[in]  dstImage destination image buffer address to save the cropped image.
  \param[in]  dstWidth destination image width.
  \param[in]  dstHeight destination image height.
  \param[in]  bpp number of bits per pixel.
  \return     return error status zero for success negative value for error.
*/
extern int crop_and_interpolate( uint8_t const *srcImage, \
        uint32_t srcWidth, uint32_t srcHeight, \
        uint8_t *dstImage, uint32_t dstWidth, \
        uint32_t dstHeight, uint32_t bpp);

/* pool area for crop the captured camera image.
 *  Allocated in the "lcd_crop_and_interpolate_buf" section.
 */
uint8_t crop_and_interpolate_buffer_pool[CRP_FRAMEBUFFER_POOL_SIZE] \
        __attribute__((section("lcd_crop_and_interpolate_buf")));


/* Required convert captured image data format to RGB image format.
 *
 *  - for ARX3A0 Camera sensor,
 *     selected Bayer output format:
 *      in-order to get the color image,
 *       Bayer format must be converted in to RGB format.
 *       User can use below provided
 *        "Open-Source" code for Bayer to RGB Conversion
 *        which uses DC1394 library.
 */

#define LCD_BUF_BASE  SRAM0_BASE
#define LCD_BUF_SIZE  ( CRP_FRAMEBUFFER_POOL_SIZE + LCD_FRAMEBUFFER_POOL_SIZE )

#endif

/* Enable image conversion Bayer to RGB. */
#define IMAGE_CONVERSION_BAYER_TO_RGB_EN      1

/* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
/* @Note: Bayer to RGB configurations
 *        are directly borrowed from "Open-Source" code for
 *        Bayer to RGB Conversion, for detail refer bayer2rgb.c.
 *
 * Selected Bayer to RGB configurations:
 *   - converted image format : tiff
 *   - bpp bit per pixel      : 8-bit
 */
#define TIFF_HDR_NUM_ENTRY 8
#define TIFF_HDR_SIZE 10+TIFF_HDR_NUM_ENTRY*12
/* bpp bit per pixel
 *  Valid parameters are:
 *   -  8-bit
 *   - 16-bit
 */
#define BITS_PER_PIXEL_8_BIT      8
#define BITS_PER_PIXEL            BITS_PER_PIXEL_8_BIT


/* pool size for Camera frame buffer for Bayer to RGB conversion:
 *   which will be frame width x frame height x (bpp / 8) * 3 + tiff header(106 Bytes).
 */
#if IMAGE_CROP_AND_INTERPOLATE_EN
#define BAYER_TO_RGB_BUFFER_POOL_SIZE   \
    ( (CAM_FRAME_WIDTH) * (CAM_FRAME_HEIGHT) * (BITS_PER_PIXEL / 8) * RGB_BYTES_PER_PIXEL + TIFF_HDR_SIZE )
#else
#define BAYER_TO_RGB_BUFFER_POOL_SIZE   \
    ( (LCD_FRAME_WIDTH) * (LCD_FRAME_HEIGHT) * (BITS_PER_PIXEL / 8) * RGB_BYTES_PER_PIXEL + TIFF_HDR_SIZE )
#endif
/* pool area for Camera frame buffer for Bayer to RGB conversion.
 *  Allocated in the "camera_frame_bayer_to_rgb_buf" section.
 */
uint8_t bayer_to_rgb_buffer_pool[BAYER_TO_RGB_BUFFER_POOL_SIZE] \
        __attribute__((section("camera_frame_bayer_to_rgb_buf")));

#define CAM_BUF_BASE  SRAM1_BASE
#define CAM_BUF_SIZE  ( CAMERA_FRAMEBUFFER_POOL_SIZE + BAYER_TO_RGB_BUFFER_POOL_SIZE )

/* Optional:
 *  Camera Image Conversions
 */
typedef enum {
    BAYER_TO_RGB_CONVERSION   = (1 << 0),
}IMAGE_CONVERSION;

#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

/* Camera callback events */
typedef enum {
    CAM_CB_EVENT_ERROR        = (1 << 0),
    DISP_CB_EVENT_ERROR       = (1 << 1),
    CAM_VSYNC_CB_EVENT        = (1 << 2)
}CB_EVENTS;

/**
  \fn          void Camera_callback(uint32_t event)
  \brief       Camera isr callback
  \param[in]   event: Camera Event
  \return      none
  */
void Camera_callback(uint32_t event)
{
    if(event & ARM_CAMERA_CONTROLLER_EVENT_CAMERA_FRAME_VSYNC_DETECTED)
    {
        /* Transfer Success: Frame VSYNC detected, Wake-up Thread. */
        tx_event_flags_set(&event_flags, CAM_VSYNC_CB_EVENT, TX_OR);
    }

    if(event & ARM_CAMERA_CONTROLLER_EVENT_ERR_CAMERA_FIFO_OVERRUN)
    {
        /* Transfer Error: Received FIFO over-run, Wake-up Thread. */
        tx_event_flags_set(&event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    }

    if(event & ARM_CAMERA_CONTROLLER_EVENT_ERR_CAMERA_FIFO_UNDERRUN)
    {
        /* Transfer Error: Received FIFO under-run, Wake-up Thread. */
        tx_event_flags_set(&event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    }

    if(event & ARM_CAMERA_CONTROLLER_EVENT_MIPI_CSI2_ERROR)
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

    /* Configure GPIO Pin : P3_8 as I3C_SDA_B */
    ret = PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_8, PINMUX_ALTERNATE_FUNCTION_3);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: i3c PINMUX failed.\r\n");
        return -1;
    }

    /* Configure GPIO Pin : P3_9 as I3C_SCL_B */
    ret = PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_9, PINMUX_ALTERNATE_FUNCTION_4);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: i3c PINMUX failed.\r\n");
        return -1;
    }

    /* Pin-Pad P3_8 as I3C_SDA_B
     * Pad function: PAD_FUNCTION_READ_ENABLE |
     *      PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_UP |
     *      PAD_FUNCTION_DRIVER_OPEN_DRAIN
     */
    ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_8, PAD_FUNCTION_READ_ENABLE |
            PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_UP |
            PAD_FUNCTION_DRIVER_OPEN_DRAIN);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: i3c PINPAD failed.\r\n");
        return -1;
    }

    /* Pin-Pad P3_9 as I3C_SCL_B
     * Pad function: PAD_FUNCTION_READ_ENABLE |
     *      PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_UP |
     *      PAD_FUNCTION_DRIVER_OPEN_DRAIN
     */
    ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_9,PAD_FUNCTION_READ_ENABLE |
            PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_UP |
            PAD_FUNCTION_DRIVER_OPEN_DRAIN);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: i3c PINPAD failed.\r\n");
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

    /* @Note: Below GPIO pins are configured for Camera.
     *
     *         For ASIC A1 CPU Board
     *         - P2_7 as CAM_XVCLK_B
     */
#define ASIC_A1_CPU_BOARD_ENABLE       1  /* Enable ASIC A1 CPU Board */

#if ASIC_A1_CPU_BOARD_ENABLE
    /* Configure GPIO Pin : P2_7 as CAM_XVCLK_B */
    ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_7, PINMUX_ALTERNATE_FUNCTION_6);

    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Camera Pin-Mux failed.\r\n");
        return -1;
    }
#endif

    return 0;
}

/**
  \fn         static void MPU_Load_Regions(void)
  \brief      Load the MPU regions from the given table
  \note       This function loads the region and also sets the
              attrbutes for the regions.
  \param[in]  None
  \return     None
  */
static void MPU_Load_Regions(void)
{
    static const ARM_MPU_Region_t mpu_table[] __STARTUP_RO_DATA_ATTRIBUTE = {
#if IMAGE_CROP_AND_INTERPOLATE_EN
        {
        /* LCD Buffer Region making it as Non-sharable, read/write, non-privileged, non-executable */
            .RBAR = ARM_MPU_RBAR(LCD_BUF_BASE, ARM_MPU_SH_NON, 0UL, 1UL, 0UL),
            .RLAR = ARM_MPU_RLAR((LCD_BUF_BASE + LCD_BUF_SIZE), 1UL)
        },
#endif
        {
        /* Camera Buffer Region making it as Non-sharable, read/write, non-privileged, non-executable */
            .RBAR = ARM_MPU_RBAR(CAM_BUF_BASE, ARM_MPU_SH_NON, 0UL, 1UL, 0UL),
            .RLAR = ARM_MPU_RLAR((CAM_BUF_BASE + CAM_BUF_SIZE), 1UL)
        },
        {
        /* Low Power Peripheral Regions */
            .RBAR = ARM_MPU_RBAR(0x70000000UL, ARM_MPU_SH_NON, 0UL, 1UL, 1UL),
            .RLAR = ARM_MPU_RLAR(0x72FFFFFFUL, 0UL)
        },
    };

    /* Define the possible Attribute regions */
     ARM_MPU_SetMemAttr(0UL, ARM_MPU_ATTR_DEVICE); /* Attr0, Device Memory */
     /* Attr1, Normal Memory, Cached, Write-through, Read Allocation*/
     ARM_MPU_SetMemAttr(1UL, ARM_MPU_ATTR(
                             ARM_MPU_ATTR_MEMORY_(0,0,1,0),
                             ARM_MPU_ATTR_MEMORY_(0,0,1,0)));

    /* Load the regions from the table */
    ARM_MPU_Load(0U, &mpu_table[0], sizeof(mpu_table)/sizeof(ARM_MPU_Region_t));;
}

/**
  \fn         void MPU_Clear_All_Regions(void)
  \brief      Clear all the MPU registers
  \note       This function disables the MPU and clear
              all the existing regions.
  \param[in]  None
  \return    None
  */
static void MPU_Clear_All_Regions(void)
{
    MPU_Type* mpu = MPU;
    /* Retrieve the number of regions */
    uint32_t num_regions = (mpu->TYPE >> 8);
    uint32_t cnt;

    ARM_MPU_Disable();

    for(cnt = 0U; cnt < num_regions; cnt++)
            ARM_MPU_ClrRegion(cnt);
}

/**
  \fn         void MPU_Setup(void)
  \brief      Configure the MPU.
  \note       This function disables the MPU and loads the regions
              from the table. Once it is loaded, MPU is enabled.
  \param[in]  None
  \return     None
 */
static void MPU_Setup(void)
{
#define MPU_CONTROL (MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk)

    MPU_Clear_All_Regions();
    MPU_Load_Regions();
    ARM_MPU_Enable(MPU_CONTROL);
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

/* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
/**
  \fn          int camera_image_conversion(IMAGE_CONVERSION image_conversion,
  uint8_t  *src,
  uint8_t  *dest,
  uint32_t  frame_width,
  uint32_t  frame_height)
  \brief       Convert image data from one format to any other image format.
  - Supported conversions
  - Bayer(RAW) to RGB Conversion
  - User can use below provided
  "Open-Source" Bayer to RGB Conversion code
  which uses DC1394 library.
  - This code will,
  - Add header for tiff image format
  - Convert RAW Bayer to RGB depending on
  - bpp bit per pixel 8/16 bit
  - DC1394 Color Filter
  - DC1394 Bayer interpolation methods
  - Output image size will be
  - width x height x (bpp / 8) x 3 + tiff header(106 Bytes)
  \param[in]   image_conversion : image conversion methods \ref IMAGE_CONVERSION
  \param[in]   src              : Source address, Pointer to already available
  image data Address
  \param[in]   dest             : Destination address,Pointer to Address,
  where converted image data will be stored.
  \param[in]   frame_width      : image frame width
  \param[in]   frame_height     : image frame height
  \return      success          :  0
  \return      failure          : -1
  */
int camera_image_conversion(IMAGE_CONVERSION  image_conversion,
        uint8_t          *src,
        uint8_t          *dest,
        uint32_t          frame_width,
        uint32_t          frame_height)
{
    /* Bayer to RGB Conversion. */
    extern int32_t bayer_to_RGB(uint8_t  *src,   uint8_t  *dest,   \
            uint32_t  width, uint32_t  height);

    int ret = 0;

    switch(image_conversion)
    {
        case BAYER_TO_RGB_CONVERSION:
            printf("\r\n Start Bayer to RGB Conversion: \r\n");
            printf("\t Frame Buffer Addr: 0x%X \r\n \t Bayer_to_RGB Addr: 0x%X\n", \
                    (uint32_t) src, (uint32_t) dest);
            ret = bayer_to_RGB(src, dest, frame_width, frame_height);
            if(ret != 0)
            {
                printf("\r\n Error: CAMERA image conversion: Bayer to RGB failed.\r\n");
                return -1;
            }
            break;

        default:
            return -1;
    }

    return 0;
}
#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

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
    it gives image in frame resolution560x560
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
    ULONG row = 0, col = 0, index = 0, row_location = 0;
    ULONG wait_timer_ticks = TX_TIMER_TICKS_PER_SECOND;

    ARM_CAMERA_RESOLUTION camera_resolution = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> ARX3A0 Camera Sensor and ILI9806E LCD Panel demo "
            "with Azure RTOS ThreadX is starting up!!! <<< \r\n");


    /* Allocated memory address for
     *   - Camera frame buffer and
     *   - Camera frame buffer for Bayer to RGB Conversion.
     */
    printf("\n \t camera frame buffer          pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            CAMERA_FRAMEBUFFER_POOL_SIZE, (uint32_t) cam_framebuffer_pool);

    printf("\n \t bayer_to_rgb buffer          pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            BAYER_TO_RGB_BUFFER_POOL_SIZE, (uint32_t) bayer_to_rgb_buffer_pool);

#if IMAGE_CROP_AND_INTERPOLATE_EN
    /* Allocated memory address for
     *   - Crop and interpolate buffer and
     *   - LCD frame buffer.
     */
    printf("\n \t lcd frame buffer             pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            LCD_FRAMEBUFFER_POOL_SIZE, (uint32_t) lcd_framebuffer_pool);

    printf("\n \t crop_and_interpolate buffer  pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            CRP_FRAMEBUFFER_POOL_SIZE, (uint32_t) crop_and_interpolate_buffer_pool);
#endif

    /* Initialize i3c and Camera hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != 0)
    {
        printf("\r\n Error: CAMERA Hardware Initialize failed.\r\n");
        return;
    }

    version = CAMERAdrv->GetVersion();
    printf("\r\n Camera driver version api:0x%X driver:0x%X \r\n",version.api, version.drv);

    version = CDCdrv->GetVersion();
    printf("\r\n CDC driver version api:0x%X driver:0x%X \r\n",version.api, version.drv);

    /* Initialize CAMERA driver with Camera Resolution */
    if (ARX3A0_CAMERA_RESOLUTION == ARX3A0_CAMERA_RESOLUTION_560x560)
    {
        camera_resolution = CAMERA_RESOLUTION_560x560;
    }
    else if (ARX3A0_CAMERA_RESOLUTION == ARX3A0_CAMERA_RESOLUTION_480x480)
    {
        camera_resolution = CAMERA_RESOLUTION_480x480;
    }

    /* Initialize Display Driver CDC200*/
    ret = CDCdrv->Initialize(Display_callback);
    if(ret != 0)
    {
        printf("\r\n Error: CDC200 Initialize failed.\r\n");
        return;
    }

    /* Initialize Camera Driver*/
    ret = CAMERAdrv->Initialize(camera_resolution, Camera_callback);
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
#if IMAGE_CROP_AND_INTERPOLATE_EN
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)(lcd_framebuffer_pool));
#else
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)(bayer_to_rgb_buffer_pool + TIFF_HDR_SIZE));
#endif

    if(ret != 0)
    {
        printf("\r\n Error: CDC200 Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Configure Camera */
    ret = CAMERAdrv->Control(CAMERA_SENSOR_CONFIGURE, CAMERA_RESOLUTION_560x560);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Control configuration for camera events */
    ret = CAMERAdrv->Control(CAMERA_EVENTS_CONFIGURE, \
            ARM_CAMERA_CONTROLLER_EVENT_CAMERA_FRAME_VSYNC_DETECTED | \
            ARM_CAMERA_CONTROLLER_EVENT_ERR_CAMERA_FIFO_OVERRUN | \
            ARM_CAMERA_CONTROLLER_EVENT_ERR_CAMERA_FIFO_UNDERRUN);
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
 * crop and interpolate the image and
 * copy it to Display Frame buffer*/
    for(;;)
    {
        ret = tx_event_flags_get(&event_flags,        \
                CAM_VSYNC_CB_EVENT                 |  \
                CAM_CB_EVENT_ERROR                 |  \
                DISP_CB_EVENT_ERROR,                  \
                TX_OR_CLEAR,                          \
                &actual_events,                       \
                wait_timer_ticks);
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

        ret = camera_image_conversion(BAYER_TO_RGB_CONVERSION,
                cam_framebuffer_pool,
                bayer_to_rgb_buffer_pool,
                CAM_FRAME_WIDTH,
                CAM_FRAME_HEIGHT);
        if(ret != 0)
        {
            printf("\r\n Error: CAMERA image conversion failed.\r\n");
            goto error_poweroff_camera;
        }

#if IMAGE_CROP_AND_INTERPOLATE_EN
        ret = crop_and_interpolate((void *)(bayer_to_rgb_buffer_pool + TIFF_HDR_SIZE),
                CAM_FRAME_WIDTH,
                CAM_FRAME_HEIGHT,
                (void *)crop_and_interpolate_buffer_pool,
                CRP_FRAME_WIDTH,
                CRP_FRAME_HEIGHT,
                RGB_BYTES_PER_PIXEL * 8);
        if(ret != 0)
        {
            printf("\r\n Error: CAMERA image crop and interpolate failed.\r\n");
            goto error_poweroff_camera;
        }

        for (row = 0, row_location = 0, index =0; row < CRP_FRAME_HEIGHT; row++) {  //height
            row_location = row * CRP_FRAME_HEIGHT * RGB_BYTES_PER_PIXEL;
            for (col = 0; col < CRP_FRAME_WIDTH; col++) { //width
                lcd_framebuffer_pool[row_location + (col * RGB_BYTES_PER_PIXEL)] = crop_and_interpolate_buffer_pool[index++]; //R
                lcd_framebuffer_pool[row_location + (col * RGB_BYTES_PER_PIXEL) + 1 ] = crop_and_interpolate_buffer_pool[index++]; //G
                lcd_framebuffer_pool[row_location + (col * RGB_BYTES_PER_PIXEL) + 2 ] = crop_and_interpolate_buffer_pool[index++]; //B

             }
        }
#endif
    }

error_poweroff_camera:
    /* Power off CAMERA peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
        printf("\r\n Error: CAMERA Power OFF failed.\r\n");

error_poweroff_cdc:
    /* Power off CDC200 peripheral */
    ret = CDCdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
        printf("\r\n Error: CDC200 Power OFF failed.\r\n");

error_uninitialize_camera:
    /* Un-initialize CAMERA driver */
    ret = CAMERAdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
        printf("\r\n Error: CAMERA Uninitialize failed.\r\n");

error_uninitialize_cdc:
    /* Un-initialize CDC200 driver */
    ret = CDCdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
        printf("\r\n Error: CDC200 Uninitialize failed.\r\n");

    printf("\r\n XXX Camera demo thread is exiting XXX...\r\n");

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
        printf("ERROR: Could not release byte allocate\n");

error_delete_event_flag:
    status = tx_event_flags_delete(&event_flags);
    if (status != TX_SUCCESS)
        printf("ERROR: Could not delete event flags\n");

error_delete_byte_pool:
    status = tx_byte_pool_delete(&byte_pool_0);
    if (status != TX_SUCCESS)
        printf("ERROR: Could not delete byte pool\n");

    while(1);
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
