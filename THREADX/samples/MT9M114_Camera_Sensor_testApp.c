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
 * @file     MT9M114_Camera_Sensor_testApp.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     25-Oct-2021
 * @brief    TestApp to verify MT9M114 Camera Sensor with
 *            Azure RTOS ThreadX as an Operating System.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


/* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Project Includes */
/* Camera Controller Driver */
#include "Driver_Camera_Controller.h"

/* Camera Resolution. */
#include "Camera_Common.h"

/* PINMUX Driver */
#include "Driver_PINMUX_AND_PINPAD.h"

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CAMERA_CONTROLLER Driver_CAMERA0;
static ARM_DRIVER_CAMERA_CONTROLLER *CAMERAdrv = &Driver_CAMERA0;

void camera_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE             1024
#define DEMO_BYTE_POOL_SIZE         9120

TX_THREAD               Camera_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    camera_event_flags;


/* @Note: MT9M114 Camera Sensor configurations
 *        are directly borrowed from MT9M114 Camera Sensor drivers,
 *        for detail refer MT9M114 driver.
 *
 * Selected MT9M114 Camera Sensor configurations:
 *   - Interface     : Parallel
 *   - Resolution    : VGA 640x480
 *   - Output Format : RAW Bayer10
 */

/* Supported MT9M114 Camera Sensor Output Format.
 *  (Currently supports only RAW BAYER10 Format.)
 */
#define MT9M114_CAMERA_OUTPUT_FORMAT_RAW_BAYER10    0

/* User can select from supported MT9M114 Camera Sensor Output Format. */
#define MT9M114_USER_SELECT_CAMERA_OUTPUT_FORMAT    MT9M114_CAMERA_OUTPUT_FORMAT_RAW_BAYER10

/* For MT9M114 Camera Sensor RAW BAYER Output Format:
 * As per data-sheet "AND9534/D"
 *  section: Obtaining Bayer Data (Table 18),
 *   When using any of the RAW Bayer modes,
 *    it is essential that the user adds on the
 *    additional border pixels 8x8(WxH) for demosaic.
 */
#if (MT9M114_USER_SELECT_CAMERA_OUTPUT_FORMAT == MT9M114_CAMERA_OUTPUT_FORMAT_RAW_BAYER10)
#define MT9M114_RAW_BAYER_FORMAT_ADDITIONAL_BORDER_WIDTH      8
#define MT9M114_RAW_BAYER_FORMAT_ADDITIONAL_BORDER_HEIGHT     8

#define MT9M114_ADDITIONAL_WIDTH           MT9M114_RAW_BAYER_FORMAT_ADDITIONAL_BORDER_WIDTH
#define MT9M114_ADDITIONAL_HEIGHT          MT9M114_RAW_BAYER_FORMAT_ADDITIONAL_BORDER_HEIGHT
#else
#define MT9M114_ADDITIONAL_WIDTH           0
#define MT9M114_ADDITIONAL_HEIGHT          0
#endif

/* MT9M114 Camera Sensor Resolution. */
#define MT9M114_CAMERA_RESOLUTION_VGA_640x480       0
#define MT9M114_CAMERA_RESOLUTION                   MT9M114_CAMERA_RESOLUTION_VGA_640x480

#if (MT9M114_CAMERA_RESOLUTION == MT9M114_CAMERA_RESOLUTION_VGA_640x480)
#define FRAME_WIDTH        (640 + MT9M114_ADDITIONAL_WIDTH)
#define FRAME_HEIGHT       (480 + MT9M114_ADDITIONAL_HEIGHT)
#endif

/* Allocate Camera frame buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

/* pool size for Camera frame buffer:
 *  which will be frame width x frame height
 */
#define FRAMEBUFFER_POOL_SIZE   ( (FRAME_WIDTH) * (FRAME_HEIGHT) )

/* pool area for Camera frame buffer.
 *  Allocated in the "camera_frame_buf" section.
 */
uint8_t framebuffer_pool[FRAMEBUFFER_POOL_SIZE] \
                 __attribute__((section("camera_frame_buf")));

/* (optional)
 * if required convert captured image data format to any other image format.
 *
 *  - for MT9M114 Camera sensor,
 *     selected Bayer output format:
 *      in-order to get the color image,
 *       Bayer format must be converted in to RGB format.
 *       User can use below provided
 *        "Open-Source" code for Bayer to RGB Conversion
 *        which uses DC1394 library.
 */
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
#define BAYER_TO_RGB_BUFFER_POOL_SIZE   \
    ( (FRAME_WIDTH) * (FRAME_HEIGHT) * (BITS_PER_PIXEL / 8) * 3 + TIFF_HDR_SIZE )

/* pool area for Camera frame buffer for Bayer to RGB conversion.
 *  Allocated in the "camera_frame_bayer_to_rgb_buf" section.
 */
uint8_t bayer_to_rgb_buffer_pool[BAYER_TO_RGB_BUFFER_POOL_SIZE] \
                __attribute__((section("camera_frame_bayer_to_rgb_buf")));

/* Optional:
 *  Camera Image Conversions
 */
typedef enum {
  BAYER_TO_RGB_CONVERSION   = (1 << 0),
}IMAGE_CONVERSION;

#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

/* Camera callback events */
typedef enum {
  CAM_CB_EVENT_FRAME_VSYNC_DETECTED = (1 << 0),
  CAM_CB_EVENT_CAPTURE_STOPPED      = (1 << 1),
  CAM_CB_EVENT_ERROR                = (1 << 2)
}CAMERA_CB_EVENTS;


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
    tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_FRAME_VSYNC_DETECTED, TX_OR);
    printf("\r\n \t\t >>> Camera CB Success: Camera frame VSYNC detected. <<< \r\n");
  }

  if(event & ARM_CAMERA_CONTROLLER_EVENT_CAMERA_CAPTURE_STOPPED)
  {
    /* Transfer Success: Capture Stop detected, Wake-up Thread. */
    tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_CAPTURE_STOPPED, TX_OR);
    printf("\r\n \t\t >>> Camera CB Success: Camera Capture Stop detected. <<< \r\n");
  }

  if(event & ARM_CAMERA_CONTROLLER_EVENT_ERR_CAMERA_FIFO_OVERRUN)
  {
    /* Transfer Error: Received FIFO over-run, Wake-up Thread. */
    tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    printf("\r\n \t\t >>> Camera CB Error: FIFO over-run. <<< \r\n");
  }

  if(event & ARM_CAMERA_CONTROLLER_EVENT_ERR_CAMERA_FIFO_UNDERRUN)
  {
    /* Transfer Error: Received FIFO under-run, Wake-up Thread. */
    tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    printf("\r\n \t\t >>> Camera CB Error: FIFO under-run. <<< \r\n");
  }

  if(event & ARM_CAMERA_CONTROLLER_EVENT_ERR_HARDWARE)
  {
    /* Transfer Error: Received Hardware error, Wake-up Thread. */
    tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    printf("\r\n \t\t >>> Camera CB Error: hardware error <<< \r\n");
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

  /* i3c Pin-Pad */

  /* Pin-Pad P3_8 as I3C_SDA_B
   * Pad function: weak pull up(0x8) + read enable(0x01)
   *               + Output drive strength 4mA(0x20)
   */
  ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_8,  \
                        (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: i3c PINPAD failed.\r\n");
    return -1;
  }

  /* Pin-Pad P3_9 as I3C_SCL_B
   * Pad function: weak pull up(0x8) + read enable(0x01)
   *               + Output drive strength 4mA(0x20)
   */
  ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_9,  \
                        (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));
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
   *         - P2_4 as CAM_HSYNC_B
   *         - P2_5 as CAM_VSYNC_B
   *
   *         For ASIC A0 CPU Board
   *         - P2_6 as CAM_PCLK_B
   *                (OR)
   *         For ASIC A1 CPU Board
   *         - P3_14 as CAM_PCLK_A
   *
   *           (Note:  CAM_XVCLK_B is not configured.)
   *
   *         - Data Lines D0-D7
   *           - P2_28 as CAM_D0_A
   *           - P2_29 as CAM_D1_A
   *
   *           - P2_10 as CAM_D2_B
   *           - P2_11 as CAM_D3_B
   *           - P2_12 as CAM_D4_B
   *           - P2_13 as CAM_D5_B
   *           - P2_14 as CAM_D6_B
   *           - P2_15 as CAM_D7_B
   */

  /* Configure GPIO Pin : P2_4 as CAM_HSYNC_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_7);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Configure GPIO Pin : P2_5 as CAM_VSYNC_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_5, PINMUX_ALTERNATE_FUNCTION_6);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

#define ASIC_A0_CPU_BOARD_ENABLE       0  /* Enable ASIC A0 CPU Board */
                /* (OR) */
#define ASIC_A1_CPU_BOARD_ENABLE       1  /* Enable ASIC A1 CPU Board */

#if ASIC_A0_CPU_BOARD_ENABLE
  /* Configure GPIO Pin : P2_6 as CAM_PCLK_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_6, PINMUX_ALTERNATE_FUNCTION_6);
#endif

#if ASIC_A1_CPU_BOARD_ENABLE
  /* Configure GPIO Pin : P3_14 as CAM_PCLK_A */
  ret = PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_14, PINMUX_ALTERNATE_FUNCTION_6);
#endif

  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Data Lines: D0-D7 */

  /* Configure GPIO Pin : P2_28 as CAM_D0_A */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_28, PINMUX_ALTERNATE_FUNCTION_6);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Configure GPIO Pin : P2_29 as CAM_D1_A */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_29, PINMUX_ALTERNATE_FUNCTION_6);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Configure GPIO Pin : P2_10 as CAM_D2_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_10, PINMUX_ALTERNATE_FUNCTION_6);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Configure GPIO Pin : P2_11 as CAM_D3_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_11, PINMUX_ALTERNATE_FUNCTION_6);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Configure GPIO Pin : P2_12 as CAM_D4_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_12, PINMUX_ALTERNATE_FUNCTION_6);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Configure GPIO Pin : P2_13 as CAM_D5_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_13, PINMUX_ALTERNATE_FUNCTION_6);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Configure GPIO Pin : P2_14 as CAM_D6_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_14, PINMUX_ALTERNATE_FUNCTION_5);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: Camera Pin-Mux failed.\r\n");
    return -1;
  }

  /* Configure GPIO Pin : P2_15 as CAM_D7_B */
  ret = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_15, PINMUX_ALTERNATE_FUNCTION_6);
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
  \fn          void camera_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify MT9M114 Camera Sensor with
                Azure RTOS ThreadX as an Operating System.

               This demo thread does:
                 - initialize i3c and Camera hardware pins
                    using PinMux Driver;
                 - initialize Camera driver with Camera Resolution
                 - capture one frame
                   - captured data will be stored in to allocated
                     frame buffer address
                 - stop Camera capture
                 - (optional)
                   -if required convert captured image format
                     in to any other image format
                    - for MT9M114 Camera sensor,
                      - selected Bayer output format;
                        - in-order to get the color image,
                           Bayer format must be converted in to RGB format.
                        - User can use below provided
                           "Open-Source" code for Bayer to RGB Conversion
                           which uses DC1394 library.
                 - dump captured/converted image data from memory address
                    using any debugger
                 - display image
  \param[in]   thread_input : thread input
  \return      none
*/
void camera_demo_thread_entry(ULONG thread_input)
{
  INT   ret = 0;
  ULONG actual_events = 0;
  ULONG wait_timer_tickes = 0;

  ARM_CAMERA_RESOLUTION camera_resolution = 0;
  ARM_DRIVER_VERSION version;

  printf("\r\n \t\t >>> MT9M114 Camera Sensor demo with Azure RTOS ThreadX is starting up!!! <<< \r\n");

  /* Allocated memory address for
   *   - Camera frame buffer and
   *   - (Optional) Camera frame buffer for Bayer to RGB Conversion.
   */
  printf("\n \t frame buffer        pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
           FRAMEBUFFER_POOL_SIZE, (uint32_t) framebuffer_pool);

#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
  printf("\n \t bayer_to_rgb buffer pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
           BAYER_TO_RGB_BUFFER_POOL_SIZE, (uint32_t) bayer_to_rgb_buffer_pool);
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

  /* Initialize CAMERA driver with Camera Resolution */
  if (MT9M114_CAMERA_RESOLUTION == MT9M114_CAMERA_RESOLUTION_VGA_640x480)
  {
    camera_resolution = CAMERA_RESOLUTION_VGA_640x480;
  }

  ret = CAMERAdrv->Initialize(camera_resolution, Camera_callback);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: CAMERA Initialize failed.\r\n");
    return;
  }

  /* Power up Camera peripheral */
  ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: CAMERA Power Up failed.\r\n");
    goto error_uninitialize;
  }

  /* Wait sometime for Camera Sensor to setup,
   *  otherwise captured image quality will not be good.
   *  User can adjust this delay as per Camera Sensor.
   *
   * @Observation for MT9M114 Camera Sensor:
   *  - Proper delay is required for:
   *    - Camera Sensor to setup after Soft Reset.
   *    - Camera Sensor Lens to come-out from Shutter and gets steady,
   *       otherwise captured image will be less bright/dull.
   *       adjust this delay if captured image is not proper.
   */
  printf("\r\n Wait for sometime for Camera Sensor to setup,");
  printf("\r\n  otherwise captured image quality will not be good,");
  printf("\r\n  User can adjust this delay as per Camera Sensor.\r\n");
  /* Sleep for n second. */
  tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 3);

  /* Let's Start Capturing Camera Frame...
   *   Camera Controller will capture one frame,
   *   (store data in to allocated frame buffer address)
   *   then it gets stop.
   */
  printf("\r\n Let's Start Capturing Camera Frame...\r\n");
  ret = CAMERAdrv->CaptureFrame(framebuffer_pool);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: CAMERA Capture Frame failed.\r\n");
    goto error_poweroff;
  }

  /* wait till any event success/error comes in isr callback,
   *  and if event is set then clear that event.
   * if the event flags are not set,
   *  this service suspends for a maximum of 'n' timer-ticks.
   */
  wait_timer_tickes = 2000;
  ret = tx_event_flags_get(&camera_event_flags,                  \
                           CAM_CB_EVENT_FRAME_VSYNC_DETECTED  |  \
                            CAM_CB_EVENT_CAPTURE_STOPPED      |  \
                            CAM_CB_EVENT_ERROR,                  \
                           TX_OR_CLEAR,                          \
                           &actual_events,                       \
                           wait_timer_tickes);
  if (ret != TX_SUCCESS)
  {
    printf("Error: CAMERA tx_event_flags_get failed.\n");
    goto error_poweroff;
  }

  if(actual_events & CAM_CB_EVENT_ERROR)
  {
    /* Error: Camera Capture Frame failed. */
    printf("\r\n \t\t >> Error: CAMERA Capture Frame failed. \r\n");
    goto error_poweroff;
  }

  /* Okay, we have received Success: Camera Capture Frame VSYNC detected.
   * now stop Camera Capture.
   */
  ret = CAMERAdrv->Stop();
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: CAMERA stop Capture failed.\r\n");
    goto error_poweroff;
  }

  /* (optional)
   * if required convert captured image data format to any other image format.
   *  - for MT9M114 Camera sensor,
   *     selected Bayer output format:
   *      in-order to get the color image,
   *       Bayer format must be converted in to RGB format.
   *       User can use below provided
   *        "Open-Source" code for Bayer to RGB Conversion
   *        which uses DC1394 library.
   */
  /* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
  ret = camera_image_conversion(BAYER_TO_RGB_CONVERSION,
                                framebuffer_pool,
                                bayer_to_rgb_buffer_pool,
                                FRAME_WIDTH,
                                FRAME_HEIGHT);
  if(ret != 0)
  {
    printf("\r\n Error: CAMERA image conversion failed.\r\n");
    return;
  }
#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

  /* How to dump captured/converted image data from memory address?
   *  To dump memory using ARM DS(Development Studio) and Ulink Pro Debugger
   *
   *  Use below command in "Commands" tab:
   *   dump binary memory path_with_filename.fileformat starting_address ending_address
   *
   *   example:(update user directory name)
   *    dump binary memory /home/user/camera_dump/cam_image0_640p.bin 0x8000000 0x804D33F
   *
   *   Bayer to RGB:
   *    dump binary memory /home/user/camera_dump/cam_image0_Bayer_to_RGB_640p.tif 0x8000000 0x80E7A29
   *
   *   This command will dump memory from staring address to ending address
   *   and store it in to given path with filename.
   */
  printf("\n To dump memory using ARM DS and Ulink Pro Debugger:");
  printf("\n  Use below command in Commands tab: update user directory name \r\n");

#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
  printf("\n   dump binary memory /home/user/camera_dump/cam_image0_Bayer_to_RGB_640p.tif 0x%X 0x%X \r\n", \
         (uint32_t) bayer_to_rgb_buffer_pool, (uint32_t) (bayer_to_rgb_buffer_pool + BAYER_TO_RGB_BUFFER_POOL_SIZE - 1));
#else
  printf("\n   dump binary memory /home/user/camera_dump/cam_image0_640p.bin 0x%X 0x%X \r\n", \
         (uint32_t) framebuffer_pool, (uint32_t) (framebuffer_pool + FRAMEBUFFER_POOL_SIZE - 1));
#endif

  printf("\n  This command will dump memory from staring address to ending address \r");
  printf("\n  and store it in to given path with filename.\r\n\r\n");

  printf("\r\n\r\n XXX Camera demo thread is halting here! XXX...\r\n");
  printf("\r\n Now User can dump captured/converted image data from memory address using any debugger!!!\r\n");

  /* wait forever. */
  while(1);


error_poweroff:

  /* Power off CAMERA peripheral */
  ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: CAMERA Power OFF failed.\r\n");
  }

error_uninitialize:

  /* Un-initialize CAMERA driver */
  ret = CAMERAdrv->Uninitialize();
  if(ret != ARM_DRIVER_OK)
  {
    printf("\r\n Error: CAMERA Uninitialize failed.\r\n");
  }

  printf("\r\n XXX Camera demo thread is exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
  /* Enter the ThreadX kernel.  */
  tx_kernel_enter();
}

/* Define what the initial system looks like.  */
void tx_application_define(void *first_unused_memory)
{
  CHAR    *pointer = TX_NULL;
  INT      status  = 0;

  /* Create a byte memory pool from which to allocate the thread stacks.  */
  status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
  if (status != TX_SUCCESS)
  {
    printf("Could not create byte pool\n");
    return;
  }

  /* Create the event flags group used by i2c thread */
  status = tx_event_flags_create(&camera_event_flags, "event flags I2C");
  if (status != TX_SUCCESS)
  {
    printf("Could not create event flags\n");
    return;
  }

  /* Allocate the stack for thread.  */
  status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
  if (status != TX_SUCCESS)
  {
    printf("Could not create byte allocate\n");
    return;
  }

  /* Create the main thread.  */
  status = tx_thread_create(&Camera_thread, "Camera_thread", camera_demo_thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
  if (status != TX_SUCCESS)
  {
    printf("Could not create thread \n");
    return;
  }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
