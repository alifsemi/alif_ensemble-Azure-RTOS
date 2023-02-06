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
 * @file     fx_sd_driver.h
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    SD Driver FileX entry APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef FX_SD_DRIVER_H
#define FX_SD_DRIVER_H

#include "fx_api.h"
#include "sd.h"

#ifdef  __cplusplus
extern "C"
{
#endif

VOID _fx_sd_driver(FX_MEDIA *media_ptr);

#ifdef  __cplusplus
}
#endif

#endif
