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
 * @file     ux_dcd_dwc3_app.h
 * @author   anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     9-June-2022
 * @brief    Header file for the application required driver info.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef UX_DCD_DWC3_H
#define UX_DCD_DWC3_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard
   C is used to process the API information.  */

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

#include "ux_api.h"


/* Define USB Application required DWC3 prototypes.  */

UINT  _ux_dcd_dwc3_initialize();

/* Determine if a C++ compiler is being used.  If so, complete the standard
   C conditional started above.  */
#ifdef   __cplusplus
        }
#endif


#endif
