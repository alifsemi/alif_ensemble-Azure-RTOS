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
 * @file     nxde_icmpv6_ra_prefix_callback_set.c
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     3-Jan-2023
 * @brief    Implementation of Router Advertisement callback mechanism.
 * @bug      None.
 * @Note     None
 ******************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** NetX Component                                                        */
/**                                                                       */
/**   Internet Protocol (IP)                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define NX_SOURCE_CODE


/* Include necessary system files.  */

#include "nx_api.h"
#include "nx_icmpv6.h"

#ifdef FEATURE_NX_IPV6
/* Bring in externs for caller checking code.  */
NX_CALLER_CHECKING_EXTERNS
#endif /* FEATURE_NX_IPV6 */


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nxde_icmpv6_ra_prefix_callback_set                  PORTABLE C     */
/*                                                         6.1            */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function checks for errors for ra_prefix callback register.    */
/*                                                                        */
/*    Derived from nxde_icmpv6_ra_flag_callback_set.c authored by         */
/*    Yuxin Zhou, Microsoft corporation.                                  */
/*                                                                        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ip_ptr                            IP control block pointer          */
/*    icmpv6_ra_prefix_callback           Application callback function     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                Actual completion status      */
/*    NX_PTR_ERROR                          Invalid pointer input         */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application Code                                                    */
/*                                                                        */
/**************************************************************************/
UINT  _nxde_icmpv6_ra_prefix_callback_set(NX_IP *ip_ptr, VOID (*icmpv6_ra_prefix_callback)(NX_IP *ip_ptr, ULONG *prefix, ULONG prefix_length))
{
#ifdef FEATURE_NX_IPV6
UINT status;


    /* Check for invalid input pointers.  */
    if ((ip_ptr == NX_NULL) || (ip_ptr -> nx_ip_id != NX_IP_ID))
    {
        return(NX_PTR_ERROR);
    }

    /* Check for appropriate caller.  */
    NX_INIT_AND_THREADS_CALLER_CHECKING

    /* Call actual IPv6 enable function.  */
    status =  _nxd_icmpv6_ra_prefix_callback_set(ip_ptr, icmpv6_ra_prefix_callback);

    /* Return completion status.  */
    return(status);
#else /* !FEATURE_NX_IPV6 */
    NX_PARAMETER_NOT_USED(ip_ptr);
    NX_PARAMETER_NOT_USED(icmpv6_ra_prefix_callback);

    return(NX_NOT_SUPPORTED);
#endif /* FEATURE_NX_IPV6 */
}

