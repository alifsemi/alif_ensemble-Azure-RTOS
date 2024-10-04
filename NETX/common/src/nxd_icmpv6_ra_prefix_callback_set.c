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
 * @file     nxd_icmpv6_ra_prefix_callback_set.c
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
/**   Internet Control Message Protocol for IPv6 (ICMPv6)                 */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define NX_SOURCE_CODE


/* Include necessary system files.  */

#include "nx_api.h"
#include "nx_icmpv6.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nxd_icmpv6_ra_prefix_callback_set                   PORTABLE C   */
/*                                                           6.1          */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function registers an application callback routine that NetX   */
/*    Duo calls whenever router advertisement is received.                */
/*                                                                        */
/*    Derived from nxd_icmpv6_ra_flag_callback_set.c authored by          */
/*    Yuxin Zhou, Microsoft corporation.                                  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ip_ptr                            IP control block pointer          */
/*    icmpv6_ra_prefix_callback         Application callback function     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                            Completion status                 */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    tx_mutex_get                          Get protection mutex          */
/*    tx_mutex_put                          Put protection mutex          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application Code                                                    */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
UINT  _nxd_icmpv6_ra_prefix_callback_set(NX_IP *ip_ptr, VOID (*icmpv6_ra_prefix_callback)(NX_IP *ip_ptr, ULONG *prefix, ULONG prefix_length))
{

#ifdef FEATURE_NX_IPV6
    /* Get mutex protection.  */
    tx_mutex_get(&(ip_ptr -> nx_ip_protection), TX_WAIT_FOREVER);

    /* Setup the IP address change callback function and the additional information pointers. */
    ip_ptr -> nx_icmpv6_ra_prefix_callback = icmpv6_ra_prefix_callback;

    /* Release mutex protection.  */
    tx_mutex_put(&(ip_ptr -> nx_ip_protection));

    /* Return completion status.  */
    return(NX_SUCCESS);

#else /* !FEATURE_NX_IPV6 */
    NX_PARAMETER_NOT_USED(ip_ptr);
    NX_PARAMETER_NOT_USED(icmpv6_ra_prefix_callback);

    return(NX_NOT_SUPPORTED);

#endif /* FEATURE_NX_IPV6 */
}

