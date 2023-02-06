/*
 * sample_config.h
 *
 * This header file gives flexibility to user to enable or disable features
 * of application using macros. Also user settings and values can be updated
 * 
  *
 * Author   : Srinivasa Rao Ragolu <srinivasa@alifsemi.com>
 *
 * Copyright (C) 2021 ALIF SEMICONDUCTOR
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
  \file sample_config.h
  \brief : meant for updating user settings, flags and values 
 */

#ifndef SAMPLE_CONFIG_H
#define SAMPLE_CONFIG_H



#ifdef __cplusplus
extern   "C" {
#endif

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h> IOT Configuration
// =========================================


/* This sample uses Symmetric key (SAS) to connect to IoT Hub by default,
   simply defining USE_DEVICE_CERTIFICATE and setting your device certificate in sample_device_identity.c
   to connect to IoT Hub with x509 certificate. Set up X.509 security in your Azure IoT Hub,
   refer to https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-security-x509-get-started  */
/* #define USE_DEVICE_CERTIFICATE                      1  */

/*
TODO`s: Configure core settings of application for your IoTHub.
        By default features are disabled. To enable any feature
        comment specific macro mentioned below
*/

/* Defined, DPS is enabled.  */
/*
#define ENABLE_DPS_SAMPLE
*/

/* Defined, telemetry is disabled.  */
//#define DISABLE_TELEMETRY_SAMPLE


/* Defined, C2D is disabled.  */
#define DISABLE_C2D_SAMPLE


/* Defined, Direct method is disabled.  */
#define DISABLE_DIRECT_METHOD_SAMPLE


/* Defined, Device twin is disabled.  */

#define DISABLE_DEVICE_TWIN_SAMPLE


#ifndef ENABLE_DPS_SAMPLE

/* Required when DPS is not used.  */
/* These values can be picked from device connection string which is of format : HostName=<host1>;DeviceId=<device1>;SharedAccessKey=<key1>
   HOST_NAME can be set to <host1>,
   DEVICE_ID can be set to <device1>,
   DEVICE_SYMMETRIC_KEY can be set to <key1>.  */
#ifndef HOST_NAME
#define HOST_NAME                                   "srini-IoTHub.azure-devices.net"
#endif /* HOST_NAME */

//   <o> DEVICE ID
//   <i> Defines the Name of the IOT node.
//   <i> Default:
#ifndef DEVICE_ID
#define DEVICE_ID                                   "myIOTDevice"
#endif /* DEVICE_ID */

#else /* !ENABLE_DPS_SAMPLE */

/* Required when DPS is used.  */
#ifndef ENDPOINT
#define ENDPOINT                                    ""
#endif /* ENDPOINT */

#ifndef ID_SCOPE
#define ID_SCOPE                                    ""
#endif /* ID_SCOPE */

#ifndef REGISTRATION_ID
#define REGISTRATION_ID                             ""
#endif /* REGISTRATION_ID */

#endif /* ENABLE_DPS_SAMPLE */

/* Optional SYMMETRIC KEY.  */
#ifndef DEVICE_SYMMETRIC_KEY
#define DEVICE_SYMMETRIC_KEY                        "TLfDsjaD2Y3N/m8uC1cYmfGlvhBbSk9cW2idykoQ8ps="
#endif /* DEVICE_SYMMETRIC_KEY */

/* Optional module ID.  */
#ifndef MODULE_ID
#define MODULE_ID                                   ""
#endif /* MODULE_ID */

#if (USE_DEVICE_CERTIFICATE == 1)

/* Using X509 certificate authenticate to connect to IoT Hub,
   set the device certificate as your device.  */

/* Device Key type. */
#ifndef DEVICE_KEY_TYPE
#define DEVICE_KEY_TYPE                             NX_SECURE_X509_KEY_TYPE_RSA_PKCS1_DER
#endif /* DEVICE_KEY_TYPE */

#endif /* USE_DEVICE_CERTIFICATE */

// </h>
//------------- <<< end of configuration section >>> ---------------------------

/*
END TODO section
*/

/* Define the Azure RTOS IOT thread stack and priority.  */
#ifndef NX_AZURE_IOT_STACK_SIZE
#define NX_AZURE_IOT_STACK_SIZE                     (2048)
#endif /* NX_AZURE_IOT_STACK_SIZE */

#ifndef NX_AZURE_IOT_THREAD_PRIORITY
#define NX_AZURE_IOT_THREAD_PRIORITY                (4)
#endif /* NX_AZURE_IOT_THREAD_PRIORITY */

#ifndef SAMPLE_MAX_BUFFER
#define SAMPLE_MAX_BUFFER                           (256)
#endif /* SAMPLE_MAX_BUFFER */

/* Define the sample thread stack and priority.  */
#ifndef SAMPLE_STACK_SIZE
#define SAMPLE_STACK_SIZE                           (2048)
#endif /* SAMPLE_STACK_SIZE */

#ifndef SAMPLE_THREAD_PRIORITY
#define SAMPLE_THREAD_PRIORITY                      (16)
#endif /* SAMPLE_THREAD_PRIORITY */

/* Define sample properties count.  */
#define MAX_PROPERTY_COUNT                          2

#ifdef __cplusplus
}
#endif
#endif /* SAMPLE_CONFIG_H */
