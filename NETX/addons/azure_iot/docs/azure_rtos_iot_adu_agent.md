# Azure IoT ADU Agent

**nx_azure_iot_adu_agent_start**
***
<div style="text-align: right"> Start Azure IoT ADU agent. </div>

**Prototype**
```c
UINT nx_azure_iot_adu_agent_start(NX_AZURE_IOT_ADU_AGENT *adu_agent_ptr,
                                  NX_AZURE_IOT_HUB_CLIENT *iothub_client_ptr,
                                  const UCHAR *manufacturer, UINT manufacturer_length,
                                  const UCHAR *model, UINT model_length,
                                  const UCHAR *provider, UINT provider_length,
                                  const UCHAR *name, UINT name_length,
                                  const UCHAR *version, UINT version_length,
                                  VOID (*adu_agent_update_notify)(NX_AZURE_IOT_ADU_AGENT *adu_agent_ptr,
                                                                  UCHAR *provider, UINT provider_length,
                                                                  UCHAR *name, UINT name_length,
                                                                  UCHAR *version, UINT version_length),
                                  VOID (*adu_agent_driver)(NX_AZURE_IOT_ADU_AGENT_DRIVER *));
```
**Description**

<p>This routine starts the ADU agent.</p>

**Parameters**

| Name | Description |
| - |:-|
| adu_agent_ptr [in] | A pointer to a `NX_AZURE_IOT_ADU_AGENT`. |
| iothub_client_ptr [in] | A pointer to a `NX_AZURE_IOT_HUB_CLIENT`.|
| manufacturer [in] | A pointer to the manufacturer. Must be NULL terminated string. |
| manufacturer_length [in] | Length of the manufacturer.  |
| model [in]  | A pointer to the model. Must be NULL terminated string. |
| model_length [in] | Length of the model. |
| provider [in]  | A pointer to the update provider. Must be NULL terminated string. |
| provider_length [in] | Length of the update provider. |
| name [in]  | A pointer to the update name. Must be NULL terminated string. |
| name_length [in] | Length of the update name. |
| version [in]  | A pointer to the update version. Must be NULL terminated string. |
| version_length [in] | Length of the update version. |
| adu_agent_update_notify [in] | Pointer to a callback function invoked once update is received. |
| adu_agent_driver [in] | User supplied driver for flash operation. |

**Return Values**
* NX_AZURE_IOT_SUCCESS Successfully started the Azure IoT ADU agent.
* NX_AZURE_IOT_INVALID_PARAMETER Fail to start the Azure IoT ADU agent due to invalid parameter.
* NX_AZURE_IOT_NO_AVAILABLE_CIPHER Fail to start the Azure IoT ADU agent due to no available cipher.
* NX_AZURE_IOT_INSUFFICIENT_BUFFER_SPACE Fail to start the Azure IoT ADU agent due to insufficient buffer space.

**Allowed From**

Threads

**Example**

**See Also**

<div style="page-break-after: always;"></div>

**nx_azure_iot_adu_agent_proxy_update_add**
***
<div style="text-align: right"> Add proxy update on device update agent. </div>

**Prototype**
```c
UINT nx_azure_iot_adu_agent_proxy_update_add(NX_AZURE_IOT_ADU_AGENT *adu_agent_ptr,
                                             const UCHAR *provider, UINT provider_length,
                                             const UCHAR *name, UINT name_length,
                                             const UCHAR *version, UINT version_length,
                                             VOID (*adu_agent_driver)(NX_AZURE_IOT_ADU_AGENT_DRIVER *));
```
**Description**

<p>This routine adds the proxy update on device update agent. Provider, name and version is a string capturing the update id for a device. Combination of provider and name must be globally unique, when receiving the update, agent checks the provider and name for each device. If the version is provided, agent compares the version to see if the update is installed or not, otherwise, agent lets the driver to check if the update is installed or not. </p>

**Parameters**

| Name | Description |
| - |:-|
| adu_agent_ptr [in] | A pointer to a `NX_AZURE_IOT_ADU_AGENT`. |
| provider [in]  | A pointer to the update provider. Must be NULL terminated string. |
| provider_length [in] | Length of the update provider. |
| name [in]  | A pointer to the update name. Must be NULL terminated string. |
| name_length [in] | Length of the update name. |
| version [in]  | A pointer to the update version. Must be NULL terminated string. |
| version_length [in] | Length of the update version. |
| adu_agent_driver [in] | User supplied driver for flash operation. |

**Return Values**
* NX_AZURE_IOT_SUCCESS Successfully started the Azure IoT ADU agent.
* NX_AZURE_IOT_INVALID_PARAMETER Fail to start the Azure IoT ADU agent due to invalid parameter.
* NX_AZURE_IOT_NO_MORE_ENTRIES Fail to start the Azure IoT ADU agent due to no more entries.

**Allowed From**

Threads

**Example**

**See Also**

<div style="page-break-after: always;"></div>

**nx_azure_iot_adu_agent_stop**
***
<div style="text-align: right"> Stop Azure IoT ADU agent. </div>

**Prototype**
```c
UINT nx_azure_iot_adu_agent_stop(NX_AZURE_IOT_ADU_AGENT *adu_agent_ptr);
```
**Description**

<p>This routine stops the ADU agent.</p>

**Parameters**
|               |               |
| - |:-|
| adu_agent_ptr [in]    | A pointer to a `NX_AZURE_IOT_ADU_AGENT` |


**Return Values**
* NX_AZURE_IOT_SUCCESS Successfully stopped the Azure IoT ADU agent.
* NX_AZURE_IOT_INVALID_PARAMETER Fail to stop the Azure IoT ADU agent due to invalid parameter.

**Allowed From**

Threads

**Example**

**See Also**

<div style="page-break-after: always;"></div>

**nx_azure_iot_adu_agent_update_download_install**
***
<div style="text-align: right"> Start to download and install the new update. </div>

**Prototype**
```c
UINT nx_azure_iot_adu_agent_update_download_install(NX_AZURE_IOT_ADU_AGENT *adu_agent_ptr);
```
**Description**

<p>The routine starts to download and install the new update.</p>

**Parameters**
|               |               |
| - |:-|
| adu_agent_ptr [in]    | A pointer to a `NX_AZURE_IOT_ADU_AGENT` |


**Return Values**
* NX_AZURE_IOT_SUCCESS Successfully started to download and install the new update.
* NX_AZURE_IOT_INVALID_PARAMETER Fail to download and install the new update due to invalid parameter.

**Allowed From**

Threads

**Example**

**See Also**

<div style="page-break-after: always;"></div>

**nx_azure_iot_adu_agent_update_apply**
***
<div style="text-align: right"> Start to apply the new update. </div>

**Prototype**
```c
UINT nx_azure_iot_adu_agent_update_apply(NX_AZURE_IOT_ADU_AGENT *adu_agent_ptr);
```
**Description**

<p>The routine starts to apply the new update. The device may reboot after applying the update successfully.</p>

**Parameters**
|               |               |
| - |:-|
| adu_agent_ptr [in]    | A pointer to a `NX_AZURE_IOT_ADU_AGENT` |


**Return Values**
* NX_AZURE_IOT_SUCCESS Successfully started to apply the new update.
* NX_AZURE_IOT_INVALID_PARAMETER Fail to apply the new update due to invalid parameter.

**Allowed From**

Threads

**Example**

**See Also**

<div style="page-break-after: always;"></div>
