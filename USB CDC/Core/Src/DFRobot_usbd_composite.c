/*
 * DFRobot_usbd_composite.c
 *
 *  Created on: Nov 6, 2020
 *      Author: Administrator
 */


#include "DFRobot_usbd_composite.h"
#include "DFRobot_usbd_cdc.h"
#include "DFRobot_usbd_cdc_if.h"
#include "DFRobot_usbd_keyboard_hid.h"
#include "DFRobot_usbd_keyboard_hid_if.h"

static DFR_USBD_KEYBOARD_HID_HandleTypeDef *pHIDData;

static DFR_USBD_CDC_HandleTypeDef *pCDCData;


static uint8_t  DFR_USBD_Composite_Init (USBD_HandleTypeDef *pdev,
                                     uint8_t cfgidx);

static uint8_t  DFR_USBD_Composite_DeInit (USBD_HandleTypeDef *pdev,
                                       uint8_t cfgidx);

static uint8_t  DFR_USBD_Composite_EP0_RxReady(USBD_HandleTypeDef *pdev);

static uint8_t  DFR_USBD_Composite_Setup (USBD_HandleTypeDef *pdev,
                                      USBD_SetupReqTypedef *req);

static uint8_t  DFR_USBD_Composite_DataIn (USBD_HandleTypeDef *pdev,
                                       uint8_t epnum);

static uint8_t  DFR_USBD_Composite_DataOut (USBD_HandleTypeDef *pdev,
                                        uint8_t epnum);

static uint8_t  *DFR_USBD_Composite_GetFSCfgDesc (uint16_t *length);

static uint8_t  *DFR_USBD_Composite_GetDeviceQualifierDescriptor (uint16_t *length);

USBD_ClassTypeDef  DFR_USBD_COMPOSITE =
{
  DFR_USBD_Composite_Init,
  DFR_USBD_Composite_DeInit,
  DFR_USBD_Composite_Setup,
  NULL, /*EP0_TxSent*/
  DFR_USBD_Composite_EP0_RxReady,
  DFR_USBD_Composite_DataIn,
  DFR_USBD_Composite_DataOut,
  NULL,
  NULL,
  NULL,
  NULL,
  DFR_USBD_Composite_GetFSCfgDesc,
  NULL,
  DFR_USBD_Composite_GetDeviceQualifierDescriptor,
};

/* USB composite device Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
__ALIGN_BEGIN uint8_t DFR_USBD_Composite_CfgFSDesc[DFR_USBD_COMPOSITE_DESC_SIZE]  __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  DFR_USBD_COMPOSITE_DESC_SIZE,
  /* wTotalLength: Bytes returned */
  0x00,
  0x03,         /*bNumInterfaces: 2 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xC0,         /*bmAttributes: bus powered */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */
  /*******************功能1  keyboard*********************************/
  /*IAD描述符*/
  0x08,
  0x0B,
  0x00,
  0x01,
  0x03,
  0x00,
  0x01,
  0x00,
  /* 17 */
  /************** Descriptor of CUSTOM HID interface ****************/
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: CUSTOM_HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /* 26 */
  /******************** Descriptor of CUSTOM_HID *************************/
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  DFR_KEYBOARD_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  DFR_USBD_KEYBOARD_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /* 35 */
  /******************** Descriptor of Custom HID endpoints ********************/
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  DFR_KEYBOARD_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  DFR_KEYBOARD_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
  0x00,
  DFR_KEYBOARD_HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
  /*42 */

  0x07,          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  DFR_KEYBOARD_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03, /* bmAttributes: Interrupt endpoint */
  DFR_KEYBOARD_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  DFR_KEYBOARD_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */
  /* 49 */
  /************************功能2 cdc*********************************/
  /*IAD描述符*/
  0x08,
  0x0B,
  0x01,
  0x02,
  0x02,
  0x02,
  0x01,
  0x00,
  /* 57 */
  /************** Descriptor of CDC interface ****************/
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  /* 66 */
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  /* 71 */
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  /* 76 */
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  /* 80 */
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  /* 85 */
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DFR_CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(DFR_CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(DFR_CDC_CMD_PACKET_SIZE),
  DFR_CDC_FS_BINTERVAL,                           /* bInterval: */
  /* 92 */
  /*---------------------------------------------------------------------------*/
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x02,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  /* 101 */
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  DFR_CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(DFR_CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(DFR_CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  /* 108 */
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  DFR_CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(DFR_CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(DFR_CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
  /* 115 */
};

#if 0
__ALIGN_BEGIN uint8_t DFR_USBD_Composite_CfgFSDesc[DFR_USBD_COMPOSITE_DESC_SIZE]  __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  DFR_USBD_COMPOSITE_DESC_SIZE,
  /* wTotalLength: Bytes returned */
  0x00,
  0x03,         /*bNumInterfaces: 2 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xC0,         /*bmAttributes: bus powered */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: CUSTOM_HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 18 */
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  DFR_KEYBOARD_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  DFR_USBD_KEYBOARD_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  DFR_KEYBOARD_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  DFR_KEYBOARD_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
  0x00,
  DFR_KEYBOARD_HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */

  0x07,          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  DFR_KEYBOARD_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03, /* bmAttributes: Interrupt endpoint */
  DFR_KEYBOARD_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  DFR_KEYBOARD_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */
  /* 41 */

  /************** Descriptor of CDC interface ****************/
  /* 41 */
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  /* 50 */
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  /* 55 */
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  /* 60 */
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  /* 64 */
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  /* 69 */
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DFR_CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(DFR_CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(DFR_CDC_CMD_PACKET_SIZE),
  DFR_CDC_HS_BINTERVAL,                           /* bInterval: */
  /*---------------------------------------------------------------------------*/
  /* 76 */
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x02,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  /* 85 */
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  DFR_CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(DFR_CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(DFR_CDC_DATA_HS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  /* 92 */
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  DFR_CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(DFR_CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(DFR_CDC_DATA_HS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
  /* 99 */
};
#endif


/* USB Standard Device Descriptor */
__ALIGN_BEGIN  uint8_t DFR_USBD_Composite_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]  __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


/**
* @brief  USBD_Composite_Init
*         Initialize the Composite interface
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  DFR_USBD_Composite_Init (USBD_HandleTypeDef *pdev,
                                     uint8_t cfgidx)
{
  uint8_t res = 0;

  pdev->pUserData =  &DFR_USBD_KEYBOARD_fops_FS;
  res +=  DFR_USBD_KEYBOARD_HID.Init(pdev,cfgidx);
  pHIDData = pdev->pClassData;
  pdev->pUserData = &DFR_USBD_CDC_fops_FS;
  res +=  DFR_USBD_CDC.Init(pdev,cfgidx);
  pCDCData = pdev->pClassData;
  return res;
}

/**
* @brief  USBD_Composite_DeInit
*         DeInitilaize  the Composite configuration
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status
*/
static uint8_t  DFR_USBD_Composite_DeInit (USBD_HandleTypeDef *pdev,
                                       uint8_t cfgidx)
{
  uint8_t res = 0;
  pdev->pClassData = pHIDData;
  pdev->pUserData = &DFR_USBD_KEYBOARD_fops_FS;
  res +=  DFR_USBD_KEYBOARD_HID.DeInit(pdev,cfgidx);

  pdev->pClassData = pCDCData;
  pdev->pUserData = &DFR_USBD_CDC_fops_FS;
  res +=  DFR_USBD_CDC.DeInit(pdev,cfgidx);

  return res;
}


static uint8_t  DFR_USBD_Composite_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  DFR_USBD_KEYBOARD_HID.EP0_RxReady(pdev);
  return DFR_USBD_CDC.EP0_RxReady(pdev);
}



/**
* @brief  USBD_Composite_Setup
*         Handle the Composite requests
* @param  pdev: device instance
* @param  req: USB request
* @retval status
*/
static uint8_t  DFR_USBD_Composite_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (req->bmRequest & USB_REQ_RECIPIENT_MASK)
  {
  case USB_REQ_RECIPIENT_INTERFACE:
    switch(req->wIndex)
    {
    case USBD_KEYBOARD_INTERFACE:
      pdev->pClassData = pHIDData;
      pdev->pUserData =  &DFR_USBD_KEYBOARD_fops_FS;
      return(DFR_USBD_KEYBOARD_HID.Setup(pdev, req));

    case USBD_CDC_INTERFACE:
      pdev->pClassData = pCDCData;
      pdev->pUserData =  &DFR_USBD_CDC_fops_FS;
      return(DFR_USBD_CDC.Setup (pdev, req));

    default:
      break;
    }
    break;

  case USB_REQ_RECIPIENT_ENDPOINT:
    switch(req->wIndex)
    {
    case DFR_KEYBOARD_HID_EPIN_ADDR:
    case DFR_KEYBOARD_HID_EPOUT_ADDR:
      pdev->pClassData = pHIDData;
      pdev->pUserData =  &DFR_USBD_KEYBOARD_fops_FS;
      return(DFR_USBD_KEYBOARD_HID.Setup(pdev, req));

    case DFR_CDC_IN_EP:
    case DFR_CDC_OUT_EP:
      pdev->pClassData = pCDCData;
      pdev->pUserData =  &DFR_USBD_CDC_fops_FS;
      return(DFR_USBD_CDC.Setup (pdev, req));

    default:
      break;
    }
    break;
  }
  return USBD_OK;
}




/**
* @brief  USBD_Composite_DataIn
*         handle data IN Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
uint8_t  DFR_USBD_Composite_DataIn (USBD_HandleTypeDef *pdev,
                                uint8_t epnum)
{
  switch(epnum | 0x80)
  {
  case DFR_KEYBOARD_HID_EPIN_ADDR:
    pdev->pClassData = pHIDData;
    pdev->pUserData =  &DFR_USBD_KEYBOARD_fops_FS;
    return(DFR_USBD_KEYBOARD_HID.DataIn(pdev,epnum));

  case DFR_CDC_IN_EP:
    pdev->pClassData = pCDCData;
    pdev->pUserData =  &DFR_USBD_CDC_fops_FS;
    return(DFR_USBD_CDC.DataIn(pdev,epnum));

  default:
    break;

  }
  return USBD_FAIL;
}


/**
* @brief  USBD_Composite_DataOut
*         handle data OUT Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
uint8_t  DFR_USBD_Composite_DataOut (USBD_HandleTypeDef *pdev,
                                 uint8_t epnum)
{
  switch(epnum)
  {
  case DFR_KEYBOARD_HID_EPOUT_ADDR:
    pdev->pClassData = pHIDData;
    pdev->pUserData =  &DFR_USBD_KEYBOARD_fops_FS;
    return(DFR_USBD_KEYBOARD_HID.DataOut(pdev,epnum));

  case DFR_CDC_OUT_EP:
    pdev->pClassData = pCDCData;
    pdev->pUserData =  &DFR_USBD_CDC_fops_FS;
    return(DFR_USBD_CDC.DataOut(pdev,epnum));

  default:
    break;

  }
  return USBD_FAIL;
}



/**
* @brief  USBD_Composite_GetHSCfgDesc
*         return configuration descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *DFR_USBD_Composite_GetFSCfgDesc (uint16_t *length)
{
  *length = sizeof (DFR_USBD_Composite_CfgFSDesc);
  return DFR_USBD_Composite_CfgFSDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *DFR_USBD_Composite_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (DFR_USBD_Composite_DeviceQualifierDesc);
  return DFR_USBD_Composite_DeviceQualifierDesc;
}
