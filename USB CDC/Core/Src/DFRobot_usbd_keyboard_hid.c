/*
 * DFRobot_usbd_keyboard_hid.c
 *
 *  Created on: 2020年11月9日
 *      Author: Administrator
 */

/* Includes ------------------------------------------------------------------*/
#include "DFRobot_usbd_keyboard_hid.h"
#include "usbd_ctlreq.h"

static uint8_t  DFR_USBD_KEYBOARD_HID_Init(USBD_HandleTypeDef *pdev,
                                     uint8_t cfgidx);

static uint8_t  DFR_USBD_KEYBOARD_HID_DeInit(USBD_HandleTypeDef *pdev,
                                       uint8_t cfgidx);

static uint8_t  DFR_USBD_KEYBOARD_HID_Setup(USBD_HandleTypeDef *pdev,
                                      USBD_SetupReqTypedef *req);

static uint8_t  *DFR_USBD_KEYBOARD_HID_GetFSCfgDesc(uint16_t *length);

static uint8_t  *DFR_USBD_KEYBOARD_HID_GetHSCfgDesc(uint16_t *length);

static uint8_t  *DFR_USBD_KEYBOARD_HID_GetOtherSpeedCfgDesc(uint16_t *length);

static uint8_t  *DFR_USBD_KEYBOARD_HID_GetDeviceQualifierDesc(uint16_t *length);

static uint8_t  DFR_USBD_KEYBOARD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  DFR_USBD_KEYBOARD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  DFR_USBD_KEYBOARD_HID_EP0_RxReady(USBD_HandleTypeDef  *pdev);
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables
  * @{
  */

USBD_ClassTypeDef  DFR_USBD_KEYBOARD_HID =
{
  DFR_USBD_KEYBOARD_HID_Init,
  DFR_USBD_KEYBOARD_HID_DeInit,
  DFR_USBD_KEYBOARD_HID_Setup,
  NULL, /*EP0_TxSent*/
  DFR_USBD_KEYBOARD_HID_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
  DFR_USBD_KEYBOARD_HID_DataIn, /*DataIn*/
  DFR_USBD_KEYBOARD_HID_DataOut,
  NULL, /*SOF */
  NULL,
  NULL,
  DFR_USBD_KEYBOARD_HID_GetHSCfgDesc,
  DFR_USBD_KEYBOARD_HID_GetFSCfgDesc,
  DFR_USBD_KEYBOARD_HID_GetOtherSpeedCfgDesc,
  DFR_USBD_KEYBOARD_HID_GetDeviceQualifierDesc,
};

/* USB CUSTOM_HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t DFR_USBD_KEYBOARD_HID_CfgFSDesc[DFR_USB_KEYBOARD_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  DFR_USB_KEYBOARD_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
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
};

/* USB CUSTOM_HID device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t DFR_USBD_KEYBOARD_HID_CfgHSDesc[DFR_USB_KEYBOARD_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  DFR_USB_KEYBOARD_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
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
  DFR_KEYBOARD_HID_HS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */

  0x07,          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  DFR_KEYBOARD_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03, /* bmAttributes: Interrupt endpoint */
  DFR_KEYBOARD_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  DFR_KEYBOARD_HID_HS_BINTERVAL,  /* bInterval: Polling Interval */
  /* 41 */
};

/* USB CUSTOM_HID device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t DFR_USBD_KEYBOARD_HID_OtherSpeedCfgDesc[DFR_USB_KEYBOARD_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  DFR_USB_KEYBOARD_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
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
};

/* USB CUSTOM_HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t DFR_USBD_KEYBOARD_HID_Desc[DFR_USB_KEYBOARD_HID_DESC_SIZ] __ALIGN_END =
{
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
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t DFR_USBD_KEYBOARD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Functions
  * @{
  */

/**
  * @brief  USBD_CUSTOM_HID_Init
  *         Initialize the CUSTOM_HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  DFR_USBD_KEYBOARD_HID_Init(USBD_HandleTypeDef *pdev,
                                     uint8_t cfgidx)
{
  uint8_t ret = 0U;
  DFR_USBD_KEYBOARD_HID_HandleTypeDef     *hhid;

  /* Open EP IN */
  USBD_LL_OpenEP(pdev, DFR_KEYBOARD_HID_EPIN_ADDR, USBD_EP_TYPE_INTR,
                 DFR_KEYBOARD_HID_EPIN_SIZE);

  pdev->ep_in[DFR_KEYBOARD_HID_EPIN_ADDR & 0xFU].is_used = 1U;

  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, DFR_KEYBOARD_HID_EPOUT_ADDR, USBD_EP_TYPE_INTR,
                 DFR_KEYBOARD_HID_EPOUT_SIZE);

  pdev->ep_out[DFR_KEYBOARD_HID_EPOUT_ADDR & 0xFU].is_used = 1U;

  pdev->pClassData = USBD_malloc(sizeof(DFR_USBD_KEYBOARD_HID_HandleTypeDef));

  if (pdev->pClassData == NULL)
  {
    ret = 1U;
  }
  else
  {
    hhid = (DFR_USBD_KEYBOARD_HID_HandleTypeDef *) pdev->pClassData;

    hhid->state = DFR_KEYBOARD_HID_IDLE;
    ((DFR_USBD_KEYBOARD_HID_ItfTypeDef *)pdev->pUserData)->Init();

    /* Prepare Out endpoint to receive 1st packet */
    USBD_LL_PrepareReceive(pdev, DFR_KEYBOARD_HID_EPOUT_ADDR, hhid->Report_buf,
                           DFR_USBD_KEYBOARDHID_OUTREPORT_BUF_SIZE);
  }

  return ret;
}

/**
  * @brief  USBD_CUSTOM_HID_Init
  *         DeInitialize the CUSTOM_HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  DFR_USBD_KEYBOARD_HID_DeInit(USBD_HandleTypeDef *pdev,
                                       uint8_t cfgidx)
{
  /* Close CUSTOM_HID EP IN */
  USBD_LL_CloseEP(pdev, DFR_KEYBOARD_HID_EPIN_ADDR);
  pdev->ep_in[DFR_KEYBOARD_HID_EPIN_ADDR & 0xFU].is_used = 0U;

  /* Close CUSTOM_HID EP OUT */
  USBD_LL_CloseEP(pdev, DFR_KEYBOARD_HID_EPOUT_ADDR);
  pdev->ep_out[DFR_KEYBOARD_HID_EPOUT_ADDR & 0xFU].is_used = 0U;

  /* FRee allocated memory */
  if (pdev->pClassData != NULL)
  {
    ((DFR_USBD_KEYBOARD_HID_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_Setup
  *         Handle the CUSTOM_HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  DFR_USBD_KEYBOARD_HID_Setup(USBD_HandleTypeDef *pdev,
                                      USBD_SetupReqTypedef *req)
{
  DFR_USBD_KEYBOARD_HID_HandleTypeDef *hhid = (DFR_USBD_KEYBOARD_HID_HandleTypeDef *)pdev->pClassData;
  uint16_t len = 0U;
  uint8_t  *pbuf = NULL;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        case DFR_KEYBOARD_HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case DFR_KEYBOARD_HID_REQ_GET_PROTOCOL:
          USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->Protocol, 1U);
          break;

        case DFR_KEYBOARD_HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case DFR_KEYBOARD_HID_REQ_GET_IDLE:
          USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->IdleState, 1U);
          break;

        case DFR_KEYBOARD_HID_REQ_SET_REPORT:
          hhid->IsReportAvailable = 1U;
          USBD_CtlPrepareRx(pdev, hhid->Report_buf, req->wLength);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if (req->wValue >> 8 == DFR_KEYBOARD_HID_REPORT_DESC)
          {
            len = MIN(DFR_USBD_KEYBOARD_HID_REPORT_DESC_SIZE, req->wLength);
            pbuf = ((DFR_USBD_KEYBOARD_HID_ItfTypeDef *)pdev->pUserData)->pReport;
          }
          else
          {
            if (req->wValue >> 8 == DFR_KEYBOARD_HID_DESCRIPTOR_TYPE)
            {
              pbuf = DFR_USBD_KEYBOARD_HID_Desc;
              len = MIN(DFR_USB_KEYBOARD_HID_DESC_SIZ, req->wLength);
            }
          }

          USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->AltSetting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            hhid->AltSetting = (uint8_t)(req->wValue);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }
  return ret;
}

/**
  * @brief  USBD_CUSTOM_HID_SendReport
  *         Send CUSTOM_HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t DFR_USBD_KEYBOARD_HID_SendReport(USBD_HandleTypeDef  *pdev,
                                   uint8_t *report,
                                   uint16_t len)
{
  DFR_USBD_KEYBOARD_HID_HandleTypeDef     *hhid = (DFR_USBD_KEYBOARD_HID_HandleTypeDef *)pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hhid->state == DFR_KEYBOARD_HID_IDLE)
    {
      hhid->state = DFR_KEYBOARD_HID_BUSY;
      USBD_LL_Transmit(pdev, DFR_KEYBOARD_HID_EPIN_ADDR, report, len);
    }
    else
    {
      return USBD_BUSY;
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_GetFSCfgDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *DFR_USBD_KEYBOARD_HID_GetFSCfgDesc(uint16_t *length)
{
  *length = sizeof(DFR_USBD_KEYBOARD_HID_CfgFSDesc);
  return DFR_USBD_KEYBOARD_HID_CfgFSDesc;
}

/**
  * @brief  USBD_CUSTOM_HID_GetHSCfgDesc
  *         return HS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *DFR_USBD_KEYBOARD_HID_GetHSCfgDesc(uint16_t *length)
{
  *length = sizeof(DFR_USBD_KEYBOARD_HID_CfgHSDesc);
  return DFR_USBD_KEYBOARD_HID_CfgHSDesc;
}

/**
  * @brief  USBD_CUSTOM_HID_GetOtherSpeedCfgDesc
  *         return other speed configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *DFR_USBD_KEYBOARD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = sizeof(DFR_USBD_KEYBOARD_HID_OtherSpeedCfgDesc);
  return DFR_USBD_KEYBOARD_HID_OtherSpeedCfgDesc;
}

/**
  * @brief  USBD_CUSTOM_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  DFR_USBD_KEYBOARD_HID_DataIn(USBD_HandleTypeDef *pdev,
                                       uint8_t epnum)
{
  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((DFR_USBD_KEYBOARD_HID_HandleTypeDef *)pdev->pClassData)->state = DFR_KEYBOARD_HID_IDLE;

  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  DFR_USBD_KEYBOARD_HID_DataOut(USBD_HandleTypeDef *pdev,
                                        uint8_t epnum)
{

  DFR_USBD_KEYBOARD_HID_HandleTypeDef     *hhid = (DFR_USBD_KEYBOARD_HID_HandleTypeDef *)pdev->pClassData;

  ((DFR_USBD_KEYBOARD_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(hhid->Report_buf[0],
                                                            hhid->Report_buf[1]);

  USBD_LL_PrepareReceive(pdev, DFR_KEYBOARD_HID_EPOUT_ADDR, hhid->Report_buf,
                         DFR_USBD_KEYBOARDHID_OUTREPORT_BUF_SIZE);

  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t DFR_USBD_KEYBOARD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  DFR_USBD_KEYBOARD_HID_HandleTypeDef     *hhid = (DFR_USBD_KEYBOARD_HID_HandleTypeDef *)pdev->pClassData;

  if (hhid->IsReportAvailable == 1U)
  {
    ((DFR_USBD_KEYBOARD_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(hhid->Report_buf[0],
                                                              hhid->Report_buf[1]);
    hhid->IsReportAvailable = 0U;
  }

  return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *DFR_USBD_KEYBOARD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = sizeof(DFR_USBD_KEYBOARD_HID_DeviceQualifierDesc);
  return DFR_USBD_KEYBOARD_HID_DeviceQualifierDesc;
}

/**
* @brief  USBD_CUSTOM_HID_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CUSTOMHID Interface callback
  * @retval status
  */
uint8_t  DFR_USBD_KEYBOARD_HID_RegisterInterface(USBD_HandleTypeDef   *pdev,
                                           DFR_USBD_KEYBOARD_HID_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;

  if (fops != NULL)
  {
    pdev->pUserData = fops;
    ret = USBD_OK;
  }

  return ret;
}
/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
