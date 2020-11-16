/*
 * DFRobot_usbd_keyboard.h
 *
 *  Created on: Nov 6, 2020
 *      Author: Administrator
 */

#ifndef INC_DFROBOT_USBD_KEYBOARD_HID_H_
#define INC_DFROBOT_USBD_KEYBOARD_HID_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CUSTOM_HID
  * @brief This file is the Header file for USBD_customhid.c
  * @{
  */


/** @defgroup USBD_CUSTOM_HID_Exported_Defines
  * @{
  */
#define DFR_KEYBOARD_HID_EPIN_ADDR                 0x81U
#define DFR_KEYBOARD_HID_EPIN_SIZE                 0x02U

#define DFR_KEYBOARD_HID_EPOUT_ADDR                0x01U
#define DFR_KEYBOARD_HID_EPOUT_SIZE                0x02U

#define DFR_USB_KEYBOARD_HID_CONFIG_DESC_SIZ       41U
#define DFR_USB_KEYBOARD_HID_DESC_SIZ              9U
#define DFR_USBD_KEYBOARD_HID_REPORT_DESC_SIZE     63

#ifndef DFR_KEYBOARD_HID_HS_BINTERVAL
#define DFR_KEYBOARD_HID_HS_BINTERVAL            0x05U
#endif /* CUSTOM_HID_HS_BINTERVAL */

#ifndef DFR_KEYBOARD_HID_FS_BINTERVAL
#define DFR_KEYBOARD_HID_FS_BINTERVAL            0x05U
#endif /* CUSTOM_HID_FS_BINTERVAL */

#ifndef DFR_USBD_KEYBOARDHID_OUTREPORT_BUF_SIZE
#define DFR_USBD_KEYBOARDHID_OUTREPORT_BUF_SIZE  0x02U
#endif /* USBD_CUSTOMHID_OUTREPORT_BUF_SIZE */
#ifndef DFR_USBD_KEYBOARD_HID_REPORT_DESC_SIZE
#define DFR_USBD_KEYBOARD_HID_REPORT_DESC_SIZE   163U
#endif /* USBD_CUSTOM_HID_REPORT_DESC_SIZE */

#define DFR_KEYBOARD_HID_DESCRIPTOR_TYPE           0x21U
#define DFR_KEYBOARD_HID_REPORT_DESC               0x22U

#define DFR_KEYBOARD_HID_REQ_SET_PROTOCOL          0x0BU
#define DFR_KEYBOARD_HID_REQ_GET_PROTOCOL          0x03U

#define DFR_KEYBOARD_HID_REQ_SET_IDLE              0x0AU
#define DFR_KEYBOARD_HID_REQ_GET_IDLE              0x02U

#define DFR_KEYBOARD_HID_REQ_SET_REPORT            0x09U
#define DFR_KEYBOARD_HID_REQ_GET_REPORT            0x01U
/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
  DFR_KEYBOARD_HID_IDLE = 0U,
  DFR_KEYBOARD_HID_BUSY,
}
DFR_KEYBOARD_HID_StateTypeDef;

typedef struct _DFR_USBD_KEYBOARD_HID_Itf
{
  uint8_t                  *pReport;
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* OutEvent)(uint8_t event_idx, uint8_t state);

} DFR_USBD_KEYBOARD_HID_ItfTypeDef;

typedef struct
{
  uint8_t              Report_buf[DFR_USBD_KEYBOARDHID_OUTREPORT_BUF_SIZE];
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  uint32_t             IsReportAvailable;
  DFR_KEYBOARD_HID_StateTypeDef     state;
}
DFR_USBD_KEYBOARD_HID_HandleTypeDef;

extern USBD_ClassTypeDef  DFR_USBD_KEYBOARD_HID;
#define DFR_USBD_KAYBOARD_HID_CLASS    &DFR_USBD_KEYBOARD_HID
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t DFR_USBD_KEYBOARD_HID_SendReport(USBD_HandleTypeDef *pdev,
                                   uint8_t *report,
                                   uint16_t len);



uint8_t  DFR_USBD_KEYBOARD_HID_RegisterInterface(USBD_HandleTypeDef   *pdev,
                                           DFR_USBD_KEYBOARD_HID_ItfTypeDef *fops);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* INC_DFROBOT_USBD_KEYBOARD_HID_H_ */
