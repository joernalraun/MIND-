/*
 * DFRobot_usbd_keyboard_hid_if.h
 *
 *  Created on: Nov 6, 2020
 *      Author: Administrator
 */

#ifndef INC_DFROBOT_USBD_KEYBOARD_HID_IF_H_
#define INC_DFROBOT_USBD_KEYBOARD_HID_IF_H_


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "DFRobot_usbd_keyboard_hid.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CUSTOM_HID USBD_CUSTOM_HID
  * @brief Usb custom human interface device module.
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Defines USBD_CUSTOM_HID_Exported_Defines
  * @brief Defines.
  * @{
  */

/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Types USBD_CUSTOM_HID_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Macros USBD_CUSTOM_HID_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CUSTOMHID Interface callback. */
extern DFR_USBD_KEYBOARD_HID_ItfTypeDef DFR_USBD_KEYBOARD_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_FunctionsPrototype USBD_CUSTOM_HID_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* INC_DFROBOT_USBD_KEYBOARD_HID_IF_H_ */
