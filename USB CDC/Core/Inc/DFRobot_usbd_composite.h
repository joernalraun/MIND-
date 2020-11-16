/*
 * usbd_composite.h
 *
 *  Created on: Nov 6, 2020
 *      Author: Administrator
 */

#ifndef INC_DFROBOT_USBD_COMPOSITE_H_
#define INC_DFROBOT_USBD_COMPOSITE_H_
#ifdef __cplusplus
extern "C" {
#endif
#include  "usbd_ioreq.h"
#define USBD_KEYBOARD_INTERFACE     0
#define USBD_CDC_INTERFACE          1
#define DFR_USBD_COMPOSITE_DESC_SIZE 115
extern USBD_ClassTypeDef  DFR_USBD_COMPOSITE;
#ifdef __cplusplus
}
#endif
#endif /* INC_DFROBOT_USBD_COMPOSITE_H_ */
