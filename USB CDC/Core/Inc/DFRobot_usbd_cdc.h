/*
 * DFRobot_usbd_cdc.h
 *
 *  Created on: Nov 6, 2020
 *      Author: Administrator
 */

#ifndef INC_DFROBOT_USBD_CDC_H_
#define INC_DFROBOT_USBD_CDC_H_

#ifdef __cplusplus
extern "C" {
#endif
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_cdc
  * @brief This file is the Header file for usbd_cdc.c
  * @{
  */


/** @defgroup usbd_cdc_Exported_Defines
  * @{
  */
#define DFR_CDC_IN_EP                            0x84U  /* EP3 for data IN */
#define DFR_CDC_OUT_EP                           0x04U  /* EP3 for data OUT */
#define DFR_CDC_CMD_EP                           0x82U  /* EP4 for CDC commands */

#define DFR_CDC_DATA_HS_MAX_PACKET_SIZE          512U  /* Endpoint IN & OUT Packet size */
#define DFR_CDC_DATA_FS_MAX_PACKET_SIZE          64U  /* Endpoint IN & OUT Packet size */
#define DFR_CDC_CMD_PACKET_SIZE                  8U  /* Control Endpoint Packet size */

#ifndef DFR_CDC_HS_BINTERVAL
#define DFR_CDC_HS_BINTERVAL                     0x10U
#endif /* DFR_CDC_HS_BINTERVAL */

#ifndef DFR_CDC_FS_BINTERVAL
#define DFR_CDC_FS_BINTERVAL                     0x10U
#endif /* DFR_CDC_FS_BINTERVAL */

#ifndef DFR_CDC_HS_BINTERVAL
#define DFR_CDC_HS_BINTERVAL                          0x10U
#endif /* CDC_HS_BINTERVAL */

#ifndef DFR_CDC_FS_BINTERVAL
#define DFR_CDC_FS_BINTERVAL                          0x10U
#endif /* CDC_FS_BINTERVAL */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define DFR_CDC_DATA_HS_MAX_PACKET_SIZE                 512U  /* Endpoint IN & OUT Packet size */
#define DFR_CDC_DATA_FS_MAX_PACKET_SIZE                 64U  /* Endpoint IN & OUT Packet size */
#define DFR_CDC_CMD_PACKET_SIZE                         8U  /* Control Endpoint Packet size */

#define DFR_USB_CDC_CONFIG_DESC_SIZ                     67U
#define DFR_CDC_DATA_HS_IN_PACKET_SIZE                  DFR_CDC_DATA_HS_MAX_PACKET_SIZE
#define DFR_CDC_DATA_HS_OUT_PACKET_SIZE                 DFR_CDC_DATA_HS_MAX_PACKET_SIZE

#define DFR_CDC_DATA_FS_IN_PACKET_SIZE                  DFR_CDC_DATA_FS_MAX_PACKET_SIZE
#define DFR_CDC_DATA_FS_OUT_PACKET_SIZE                 DFR_CDC_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define DFR_CDC_SEND_ENCAPSULATED_COMMAND               0x00U
#define DFR_CDC_GET_ENCAPSULATED_RESPONSE               0x01U
#define DFR_CDC_SET_COMM_FEATURE                        0x02U
#define DFR_CDC_GET_COMM_FEATURE                        0x03U
#define DFR_CDC_CLEAR_COMM_FEATURE                      0x04U
#define DFR_CDC_SET_LINE_CODING                         0x20U
#define DFR_CDC_GET_LINE_CODING                         0x21U
#define DFR_CDC_SET_CONTROL_LINE_STATE                  0x22U
#define DFR_CDC_SEND_BREAK                              0x23U
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} DFR_USBD_CDC_LineCodingTypeDef;

typedef struct _DFR_USBD_CDC_Itf
{
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);

} DFR_USBD_CDC_ItfTypeDef;


typedef struct
{
  uint32_t data[DFR_CDC_DATA_HS_MAX_PACKET_SIZE / 4U];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
}
DFR_USBD_CDC_HandleTypeDef;



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  DFR_USBD_CDC;
#define DFR_USBD_CDC_CLASS    &DFR_USBD_CDC
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  DFR_USBD_CDC_RegisterInterface(USBD_HandleTypeDef   *pdev,
                                    DFR_USBD_CDC_ItfTypeDef *fops);

uint8_t  DFR_USBD_CDC_SetTxBuffer(USBD_HandleTypeDef   *pdev,
                              uint8_t  *pbuff,
                              uint16_t length);

uint8_t  DFR_USBD_CDC_SetRxBuffer(USBD_HandleTypeDef   *pdev,
                              uint8_t  *pbuff);

uint8_t  DFR_USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev);

uint8_t  DFR_USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* INC_DFROBOT_USBD_CDC_H_ */
