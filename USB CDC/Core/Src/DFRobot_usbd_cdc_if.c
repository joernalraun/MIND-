/* Includes ------------------------------------------------------------------*/
#include <DFRobot_usbd_cdc.h>
#include <DFRobot_usbd_cdc_if.h>

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000

//#define bootAddress    0x8000000
//typedef void (*pFunction)(void);
//uint32_t jumpAddress;
//pFunction jump_To_Bootloader;
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t DFR_CDC_Init_FS(void);
static int8_t DFR_CDC_DeInit_FS(void);
static int8_t DFR_CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t DFR_CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

DFR_USBD_CDC_ItfTypeDef DFR_USBD_CDC_fops_FS =
{
  DFR_CDC_Init_FS,
  DFR_CDC_DeInit_FS,
  DFR_CDC_Control_FS,
  DFR_CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DFR_CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  DFR_USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  DFR_USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DFR_CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

//void userBootStart(void){
//	for(int i=0;i<2;i++){
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
//	  HAL_Delay(1000);
//	}
//	if(((*(__IO uint32_t *)bootAddress) & 0x2FFE0000) == 0x20000000){
//		jumpAddress = *(__IO uint32_t *)(bootAddress +4);
//		jump_To_Bootloader  = (pFunction) jumpAddress;
//		__set_MSP(*(__IO uint32_t *) bootAddress);
//		jump_To_Bootloader();
//	}
//}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DFR_CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	static uint8_t count;
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case DFR_CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case DFR_CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case DFR_CDC_SET_COMM_FEATURE:

    break;

    case DFR_CDC_GET_COMM_FEATURE:

    break;

    case DFR_CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case DFR_CDC_SET_LINE_CODING:
    	if(pbuf[0]+(pbuf[1]<<8)==1200)
    	{
    		if(count>4){
    			NVIC_SystemReset();
    		}else
    			count++;
    	}
    break;

    case DFR_CDC_GET_LINE_CODING:


    break;

    case DFR_CDC_SET_CONTROL_LINE_STATE:

    break;

    case DFR_CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DFR_CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
//  DFR_USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
//  DFR_USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//  return (USBD_OK);
	flag2=1;
	memcpy(my_RxBuf,Buf,*Len);
      my_RxLength=*Len;
      DFR_USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
      DFR_USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	return 1;
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t DFR_CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  DFR_USBD_CDC_HandleTypeDef *hcdc = (DFR_USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  DFR_USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = DFR_USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
