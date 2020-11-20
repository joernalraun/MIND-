/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#ifdef __GNUC__

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#else

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#endif



PUTCHAR_PROTOTYPE

{

//同样USART2改为你的串口

HAL_UART_Transmit(&huart1, (uint8_t*)&ch,1,HAL_MAX_DELAY);

    return ch;

}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
#define ApplicationAddress    0x800e000
typedef void (*pFunction)(void);
uint32_t jumpAddress;
pFunction jump_To_Application;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//extern uint8_t FIFOBUFFER[10000];
//extern uint16_t head;
//extern uint16_t tail;

extern uint8_t my_RxBuf[64];
extern uint8_t flag2;
extern uint8_t flag3;
extern uint32_t my_RxLength;
extern uint32_t SIZE;
extern int valid_data;
extern uint32_t writeFlashData[512];
uint32_t addr = 0x0800e000;
uint32_t print_addr=0x0800e000;
uint32_t temp1=0;
extern uint32_t temp_count;
extern uint32_t count=0;
uint8_t flash_flag=0;
uint8_t flag=0;



void userAppStart(void){
	  for(int i=0;i<2;i++){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		  HAL_Delay(1000);
	  }
    for(int i = 0; i < 8; i++) {
	   NVIC->ICER[i] = 0xFFFFFFFF; // 关闭中断
	   NVIC->ICPR[i] = 0xFFFFFFFF; // 清除中断标志位
    }
	if(((*(__IO uint32_t *)ApplicationAddress) & 0x2FFE0000) == 0x20000000){
		jumpAddress = *(__IO uint32_t *)(ApplicationAddress +4);
		jump_To_Application  = (pFunction) jumpAddress;
		__set_MSP(*(__IO uint32_t *) ApplicationAddress);
		jump_To_Application();
	}
}

void erasureflash(uint8_t page){
	HAL_FLASH_Unlock();
	//2、擦除FLASH
	//初始化FLASH_EraseInitTypeDef
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = addr;
	f.NbPages = page;
	//设置PageError
	uint32_t PageError = 0;
	//调用擦除函数
	HAL_FLASHEx_Erase(&f, &PageError);
	HAL_FLASH_Lock();
}

void writeFlash(int data){
//	HAL_FLASH_Unlock();
	HAL_FLASH_Program(TYPEPROGRAM_WORD, addr,data);
	addr+=4;
//	HAL_FLASH_Lock();
}

void writeFlashTest(void){
    //1、解锁FLASH
	HAL_FLASH_Unlock();
//	//2、擦除FLASH
//	//初始化FLASH_EraseInitTypeDef
//	FLASH_EraseInitTypeDef f;
//	f.TypeErase = FLASH_TYPEERASE_PAGES;
//	f.PageAddress = addr;
//	f.NbPages = 1;
//	//设置PageError
//	uint32_t PageError = 0;
//	//调用擦除函数
//	HAL_FLASHEx_Erase(&f, &PageError);
	//3、对FLASH烧写
	for(int i=0;i<valid_data;i++){
		HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, writeFlashData[i]);
		addr+=4;
	}
	//4、锁住FLASH
	HAL_FLASH_Lock();
}
//FLASH读取数据测试
void printFlashTest(void){
#if 1
	for(int i=0;i<19284;i++){//19284 valid_data
	  uint32_t temp = *(__IO uint32_t*)(print_addr);
	  printf("addr:0x%lx, data:0x%lx\r\n", print_addr, temp);
	  print_addr+=4;
	}
#endif
}

int c;
void  firmware_update(){
  if(flag3==1){
	  flash_flag=0;
	  erasureflash(SIZE);
	  flag3=0;
  }
  if(flag2==1){
	  flash_flag=0;
	  HAL_TIM_Base_Stop_IT((TIM_HandleTypeDef *)&htim1);
	  if(my_RxLength<64){
		  writeFlashTest();
//		  printFlashTest();
		  userAppStart();
	  }else{
		  writeFlashTest();
//		  printFlashTest();
	  }
	  /*
	if(my_RxLength<64){
		valid_data=my_RxLength/4+count;
		for(int i=0;i<(my_RxLength/4);i++){
			temp1=0;
			for(int j=0;j<4;j++){
				temp1 |= ((my_RxBuf[4*i+j])<<(8*j));
			}
			writeFlashData[temp_count++]=temp1;
		}
		writeFlashTest();
		userAppStart();
	}else{
		for(int i=0;i<(my_RxLength/4);i++){
			temp1=0;
			for(int j=0;j<4;j++){
				temp1 |= ((my_RxBuf[4*i+j])<<(8*j));
			}
			writeFlashData[temp_count++]=temp1;
			count++;
		}
		if(temp_count>511){
			c++;
			printf("c==%d\n",c*2048);
			valid_data=count;
			writeFlashTest();
			temp_count=0;
			count=0;
		}
	}
	*/
	flag2=0;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM1){
		if(flash_flag>0)
		  HAL_TIM_Base_Stop_IT((TIM_HandleTypeDef *)&htim1);
		flash_flag++;
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,0);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)&htim1);
//  printFlashTest();
  while (1)
  {
	if(flash_flag>1)
	  userAppStart();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	firmware_update();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
