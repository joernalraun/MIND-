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
#include "stdio.h"//别忘了她

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

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
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t my_RxBuf[100];
extern uint8_t flag2;
extern uint32_t my_RxLength;
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t flag0=0;
uint8_t flag1=0;
static float old_x=1.6;
static float old_y=1.6;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t key_up[8];

extern uint8_t a_down[8];
extern uint8_t d_down[8];

extern uint8_t w_down[8];
extern uint8_t s_down[8];

extern uint8_t j_down[8];
extern uint8_t k_down[8];
extern uint8_t l_down[8];
extern uint8_t i_down[8];
extern uint8_t o_down[8];
extern uint8_t p_down[8];
extern uint8_t space_down[8];

uint8_t start_down[8]={0x01,0x00,0x3e,0x00,0x00,0x00,0x00,0x00};
uint8_t pause_down[8]={0x01,0x00,0x3f,0x00,0x00,0x00,0x00,0x00};
uint8_t full_screem_down[8]={0x01,0x00,0x45,0x00,0x00,0x00,0x00,0x00};

float ad0,ad1;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(GPIO_Pin == GPIO_PIN_3){
		 if(1==HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }else{
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,j_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
	 }else if(GPIO_Pin == GPIO_PIN_2){
		 if(1==HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }else{
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,k_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
	 }else if(GPIO_Pin == GPIO_PIN_1){
		 if(1==HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }
		 else
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,l_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
	 }else if(GPIO_Pin == GPIO_PIN_0){
		 if(1==HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }
		 else
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,i_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	 }else if(GPIO_Pin == GPIO_PIN_9){
		 if(1==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }
		 else
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,o_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
	 }else if(GPIO_Pin == GPIO_PIN_8){
		 if(1==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }
		 else
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,p_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
	 }else if(GPIO_Pin == GPIO_PIN_7){
		 if(1==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }
		 else
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,space_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	 }else if(GPIO_Pin == GPIO_PIN_4){
		 if(1==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }
		 else
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,start_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
	 }else if(GPIO_Pin == GPIO_PIN_5){
		 if(1==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }
		 else
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,pause_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
	 }else if(GPIO_Pin == GPIO_PIN_6){
		 if(1==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		 }
		 else
		 {
			 DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,full_screem_down,8);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		 }
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
	 }else{
		 ;
	 }
}

void reportAnalogstick(){
	if(fabs(old_x-ad0)>=1){
//		HAL_Delay(1);
		printf("old_x==%f\n",old_x);
		printf("ad0==%f\n",ad0);
		if(ad0<0.6){
			old_x=ad0;
			DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,a_down,8);
		}else if(ad0>2.6){
			old_x=ad0;
			DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,d_down,8);
		}else{
			old_x=1.6;
			DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
		}

	}

	if(fabs(old_y-ad1)>=1){
//		HAL_Delay(1);
		printf("old_y==%f\n",old_y);
		printf("ad1==%f\n",ad1);
		if(ad1<0.6){
			old_y=ad1;
			DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,s_down,8);
		}else if(ad1>2.6){
			old_y=ad1;
			DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,w_down,8);
		}else{
			old_y=1.6;
			DFR_USBD_KEYBOARD_HID_SendReport(&hUsbDeviceFS,key_up,8);
		}

	}
}

void LED_test(){
	while(1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
		HAL_Delay(1000);
	}
}

void user_pwm_setvalue(uint16_t value){
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void PWMtest(){

}

void BMI160test(void){
	begin(0,0x69);
	while(1){
		printf("x==%f,y==%f,z==%f,\n",getAccX(),getAccY(),getAccZ());
		printf("STEP==%d\n",getstep());
		HAL_Delay(1000);
	}
}

void HUKSYLENS_test(){
}

void M_TEST(){
	while(1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,1);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,0);
		HAL_Delay(1000);
	}
}


uint32_t ADC_Value[100];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  ad0 = (float)ADC_Value[0] / 4096 * 3.3;
  ad1 = (float)ADC_Value[1] / 4096 * 3.3;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
//		reportAnalogstick();
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
	SCB->VTOR=0x0800e000;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,0);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)&htim1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADC_Start_DMA(&hadc1,ADC_Value,100);
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  reportAnalogstick();
	  if(flag2==1){
		  firmata_parse(my_RxBuf,my_RxLength);
		  flag2=0;
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Period = 10000-1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1080-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_key_GPIO_Port, led_key_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTAR_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : i3_Pin l2_Pin k1_Pin j0_Pin */
  GPIO_InitStruct.Pin = i3_Pin|l2_Pin|k1_Pin|j0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : led_key_Pin */
  GPIO_InitStruct.Pin = led_key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_key_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTAR_Pin PC10 */
  GPIO_InitStruct.Pin = MOTAR_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : start_Pin pause_Pin full_screem_Pin space_Pin 
                           p5_Pin o4_Pin */
  GPIO_InitStruct.Pin = start_Pin|pause_Pin|full_screem_Pin|space_Pin 
                          |p5_Pin|o4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
