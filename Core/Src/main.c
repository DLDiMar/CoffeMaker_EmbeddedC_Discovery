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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32l4xx_hal_gpio.h"
//#include "stm32l475e_iot01_tsensor.h"
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Define different coffee maker states
typedef enum
{
	off_State,
	idle_State,
	strongBrew_Idle_State,
	strongSmall_State,
	strongMedium_State,
	strongLarge_State,
	strongXLarge_State,
	small_State,
	medium_State,
	large_State,
	xlarge_State,
	brewFinish_State,

}	coffeeSystemState;

// Define different coffee maker events
typedef enum
{
	pressOn_Event,
	timeOut_Event,
	strongOn_Event,
	strongSmall_Event,
	strongMedium_Event,
	strongLarge_Event,
	strongXLarge_Event,
	small_Event,
	medium_Event,
	large_Event,
	xlarge_Event,
	finishBrew_Event,
	turnOff_Event,

}	coffeeSystemEvent;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define "Water Level" input value max value
#define waterLevel_MAX (4095)

// Define Max Idle Time before Auto-off (90 seconds)
#define idleTime (90000)

// Pre-set Console Messages
char *msgPowerOn = "Power: On";
char *msgPowerOff = "Power: Off";
char *msgAutoOff = "Auto Off: Activate";
char *msgWaterLevel = "Water Level: ";
char *msgFinishBrew = "Brew Finished";
char *msgWaterFull = "Cup Level: Full";
char *msgTemperature = "Coffee Temperature: ";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */


//Prototypes of event handlers
coffeeSystemState pressOnHandler(void)
{
	system("clear");
	printf("%s \r\n", msgPowerOn);									// Display Power On message
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);		// Power Button On
	return idle_State;
}

coffeeSystemState strongOnHandler(void)
{
	HAL_GPIO_WritePin(GPIOB, StrongBrew_Pin, GPIO_PIN_SET);			// Strong Brew On
	return strongBrew_Idle_State;
}

coffeeSystemState strongSmallHandler(void)
{
	HAL_GPIO_WritePin(GPIOD, SmallCup_LED_Pin, GPIO_PIN_SET);		// Small Cup On
	return strongSmall_State;
}

coffeeSystemState strongMediumHandler(void)
{
	HAL_GPIO_WritePin(GPIOA, MediumCup_LED_Pin, GPIO_PIN_SET);		// Medium Cup On
	return strongMedium_State;
}

coffeeSystemState strongLargeHandler(void)
{
	HAL_GPIO_WritePin(GPIOB, LargeCup_LED_Pin, GPIO_PIN_SET);		// Large Cup On
	return strongLarge_State;
}

coffeeSystemState strongXLargeHandler(void)
{
	HAL_GPIO_WritePin(GPIOB, XLargeCup_LED_Pin, GPIO_PIN_SET);		// XLarge Cup On
	return strongXLarge_State;
}

coffeeSystemState smallHandler(void)
{
	HAL_GPIO_WritePin(GPIOD, SmallCup_LED_Pin, GPIO_PIN_SET);		// Small Cup On
	return small_State;
}

coffeeSystemState mediumHandler(void)
{
	HAL_GPIO_WritePin(GPIOA, MediumCup_LED_Pin, GPIO_PIN_SET);		// Medium Cup On
	return medium_State;
}

coffeeSystemState largeHandler(void)
{
	HAL_GPIO_WritePin(GPIOB, LargeCup_LED_Pin, GPIO_PIN_SET);		// Large Cup On
	return large_State;
}

coffeeSystemState xlargeHandler(void)
{
	HAL_GPIO_WritePin(GPIOB, XLargeCup_LED_Pin, GPIO_PIN_SET);		// XLarge Cup On
	return xlarge_State;
}

coffeeSystemState finishBrewHandler(void)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);		// Power Button On
	HAL_GPIO_WritePin(GPIOB, StrongBrew_Pin, GPIO_PIN_RESET);		// Strong Brew Off
	HAL_GPIO_WritePin(GPIOA, WaterLevel_Pin, GPIO_PIN_SET);			// Water Level On
	HAL_GPIO_WritePin(GPIOD, SmallCup_LED_Pin, GPIO_PIN_RESET);		// Small Cup Off
	HAL_GPIO_WritePin(GPIOA, MediumCup_LED_Pin, GPIO_PIN_RESET);	// Medium Cup Off
	HAL_GPIO_WritePin(GPIOB, LargeCup_LED_Pin, GPIO_PIN_RESET);		// Large Cup Off
	HAL_GPIO_WritePin(GPIOB, XLargeCup_LED_Pin, GPIO_PIN_RESET);	// XLarge Cup Off
	return brewFinish_State;
}

coffeeSystemState turnOffHandler(void)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);	// Power Button Off
	HAL_GPIO_WritePin(GPIOB, StrongBrew_Pin, GPIO_PIN_RESET);		// Strong Brew Off
	HAL_GPIO_WritePin(GPIOA, WaterLevel_Pin, GPIO_PIN_RESET);		// Water Level Off
	HAL_GPIO_WritePin(GPIOD, SmallCup_LED_Pin, GPIO_PIN_RESET);		// Small Cup Off
	HAL_GPIO_WritePin(GPIOA, MediumCup_LED_Pin, GPIO_PIN_RESET);	// Medium Cup Off
	HAL_GPIO_WritePin(GPIOB, LargeCup_LED_Pin, GPIO_PIN_RESET);		// Large Cup Off
	HAL_GPIO_WritePin(GPIOB, XLargeCup_LED_Pin, GPIO_PIN_RESET);	// XLarge Cup Off
	system("clear");
	printf("%s \r\n", msgPowerOff);									// Diaplay Power off message
	return off_State;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
	return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	coffeeSystemState coffeeNextState = off_State;					// Start in Off State
	coffeeSystemEvent coffeeNewEvent;								// Define Event of system

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Initiate Board Support Package: Temperature Sensor HTS221
  //BSP_TSENSOR_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  // Initialize ADC for Water Level
	  HAL_ADC_Start(&hadc1);

	  //coffeeSystemEvent coffeeNewEvent = readCoffeeEvent();

	  // Power Button Status
	  uint32_t buttonPowerStatus = HAL_GPIO_ReadPin(BUTTON_EXTI13_GPIO_Port, BUTTON_EXTI13_Pin);
	  // Strong Brew Button Status
	  uint32_t buttonStrongBrewStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_STRONGBREW_Pin);
	  // Small Button Status
	  uint32_t buttonSmallStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_SMALL_Pin);
	  // Medium Button Status
	  uint32_t buttonMediumStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_MEDIUM_Pin);
	  // Large Button Status
	  uint32_t buttonLargeStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_LARGE_Pin);
	  // XLarge Button Status
	  uint32_t buttonXLargeStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_XLARGE_Pin);
	  // Water Level status
	  float adcResult0 = HAL_ADC_GetValue(&hadc1);

	  // Convert Water Level as a percentage of max ADC value
	  float waterLevelPercent = (adcResult0/4095)*100;

	  // Function to output console details during brewing process
	  void outputBrewingDetails(void)
	  {
			  system("clear");
			  // Display Water Level Status
			  printf("%s %f %% \r\n", msgWaterLevel, waterLevelPercent);
			  //float temperature = BSP_TSENSOR_ReadTemp();
			  //int value = temperature;
			  //printf("Temperature: %d %%C\n\r", value);
			  HAL_Delay(500);
	  }

	  switch(coffeeNextState)
	  {
	  	  case off_State:
	  	  {
	  		  if(buttonPowerStatus == GPIO_PIN_RESET)			// If Power button is pressed...
	  		  {
	  			  coffeeNextState = pressOnHandler();			// ...Do Power On Event
	  		  }
	  	  }
	  	  break;

	  	  case idle_State:
	  	  {
	  		  // Count time for Auto-Off function
	  		  uint32_t count = 0;
	  		  count = HAL_GetTick();
	  		  if(buttonStrongBrewStatus == GPIO_PIN_SET)		// If Strong Brew Button is pressed...
	  		  {
	  			  coffeeNextState = strongOnHandler();			// ...Do Strong Brew Idle Event
	  		  }
	  		  else if(buttonSmallStatus == GPIO_PIN_SET)		// If Small button is pressed...
	  		  {
	  			  coffeeNextState = smallHandler();				// ...Do Small Brew Event
	  		  }
	  		  else if(buttonMediumStatus == GPIO_PIN_SET)		// If Medium Brew Button is pressed...
	  		  {
	  			  coffeeNextState = mediumHandler();			// ...Do Medium Brew Event
	  		  }
	  		  else if(buttonLargeStatus == GPIO_PIN_SET)		// If Large Brew Button is pressed...
	  		  {
	  			  coffeeNextState = largeHandler();				// ...Do Large Brew Event
	  		  }
	  		  else if(buttonXLargeStatus == GPIO_PIN_SET)		// If Extra Large Brew Button is pressed...
	  		  {
	  			  coffeeNextState = xlargeHandler();			// ...Do Extra Large Brew Event
	  		  }
	  		  else if(adcResult0 == waterLevel_MAX)				// If water level is at MAX...
	  		  {
	  			  printf("%s \r\n", msgWaterFull);				// ... Display message water level is full
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finish Brew Event
	  		  }
	  		  else if(count >= idleTime)						// If idle time is greater than allowable time...
	  		  {
	  			  printf("%s \r\n", msgAutoOff);				// ...Tell user Auto-Off will initiate
	  			  coffeeNextState = turnOffHandler();			// ... Then turn off coffee machine
	  		  }
	  	  }
	  	  break;

	  	  case strongBrew_Idle_State:
	  	  {
	  		  if(buttonSmallStatus == GPIO_PIN_SET)				// If Small button is pressed...
	  		  {
	  			  coffeeNextState = smallHandler();				// ...Do Small Brew Event
	  		  }
	  		  else if(buttonMediumStatus == GPIO_PIN_SET)		// If Medium Brew Button is pressed...
	  		  {
	  			  coffeeNextState = mediumHandler();			// ...Do Medium Brew Event
	  		  }
	  		  else if(buttonLargeStatus == GPIO_PIN_SET)		// If Large Brew Button is pressed...
	  		  {
	  			  coffeeNextState = largeHandler();				// ...Do Large Brew Event
	  		  }
	  		  else if(buttonXLargeStatus == GPIO_PIN_SET)		// If Extra Large Brew Button is pressed...
	  		  {
	  			  coffeeNextState = xlargeHandler();			// ...Do Extra Large Brew Event
	  		  }
	  	  }
	  	  break;

	  	  case small_State:
	  	  {
	  		  outputBrewingDetails();
	  		  if (adcResult0 == waterLevel_MAX)					// When Water Level is Full...
	  		  {
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finished Brew Event
	  		  }
	  	  }
	  	  break;

	  	  case medium_State:
	  	  {
	  		  outputBrewingDetails();
	  		  if (adcResult0 == waterLevel_MAX)					// When Water Level is Full...
	  		  {
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finished Brew Event
	  		  }
	  	  }
	  	  break;

	  	  case large_State:
	  	  {
	  		  outputBrewingDetails();
	  		  if (adcResult0 == waterLevel_MAX)					// When Water Level is Full...
	  		  {
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finished Brew Event
	  		  }
	  	  }
	  	  break;

	  	  case xlarge_State:
	  	  {
	  		  outputBrewingDetails();
	  		  if (adcResult0 == waterLevel_MAX)					// When Water Level is Full...
	  		  {
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finished Brew Event
	  		  }
	  	  }
	  	  break;


	  	  case strongSmall_State:
	  	  {
	  		  outputBrewingDetails();
	  		  if (adcResult0 == waterLevel_MAX)					// When Water Level is Full...
	  		  {
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finished Brew Event
	  		  }
	  	  }
	  	  break;

	  	  case strongMedium_State:
	  	  {
	  		  outputBrewingDetails();
	  		  if (adcResult0 == waterLevel_MAX)					// When Water Level is Full...
	  		  {
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finished Brew Event
	  		  }
	  	  }
	  	  break;

	  	  case strongLarge_State:
	  	  {
	  		  outputBrewingDetails();
	  		  if (adcResult0 == waterLevel_MAX)					// When Water Level is Full...
	  		  {
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finished Brew Event
	  		  }
	  	  }
	  	  break;

	  	  case strongXLarge_State:
	  	  {
	  		  outputBrewingDetails();
	  		  if (adcResult0 == waterLevel_MAX)					// When Water Level is Full...
	  		  {
	  			  coffeeNextState = finishBrewHandler();		// ... Do Finished Brew Event
	  		  }
	  	  }
	  	  break;

	  	  case brewFinish_State:
	  	  {
	  		  system("clear");
	  		  printf("%s \r\n", msgFinishBrew);					// Display on Console brew is finished
	  		  HAL_Delay(2000);
	  		  coffeeNextState = turnOffHandler();				// Turn off machine event
	  	  }
	  	  break;

	  }


	// Define Buttons and Status values

/*
	  // Power Button Status
	  uint32_t buttonPowerStatus = HAL_GPIO_ReadPin(BUTTON_EXTI13_GPIO_Port, BUTTON_EXTI13_Pin);
	  // Strong Brew Button Status
	  uint32_t buttonStrongBrewStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_STRONGBREW_Pin);
	  // Small Button Status
	  uint32_t buttonSmallStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_SMALL_Pin);
	  // Medium Button Status
	  uint32_t buttonMediumStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_MEDIUM_Pin);
	  // Large Button Status
	  uint32_t buttonLargeStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_LARGE_Pin);
	  // XLarge Button Status
	  uint32_t buttonXLargeStatus = HAL_GPIO_ReadPin(GPIOC, BUTTON_XLARGE_Pin);
*/

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|MediumCup_LED_Pin|ARD_D4_Pin|SPBTLE_RF_RST_Pin
                          |ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LargeCup_LED_Pin|XLargeCup_LED_Pin|StrongBrew_Pin|ISM43362_BOOT0_Pin
                          |ISM43362_WAKEUP_Pin|LED2_Pin|SPSGRF_915_SDN_Pin|WaterLevel_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|SmallCup_LED_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin */
  GPIO_InitStruct.Pin = SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_XLARGE_Pin BUTTON_LARGE_Pin BUTTON_MEDIUM_Pin BUTTON_SMALL_Pin
                           BUTTON_STRONGBREW_Pin */
  GPIO_InitStruct.Pin = BUTTON_XLARGE_Pin|BUTTON_LARGE_Pin|BUTTON_MEDIUM_Pin|BUTTON_SMALL_Pin
                          |BUTTON_STRONGBREW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MediumCup_LED_Pin ARD_D4_Pin */
  GPIO_InitStruct.Pin = MediumCup_LED_Pin|ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LargeCup_LED_Pin StrongBrew_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin
                           LED2_Pin SPSGRF_915_SDN_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = LargeCup_LED_Pin|StrongBrew_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin
                          |LED2_Pin|SPSGRF_915_SDN_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : XLargeCup_LED_Pin WaterLevel_Pin */
  GPIO_InitStruct.Pin = XLargeCup_LED_Pin|WaterLevel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SmallCup_LED_Pin */
  GPIO_InitStruct.Pin = SmallCup_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SmallCup_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
