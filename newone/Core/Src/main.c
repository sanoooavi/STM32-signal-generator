/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dac.h"
#include "math.h"
#include "../../Drivers/LCD16x2Lib/LCD.h"
#include "stdio.h"
#include <stdlib.h>
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
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//defined variables
volatile uint16_t frequency;
volatile uint16_t duration;
volatile uint16_t wave_model;
volatile uint8_t flag=1;
//for printing waves
volatile uint8_t index=0;
volatile uint8_t interupt_occured=1;
void create_square_pulse(void);
void create_trangle_pulse(void);
void create_saw_tooth_pulse(void);
void create_sin_pulse(void);
void create_abs_pulse(void);
void create_step_pulse(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
const uint8_t square_pule_lookUp[200] = {
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,127,127,127,127,127,127,127,127,127,127,
127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127 };
const uint8_t triangle[200] = {0,  3,  5,  8, 10, 13, 15, 18, 20, 23, 25, 28, 30,33, 36, 38, 41, 43, 46, 48, 51, 53, 56, 58, 61, 64,
	66, 69, 71, 74, 76, 79, 81, 84, 86, 89, 91, 94, 97,99,102,104,107,109,112,114,117,119,122,124,127,130,132,135,137,140,142,145,147,
	150,152,155,157,160,163,165,168,170,173,175,178,180,183,185,188,190,193,196,198,201,203,206,208,211,213,216,218,221,224,226,229,231
	,234,236,239,241,244,246,249,251,254,251,249,246,244,241,239,236,234,231,229,226,224,221,218,216,213,211,208,206,203,201,198,196,193,
	190,188,185,183,180,178,175,173,170,168,165,163,160,157,155,152,150,147,145,142,140,137,135,132,130,127,124,122,119,117,114,112,109,
	107,104,102, 99, 97, 94, 91, 89, 86, 84, 81,79, 76, 74, 71, 69, 66, 64, 61, 58, 56, 53, 51, 48,46, 43, 41, 38, 36, 33, 30, 28, 25, 23,
20, 18, 15,13, 10,  8,  5,  3 };
const uint8_t step_lookUp_table[]={
0,0,0,0,0,20,20,20,20,20,20,20,38,38,38,38,38,38,38,56,56,56,56,56,56,56,74,74,74,74,74,74,74,92,92,92,92,92,92,92,110,
110,110,110,110,110,110,128,128,128,128,128,128,128,146,146,146,146,146,146,146,164,164,164,164,164,164,164,182,182,182,
182,182,182,182,200,200,200,200,200,200,200,218,218,218,218,218,218,218,236,236,236,236,236,236,236,254,254,254,254,
254,254,254,254,236,236,236,236,236,236,236,218,218,218,218,218,218,218,200,200,200,200,200,200,200,182,182,182,182,182,182,182,164,164,164,164,164,164,164,146,146,146,146,146,146,146,128,128,128,128,128,128,128,110,110,110,110,110,110,
110,92,92,92,92,92,92,92,74,74,74,74,74,74,74,56,56,56,56,56,56,56,38,38,38,38,38,38,38,20,20,20,20,20,20,20,0,0,0,0,0};
const uint8_t sin_lookUp_table[]={127,131,135,139,143,147,151,155,159,162,166,170,174,177,181,185,188,192,195,198,202,205,208,211,214,
	217,220,222,225,227,230,232,234,236,238,240,242,244,245,246,248,249,250,251,252,252,253,253,254,254,254,254,254,253,253,252,252,251,
	250,249,248,246,245,244,242,240,238,236,234,232,230,227,225,222,220,217,214,211,208,205,202,198,195,192,188,185,181,177,174,170,166,
	162,159,155,151,147,143,139,135,131,127,123,119,115,111,107,103, 99, 95, 92, 88, 84, 80, 77, 73, 69, 66,62, 59, 56, 52, 49, 46, 43,
	40, 37, 34, 32, 29, 27,24, 22, 20, 18, 16, 14, 12, 10,  9,  8,  6,  5,  4,3,  2,  2,  1,  1,  0,  0,  0,  0,  0,  1,  1,  2, 2,  3, 
	4,  5,  6,  8,  9, 10, 12, 14, 16, 18, 20,22, 24, 27, 29, 32, 34, 37, 40, 43, 46, 49, 52, 56,59, 62, 66, 69, 73, 77, 80, 84, 88, 92,
95, 99,103,107,111,115,119,123};
const uint8_t abs_lookUp_table[]={127,131,135,139,143,147,151,155,159,162,166,170,174,177,181,185,188,192,195,198,202,205,208,211,214,
	217,220,222,225,227,230,232,234,236,238,240,242,244,245,246,248,249,250,251,252,252,253,253,254,254,254,254,254,253,253,252,252,251,
	250,249,248,246,245,244,242,240,238,236,234,232,230,227,225,222,220,217,214,211,208,205,202,198,195,192,188,185,181,177,174,170,166,
	162,159,155,151,147,143,139,135,131,127,131,135,139,143,147,151,155,159,162,166,170,174,177,181,185,188,192,195,198,202,205,208,211,214,
	217,220,222,225,227,230,232,234,236,238,240,242,244,245,246,248,249,250,251,252,252,253,253,254,254,254,254,254,253,253,252,252,251,
	250,249,248,246,245,244,242,240,238,236,234,232,230,227,225,222,220,217,214,211,208,205,202,198,195,192,188,185,181,177,174,170,166,
	162,159,155,151,147,143,139,135,131};
const uint8_t saw_tooth_lookUp_table[]={0,  1,  3,  4,  5,  6,  8,  9, 10, 11, 13, 14, 15,17, 18, 19, 20, 22, 23, 24, 25, 27, 28, 29, 
	30, 32,33, 34, 36, 37, 38, 39, 41, 42, 43, 44, 46, 47, 48,50, 51, 52, 53, 55, 56, 57, 58, 60, 61, 62, 64, 65,66, 67, 69, 70, 71, 72,
	74, 75, 76, 77, 79, 80, 81,83, 84, 85, 86, 88, 89, 90, 91, 93, 94, 95, 97, 98,99,100,102,103,104,105,107,108,109,110,112,113,114,116,117,
	118,119,121,122,123,124,126,127,128,130,131,132,133,135,136,137,138,140,141,142,144,145,146,147,149,150,151,152,154,155,156,157,159,160,
	161,163,164,165,166,168,169,170,171,173,174,175,177,178,179,180,182,183,184,185,187,188,189,190,192,193,194,196,197,198,199,201,202,203,
	204,206,207,208,210,211,213,215,216,217,218,220,221,222,224,225,226,227,229,230,231,232,234,235,236,237,239,240,241,243,244,245,246,
 248,249,250,251,253,255};

	
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


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
//  MX_TIM3_Init();
 // MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	LCD_Init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//LCD_Puts(4, 0, "helllllllllo");
	/*LCD_Puts(0,0,"waiting...");
	uint8_t *received = (uint8_t*) malloc(6 * sizeof(uint8_t));
//	uint8_t *tx_buffer_slave = (uint8_t*) malloc(6 * sizeof(uint8_t));
	// uint8_t received[6];
	 HAL_SPI_Receive(&hspi1,received,6,HAL_MAX_DELAY);
	 LCD_Clear();
	 char str[33];
	 
    frequency=(received[0]<<8)|received[1];
  	duration=(received[2]<<8)|received[3];
    wave_model=(received[4]<<8)|received[5];
	 
	 sprintf(str,"freq is %06d",frequency);
	 LCD_Puts(0,0,str);
	 sprintf(str,"du is %06d",duration);
	 LCD_Puts(0,1,str);
	 HAL_Delay(1000);
	 LCD_Clear();
	 sprintf(str,"model is %01d",wave_model);
	 LCD_Puts(0,0,str);
	 //set the duratuon for the timer
	 HAL_Delay(1000);
 //  MX_TIM3_Init();

	 //now it should transmit what frequency it got
   HAL_Delay(100);
	 LCD_Clear();
	 LCD_Puts(0,0,"done config");
	 MX_TIM3_Init();
	 //	 MX_TIM2_Init();
	 HAL_Delay(1);
 if(wave_model==1){
		LCD_Clear();
		LCD_Puts(0,0,"sin");
		HAL_Delay(100);
  	create_sin_pulse();
	}
	else if(wave_model==2){
		LCD_Clear();
		LCD_Puts(0,0,"square pulse");
		HAL_Delay(100);
	  create_square_pulse();
	}
	else if(wave_model==3){
		LCD_Clear();
		LCD_Puts(0,0,"triangle");
		HAL_Delay(100);
   	create_trangle_pulse();
	}
	else if(wave_model==4){}
	else if(wave_model==5){}
	else if(wave_model==6){
		LCD_Clear();
		LCD_Puts(0,0,"saw_tooth");
		HAL_Delay(100);
		create_saw_tooth_pulse();
	 }
	 HAL_Delay(100);
	 //NOW the salve should send that it work is done and waiting for another command....
	 uint8_t *slave_send = (uint8_t*) malloc(1 * sizeof(uint8_t));
   slave_send[0]=(uint8_t)1;
	 HAL_SPI_Transmit(&hspi1,slave_send,1,HAL_MAX_DELAY);*/
	 
	
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		LCD_Puts(0,0,"waiting...");
	 uint8_t *received = (uint8_t*) malloc(6 * sizeof(uint8_t));
//	uint8_t *tx_buffer_slave = (uint8_t*) malloc(6 * sizeof(uint8_t));
	// uint8_t received[6];
	 HAL_SPI_Receive(&hspi1,received,6,HAL_MAX_DELAY);
	 LCD_Clear();
	 char str[33];
	 
    frequency=(received[0]<<8)|received[1];
  	duration=(received[2]<<8)|received[3];
    wave_model=(received[4]<<8)|received[5];
	 
	 sprintf(str,"freq is %06d",frequency);
	 LCD_Puts(0,0,str);
	 sprintf(str,"du is %06d",duration);
	 LCD_Puts(0,1,str);
	 HAL_Delay(1000);
	 LCD_Clear();
	 sprintf(str,"model is %01d",wave_model);
	 LCD_Puts(0,0,str);
	 //set the duratuon for the timer
	 HAL_Delay(1000);
   //  MX_TIM3_Init();

	 //now it should transmit what frequency it got
   HAL_Delay(100);
	 LCD_Clear();
	 LCD_Puts(0,0,"done config");
	 MX_TIM3_Init();
	 MX_TIM2_Init();
	 HAL_Delay(1);
 if(wave_model==1){
		LCD_Clear();
		LCD_Puts(0,0,"sin");
		HAL_Delay(100);
  	create_sin_pulse();
	}
	else if(wave_model==2){
		LCD_Clear();
		LCD_Puts(0,0,"square pulse");
		HAL_Delay(100);
	  create_square_pulse();
	}
	else if(wave_model==3){
		LCD_Clear();
		LCD_Puts(0,0,"abs");
		HAL_Delay(100);
   	create_abs_pulse();
	}
	else if(wave_model==4){
	LCD_Clear();
		LCD_Puts(0,0,"step");
		HAL_Delay(100);
   	create_step_pulse();}
	else if(wave_model==5){
		LCD_Clear();
		LCD_Puts(0,0,"triangle");
		HAL_Delay(100);
   	create_trangle_pulse();}
	else if(wave_model==6){
		LCD_Clear();
		LCD_Puts(0,0,"saw_tooth");
		HAL_Delay(100);
		create_saw_tooth_pulse();
	 }
	 HAL_Delay(100);
	 LCD_Clear();
	 frequency=0;
	 duration=0;
   wave_model=0;
   flag=1;
   index=0;
   interupt_occured=1;
	 //NOW the salve should send that it work is done and waiting for another command....
	 /*uint8_t *slave_send = (uint8_t*) malloc(1 * sizeof(uint8_t));
   slave_send[0]=(uint8_t)22;
	 HAL_SPI_Transmit(&hspi1,slave_send,1,HAL_MAX_DELAY);*/
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  //uint16_t arr=(1000000/frequency)/200;
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (16000000/(frequency*200))-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	
  htim2.Init.Period =5000/frequency;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = duration;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void create_sin_pulse(void){
		HAL_TIM_Base_Start_IT(&htim3);	
		HAL_TIM_Base_Start_IT(&htim2);
  while (flag && interupt_occured)
    {
          GPIOC->ODR=  sin_lookUp_table[index];
    }
		GPIOC->ODR=0;
		LCD_Clear();
		LCD_Puts(0,0,"INTERUPT");
}
void create_step_pulse(void){
		HAL_TIM_Base_Start_IT(&htim3);	
		HAL_TIM_Base_Start_IT(&htim2);
 while (flag && interupt_occured)
    {     
           GPIOC->ODR=step_lookUp_table[index];	
    }
		GPIOC->ODR=0;
		LCD_Clear();
		LCD_Puts(0,0,"INTERUPT");
}
void create_abs_pulse(void){
		HAL_TIM_Base_Start_IT(&htim3);	
		HAL_TIM_Base_Start_IT(&htim2);
 while (flag && interupt_occured)
    {
        GPIOC->ODR=abs_lookUp_table[index];		
    }
		GPIOC->ODR=0;
		LCD_Clear();
		LCD_Puts(0,0,"INTERUPT");
}

void create_square_pulse(void){
		HAL_TIM_Base_Start_IT(&htim3);
		HAL_TIM_Base_Start_IT(&htim2);
 while (flag && interupt_occured)
    {
       GPIOC->ODR=square_pule_lookUp[index]; 
    }
		GPIOC->ODR=0;
		LCD_Clear();
		LCD_Puts(0,0,"INTERUPT");
}

	
void create_trangle_pulse(void){
		HAL_TIM_Base_Start_IT(&htim3);
		HAL_TIM_Base_Start_IT(&htim2);
 while (flag && interupt_occured)
    {
        GPIOC->ODR=triangle[index];		
    }
		GPIOC->ODR=0;
		LCD_Clear();
		LCD_Puts(0,0,"INTERUPT");
}
void create_saw_tooth_pulse(void){
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
 while (flag && interupt_occured)
    {
			//interupt_occured=0;
			GPIOC->ODR=saw_tooth_lookUp_table[index];
    }
		GPIOC->ODR=0;
		LCD_Clear();
		LCD_Puts(0,0,"INTERUPT");
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim2.Instance)
    {
       index=(index+1)%200;
    }
  if (htim->Instance == htim3.Instance)
  {
		
		//  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
 // if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
  //{
 //  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE); // Clear the interrupt flag
    // Disable the signal by setting the appropriate GPIO pin to a low level
		GPIOC->ODR=0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1| GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		flag=0;
		interupt_occured=0;
		HAL_TIM_Base_Stop(&htim2);
  /* USER CODE END TIM3_IRQn 1 */
//}
   
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
