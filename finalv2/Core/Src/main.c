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
#include "../../Drivers/LCD16x2Lib/LCD.h"
#include "stdio.h"
#include <stdlib.h> // needed for malloc function
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
 ADC_HandleTypeDef hadc1;
 SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
int read_keypad(void);
int get_timer_from_p(void);
int read_one_from_keypad(void);
int get_row(void);
int get_frequence_from_p(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
	 MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   	LCD_Puts(4, 0, "99243018");
 	HAL_Delay(2000);
 	LCD_Clear();
	LCD_Puts(0, 0, "1.sin 2.squ 3.ab");
	LCD_Puts(0, 1, "4.stp 5.tri 6.sa");
	HAL_Delay(5000);
	int input=-10;
	char str[33];
	while(1){
  	input=read_keypad();
		LCD_Clear();
		sprintf(str,"%01d",input);
		LCD_Puts(0,0,str);
		HAL_Delay(2000);
		if(input>=1 && input<=6)
				break;
		else 
			continue;
	}
	
	LCD_Clear();
	LCD_Puts(0,0,"You chose wave:");
	sprintf(str, "%01d", input);
	LCD_Puts(4, 1,str);
	HAL_Delay(2000);
	HAL_Delay(10);
	LCD_Clear();
	LCD_Puts(0, 0,"select time :");
	uint16_t duration_on_lcd,freq_on_lcd;
	//uint16_t duration, frequency;
	while(read_one_from_keypad()!=-2){
			duration_on_lcd=get_timer_from_p();
		  //duration_on_lcd=duration* 9500/ 4095+500;
			sprintf(str, "%05d", duration_on_lcd);
			LCD_Clear();
			LCD_Puts(4, 1,str);
   }
	LCD_Clear();
	LCD_Puts(0, 0,"select frequency :");
		while(read_one_from_keypad()!=-2){
				freq_on_lcd=get_frequence_from_p();
	  		//freq_on_lcd= frequency* 999/4095 + 1;
				sprintf(str, "%05d", freq_on_lcd);
				LCD_Clear();
				LCD_Puts(4, 1,str);
			}
	   
    uint16_t data[3];
		data[0]=freq_on_lcd;
		data[1]=duration_on_lcd;
		data[2]=input;
		//declaring rx_and tx_buffer for the master to send and receive from the slave
		uint8_t *tx_buffer = (uint8_t*) malloc(6 * sizeof(uint8_t));
		uint8_t *rx_buffer = (uint8_t*) malloc(1 * sizeof(uint8_t));
		LCD_Clear();	
		for (int i = 0; i < 3; i++) {
    tx_buffer[i * 2] = (uint8_t)(data[i] >> 8); // Most significant byte
    tx_buffer[i * 2 + 1] = (uint8_t)(data[i]); // Least significant byte
		sprintf(str, "%msb is %08d",tx_buffer[i*2]);
		LCD_Puts(0, 0,str);
		sprintf(str, "lsb is %08d",tx_buffer[i*2+1]);
		LCD_Puts(0, 1,str);
  	HAL_Delay(1000);
		LCD_Clear();
    }
		//send the data to the salve
		HAL_SPI_Transmit(&hspi1,tx_buffer,6,HAL_MAX_DELAY);
		LCD_Puts(0,0,"done");
	  uint16_t freq_r=0,model_r=0,du_r=0;
		HAL_Delay(100);
		//LCD_Clear();
		//converting to mod
		
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
		//rx_buffer[0]=0;
		//receiving data from the salve and check if we have sent it right
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC5 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int get_timer_from_p(void){
  uint16_t adc_value;
  uint16_t time_ms;
  HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    // Start ADC conversion
   // HAL_ADC_Start(&hadc1);
    // Wait for ADC conversion to complete
   // HAL_ADC_PollForConversion(&hadc1, 100);
    // Read ADC value and calculate time
  //  adc_value = HAL_ADC_GetValue(&hadc1);
  //time_ms=adc_value;
 //time_ms=potentiometerValue+100;
    time_ms = adc_value * 9500 / 4095 + 500; // Map ADC value to 500-10000ms range
	//  time_ms=adc_value;
    // Do something with time, such as delay for that amount of time
    //HAL_Delay(time_ms);
    // Wait for next iteration
  // HAL_Delay(100);
return time_ms;
}
int get_frequence_from_p(void){
uint16_t adc_value;
  uint16_t frequencee;
 HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
	 // frequencee=adc_value;
    frequencee = adc_value * 999/4095+ 1; // Map ADC value to 500-10000ms range
return frequencee;
}

int get_row(void)
{
  int input_state=1;
  int row=-1;
  input_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);//pin 3 be satr aval motasel hast
  if (input_state ==0)//age input =1 bashe pas satr 1 ro drim
  {
   row=1;
   return row;
  }
   input_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
  if (input_state ==0)
  {
   row=2;
   return row;
  }
    input_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
  if (input_state ==0)
  {
   row=3;
   return row;
  }
    input_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
  if (input_state ==0)
  {
   row=4;
   return row;
  }
return row;
}
int read_one_from_keypad(void){
int key_pad_get[4][3]={
{1,2,3},
{4,5,6},
{7,8,9},
{-1,0,-2}
};
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
int col=-1,row=-1;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_RESET);//miam port_c_2=1 miknm yani sotun aval
  HAL_Delay(30);
  row=get_row();

  if (row!=-1)
  {
   col=1;
   return key_pad_get[row-1][col-1] ;
  }
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_SET);//dobare pin2_c ro 0 miknm
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);//pin 1 ro 1 miknm
  HAL_Delay(30);
  row=get_row();
  if (row!=-1)
  {
   col=2;
   return key_pad_get[row-1][col-1] ;
  }
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
  HAL_Delay(30);
  row=get_row();
  if (row!=-1)
  {
      col=3;
      return key_pad_get[row-1][col-1] ;
  }
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
return -10;
}

int read_keypad(void)
{
//LCD_Puts(4,0,"processing");
int key_pad_get[4][3]={
{1,2,3},
{4,5,6},
{7,8,9},
{-1,0,-2}
};
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
int col=-1,row=-1;
while (1){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_RESET);//miam port_c_2=1 miknm yani sotun aval
  HAL_Delay(30);
  row=get_row();

  if (row!=-1)
  {
   col=1;
   return key_pad_get[row-1][col-1] ;
  }
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_SET);//dobare pin2_c ro 0 miknm
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);//pin 1 ro 1 miknm
  HAL_Delay(30);
  row=get_row();
  if (row!=-1)
  {
   col=2;
   return key_pad_get[row-1][col-1] ;
  }
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
  HAL_Delay(30);
  row=get_row();
  if (row!=-1)
  {
      col=3;
      return key_pad_get[row-1][col-1] ;
  }
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
 }
return -1;
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