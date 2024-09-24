/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "MX25L.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define FLASH_TEST
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c4;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C4_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t rxData[2];
uint8_t Modem_RX[1000];
uint16_t Modem_RX_Index = 0;
uint8_t Modem_RX_Flag = 0;
uint32_t Modem_RX_Time = 0;

uint8_t Debug_RX[1000];
uint16_t Debug_RX_Index = 0;
uint8_t Debug_RX_Flag = 0;
uint8_t debug_echo = 1;
uint8_t adc_complete_flag1 = 0;
uint8_t adc_complete_flag2 = 0;
__IO uint32_t adc_results1[32];
uint32_t adc_results2[32];

/* Variable to report ADC sequencer status */
uint8_t         ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

int _write(int fd, char *str, int len)
{
#if 0
	for(int i=0; i<len; i++)
	{
		HAL_UART_Transmit(&huart5, (uint8_t *)&str[i], 1, 0xFFFF);
	}
#endif
	HAL_UART_Transmit(&huart5, (uint8_t *)str, len, 0xFFFF);
	return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    /*
        This will be called once data is received successfully,
        via interrupts.
    */

     /*
       loop back received data
     */
	if(huart == &huart5)
	{
		 HAL_UART_Receive_IT(&huart5, rxData, 1);
		 if(debug_echo)
				 HAL_UART_Transmit(&huart5, rxData, 1, 1000);

		 Debug_RX[Debug_RX_Index++] = rxData[0];
		 if(rxData[0]=='\n')
		 {
			 Debug_RX_Flag = 1;
		 }
	}
	else if(huart == &huart2)
	{
		HAL_UART_Receive_IT(&huart2, rxData, 1);
		Modem_RX[Modem_RX_Index++] = rxData[0];
		if((Modem_RX[Modem_RX_Index-1] == '\n')&&(Modem_RX[Modem_RX_Index-2] == '\r'))
		{
			Modem_RX[Modem_RX_Index-1] = '.';
			Modem_RX[Modem_RX_Index-2] = '.';
			Modem_RX_Time = HAL_GetTick();
			Modem_RX_Flag |= 0x01;
			if((Modem_RX[Modem_RX_Index-4] == 'O')&&(Modem_RX[Modem_RX_Index-3] == 'K'))
			{
				Modem_RX_Flag |= 0x02;
			}
			else if((Modem_RX[Modem_RX_Index-5] == 'R')&&(Modem_RX[Modem_RX_Index-4] == 'O')&&(Modem_RX[Modem_RX_Index-3] == 'R'))
			{
				Modem_RX_Flag |= 0x02;
			}
		}
	}
}
#if 0
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc == &hadc1)
    {
        adc_complete_flag1 = 1;
    }
#if 0
    else if(hadc == &hadc2)
    {
        adc_complete_flag2 = 1;
    }
#endif
}
#endif

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Report to main program that ADC sequencer has reached its end */
  if(hadc == &hadc1)
  {
    ubSequenceCompleted = SET;
  }
}


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
  uint32_t tickFreq = 0;
  uint16_t adcVal[2]= {0,};
  uint8_t adcIndex = 0;
  uint16_t adcCount =0;
  uint8_t SPI_tx_data[4]={0,};
  uint8_t SPI_rx_data[4]={0,};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //uint8_t ATCMD[5] = "AT\r\n";
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C4_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart5, rxData, 1);
  HAL_UART_Receive_IT(&huart2, rxData, 1);
  HAL_UART_Transmit(&huart5, (uint8_t *)"Start!!\r\n",9 , 0xFFFF);
  printf("Hello World!\r\n");
  HAL_GPIO_WritePin(GPIOD, RF_POWER_Pin|NRESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
  SPI_tx_data[0] = 0x9f;
  HAL_SPI_TransmitReceive(&hspi1, SPI_tx_data, SPI_rx_data, 4, 100);
  printf("MX25L3233 RDID =0x%02x%02x%02x\r\n", SPI_rx_data[1], SPI_rx_data[2], SPI_rx_data[3]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("PWRKEY Down\r\n");
  HAL_GPIO_WritePin(GPIOD, PWRKEY_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  printf("PWRKEY UP\r\n");
  HAL_GPIO_WritePin(GPIOD, PWRKEY_Pin, GPIO_PIN_RESET);
  tickFreq =  HAL_GetTickFreq();
#ifdef FLASH_TEST
  ////////////////Flash TEST/////////
  MX25L_SetStatus(0x00);
  	uint8_t status = MX25L_GetStatus();
  	printf("\r\nGet status:%d\r\n",status);
  	printf("-----------------\r\n");
  	uint8_t buffer[257];
  	MX25L_Get_Identification((uint8_t *)buffer);
  	printf("Identification read:\r\n");
  	printf("Byte 0: %x\r\n", buffer[0]);
  	printf("Byte 1: %x\r\n", buffer[1]);
  	printf("Byte 2: %x\r\n", buffer[2]);
  	printf("-----------------\r\n");

  	printf("Clearing chip ... ");
  	MX25L_ChipErase();
  	printf("done\r\n");
  	printf("-----------------\r\n");
  	buffer[256];
  	uint8_t ch = 0x21;
  	uint8_t page = 0;
  	uint8_t endFlash = 1;
  	////////////////////
#endif
  //HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_results1, 4);

#if 0
  //adc_done = 0;
  adc_complete_flag1 = 0;
  adc_complete_flag2 = 0;
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_results1, 32);
  //HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adc_results2, 32);
#if 1
  while (!adc_complete_flag1) {
    HAL_Delay(1);
  }
  adc_complete_flag1 =0;
#if 0
  while (!adc_complete_flag2) {
    HAL_Delay(1);
  }
  adc_complete_flag2 =0;
#endif
#endif
  
  for (int i = 0; i < 32; i++) {
    printf("ADC%d - 1: %5d(%04X)\n", i + 1, (adc_results1[i]&0x0FFF), (adc_results1[i]&0x0FFF));
  }
  
  HAL_ADC_Stop_DMA(&hadc1);
#endif
#if 0
  /* Run the ADC calibration in single-ended mode */
   if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
   {
     /* Calibration Error */
     Error_Handler();
   }


   /*##-3- Start the conversion process #######################################*/
   if (HAL_ADC_Start(&hadc1) != HAL_OK)
   {
     /* Start Conversation Error */
	 printf("ADC Start Error\n");
	 //continue;
     //Error_Handler();
   }
   for (int i = 0; i < 32; i++) {
   /*##-4- Wait for the end of conversion #####################################*/
   /*  For simplicity reasons, this example is just waiting till the end of the
       conversion, but application may perform other tasks while conversion
       operation is ongoing. */
   //if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
   HAL_StatusTypeDef result;
   //result = HAL_ADC_PollForConversion(&hadc1, 10);
   if((result = HAL_ADC_PollForConversion(&hadc1, 10)) != HAL_OK)
   {
     /* End Of Conversion flag not set on time */
     //Error_Handler();
	   printf("ERROR %d %d\n", i, result);
   }
   else
   {
     /* ADC conversion completed */
     /*##-5- Get the converted value of regular channel  ########################*/
	   adc_results1[0] = HAL_ADC_GetValue(&hadc1);
	   printf("ADC%2d - 1: %5d(%04X)\n",i+1, (adc_results1[0]&0x0FFF), (adc_results1[0]&0x0FFF));
   }
  }
   #endif
   
  while (1)
  {
  /* Start ADC conversion */
    /* Since sequencer is enabled in discontinuous mode, this will perform    */
    /* the conversion of the next rank in sequencer.                          */
    /* Note: For this example, conversion is triggered by software start,     */
    /*       therefore "HAL_ADC_Start()" must be called for each conversion.  */
    /*       Since DMA transfer has been initiated previously by function     */
    /*       "HAL_ADC_Start_DMA()", this function will keep DMA transfer      */
    /*       active.                                                          */
  #if 0
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        printf("start Error\r\n");
      Error_Handler(); 
    }
    #endif
     
    /* After each intermediate conversion, 
       - EOS remains reset (EOS is set only every third conversion)
       - EOC is set then immediately reset by DMA reading of DR register.
     Therefore, the only reliable flag to check the conversion end is EOSMP
     (end of sampling flag).
     Once EOSMP is set, the end of conversion will be reached when the successive
     approximations are over. 
     RM indicates 12.5 ADC clock cycles for the successive approximations
     duration with a 12-bit resolution, or 185.ns at 80 MHz.
     Therefore, it is known that the conversion is over at
     HAL_ADC_PollForConversion() API return */
    /* Note: Function "HAL_ADC_PollForConversion(&AdcHandle, 1)" could   */
    /*       be used here, but with a different ADC configuration        */
    /*      (this function cannot be used if ADC configured in DMA mode  */
    /*       and polling for end of each conversion):                    */
    /*       a possible configuration is ADC polling for the entire      */
    /*       sequence (ADC init parameter "EOCSelection" set             */
    /*       to ADC_EOC_SEQ_CONV) (this also induces that ADC            */
    /*       discontinuous mode must be disabled).                       */
    #if 0
    if (HAL_ADC_PollForEvent(&hadc1, ADC_EOSMP_EVENT, 10) != HAL_OK)
    {
        printf("adc event Error\r\n");
      Error_Handler(); 
    }  
    #endif
    if (ubSequenceCompleted == SET)
    {
        ubSequenceCompleted = RESET;
        printf("ADC 1: %5d(%04X) 2: %5d(%04X) 3: %5d(%04X) 4: %5d(%04X)\n", (adc_results1[0]&0x0FFF), (adc_results1[0]&0x0FFF),
            (adc_results1[1]&0x0FFF), (adc_results1[1]&0x0FFF),
            (adc_results1[2]&0x0FFF), (adc_results1[2]&0x0FFF),
            (adc_results1[3]&0x0FFF), (adc_results1[3]&0x0FFF));
        //HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_results1, 4);
    }
    #if 0
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
    {
        printf("stop Error\r\n");
      Error_Handler(); 
    }
    #endif

#ifdef FLASH_TEST
	  if(endFlash)
	  {
		  memset(buffer,ch, 256);
		printf("Writing page %d ... ", page);
		MX25L_Write(page * 256, buffer, 256);
		printf("done\r\n");
		printf("Data: %s\r\n", buffer);
		printf("Reading page %d ... ", page);
		memset(buffer,0, 256);
		MX25L_Read(page * 256, buffer, 256);
		//buffer[256] = {00;
		printf("done\r\n");
		printf("Data: %s\r\n", buffer);
		printf("\r\n\r\n");
		ch ++;
		if(ch > 0x7E) ch = 0x21;
		page ++;
		if(page > 200)
			{
				endFlash = 0;
				printf("Flash test Finish!!\r\n");
			}
		HAL_Delay(1000);
	  }
#endif
#if 0
	  if(adcCount++ > 500)
	  {
		  adcCount = 0;
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  adcVal[adcIndex++] = HAL_ADC_GetValue(&hadc1);
		  printf("ADC 0:%4d, 1:%4d\r\n", adcVal[0], adcVal[1]);
		  if(adcIndex > 1)
			  adcIndex = 0;
	  }
#endif
	  tickFreq =  HAL_GetTickFreq();
	  if(Modem_RX_Flag & 0x02 )
	  {
		  Modem_RX_Flag = 0;
		  Modem_RX[Modem_RX_Index]=0;
		  printf("[Modem RX] : %s\r\n", Modem_RX);
		  Modem_RX_Index = 0;
	  }
	  else if(Debug_RX_Flag == 1)
	  {
		  Debug_RX[Debug_RX_Index] =0;
		  printf("[Debug TX] %s", Debug_RX);
		  HAL_UART_Transmit(&huart2, Debug_RX, Debug_RX_Index, 1000);
		  if(strncmp((char *)Debug_RX, "echo off", 8)==0)
		  {
			  printf("Echo OFF\r\n");
			  debug_echo = 0;
		  }
		  else if(strncmp((char *)Debug_RX, "echo on", 7)==0)
		  {
			  printf("Echo ON\r\n");
			  debug_echo = 1;
		  }
		  Debug_RX_Index =0;
		  Debug_RX_Flag = 0;
	  }
	  else if(Modem_RX_Flag & 0x01)
	  {
		  if((HAL_GetTick() - Modem_RX_Time) > (500 + tickFreq))
		  {
			  //printf("M %ld,F:%ld\r\n", (HAL_GetTick() - Modem_RX_Time), tickFreq);
			  Modem_RX_Time = HAL_GetTick();
			  Modem_RX_Flag |= 0x02;
		  }
	  }
	  else if((Modem_RX_Index > 0)&&((HAL_GetTick() - Modem_RX_Time) > (20000 + tickFreq)))
	  {
		  Modem_RX_Flag |= 0x02;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return 0;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10D19CE4;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRESET_GPIO_Port, NRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PWRKEY_Pin|RF_POWER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRESET_Pin */
  GPIO_InitStruct.Pin = NRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWRKEY_Pin RF_POWER_Pin */
  GPIO_InitStruct.Pin = PWRKEY_Pin|RF_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	printf("ERROR\n");
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
