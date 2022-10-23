/* USER CODE BEGIN Header */
/**
*******************************************************
Info:		Mini Project Part B
Author:		Avile Dlukwana and Kutlwisiso Belebesi
*******************************************************
Light-of-Things Concept
EEE3065/6S mini project part B receiver
******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "retarget.h"
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
 ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;


TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char buffer[100];

//TO DO:
//TASK 1
//Create global variables for debouncing and delay interval
volatile uint32_t start = 1;
volatile uint32_t delay = 1000; //initial value of the delay 1hz
volatile uint32_t adcValue = 0 ;
uint32_t binaryNum[12];
volatile uint32_t count = 0 ;
volatile uint32_t checkpoint = 0 ;
volatile uint16_t Tot_Samples[50] ;
uint32_t received_samples[12];
uint16_t Samples[4];
int Recievercheckpoint = 0; //check point received from transmitter
int Trans_checkpoint = 0; //check point received from transmitter
int samples_value = 0; //length of the current samples received
volatile int pressed = 0  ;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
uint32_t pollADC(void);
uint32_t ADCtoCRR(uint32_t adc_val);
void decToBinary(uint32_t n) ;
void binaryToDecimal(uint32_t *message, uint16_t *Samples ) ;


//void receive_data (uint8_t value);
int data_validation(int check);


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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  RetargetInit(&huart2);
  /* USER CODE BEGIN 2 */

  //TO DO:
  //Create variables needed in while loop

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //Start the PWM on TIM3 Channel 4 (Green LED)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("\rWELCOME \n");
  while(1)
  {
	  printf("\rPress T to get ready for transmission and start transmitter,\n") ;
	  printf("\rPress L to display received samples,\n");
	  printf("\rPress C to clear samples, E to exit.\n");
	  char buff;
	  scanf("%s", &buff);
	  if(buff == 'T')
	  {
		  while(!pressed)
		  {
			  pressed = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
		  }
		  if(pressed)
		  {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,1 );

			  printf("\rReceiving...\n");
			  HAL_Delay (1000); // wait for 500 ms
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,0 );
			  int i = 0  ;

			  while(i < 12)
			  {
				  received_samples[i] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
				  printf("\rbits = %lu\n", received_samples[i]);
				  HAL_Delay (1100); // wait for 500 ms
				  i++ ;
			  }
			  i = 0 ;
			  while(i < 4)
			  {
				  Samples[i] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
				  printf("\rCheckbits = %lu\n", Samples[i]);
				  HAL_Delay (1100); // wait for 500 ms
				  i++ ;
			  }

			  binaryToDecimal(received_samples,Samples) ;
			  Tot_Samples[checkpoint] = samples_value ;
			  ++checkpoint ;

			//check to see if the length is the same as transmitter checkpoint
			  printf("\rSample Value = %lu, checkpoint=%lu\n", samples_value, Trans_checkpoint);
			/*  char buf[100];
			  printf("\r\nYour name: \n");
			  scanf("%s", buf);
			  printf("\r\nHello, %s!\r\n", buf);
			  */
			if(!data_validation(Trans_checkpoint))
			{
				Tot_Samples[checkpoint] = 0 ;
				checkpoint = Trans_checkpoint ;
				for(int delay = 0 ; delay < 2000 ; delay+= 150)
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,1 );
					HAL_Delay (100); // wait for 500 ms
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,0 );
					HAL_Delay (100); // wait for 500 ms
				}
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,1 );
				HAL_Delay (2000); // wait for 500 ms
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,0 );
			}
		  }
		  pressed = 0 ;
		  printf("\n");
		  printf("\n");

	  }
	else if(buff == 'L')
	{
		if(checkpoint == 0)
		{
			  printf("\rNo Samples\n") ;
		}
		else
		{
			printf("\rSamples\n") ;
		  int i = 0 ;
		  while(i < checkpoint)
		  {
			  printf("\r%d\n", Tot_Samples[i]);
			  i++ ;
		  }
		}
		printf("\n") ;
		printf("\n") ;
	}

	else if(buff == 'C')
	{
		printf("\rClearing...\n") ;
		int i = 0 ;
		while(i < checkpoint)
		{
			Tot_Samples[i] = 0 ;
			i++ ;
		}
		printf("\rComplete\n") ;
		checkpoint = 0 ;
		printf("\n") ;
		printf("\n") ;
	}
	else if(buff == 'E')
	{
		printf("\rexiting...\n") ;
		printf("\rPress Reset button to start program.\n") ;
		while(1) ;
	}
	else
	{
		printf("\rIncorrect User input !!\n") ;
		printf("\n") ;
		printf("\n") ;

	}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 47999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 9600;
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

/*Prints out values on Putty	*/
void debugPrintln(UART_HandleTypeDef * uart_handle, char _out[])
{
	HAL_UART_Transmit(uart_handle, (uint8_t *)_out, strlen(_out),60);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(uart_handle, (uint8_t *)newline, 2, 60);
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
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}
void binaryToDecimal(uint32_t *message, uint16_t *Samples )
{
    // Initializing base value to 1, i.e 2^0
    int base = 1 ;
    samples_value = 0 ;
    for (int i = 0; i < 12; i++) {
        if (message[i] == 1)
        	samples_value += base;
        base = base * 2;
    }
    base = 1 ;
    Trans_checkpoint = 0 ;
    for (int i = 0; i < 4; i++) {
        if (Samples[i] == 1)
        	Trans_checkpoint += base;
        base = base * 2;
    }

}

void decToBinary(uint32_t n)
{
    // array to store binary number
    // counter for binary array
	for(int i = 0 ; i < 12 ; i++)
	{
        binaryNum[i] = 0 ;
	}
    int i = 0;
    while (n > 0) {
        // storing remainder in binary array
        binaryNum[i] = n % 2;
        n = n / 2;
        i++;
    }
    count = i ;
}

int data_validation(int check)
{
	//if the received samples is the same as the checkpoint from the receiver return true [1]
	if (check == checkpoint)
	{
		return 1;
	}
	else
	{
		return 0;
	}
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  //////////////////////////////////////////////////////////
  //this is the configuration for GPIO input 5
  //////////////////////////////////////////////////
  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	//TO DO:
	//TASK 1
	//Switch delay frequency
	if(HAL_GetTick() - start > 500)
	{
		//Trans_checkpoint ;
		//adcValue = pollADC();
	}
	start = HAL_GetTick();
	HAL_GPIO_EXTI_IRQHandler(B1_Pin); // Clear interrupt flags

}

uint32_t pollADC(void){
	//TO DO:
	//TASK 2
	// Complete the function body
	 HAL_ADC_Start(&hadc);
	 HAL_ADC_PollForConversion(&hadc, 1000);
	 uint32_t val = HAL_ADC_GetValue(&hadc);
	 HAL_ADC_Stop(&hadc);
	return val;
}

uint32_t ADCtoCRR(uint32_t adc_val){
	//TO DO:
	//TASK 3
	// Complete the function body
	double adc_value = adc_val ;
	double percentage = adc_value/4095 ;
	uint32_t val = percentage * 47999;
	//HAL_TIM_SetCompare(htim3,TIM_CHANNEL_4,ccr_val);
	//HINT: The CRR value for 100% DC is 47999 (DC = CRR/ARR = CRR/47999)
	//HINT: The ADC range is approx 0 - 4095
	//HINT: Scale number from 0-4096 to 0 - 47999
	return val;
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
