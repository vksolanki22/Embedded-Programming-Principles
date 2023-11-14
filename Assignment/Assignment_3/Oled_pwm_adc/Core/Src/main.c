/* USER CODE BEGIN Header */
/*******************************************************************************
 * FILE               : main.c
 * PROJECT			  : PROG8126 - Assignment #3 - LCD RGB temp LED
 * PROGRAMMER		  : Vivek Solanki (8925804)
 * First Version	  : 11/11/2023
 * Description        : This File contains all the required function to full fill the
 * 						requirement of Assignment 3, which should display Temparature
 * 						on Top right corner as well as it should glow the RGB LED
 * 						according to Detected Temperature as follow.
 *
 *						Below -15 C    	-->   PURPLE
 *						-15 C  to 5.0 C -->   blue
 *						5.1 C to 15 C   -->   yellow
 *						15.1 C to 25 C	-->   orange
 *						Above 25.1 C    -->   red
 *
 *	Above & Beyond	 : For Above and Beyond I have added one Push button which will
 *					   change the Temparature unit from °C to °F and Vice-Versa
 *					   Furthermore, I displayes the Temparature with Unit on OLED
 *					   with color according to given ranges.
 *  ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1331.h"
#include "logo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DEBUG_ON
//#define DEBUG_ON
typedef enum
{
	CHANNEL1 = 0, CHANNEL2, CHANNEL3, CHANNEL_MAX
} ChannelNum_e;

typedef enum
{
	TEMPERATURE_UNIT_CELSIUS = 0, TEMPERATURE_UNIT_FAHRENHEIT, TEMPERATURE_UNIT_MAX
} TemperatureUnit_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*******************************************************************************
 * FUNCTION      : setPwmDutyCycle()
 * PARAMETERS    : timer_channel_num (input) : Pass here the channel number
 *  			    Example : TIM_CHANNEL_1, TIM_CHANNEL_2 or TIM_CHANNEL_3
 *  			    percentage (input) 		 : Pass here the duty cycle percentage (0-100)
 * RETURNS       : none
 * DESCRIPTION   : This function sets the duty cycle for timer 1's
 *				 	whatever channel number is passed.
 *******************************************************************************
 */
void setPwmDutyCycle(ChannelNum_e timer_channel_num, uint16_t percentage);
/*******************************************************************************
 * FUNCTION      : getTempratureValue()
 * PARAMETERS    : uint32_t iSensorValue
 * RETURNS       : float
 * DESCRIPTION   : This Function Calculates the Temparature value from value readed
 * 				from sensor and returns float value which will be on °C.
 *******************************************************************************
 */
static float getTempratureValue(uint32_t iSensorValue);

/*******************************************************************************
 * FUNCTION      : conevertToFerenhit()
 * PARAMETERS    : float temperatureValue
 * RETURNS       : float
 * DESCRIPTION   : This Function converts the value from °C to °F
 *******************************************************************************
 */
static float conevertToFerenhit(float temperatureValue);

/*******************************************************************************
 * FUNCTION      : displayTemparatureAndGlowColour()
 * PARAMETERS    : float temperatureValue
 * RETURNS       : none
 * DESCRIPTION   : This function derives the temprature range from temperatureValue
 *					then accoding to defined raangle display Temprature string on OLED
 *					and glow LED with colour.
 *******************************************************************************
 */
static void displayTemparatureAndGlowColour(float temperatureValue);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t uhADCxConvertedValue = 0;
volatile uint8_t buttonState = 0;
TemperatureUnit_e temperatureUnit = TEMPERATURE_UNIT_CELSIUS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t print_flag = 0;
uint8_t idx;
char tempSign[14];

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	float temperatureValue = 0.0;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init ();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config ();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* USER CODE BEGIN 2 */
	MX_GPIO_Init ();
	MX_USART2_UART_Init ();
	MX_SPI1_Init ();
	MX_TIM1_Init ();
	MX_ADC1_Init ();
	ssd1331_init ();

	/* Run the ADC calibration in single-ended mode */
	if (HAL_ADCEx_Calibration_Start (&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler ();
	}

	/*
	 * Note : for the n channels HAL_TIMEx_PWMN_Start() is used
	 * for normal channels HAL_TIM_PWM_Start
	 */
	HAL_TIMEx_PWMN_Start (&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/*##-3- Start the conversion process #######################################*/
		if (HAL_ADC_Start (&hadc1) != HAL_OK)
		{
			/* Start Conversation Error */
			Error_Handler ();
		}

		/*##-4- Wait for the end of conversion #####################################*/
		/*  For simplicity reasons, this example is just waiting till the end of the
		 conversion, but application may perform other tasks while conversion
		 operation is ongoing. */
		if (HAL_ADC_PollForConversion (&hadc1, 10) != HAL_OK)
		{
			/* End Of Conversion flag not set on time */
			Error_Handler ();
		}
		else
		{
			/* ADC conversion completed */
			/*##-5- Get the converted value of regular channel  ########################*/
			uhADCxConvertedValue = HAL_ADC_GetValue (&hadc1);
			buttonState = HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_11);
			HAL_ADC_Stop (&hadc1);
			print_flag++;

			if (print_flag == 1000)
			{
				print_flag = 0;
			}

			// convert sensor value to TempratureValue
			temperatureValue = getTempratureValue (uhADCxConvertedValue);

			// Derive temparature unit to be displayed
			if (buttonState == GPIO_PIN_RESET)
			{
				if (temperatureUnit == TEMPERATURE_UNIT_CELSIUS)
				{
					temperatureUnit = TEMPERATURE_UNIT_FAHRENHEIT;
				}
				else
				{
					temperatureUnit = TEMPERATURE_UNIT_CELSIUS;
				}
			}

			// Making string to be printed on OLED Display with Temparature unit with Button state
			if (temperatureUnit == TEMPERATURE_UNIT_FAHRENHEIT)
			{
				idx += snprintf (tempSign, sizeof(tempSign), "%.1f",
				                 conevertToFerenhit (temperatureValue));
				snprintf (tempSign + idx, sizeof(tempSign) - idx, "%s", " F");
#ifdef DEBUG_ON
				printf ("buttonState = %d, tempSign = %s\r\n", buttonState, tempSign);
#endif

			}
			else
			{
				idx += snprintf (tempSign, sizeof(tempSign), "%.1f", temperatureValue);
				snprintf (tempSign + idx, sizeof(tempSign) - idx, "%s", " C");
#ifdef DEBUG_ON
				printf ("buttonState = %d, tempSign = %s\r\n", buttonState, tempSign);
#endif
			}
			idx = 0;

			// Sets Colour on RGB LED
			displayTemparatureAndGlowColour (temperatureValue);

			// Delay to take update temparature
			HAL_Delay (500);
		}
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler ();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess ();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler ();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler ();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode ();
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

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init (&hadc1) != HAL_OK)
	{
		Error_Handler ();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel (&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler ();
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
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init (&hspi1) != HAL_OK)
	{
		Error_Handler ();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 8;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 100;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init (&htim1) != HAL_OK)
	{
		Error_Handler ();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource (&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler ();
	}
	if (HAL_TIM_PWM_Init (&htim1) != HAL_OK)
	{
		Error_Handler ();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization (&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler ();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 100;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler ();
	}
	sConfigOC.Pulse = 120;
	if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler ();
	}
	sConfigOC.Pulse = 130;
	if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler ();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput =
	TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime (&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler ();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit (&htim1);

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
	if (HAL_UART_Init (&huart2) != HAL_OK)
	{
		Error_Handler ();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin (
	GPIOB,
	                   LD3_Pin | SSD1331_CS_Pin | SSD1331_DC_Pin | SSD1331_RES_Pin,
	                   GPIO_PIN_RESET);

	/*Configure GPIO pin : PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD3_Pin SSD1331_CS_Pin SSD1331_DC_Pin SSD1331_RES_Pin */
	GPIO_InitStruct.Pin = LD3_Pin | SSD1331_CS_Pin | SSD1331_DC_Pin | SSD1331_RES_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*******************************************************************************
 * FUNCTION      : setPwmDutyCycle()
 * PARAMETERS    : timer_channel_num (input) : Pass here the channel number
 *  			    Example : TIM_CHANNEL_1, TIM_CHANNEL_2 or TIM_CHANNEL_3
 *  			    percentage (input) 		 : Pass here the duty cycle percentage (0-100)
 * RETURNS       : none
 * DESCRIPTION   : This function sets the duty cycle for timer 1's
 *				 	whatever channel number is passed.
 *******************************************************************************
 */
void setPwmDutyCycle(ChannelNum_e timer_channel_num, uint16_t percentage)
{
	if (percentage > 100)
	{
		percentage = 100;
	}
	else if (percentage < 0)
	{
		percentage = 0;
	}
	__HAL_TIM_SET_COMPARE(&htim1, timer_channel_num, percentage);
}

/*******************************************************************************
 * FUNCTION      : getTempratureValue()
 * PARAMETERS    : uint32_t iSensorValue
 * RETURNS       : float
 * DESCRIPTION   : This Function Calculates the Temparature value from value readed
 * 				from sensor and returns float value which will be on °C.
 *******************************************************************************
 */
static float getTempratureValue(uint32_t iSensorValue)
{
	return (((3.3 * iSensorValue) / 4096.0) - 0.5) * 100;
}

/*******************************************************************************
 * FUNCTION      : conevertToFerenhit()
 * PARAMETERS    : float temperatureValue
 * RETURNS       : float
 * DESCRIPTION   : This Function converts the value from °C to °F
 *******************************************************************************
 */
static float conevertToFerenhit(float temperatureValue)
{
	return temperatureValue * (9 / 5) + 32;
}

/*******************************************************************************
 * FUNCTION      : displayTemparatureAndGlowColour()
 * PARAMETERS    : float temperatureValue
 * RETURNS       : none
 * DESCRIPTION   : This function derives the temprature range from temperatureValue
 *					then accoding to defined raangle display Temprature string on OLED
 *					and glow LED with colour.
 *******************************************************************************
 */
static void displayTemparatureAndGlowColour(float temperatureValue)
{
	int dutyCycle = 100;
	static float prevTemp;

	// To avoid over writing temprrature value because of sign
	if((prevTemp < 0.0 && temperatureValue > 0.0) || (prevTemp > 0.0 && temperatureValue < 0.0))
	{
		ssd1331_clear_screen (BLACK);
	}

	if (temperatureValue < -15.0)
	{
		ssd1331_display_string (2, 2, "Temp.:", FONT_1206, PURPLE);
		ssd1331_display_string (54, 2, tempSign, FONT_1206, PURPLE);
		setPwmDutyCycle (TIM_CHANNEL_1, dutyCycle);	//Generating Shades of red
		setPwmDutyCycle (TIM_CHANNEL_2, dutyCycle);	//Generating Shades of green
		setPwmDutyCycle (TIM_CHANNEL_3, dutyCycle);	//Generating Shades of blue
#ifdef DEBUG_ON
		printf ("PURPLE\r\n");
#endif
	}
	else if (temperatureValue >= -15.0 && temperatureValue <= 5.0)
	{
		ssd1331_display_string (2, 2, "Temp.:", FONT_1206, PURPLE);
		ssd1331_display_string (54, 2, tempSign, FONT_1206, BLUE);
		setPwmDutyCycle (TIM_CHANNEL_1, 0);
		setPwmDutyCycle (TIM_CHANNEL_2, 0);
		setPwmDutyCycle (TIM_CHANNEL_3, dutyCycle);
#ifdef DEBUG_ON
		printf ("Blue\r\n");
#endif
	}
	else if (temperatureValue >= 5.1 && temperatureValue <= 15.0)
	{
		ssd1331_display_string (2, 2, "Temp.:", FONT_1206, PURPLE);
		ssd1331_display_string (54, 2, tempSign, FONT_1206, YELLOW);
		setPwmDutyCycle (TIM_CHANNEL_1, dutyCycle);
		setPwmDutyCycle (TIM_CHANNEL_2, dutyCycle);
		setPwmDutyCycle (TIM_CHANNEL_3, 0);
#ifdef DEBUG_ON
		printf ("Yellow\r\n");
#endif
	}
	else if (temperatureValue >= 15.1 && temperatureValue <= 25.0)
	{
		ssd1331_display_string (2, 2, "Temp.:", FONT_1206, PURPLE);
		ssd1331_display_string (54, 2, tempSign, FONT_1206, ORANGE);
		setPwmDutyCycle (TIM_CHANNEL_1, 100);
		setPwmDutyCycle (TIM_CHANNEL_2, 50);
//		setPwmDutyCycle (TIM_CHANNEL_1, dutyCycle);
//		setPwmDutyCycle (TIM_CHANNEL_2, 15);
		setPwmDutyCycle (TIM_CHANNEL_3, 0);
#ifdef DEBUG_ON
		printf ("Orange\r\n");
#endif
	}
	else if (temperatureValue >= 25.1)
	{
		ssd1331_display_string (2, 0, "Temp.:", FONT_1206, PURPLE);
		ssd1331_display_string (54, 2, tempSign, FONT_1206, RED);
		setPwmDutyCycle (TIM_CHANNEL_1, dutyCycle);
		setPwmDutyCycle (TIM_CHANNEL_2, 0);
		setPwmDutyCycle (TIM_CHANNEL_3, 0);
#ifdef DEBUG_ON
		printf ("Red\r\n");
#endif
	}
	ssd1331_display_string (2, 20, "Press button to", FONT_1206, GOLDEN);
	ssd1331_display_string (2, 32, "change unit", FONT_1206, GOLDEN);

	prevTemp = temperatureValue;
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
	__disable_irq ();
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
