/* USER CODE BEGIN Header */
/*******************************************************************************
 * FILE               : main.c
 * PROJECT			  : PROG8126 - #Final Exam
 * PROGRAMMER		  : Vivek Solanki (8925804)
 * First Version	  : 13/12/2023
 * Description        : This File contains all the required function to full fill the
 * 						requirement of Final Exam, which should read the user name user height and weight
 * 						and the calculate BMI and prints that on OLED
 *
 *  ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1331.h"
#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USER_NAME_LEN			20

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum
{
	USER_INFO_STATE_WELCOME,
	USER_INFO_STATE_NAME,
	USER_INFO_STATE_HEIGHT,
	USER_INFO_STATE_WEIGHT,
	USER_INFO_STATE_BMI,
	USER_INFO_STATE_MAX
} userInfoState_e;

typedef struct
{
	char name[USER_NAME_LEN + 1];
	float height;
	float weight;
	float userBMI;
} userInfo_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

static userInfoState_e userInfoState = USER_INFO_STATE_WELCOME;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*******************************************************************************
 * FUNCTION      : startUpOLEDSplashScreen
 * PARAMETERS    : None
 * RETURNS       : nothing
 * DESCRIPTION   : displays Welcom for 2s
 * 				   on line 1 of the display and Disappears
 *******************************************************************************
 */
void startUpOLEDSplashScreen(void)
{
	char stringBuffer[16] = { 0 };
	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, sizeof(stringBuffer), "   Welcome");
	ssd1331_display_string (0, 0, stringBuffer, FONT_1608, GOLDEN);
	HAL_Delay (2000);
}

/*******************************************************************************
 * FUNCTION      : getName()
 * PARAMETERS    : userInfo_t *iuserInfo
 * RETURNS       : returns false if user name starts with 0 else 1
 * DESCRIPTION   : displays Enter Name and reads name from serial
 *******************************************************************************
 */
bool getName(userInfo_t *iuserInfo)
{
	ssd1331_clear_screen (BLACK);

	char stringBuffer[16] = { 0 };
	snprintf (stringBuffer, sizeof(stringBuffer), "Enter Name");
	ssd1331_display_string (0, 0, stringBuffer, FONT_1206, WHITE);

	scanf ("%s", iuserInfo->name);

	if (iuserInfo->name[0] == '0')
		return false;
	return true;
}

/*******************************************************************************
 * FUNCTION      : getHeight()
 * PARAMETERS    : userInfo_t *iuserInfo
 * RETURNS       : True if height is entered non zero
 * 				   False if height is entered as zero
 * DESCRIPTION   : displays Enter Height and reads Height from serial
 *
 *******************************************************************************
 */
bool getHeight(userInfo_t *iuserInfo)
{
	char stringBuffer[16] = { 0 };
	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, sizeof(stringBuffer), "Enter Height");
	ssd1331_display_string (0, 0, stringBuffer, FONT_1206, WHITE);

	scanf ("%f", &iuserInfo->height);

	if (iuserInfo->height == 0.0)
	{
		return false;
	}
	return true;
}

/*******************************************************************************
 * FUNCTION      : getWeight()
 * PARAMETERS    : userInfo_t *iuserInfo
 * RETURNS       : True if Weight is entered non zero
 * 				   False if Weight is entered as zero
 * DESCRIPTION   : displays Enter Weight and reads Weight from serial
 *
 *******************************************************************************
 */
bool getWeight(userInfo_t *iuserInfo)
{
	char stringBuffer[16] = { 0 };
	ssd1331_clear_screen (BLACK);

	snprintf (stringBuffer, sizeof(stringBuffer), "Enter Weight");
	ssd1331_display_string (0, 0, stringBuffer, FONT_1206, WHITE);

	scanf ("%f", &iuserInfo->weight);

	if (iuserInfo->weight == 0.0)
	{
		return false;
	}
	return true;
}

/*******************************************************************************
 * FUNCTION      : calculateAndDisplayBMI()
 * PARAMETERS    : userInfo_t *iuserInfo
 * RETURNS       : None
 * DESCRIPTION   : calculates and display BMI on OLED
 *
 *******************************************************************************
 */
void calculateAndDisplayBMI(userInfo_t *iuserInfo)
{
	char stringBuffer[50] = { 0 };
	ssd1331_clear_screen (BLACK);

	iuserInfo->userBMI = iuserInfo->weight / (iuserInfo->height * iuserInfo->height);

	snprintf (stringBuffer, sizeof(stringBuffer), "%s : %0.1f", iuserInfo->name,
	          iuserInfo->userBMI);

	ssd1331_display_string (0, 0, stringBuffer, FONT_1608, GOLDEN);
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
	HAL_Init ();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config ();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init ();
	MX_USART2_UART_Init ();
	MX_SPI1_Init ();
	ssd1331_init ();
	ssd1331_clear_screen (BLACK);
	/* USER CODE BEGIN 2 */

	userInfo_t userInformation;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		switch (userInfoState)
		{
			case USER_INFO_STATE_WELCOME:
			{
				startUpOLEDSplashScreen ();
				userInfoState++;
				break;
			}

			case USER_INFO_STATE_NAME:
			{
				if (getName (&userInformation))
				{
					userInfoState++;
					break;
				}
				userInfoState = USER_INFO_STATE_WELCOME;
				break;
			}

			case USER_INFO_STATE_HEIGHT:
			{
				if (getHeight (&userInformation))
				{
					userInfoState++;
					break;
				}
				userInfoState = USER_INFO_STATE_WELCOME;
				break;
			}

			case USER_INFO_STATE_WEIGHT:
			{
				if (getWeight (&userInformation))
				{
					userInfoState++;
					break;
				}
				userInfoState = USER_INFO_STATE_WELCOME;
				break;
			}

			case USER_INFO_STATE_BMI:
			{
				calculateAndDisplayBMI (&userInformation);
				HAL_Delay (1000);
				userInfoState = USER_INFO_STATE_WELCOME;
				break;
			}

			default:
			{
				userInfoState = USER_INFO_STATE_WELCOME;
				break;
			}
		}
		/* USER CODE END 3 */
	}
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
	RCC_OscInitStruct.OscillatorType =
	RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
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
	HAL_GPIO_WritePin (GPIOB,
	LD3_Pin | SSD1331_CS_Pin | SSD1331_DC_Pin | SSD1331_RES_Pin,
	                   GPIO_PIN_RESET);

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
