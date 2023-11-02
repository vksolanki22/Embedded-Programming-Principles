/* USER CODE BEGIN Header */
/*******************************************************************************
  * FILE               	   : main.c
  * PROJECT		   : PROG8126 - Assignment #1
  * PROGRAMMER		   : Vivek Solanki (8925804)
  * First Version	   : 04/10/2023
  * Description       	   : Functions in this file are used to read the input serially from the
  * 					 keyboard and based on the character entered by user if character is
  * 					 lies in between alphabet then on board LED of STM32L432KC which is on
  * 					 PB6 will blink according to morse code which 200ml Sec second delay for "."
  * 					 and 600ml Sec delay for "-"
  *
  *  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DELAY_FOR_DOT 							200
#define DELAY_FOR_DASH 							3*DELAY_FOR_DOT
#define DELAY_FOR_SPACE_BTWN_LETTERS 			DELAY_FOR_DASH
#define DELAY_FOR_SPACE_BTWN_WORDS 				7*DELAY_FOR_DOT
#define MAX_ALPHABET_INDEX						25
#define MAX_APECIAL_CHARACTER					15
#define	INDEX_FOR_SPACE_BTWN_LETTERS			MAX_ALPHABET_INDEX + MAX_APECIAL_CHARACTER + 1
#define	INDEX_FOR_SPACE_BTWN_WORDS				INDEX_FOR_SPACE_BTWN_LETTERS + 1

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	// Morse code representation for letters
	const char* morseCodes[] = {
		    ".-",    // A
		    "-...",  // B
		    "-.-.",  // C
		    "-..",   // D
		    ".",     // E
		    "..-.",  // F
		    "--.",   // G
		    "....",  // H
		    "..",    // I
		    ".---",  // J
		    "-.-",   // K
		    ".-..",  // L
		    "--",    // M
		    "-.",    // N
		    "---",   // O
		    ".--.",  // P
		    "--.-",  // Q
		    ".-.",   // R
		    "...",   // S
		    "-",     // T
		    "..-",   // U
		    "...-",  // V
		    ".--",   // W
		    "-..-",  // X
		    "-.--",  // Y
		    "--.."   // Z
			".----"	 // 1
			"..---"	 // 2
			"...--"	 // 3
			"....-"	 // 4
			"....."	 // 5
			"-...."	 // 6
			"--..."	 // 7
			"---.."	 // 8
			"----."	 // 9
			"-----"	 // 0
			".-.-.-" // .
			"--..--" // ,
			"..--.." //	?
			"-..-."	 //	/
			".--.-." // @
			"..."	 // SPACE BTWN LETTER
			"........"	 // SPACE BTWN WORDS
		};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
char alphaBetEntredByUser;
char userEnteredString[50];
uint8_t choise = 0;
bool isFirstTime = true;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void blinkAsMorseCode(uint8_t iUsrEntredChar);
static void showMenu();
static bool isSpeciqalLetter(uint8_t iUsrEntredChar);
static uint8_t getIndexforSpecialChar(uint8_t iUsrEntredChar);
static void handleSpace(uint8_t iIndex);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(isFirstTime == true)
	  {
		  showMenu();
	  }
	  isFirstTime = false;
	  scanf("%hhi",&choise);

	  switch(choise)
	  {
	  	  case 1:
	  	  {
	  		  printf("Enter Alphabet : ");

	  		  HAL_Delay(100);
	  		  scanf(" %c",&alphaBetEntredByUser);

	  		  HAL_Delay(100);
	  		  printf("character entered = %c\r\n",alphaBetEntredByUser);
	  		  blinkAsMorseCode(alphaBetEntredByUser);
	  		  break;
	  	  }

	  	  case 2:
	  	  {
	  		  printf("Enter String : \r\n");
	  		  scanf("%s",userEnteredString);
	  		  printf("string entered = %s\r\n",userEnteredString);
	  		  // Blink LED for each character in the string
	  		  for (int i = 0; i < strlen(userEnteredString); i++)
	  		  {
	  			  if(isspace((int)userEnteredString[i]))
	  			  {
	  				  if(	(isalpha((int)userEnteredString[i-1]) || isSpeciqalLetter((int)userEnteredString[i-1]))
	  						&& (isalpha((int)userEnteredString[i+1]) || isSpeciqalLetter((int)userEnteredString[i+1]))
						)
	  				  {
		  				  handleSpace(INDEX_FOR_SPACE_BTWN_LETTERS);
	  				  }
	  			  }
	  			  else
	  			  {
	  				  blinkAsMorseCode(userEnteredString[i]);
	  			  }
	  		  }

	  		  showMenu();
	  		  break;
	  	  }

	  	  default:
	  	  {
	  		  printf("Please choose from the choise list\r\n");
	  		  break;
	  	  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

static void showMenu()
{
	printf("1. Blink Character\r\n");
	printf("2. Blink String\r\n");
}

/*******************************************************************************
  * FUNCTION      : blinkAsMorseCode
  * PARAMETERS    : uint8_t iUsrEntredChar
  * RETURNS       : none
  * DESCRIPTION   : This function will derive the morse code for user given character
  * 				and then after will blink the led for that morse code with "." duration
  * 				of  200 ml sec and "-" duration of 600 ml sec
  *  ******************************************************************************
  */
static void blinkAsMorseCode(uint8_t iUsrEntredChar)
{
    uint8_t index;

   // Check is letter is lower case
   if (iUsrEntredChar >= 'a' && iUsrEntredChar <= 'z')
   {
	   //Convert it to upper case
	   iUsrEntredChar = iUsrEntredChar - 'a' + 'A';
   }

   // Check if the character is a letter
   if (iUsrEntredChar >= 'A' && iUsrEntredChar <= 'Z')
   {
       index = iUsrEntredChar - 'A';
   }
   // Check if the character is a special letter
   else if(isSpeciqalLetter(iUsrEntredChar))
   {
	   index = getIndexforSpecialChar(iUsrEntredChar);
	   if(index == 0)
	   {
		   // Unsupported character found
		   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //PB6 pin used for LED
	   }
   }

   // Loop through the Morse code representation of the character
   for (uint8_t i = 0; morseCodes[index][i] != '\0'; i++)
   {
       if (morseCodes[index][i] == '.')
       {
    	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //PB6 pin used for LED
    	    HAL_Delay(DELAY_FOR_DOT);
    	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //PB6 pin used for LED
    	    HAL_Delay(DELAY_FOR_DOT);
       }
       else if (morseCodes[index][i] == '-')
       {
    	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //PB6 pin used for LED
    	   HAL_Delay(DELAY_FOR_DASH);
           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //PB6 pin used for LED
           HAL_Delay(DELAY_FOR_DASH);
       }
   }

//   // Space between letters
//   HAL_Delay(100);

}

static void handleSpace(uint8_t iIndex)
{
	   // Loop through the Morse code representation of the character
	   for (uint8_t i = 0; morseCodes[iIndex][i] != '\0'; i++)
	   {
	       if (morseCodes[iIndex][i] == '.')
	       {
	    	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //PB6 pin used for LED
	    	    HAL_Delay(DELAY_FOR_DOT);
	    	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //PB6 pin used for LED
	    	    HAL_Delay(DELAY_FOR_DOT);
	       }
	       else if (morseCodes[iIndex][i] == '-')
	       {
	    	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //PB6 pin used for LED
	    	   HAL_Delay(DELAY_FOR_DASH);
	           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //PB6 pin used for LED
	           HAL_Delay(DELAY_FOR_DASH);
	       }
	   }
}

static bool isSpeciqalLetter(uint8_t iUsrEntredChar)
{
	const char specialChar[] = {
			'0',
			'1',
			'2',
			'3',
			'4',
			'5',
			'6',
			'7',
			'8',
			'9',
			'.',
			',',
			'?',
			'/',
			'@',
	};

	for(uint8_t idx = 0; idx < 15;idx++)
	{
		if(iUsrEntredChar == specialChar[idx])
		{
			return true;
		}
	}
	return false;
}


static uint8_t getIndexforSpecialChar(uint8_t iUsrEntredChar)
{
	switch(iUsrEntredChar)
	{
		case '1':
		{
			return MAX_ALPHABET_INDEX+1;
		}
		case '2':
		{
			return MAX_ALPHABET_INDEX+2;
		}
		case '3':
		{
			return MAX_ALPHABET_INDEX+3;
		}
		case '4':
		{
			return MAX_ALPHABET_INDEX+4;
		}
		case '5':
		{
			return MAX_ALPHABET_INDEX+5;
		}
		case '6':
		{
			return MAX_ALPHABET_INDEX+6;
		}
		case '7':
		{
			return MAX_ALPHABET_INDEX+7;
		}
		case '8':
		{
			return MAX_ALPHABET_INDEX+8;
		}
		case '9':
		{
			return MAX_ALPHABET_INDEX+9;
		}
		case '0':
		{
			return MAX_ALPHABET_INDEX+10;
		}
		case '.':
		{
			return MAX_ALPHABET_INDEX+11;
		}
		case ',':
		{
			return MAX_ALPHABET_INDEX+12;
		}
		case '?':
		{
			return MAX_ALPHABET_INDEX+3;
		}
		case '/':
		{
			return MAX_ALPHABET_INDEX+14;
		}
		case '@':
		{
			return MAX_ALPHABET_INDEX+15;
		}
		default :
		{
			return 0;
		}
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
