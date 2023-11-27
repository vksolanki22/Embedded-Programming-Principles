/* USER CODE BEGIN Header */
/*******************************************************************************
 * FILE               : main.c
 * PROJECT			   : PROG8126 - #Assignment 4
 * PROGRAMMER		   : Vivek Solanki (8925804)
 * First Version	   : 04/10/2023
 * Description        : Demonstrates a debit machine banking transaction that implements a state machine.
 * State machine diagram is found on slide 14 of Week 9 Switch Statement and State Machine.pptx
 *
 * Switches are assigned as follows
 * note: these pins are set in the debounceInit functions and do not need to be configured in cube
 * PA0			PA1			PA4			PA3
 * chequing		savings		ok			cancel
 * Note: Don't use PA2 as it is connected to VCP TX and you'll
 * lose printf output ability.
 * ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <unistd.h>
#include <stdio.h>
#include "debounce.h"
#include "ssd1331.h"
#include "fonts.h"
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	PUSHBUTTON_NONE,
	PUSHBUTTON_CHEQUING,
	PUSHBUTTON_SAVINGS,
	PUSHBUTTON_OKAY,
	PUSHBUTTON_CANCEL,
	PUSHBUTTON_MAX
} pushButton_e;

typedef enum
{
	TRANSACTION_STATE_WELCOME,
	TRANSACTION_STATE_OK_CANCEL,
	TRANSACTION_STATE_CHEQ_OR_SAVING,
	TRANSACTION_STATE_RECEIVE_PIN,
	TRANSACTION_STATE_VALIDATE_PIN,
	TRANSACTION_STATE_SUCCESS,
	TRANSACTION_STATE_FAILED,
	TRANSACTION_STATE_MAX
} transactionState_e;

typedef enum
{
	TRANSACTION_SUCCESSFULL,
	TRANSACTION_FAIL_INVALID_PIN,
	TRANSACTION_FAIL_INSUFICIENT_BALANCE,
	TRANSACTION_FAIL_USER_CALCELLED,
	TRANSACTION_FAIL_MAX
} transactionRespCode_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// PIN of USERS for authorising payment
#define STRING_BUFF_LEN_MAX					26
#define	MAX_PIN_STORAGE						10
#define	MAX_PIN_LEN__MAX					5

const char PIN[10][5] = { "1234", "5678", "1357", "2468", "9876", "5432", "1000", "1618",
                "1278", "1148" };
char userEnteredPIN[5];
bool isAccessGranted = false;
transactionRespCode_e transactionFailCause = TRANSACTION_FAIL_MAX;
char stringBuffer[STRING_BUFF_LEN_MAX];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const int16_t chequingPbPin = 0; //setting the pin assigned to each pb
static const int16_t savingsPbPin = 1;	//don't use pin 2 as it's connected
static const int16_t okPbPin = 3;		//to VCP TX
static const int16_t cancelPbPin = 4;
static transactionState_e transactionState = TRANSACTION_STATE_WELCOME;
static float chequingBalance = 10000.0;
static float savingBalance = 10000.0;
pushButton_e pbPressed = PUSHBUTTON_NONE; //will hold pushbutton defined above depending on
pushButton_e isChequingSaving = PUSHBUTTON_NONE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
void displayRaminingBalance(void);
void displayTransactionSuccessFull();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void changeSpeakerFrequency(TIM_HandleTypeDef *htim,
		uint32_t newFrequency)
{

	//HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);

	// calculate the new period based off of frequency input
	uint32_t newPeriod = 1000000000 / (newFrequency * 250);

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	htim->Instance = TIM1;
	htim->Init.Prescaler = 0;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = newPeriod;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.RepetitionCounter = 0;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(htim) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(htim) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig)
			!= HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = htim1.Init.Period/2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK)
	{
		Error_Handler();
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
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig)
			!= HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_MspPostInit(htim);

	// must restart the timer once changes are complete
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
}

/*******************************************************************************
  * FUNCTION      : ErrorSound()
  * PARAMETERS    : none
  * RETURNS       : none
  * DESCRIPTION   : Play the error sound on speaker connected on PA7
  * ******************************************************************************
  */
void ErrorSound()
{
//	printf("Playing error sound\n\r");
	for(uint8_t count = 0; count < 10; count++)
	{
		for(uint16_t freq = 0; freq < 5000; freq+=500)
		{
//			printf("Error freq=%u\r\n", freq);			// print out the new frequency
			changeSpeakerFrequency(&htim1, freq);	// set new frequency
			HAL_Delay(10);						// delay for 1 second
		}
	}

	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);

//	changeSpeakerFrequency(&htim1, 0);	// set new frequency
}


/*******************************************************************************
  * FUNCTION      : SuccessSound()
  * PARAMETERS    : none
  * RETURNS       : none
  * DESCRIPTION   : Play the success sound on speaker connected on PA7
  * ******************************************************************************
  */
void SuccessSound()
{
	uint8_t count = 0;

	if(count < 1)
	{
		for(uint16_t freq = 3000; freq > 0; freq -= 200)
		{
//			printf("Success freq=%u\r\n", freq);			// print out the new frequency
			changeSpeakerFrequency(&htim1, freq);	// set new frequency
			HAL_Delay(35);						// delay for 1 second
		}
		count++;
	}
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
//	changeSpeakerFrequency(&htim1, 0);	// set new frequency
}


/*******************************************************************************
 * FUNCTION      : waitForPBRelease
 * PARAMETERS    : pin - pin number , port- port letter ie 'A'
 * RETURNS       : nothing
 * DESCRIPTION   : Loops until the PB that is currently
 * 				pressed and at a logic low is released. Release is debounced
 *
 *
 *******************************************************************************
 */
void waitForPBRelease(const int16_t pin, const char port)
{
	while (deBounceReadPin (pin, port, 10) == 0)
	{
		//do nothing wait for key press to be released
	}
}

/*******************************************************************************
 * FUNCTION      : startUpOLEDSplashScreen
 * PARAMETERS    : None
 * RETURNS       : nothing
 * DESCRIPTION   : displays Debit Demo for 2s
 * 				on line 1 of the display and
 * 				Disappears
 *
 *
 *******************************************************************************
 */
void startUpOLEDSplashScreen(void)
{
	char stringBuffer[16] = { 0 };
	snprintf (stringBuffer, sizeof(stringBuffer), "   Debit Demo");
	ssd1331_display_string (0, 0, stringBuffer, FONT_1206, GOLDEN);
	HAL_Delay (2000);
	ssd1331_clear_screen (BLACK);
}

/*******************************************************************************
 * FUNCTION      : pulsePWM
 * PARAMETERS    : address of Timer Handle var (e.g.: &htim1) , pulseTime in ms
 * RETURNS       : nothing
 * DESCRIPTION   : Turns on the PWM for the pulseTime in ms
 * 				provided and then turns off PWM
 *
 *******************************************************************************
 */
void pulsePWM(TIM_HandleTypeDef *htim1, int32_t pulseTime)
{
	HAL_TIMEx_PWMN_Start (htim1, TIM_CHANNEL_1);
	HAL_Delay (pulseTime);
	HAL_TIMEx_PWMN_Stop (htim1, TIM_CHANNEL_1);
}

/*******************************************************************************
 * FUNCTION      : pushButtonInit
 * PARAMETERS    : None
 * RETURNS       : nothing
 * DESCRIPTION   : Calls deBounceInit to initialize ports that
 will have pushbutton on them to be inputs.
 Initializing PA0,PA1,PA4 and PA3
 Switches are assigned as follows
 PA0			PA1			PA4			PA3
 chequing	savings		ok			cancel
 Note: Don't use PA2 as it is connected to VCP TX and you'll
 lose printf output ability.
 *
 *******************************************************************************
 */
void pushButtonInit(void)
{
	deBounceInit (chequingPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit (savingsPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit (okPbPin, 'A', 1); 			//1 = pullup resistor enabled
	deBounceInit (cancelPbPin, 'A', 1); 		//1 = pullup resistor enabled
}

/*******************************************************************************
 * FUNCTION      : displayWelcome
 * PARAMETERS    : None
 * RETURNS       : nothing
 * DESCRIPTION   : clears the OLED display and displays
 * 				Welcome on line 1 of the display
 *******************************************************************************
 */
void displayWelcome(void)
{
	char stringBuffer[STRING_BUFF_LEN_MAX] = { 0 };

	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, sizeof(stringBuffer), "   Welcome ");
	ssd1331_display_string (4, 0, stringBuffer, FONT_1206, WHITE);
}

/*******************************************************************************
 * FUNCTION      : controlLEDSpeaker
 * PARAMETERS    : iTransactionState ATM machine transaction state
 * RETURNS       : None
 * DESCRIPTION   :	Glows LED as per ATM machine transaction state
 *******************************************************************************
 */
static void controlLEDSpeaker(transactionState_e iTransactionState)
{
	switch (iTransactionState)
	{
		case TRANSACTION_STATE_SUCCESS:
		{
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);		//Red LED OFF
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8, GPIO_PIN_SET);		//Green LED ON
			SuccessSound();
			HAL_Delay (3000);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);		//Green LED OFF

			break;
		}
		case TRANSACTION_STATE_FAILED:
		{
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);		//Green LED OFF
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_11, GPIO_PIN_SET);		//Red LED ON
			ErrorSound();

			HAL_Delay (3000);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);		//Red LED OFF
			break;
		}
		default:
		{
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);		//Green LED OFF
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);		//Red LED OFF
			break;
		}
	}
}

/*******************************************************************************
 * FUNCTION      : controlOLED
 * PARAMETERS    : transactionState_e iTransactionState,transactionRespCode_e iTransactionRespCode
 * RETURNS       : None
 * DESCRIPTION   : This function displays text on OLED as per
 * 				   Input parameter
 *
 *******************************************************************************
 */
static void controlOLED(transactionState_e iTransactionState,
                transactionRespCode_e iTransactionRespCode)
{
	char stringBuffer[STRING_BUFF_LEN_MAX] = { 0 };

	switch (iTransactionState)
	{
		case TRANSACTION_STATE_SUCCESS:
		{
			displayTransactionSuccessFull ();
			break;
		}

		case TRANSACTION_STATE_FAILED:
		{
			switch (iTransactionRespCode)
			{
				case TRANSACTION_FAIL_INVALID_PIN:
				{
					snprintf (stringBuffer, sizeof(stringBuffer), "%s", "Invalid Pin");

					break;
				}

				case TRANSACTION_FAIL_INSUFICIENT_BALANCE:
				{
					snprintf (stringBuffer, sizeof(stringBuffer), "%s",
					          "Insuficient balance");
					break;
				}

				case TRANSACTION_FAIL_USER_CALCELLED:
				{
					snprintf (stringBuffer, sizeof(stringBuffer), "%s",
					          "Cancled by user");
					break;
				}

				default:
				{
					snprintf (stringBuffer, sizeof(stringBuffer), "%s", "");
					break;
				}
			}

			ssd1331_clear_screen (BLACK);

			ssd1331_display_string (0, 2, "Transaction Failed", FONT_1206, RED);
			ssd1331_display_string (0, 18, stringBuffer, FONT_1206, BLUE);
			controlLEDSpeaker (iTransactionState);
			ssd1331_clear_screen (BLACK);
			displayWelcome ();

			break;
		}

		default:
		{
			break;
		}
	}
}

/*******************************************************************************
 * FUNCTION      : displayAmount
 * PARAMETERS    : float - amount(entered by user)
 * RETURNS       : none
 * DESCRIPTION   : clears the OLED display and displays
 *					the $amount received on line 1 of the display
 *******************************************************************************
 */
void displayAmount(float amount)
{
	char stringBuffer[STRING_BUFF_LEN_MAX] = { 0 };

	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, sizeof(stringBuffer), "$%.2f", amount);
	ssd1331_display_string (0, 0, stringBuffer, FONT_1206, WHITE);
}

/*******************************************************************************
 * FUNCTION      : checkIfAmountRecd
 * PARAMETERS    : float, the amount in $ to be debited
 * RETURNS       : None
 * DESCRIPTION   : Checks that does user entered amount or not
 *
 *******************************************************************************
 */
float checkIfAmountRecd()
{
	float debitAmount = 0;

	int16_t result = 0;

	printf ("waiting for debitAmount to be received on serial port\r\n"); //}
	result = scanf ("%f", &debitAmount);
	if (result == 0)		//then somehow non-float chars were entered
	{						//and nothing was assigned to %f
		return 0;
	}
	return debitAmount;
}

/*******************************************************************************
 * FUNCTION      : checkOkOrCancel
 * PARAMETERS    : None
 * RETURNS       : int8_t, BUTTONSTATE_CANCEL if cancel pressed, BUTTONSTATE_OKAY if ok
 *                 ok pressed. 0 returned if neither
 *                 has pressed.
 * DESCRIPTION   : Checks whether the OK or Cancel
 * 				button has been pressed.
 *
 *******************************************************************************
 */
pushButton_e checkOkOrCancel(void)
{
	if (deBounceReadPin (cancelPbPin, 'A', 10) == 0)
	{
		//then the cancel pushbutton has been pressed
		return PUSHBUTTON_CANCEL;
	}
	else if (deBounceReadPin (okPbPin, 'A', 10) == 0)
	{
		//then ok pressed
		return PUSHBUTTON_OKAY;
	}
	return PUSHBUTTON_NONE; //as ok or cancel was not pressed.
}

/*******************************************************************************
 * FUNCTION      : checkSavingsOrChequing
 * PARAMETERS    : None
 * RETURNS       : pushButton_e, PUSHBUTTON_SAVINGS if savings pressed, PUSHBUTTON_CHEQUING if chequing pressed. PUSHBUTTON_NONE returned if neither has pressed.
 * DESCRIPTION   : Checks whether the chequing or Savings button has been pressed.
 *
 *******************************************************************************
 */
pushButton_e checkSavingsOrChequing(void)
{
	if (deBounceReadPin (chequingPbPin, 'A', 10) == 0)
	{
		return PUSHBUTTON_CHEQUING;		//then the chequing pushbutton has been pressed
	}

	else if (deBounceReadPin (savingsPbPin, 'A', 10) == 0)
	{
		return PUSHBUTTON_SAVINGS;				//then savings pressed
	}
	else if (deBounceReadPin (cancelPbPin, 'A', 20) == 0)
	{
		return PUSHBUTTON_CANCEL;			//then the cancel pushbutton has been pressed
	}
	return PUSHBUTTON_NONE; 					//as chequing or savings was not pressed.
}

/*******************************************************************************
 * FUNCTION      : displayEnterPin
 * PARAMETERS    : None
 * RETURNS       : None
 * DESCRIPTION   : displays "Enter the Pin" on line 2 of OLED
 *
 *******************************************************************************
 */
void displayEnterPin(void)
{
	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, 25, "Enter the PIN");
	ssd1331_display_string (0, 10, stringBuffer, FONT_1206, WHITE);
	HAL_Delay (100);
}

/*******************************************************************************
 * FUNCTION      : displaytransactioncancelled
 * PARAMETERS    : None
 * RETURNS       : None
 * DESCRIPTION   : displays "transaction is cancelled" on line 2 of OLED
 *
 *******************************************************************************
 */
void displayTransactionSuccessFull()
{
	char stringBuffer[STRING_BUFF_LEN_MAX] = { 0 };
	uint8_t tIdx = 0;

	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, 25, "Transaction is approved");
	ssd1331_display_string (0, 10, stringBuffer, FONT_1206, GREEN);

	controlLEDSpeaker (TRANSACTION_STATE_SUCCESS);

	ssd1331_clear_screen (BLACK);
	displayRaminingBalance ();

	pbPressed = checkOkOrCancel ();
	pbPressed = PUSHBUTTON_OKAY;

	if (pbPressed != PUSHBUTTON_NONE)
	{
		if (pbPressed == PUSHBUTTON_CANCEL)
		{
			//then cancel was pressed.
			printf ("Cancel Pressed\r\n");
		}
		else if (pbPressed == PUSHBUTTON_OKAY)
		{
			memset (stringBuffer, 0, sizeof(stringBuffer));
			//then ok pressed
			printf ("OK Pressed\r\n");
			ssd1331_clear_screen (BLACK);

			ssd1331_clear_screen (BLACK);
			tIdx = snprintf (stringBuffer, sizeof(stringBuffer), "%s", "Balance.:$");
			if (isChequingSaving == PUSHBUTTON_CHEQUING)
			{
				snprintf (stringBuffer + tIdx, sizeof(stringBuffer) - tIdx, "%.1f",
				          chequingBalance);
				printf ("chequingBalance = %f\r\n", chequingBalance);
			}
			else if (isChequingSaving == PUSHBUTTON_SAVINGS)
			{
				snprintf (stringBuffer + tIdx, sizeof(stringBuffer) - tIdx, "%.1f",
				          savingBalance);
				printf ("savingBalance = %f\r\n", savingBalance);
			}
			ssd1331_display_string (0, 0, stringBuffer, FONT_1206, PURPLE);
			tIdx = 0;
		}
	}

	HAL_Delay (2000);
	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, 25, "Thank you");
	ssd1331_display_string (0, 10, stringBuffer, FONT_1206, WHITE);
	HAL_Delay (2000);
	ssd1331_clear_screen (BLACK);
	displayWelcome ();
}

/*******************************************************************************
 * FUNCTION      : displaySavingsChequing
 * PARAMETERS    : None
 * RETURNS       : None
 * DESCRIPTION   : displays "Savings or Chequing?" on line 2 of OLED
 *
 *******************************************************************************
 */
void displaySavingsChequing(void)
{
	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, sizeof(stringBuffer), "Savings or Chequing?");
	ssd1331_display_string (0, 10, stringBuffer, FONT_1206, WHITE);
}

/*******************************************************************************
 * FUNCTION      : displayRaminingBalance
 * PARAMETERS    : None
 * RETURNS       : None
 * DESCRIPTION   : displays "Show available balance?" on line 1 of OLED
 *
 *******************************************************************************
 */
void displayRaminingBalance(void)
{
	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, sizeof(stringBuffer), "Show available balance?");
	ssd1331_display_string (0, 2, stringBuffer, FONT_1206, CYAN);
	HAL_Delay (1000);

}

/*******************************************************************************
 * FUNCTION      : validatePin
 * PARAMETERS    : None
 * RETURNS       : None
 * DESCRIPTION   : check the pin entered and compare with globally defined PIN
 *
 *******************************************************************************
 */
void validatePin(char *userEnteredPIN)
{
	scanf ("%s", userEnteredPIN);
	printf ("Entered PIN is %s\r\n", userEnteredPIN);
	for (int i = 0; i < 10; i++)
	{
		if (strcmp (userEnteredPIN, PIN[i]) == 0)
		{
			isAccessGranted = 1;
			break;
		}
	}
}

/*******************************************************************************
 * FUNCTION      : WaitforResponse
 * PARAMETERS    : None
 * RETURNS       : None
 * DESCRIPTION   : wait for response from the bank and print message based on that
 *
 *******************************************************************************
 */
transactionState_e WaitforResponse(void)
{
	if (isAccessGranted == 1)					//transaction approved i.e pin is correct
	{
		printf ("transaction is approved\r\n");
		return TRANSACTION_STATE_SUCCESS;

	}
	else										//transaction denied i.e pin is incorrect
	{
		transactionFailCause = TRANSACTION_FAIL_INVALID_PIN;
		printf ("transaction is denied\r\n");
		return TRANSACTION_STATE_FAILED;
	}
}

/*******************************************************************************
 * FUNCTION      : displayOkOrCancel
 * PARAMETERS    : None
 * RETURNS       : None
 * DESCRIPTION   : displays "OK or Cancel?" on line 2 of OLED
 *
 *******************************************************************************
 */
void displayOkCancel(void)
{
	char stringBuffer[16] = { 0 };
	snprintf (stringBuffer, 16, "OK or Cancel?");
	ssd1331_display_string (0, 10, stringBuffer, FONT_1206, WHITE);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	// initialize the OLED
	ssd1331_init ();

	/* setup Port A bits 0,1,2 and 3, i.e.: PA0-PA3 for input */
	pushButtonInit ();
	startUpOLEDSplashScreen ();
	HAL_Delay (1000);
	displayWelcome ();
	HAL_Delay (1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		static float amount = 0;             	//used to hold the transaction amount
		//the pushbutton pressed
		/*states:
		 1   display Welcome Screen, wait for $ amount input from Serial port
		 2   @ amount Received, waiting for Ok or Cancel button
		 3   OK received, waiting for chequing or Savings button
		 4   C or S received, waiting for PIN to be entered from Serial Port
		 5   Pin Correct, send transaction data to bank. Waiting
		 for OK back from Bank If OK from Bank received. Print
		 Receipt, Record transaction. Move back to State 1.
		 6   Cancel Pressed. Display "Transaction Cancelled" back to state 1
		 */
		switch (transactionState)
		{
			case TRANSACTION_STATE_WELCOME: 	//checking if an amount has been received
			{
				printf ("\033[0H\033[0J");
				amount = checkIfAmountRecd ();
				if (amount != 0)        //returns a 0 if an transaction amount has
				{ 						//NOT been received on the serial port.
					displayAmount (amount); //but if we're we've received a debitAmount
					displayOkCancel ();	//so display it and the prompt ok or cancel
					transactionState = TRANSACTION_STATE_OK_CANCEL;	//and do that before we move on to state 2
				}
				break;
			}

			case TRANSACTION_STATE_OK_CANCEL: 		//amount has been received waiting for
			{
				pbPressed = checkOkOrCancel ();

				if (pbPressed != PUSHBUTTON_NONE)
				{
					if (pbPressed == PUSHBUTTON_CANCEL)
					{
						//then cancel was pressed.
						printf ("Cancel Pressed\r\n");
						transactionState = TRANSACTION_STATE_FAILED;
						transactionFailCause = TRANSACTION_FAIL_USER_CALCELLED;
					}
					else if (pbPressed == PUSHBUTTON_OKAY)
					{
						//then ok pressed
						printf ("OK Pressed\r\n");
						displaySavingsChequing ();
						transactionState = TRANSACTION_STATE_CHEQ_OR_SAVING;

					}
				}
				break;
			}

			case TRANSACTION_STATE_CHEQ_OR_SAVING:
			{
				isChequingSaving = checkSavingsOrChequing ();

				if (isChequingSaving != PUSHBUTTON_NONE)
				{
					if (isChequingSaving == PUSHBUTTON_CHEQUING)
					{
						if (chequingBalance - amount <= 0.0) // Check that user has sufficient fund in chequing account
						{
							transactionState = TRANSACTION_STATE_FAILED;
							transactionFailCause = TRANSACTION_FAIL_INSUFICIENT_BALANCE;
							break;
						}
						printf ("chequing Pressed\r\n");	//then chequing was pressed.
						transactionState = TRANSACTION_STATE_RECEIVE_PIN;
					}
					else if (isChequingSaving == PUSHBUTTON_SAVINGS)
					{
						if (savingBalance - amount <= 0.0) // Check that user has sufficient fund in saving account
						{
							transactionState = TRANSACTION_STATE_FAILED;
							transactionFailCause = TRANSACTION_FAIL_INSUFICIENT_BALANCE;
							break;
						}
						printf ("savings Pressed\r\n");				//then savings pressed
						transactionState = TRANSACTION_STATE_RECEIVE_PIN;
					}
					else if (isChequingSaving == PUSHBUTTON_CANCEL)
					{
						printf ("Cancel Pressed\r\n");			//then cancel was pressed.
						transactionState = TRANSACTION_STATE_FAILED;
					}
				}
				break;
			}

			case TRANSACTION_STATE_RECEIVE_PIN:
			{
				printf ("Enter the PIN\r\n");

				displayEnterPin ();
				pbPressed = checkOkOrCancel ();
				if (pbPressed == PUSHBUTTON_CANCEL)
				{
					//then cancel was pressed.
					printf ("Cancel Pressed\r\n");
					transactionState = TRANSACTION_STATE_FAILED;
				}
				transactionState = TRANSACTION_STATE_VALIDATE_PIN;
				break;
			}

			case TRANSACTION_STATE_VALIDATE_PIN:
			{
				validatePin (userEnteredPIN);		//check the pin entered
				printf ("In validating Pin\r\n");
				transactionState = WaitforResponse ();	//display message on OLED based on the response received from bank
				break;
			}

			case TRANSACTION_STATE_SUCCESS:
			{
				switch (isChequingSaving)
				{
					case PUSHBUTTON_CHEQUING:
					{
						chequingBalance = chequingBalance - amount;	// Deduct the successfully debited amount
						break;
					}
					case PUSHBUTTON_SAVINGS:
					{
						savingBalance = savingBalance - amount;	// Deduct the successfully debited amount
						break;
					}
					default:
					{
						break;
					}
				}

				controlOLED (transactionState, TRANSACTION_SUCCESSFULL);

				transactionState = TRANSACTION_STATE_WELCOME;
				break;
			}
			case TRANSACTION_STATE_FAILED:
			{
				//transaction is cancelled
				controlOLED (transactionState, transactionFailCause);
				transactionState = TRANSACTION_STATE_WELCOME;
				break;
			}

			default:
			{
				break;
			}
		} //closing brace for switch

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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LD3_Pin|SSD1331_CS_Pin|SSD1331_DC_Pin
                          |SSD1331_RES_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin SSD1331_CS_Pin SSD1331_DC_Pin SSD1331_RES_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|SSD1331_CS_Pin|SSD1331_DC_Pin|SSD1331_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
