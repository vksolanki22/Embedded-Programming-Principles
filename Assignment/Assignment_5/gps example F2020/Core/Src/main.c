/* USER CODE BEGIN Header */
/*******************************************************************************
 * FILE               : main.c
 * PROJECT			  : PROG8126 - #Assignment 5
 * PROGRAMMER		  : Vivek Solanki (8925804)
 * First Version	  : 08/12/2023
 * Description        : This File contains all the required function to full fill the
 * 						requirement of Assignment 5, which should read the GPS data string from serial
 * 						and parse that and converts that into appropriate data type using
 * 						atoi(), atof(), strtoul()
 *
 *	ABOVE AND BEYOND  : (1) To read data from serial i Used UART interrupt instead of timeout
 *						(2) Printed all data on OLED
 *						(3) Enabvled countinuous horizontal scrolling for heading of Assignment.
 *
 *  ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debounce.h"
#include "ssd1331.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REC_BUFFER_SIZE 						100
#define TEMP_BUFFER_SIZE 						5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum
{
	GPS_DATA_UTC_TIME,
	GPS_DATA_LATITUDE,
	GPS_DATA_LATITUDE_DIRECTION,
	GPS_DATA_LONGITUDE,
	GPS_DATA_LONGITUDE_DIRECTION,
	GPS_DATA_POSITION,
	GPS_DATA_SATELITE_NUMBER,
	GPS_DATA_HDOP,
	GPS_DATA_ALTITUDE,
	GPS_DATA_ALTITUDE_UNIT,
	GPS_DATA_GEOID_SEPARATION,
	GPS_DATA_GEOID_SEPARATION_UNIT,
	GPS_DATA_DGPS,
	GPS_DATA_CHECKSUM,
	GPS_DATA_MAX
} gpsData_e;

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t second2;
} dateTime_t;

typedef struct
{
	uint8_t 	degree;
	uint8_t		minute;
	uint16_t 	decimalDegree;
	char 		latiTudeDirection;
} latiTude_t;

typedef struct
{
	uint16_t 	degree;
	uint8_t 	minute;
	uint16_t 	decimalDegree;
	char 		longiTudeDirection;
} longiTude_t;

typedef struct
{
	dateTime_t 	utcTime;
	latiTude_t 	latiTude;
	longiTude_t longiTude;
	uint8_t 	position;
	uint8_t 	sateliteNumber;
	float 		hdop;
	float 		altitude;
	char 		altitudeUnit;
	char 		geoidSeparation;
	char 		geoidSeparationUnit;
	uint16_t 	dgps;
	long 		checkSum;
} gpsData_t;

char *info = "Ready to Receive GPS string\r\n"; // Prints this on console at start of the program.

/* recChar will hold a single char received on the serial port*/
char recChar = 0;
/* recBuffer will have characters that are received on the serial
 * port copied into it*/

static char recBuffer[REC_BUFFER_SIZE] = { 0 }; //static so that the previous
//values of recBuffer aren't reset each
//time we get here

/* recBufferIndex is used to keep track of where we are in recBuffer.  */
static int8_t recBufferIndex = 0;		//and again static so we don't reset our
//value each time we get here

static int8_t sentenceReceived = 0;	//flag that indicates a complete sentence has
//been received. static for the same reasons
//above

//enumerated values for use
// with if (pbPressed==value)
//type conditional statements

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/*
 * 	This is the callback function that gets called when a receive is complete.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// write some code when reception is complete

	if (recChar == '$')	//STX! so let's start storing char in the recBuffer
	{
		recBufferIndex = 0;			//$ is the first char in a sentence.
		recBuffer[recBufferIndex] = recChar;	//so we've got to store it at the
		                                        //start of our recBuffer
		recBufferIndex++;			//and inc the index so the next char
		                            //is stored in the next position in
		                            //the buffer
	}
	else if (recBufferIndex > 0)		//then we're receiving the rest of the sentence
	{
		recBuffer[recBufferIndex] = recChar;	//so place the rec'd char in the buffer
		if (recBufferIndex < REC_BUFFER_SIZE)		//and increment the index as long as
		{						//we don't exceed the size of the buffer
			recBufferIndex++;
		}
		else
		{
			recBufferIndex = 0;	//if we're we've exceeded the size of our
			printf ("recBuffer Overflow\r\n");	//buffer so let's say so
		}
	}

	if (recChar == '\r' || recChar == '\n')	//if we get a cr then we know we're at the
	{									//end of our sentence
		recBuffer[recBufferIndex] = '\0';		//put a null instead of cr in the buffer
		                                        //'cause that's what strings need
		recBufferIndex = 0;					//and reset the buffer index
		sentenceReceived = 1;	//and set the flag 'cause we've got a
		                        //complete senetence now.
	}

	HAL_UART_Receive_IT (&huart2, (uint8_t*) &recChar, 1); // now call this function to receive next data.
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/*******************************************************************************
 * FUNCTION      : startUpLCDSplashScreen()
 * PARAMETERS    : None
 * RETURNS       : None
 * DESCRIPTION   : displays GPS Parcing on line 1 of the display and Disappears
 *
 *******************************************************************************
 */
void startUpLCDSplashScreen(void)
{
	char stringBuffer[16] = { 0 };
	ssd1331_clear_screen (BLACK);
	snprintf (stringBuffer, 16, "GPS Parsing");
	ssd1331_display_string (10, 0, stringBuffer, FONT_1206, GREEN);
	HAL_Delay (10);


	// This function is defined in ssd1331.c
	// It will rotate the heading of Assignmnet (GPS Parsing) countinuous on line 1
	EnableHorizontalCountinuousScroll (0, FONT_1206);
}

/*******************************************************************************
 * FUNCTION      : parseUSTTime()
 * PARAMETERS    : gpsData_t *gpsData, char *iUstTime
 * RETURNS       : None
 * DESCRIPTION   : This function parse UTS Time date from iUstTime and stores that
 * 				   data into main gpsData structure after converting that data into
 * 				   appropriate sata type
 *
 *******************************************************************************
 */
static void parseUSTTime(gpsData_t *gpsData, char *iUstTime)
{
	char tempStr[TEMP_BUFFER_SIZE];

	// Parse hour
	strncpy (tempStr, iUstTime, 2);
	gpsData->utcTime.hour = atoi (tempStr);

	// Parse minute
	strncpy (tempStr, iUstTime + 2, 2);
	gpsData->utcTime.minute = atoi (tempStr);

	// Parse second part 1
	strncpy (tempStr, iUstTime + 4, 2);
	gpsData->utcTime.second = atoi (tempStr);

	// Parse second part 2
	strncpy (tempStr, iUstTime + 7, 2);
	gpsData->utcTime.second2 = atoi (tempStr);
}

/*******************************************************************************
 * FUNCTION      : parseLatitude()
 * PARAMETERS    : gpsData_t *gpsData, char *iLatiTude
 * RETURNS       : None
 * DESCRIPTION   : This function parse latitude data from iLatiTude and stores that
 * 				data into main gpsData structure after converting that data into
 * 				appropriate sata type
 *  *******************************************************************************
 */
static void parseLatitude(gpsData_t *gpsData, char *iLatiTude)
{
	char tempStr[TEMP_BUFFER_SIZE];

	// Parse degrees
	strncpy (tempStr, iLatiTude, 2);
	gpsData->latiTude.degree = atoi (tempStr);

	// Parse minutes
	strncpy (tempStr, iLatiTude + 2, 2);
	gpsData->latiTude.minute = atoi (tempStr);

	// Parse decimal degrees
	strncpy (tempStr, iLatiTude + 5, 4);
	gpsData->latiTude.decimalDegree = atoi (tempStr);

}

/*******************************************************************************
 * FUNCTION      : parseLongitude()
 * PARAMETERS    : gpsData_t *gpsData, char *iLongiTude
 * RETURNS       : None
 * DESCRIPTION   : This function parse longitude data from iLongiTude and stores that
 *				   data into main gpsData structure after converting that data into
 * 				   appropriate sata type
 *
 *******************************************************************************
 */
static void parseLongitude(gpsData_t *gpsData, char *iLongiTude)
{
	char tempStr[TEMP_BUFFER_SIZE];

	// Parse degrees
	strncpy (tempStr, iLongiTude, 3);
	gpsData->longiTude.degree = atoi (tempStr);

	// Parse minutes
	strncpy (tempStr, iLongiTude + 3, 2);
	gpsData->longiTude.minute = atoi (tempStr);

	// Parse decimal degrees
	strncpy (tempStr, iLongiTude + 6, 4);
	gpsData->longiTude.decimalDegree = atoi (tempStr);
}

/*******************************************************************************
 * FUNCTION      : printDataOnOLED()
 * PARAMETERS    : gpsData_t gpsData
 * RETURNS       : None
 * DESCRIPTION   : This function prints data on OLED stored in gpsData
 *
 *******************************************************************************
 */
void printDataOnOLED(gpsData_t gpsData)
{
	uint8_t index = 1;
	char stringBuffer[25] = { 0 };

	//Display UTC time on OLED
	snprintf (stringBuffer, sizeof(stringBuffer), "UTC Hour = %d", gpsData.utcTime.hour);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "UTC Min. = %d",
	          gpsData.utcTime.minute);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "UTC Second = %d",
	          gpsData.utcTime.second);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "UTC mSec = %d",
	          gpsData.utcTime.second2);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);

	// Clear OLED
	ssd1331_clear_screen (BLACK);
	startUpLCDSplashScreen ();
	index = 1;

	//Display Latitude data on OLED
	snprintf (stringBuffer, sizeof(stringBuffer), "lat. deg. = %d",
	          gpsData.latiTude.degree);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "lat. Min. = %d",
	          gpsData.latiTude.minute);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "lat. Dd = %d",
	          gpsData.latiTude.decimalDegree);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "lat. Dire. = %c",
	          gpsData.latiTude.latiTudeDirection);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);

	// Clear OLED
	ssd1331_clear_screen (BLACK);
	startUpLCDSplashScreen ();
	index = 1;

	//Display Longitude data on OLED
	snprintf (stringBuffer, sizeof(stringBuffer), "long. deg. = %d",
	          gpsData.longiTude.degree);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "long. Min. = %d",
	          gpsData.longiTude.minute);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "long. Dd = %d",
	          gpsData.longiTude.decimalDegree);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
	snprintf (stringBuffer, sizeof(stringBuffer), "long. Dire. = %c",
	          gpsData.longiTude.longiTudeDirection);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);

	// Clear OLED
	ssd1331_clear_screen (BLACK);
	startUpLCDSplashScreen ();
	index = 1;

	//Display Position of Satelite
	snprintf (stringBuffer, sizeof(stringBuffer), "Positoin = %d", gpsData.position);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;

	//Display Satelite Number
	snprintf (stringBuffer, sizeof(stringBuffer), "Sat. Num. = %d",
	          gpsData.sateliteNumber);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;

	//Display HDOP
	snprintf (stringBuffer, sizeof(stringBuffer), "HDOP = %0.3f", gpsData.hdop);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;

	//Display Altitude
	snprintf (stringBuffer, sizeof(stringBuffer), "Alt. = %0.3f", gpsData.altitude);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;

	// Clear OLED
	ssd1331_clear_screen (BLACK);
	startUpLCDSplashScreen ();
	index = 1;

	//Display Altitude unit
	snprintf (stringBuffer, sizeof(stringBuffer), "Alt. Unit = %c", gpsData.altitudeUnit);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;

	//Display Geopid Separation
	snprintf (stringBuffer, sizeof(stringBuffer), "Geoid Sep. = %c",
	          gpsData.geoidSeparation);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;

	//Display Geopid Separation unit
	snprintf (stringBuffer, sizeof(stringBuffer), "G.Sep.Unit.= %c",
	          gpsData.geoidSeparationUnit);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;

	// Clear OLED
	ssd1331_clear_screen (BLACK);
	startUpLCDSplashScreen ();
	index = 1;

	//Display DGPS Age
	snprintf (stringBuffer, sizeof(stringBuffer), "DGPS = %d", gpsData.dgps);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;

	snprintf (stringBuffer, sizeof(stringBuffer), "Checksum = %ld", gpsData.checkSum);
	ssd1331_display_string (0, index + index * FONT_1206, stringBuffer, FONT_1206, WHITE);
	index++;
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
	MX_TIM1_Init ();
	MX_USART2_UART_Init ();
	MX_SPI1_Init ();
	/* USER CODE BEGIN 2 */
	HAL_UART_Transmit_IT (&huart2, (uint8_t*) info, strlen (info)); //Transmit data in interrupt mode
	HAL_UART_Receive_IT (&huart2, (uint8_t*) &recChar, 1); //receive data from data buffer interrupt mode
	printf ("Ready to Receive GPS string\r\n");
	ssd1331_init ();
	startUpLCDSplashScreen ();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		gpsData_t gpsData;

		if (sentenceReceived)
		{
			/* your code goes here to parse the string inside recBuffer */
			printf ("recBuffer: %s\r\n", recBuffer);
			char *tokPtr = NULL;

			// Take First Data from input string
			tokPtr = strtok (recBuffer, ",");

			printf ("Identification String : %s\r\n", tokPtr);
			for (gpsData_e gpsDataIdx = GPS_DATA_UTC_TIME; gpsDataIdx < GPS_DATA_MAX;
			                gpsDataIdx++)
			{
				tokPtr = strtok (NULL, ",");

				switch (gpsDataIdx)
				{
					case GPS_DATA_UTC_TIME:
					{
						parseUSTTime (&gpsData, tokPtr);
						break;
					}

					case GPS_DATA_LATITUDE:
					{
						parseLatitude (&gpsData, tokPtr);
						break;
					}

					case GPS_DATA_LATITUDE_DIRECTION:
					{
						strncpy (&gpsData.latiTude.latiTudeDirection, tokPtr,
						         sizeof(gpsData.latiTude.latiTudeDirection));
						break;
					}

					case GPS_DATA_LONGITUDE:
					{
						parseLongitude (&gpsData, tokPtr);
						break;
					}

					case GPS_DATA_LONGITUDE_DIRECTION:
					{
						strncpy (&gpsData.longiTude.longiTudeDirection, tokPtr,
						         sizeof(gpsData.longiTude.longiTudeDirection));

						break;
					}

					case GPS_DATA_POSITION:
					{
						gpsData.position = atoi (tokPtr);
						break;
					}

					case GPS_DATA_SATELITE_NUMBER:
					{
						gpsData.sateliteNumber = atoi (tokPtr);
						break;
					}

					case GPS_DATA_HDOP:
					{
						gpsData.hdop = atof (tokPtr);
						break;
					}

					case GPS_DATA_ALTITUDE:
					{
						gpsData.altitude = atof (tokPtr);
						break;
					}

					case GPS_DATA_ALTITUDE_UNIT:
					{
						strncpy (&gpsData.altitudeUnit, tokPtr,
						         sizeof(gpsData.altitudeUnit));
						break;
					}

					case GPS_DATA_GEOID_SEPARATION:
					{
						strncpy (&gpsData.geoidSeparation, tokPtr,
						         sizeof(gpsData.geoidSeparation));
						break;
					}

					case GPS_DATA_GEOID_SEPARATION_UNIT:
					{
						strncpy (&gpsData.geoidSeparationUnit, tokPtr,
						         sizeof(gpsData.geoidSeparationUnit));
						break;
					}

					case GPS_DATA_DGPS:
					{
						gpsData.dgps = atoi (tokPtr);
						break;
					}

					case GPS_DATA_CHECKSUM:
					{
						gpsData.checkSum = strtoul (tokPtr + 1, NULL, 16);
						break;
					}

					default:
					{
						break;
					}
				}
			}

			printf ("UTC Time -> Hour = %d\r\n", gpsData.utcTime.hour);
			printf ("UTC Time -> Minute = %d\r\n", gpsData.utcTime.minute);
			printf ("UTC Time -> Second = %d\r\n", gpsData.utcTime.second);
			printf ("UTC Time -> Milisecond = %d\r\n", gpsData.utcTime.second2);
			printf ("Latitude -> Degree = %d\r\n", gpsData.latiTude.degree);
			printf ("Latitude -> Minute = %d\r\n", gpsData.latiTude.minute);
			printf ("Latitude -> DecimalDegree = %d\r\n", gpsData.latiTude.decimalDegree);
			printf ("Latitude -> Direction = %c\r\n", gpsData.latiTude.latiTudeDirection);
			printf ("Longitude -> Degree = %d\r\n", gpsData.longiTude.degree);
			printf ("Longitude -> Minute = %d\r\n", gpsData.longiTude.minute);
			printf ("Longitude -> DecimalDegree = %d\r\n",
			        gpsData.longiTude.decimalDegree);
			printf ("Longitude -> Direction = %c\r\n",
			        gpsData.longiTude.longiTudeDirection);
			printf ("Position = %d\r\n", gpsData.position);
			printf ("Satelitinumber = %d\r\n", gpsData.sateliteNumber);
			printf ("HDOP = %.3f\r\n", gpsData.hdop);
			printf ("Altitude = %.3f\r\n", gpsData.altitude);
			printf ("AltitudeUnit = %c\r\n", gpsData.altitudeUnit);
			printf ("GeoidSeparation = %c\r\n", gpsData.geoidSeparation);
			printf ("GeoidSeparationUnit = %c\r\n", gpsData.geoidSeparationUnit);
			printf ("DGPS = %d\r\n", gpsData.dgps);
			printf ("CheckSum: %ld\r\n", gpsData.checkSum);

			printDataOnOLED (gpsData);
			ssd1331_clear_screen (BLACK);
			startUpLCDSplashScreen ();

			sentenceReceived = 0;	//as we've finished processing the sentence
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 9090;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	sConfigOC.Pulse = 4045;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
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
	HAL_GPIO_WritePin (GPIOB,
	GPIO_PIN_3 | SSD1331_CS_Pin | SSD1331_DC_Pin | SSD1331_RES_Pin,
	                   GPIO_PIN_RESET);

	/*Configure GPIO pins : PB3 SSD1331_CS_Pin SSD1331_DC_Pin SSD1331_RES_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | SSD1331_CS_Pin | SSD1331_DC_Pin | SSD1331_RES_Pin;
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
