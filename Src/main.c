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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hal.h"
#include "lmic.h"
#include "oslmic.h"
#include "lorabase.h"
#include "cayenne_lpp.h"
#include <bme680/bme68x_necessary_functions.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_RESOLUTION 4095.0  //NEWWWWWWWW
#define myTIMER htim7   //  <--------- change to your setup
//#define myTIMER2 htim6 //NEWWWWWWWW

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// application router ID (LSBF) < ------- IMPORTANT
static const u1_t APPEUI[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // IMPORT FROM TTN
// unique device ID (LSBF) < ------- IMPORTANT
static const u1_t DEVEUI[8]={0xF6, 0xC8, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};// CREATE RANDOM NUMBER
// device-specific AES key (derived from device EUI (MSBF))
static const u1_t DEVKEY[16]={0xE0, 0x35, 0xCD, 0xA6, 0xF8, 0xC2, 0x20, 0xF3, 0xB1, 0x88, 0x8D, 0x10, 0x96, 0xFF, 0x69, 0xE5};// IMPORT FROM TTN

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
	memcpy(buf, APPEUI, 8);
}
// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
	memcpy(buf, DEVEUI, 8);
}
// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
	memcpy(buf, DEVKEY, 16);
}
void initsensor(){
// Here you init your sensors
}

void initfunc (osjob_t* j) {
	// intialize sensor hardware
	initsensor();
	// reset MAC state
	LMIC_reset();
	// start joining
	LMIC_startJoining();
	// init done - onEvent() callback will be invoked...
}
u2_t readsensor(){
	u2_t value = 0xDF; /// read from evrything ...make your own sensor
	return value;
}

uint32_t get_ADC_value(ADC_HandleTypeDef hadc1 , uint32_t channel)  //valeur numérique donnée par rapport à l'adc.
{
	uint32_t adc_val = 0;

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	adc_val = HAL_ADC_GetValue(&hadc1);
	return adc_val;
}

double GET_temperature(uint32_t ADC_value, double VDD) //conversion par rapport a la résolution de l'adc et la tension
{
	double temp_value = 0.0;
	double voltage = 0.0;

	voltage = (ADC_value*VDD)/ADC_RESOLUTION; //int en mV
	temp_value = (1034 - voltage)/5.48; //datasheet p12 a la fin
	return temp_value;
}

double GET_humidite()
{
	struct bme68x_data data;
	bme68x_start(&data, &hi2c1);
	if (bme68x_single_measure(&data) == 0) {

		// Measurement is successful, so continue with IAQ
		data.iaq_score = bme68x_iaq(); // Calculate IAQ

		// Create a message buffer and clear it.
		char msgBuffer[120];
		for(uint16_t i = 0; i < 120; i++){
			msgBuffer[i] = ' ';
		}
	}
	return data.humidity;
}

float GET_iaq()
{
	struct bme68x_data data;
	bme68x_start(&data, &hi2c1);
	if (bme68x_single_measure(&data) == 0) {

		// Measurement is successful, so continue with IAQ
		data.iaq_score = bme68x_iaq(); // Calculate IAQ

		// Create a message buffer and clear it.
		char msgBuffer[120];
		for(uint16_t i = 0; i < 120; i++){
			msgBuffer[i] = ' ';
		}
	}
	return data.iaq_score;
	 // Retourne la valeur de l'IAQ
}

double GET_pression()
{
	struct bme68x_data data;
	bme68x_start(&data, &hi2c1);
	if (bme68x_single_measure(&data) == 0) {

		// Measurement is successful, so continue with IAQ
		data.iaq_score = bme68x_iaq(); // Calculate IAQ

		// Create a message buffer and clear it.
		char msgBuffer[120];
		for(uint16_t i = 0; i < 120; i++){
			msgBuffer[i] = ' ';
		}
	}
	return data.pressure;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// counter
static int cnt = 0;
//static osjob_t hellojob;
static void hellofunc (osjob_t* j) {
	// say hello
	debug_str("Hello World!\r\n");
	// log counter
	debug_val("cnt = ", cnt);
	// toggle LED
	debug_led(++cnt & 1);
	// reschedule job every second
	os_setTimedCallback(j, os_getTime()+sec2osticks(1), hellofunc);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*~~~~~~~~~~~~~~~~~~~~~~~~~~*/

static osjob_t blinkjob;
static u1_t ledstate = 0;
static void blinkfunc (osjob_t* j) {
	// toggle LED
	ledstate = !ledstate;
	//debug_led(ledstate);
	// reschedule blink job
	os_setTimedCallback(j, os_getTime()+ms2osticks(100), blinkfunc);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~*/

static osjob_t reportjob;
// report sensor value every minute
static void reportfunc (osjob_t* j) {
	/*// read sensor
	u2_t val = readsensor();
	debug_val("val = ", val);
	// prepare and schedule data for transmission
	LMIC.frame[0] = val << 8;
	LMIC.frame[1] = val;
	LMIC_setTxData2(1, LMIC.frame, 2, 0); // (port 1, 2 bytes, unconfirmed)
	// reschedule job in 60 seconds
	os_setTimedCallback(j, os_getTime()+sec2osticks(15), reportfunc);*/

	/**** PARTIE 2(main) ***/
	double temperature_sensor_val = 25.0;
	double humidity_sensor_val = 0.0;
	double pressure_sensor_val = 0.0;
	double iaq_sensor_val = 0.0;
	cayenne_lpp_t lpp = {0};
	HAL_GPIO_WritePin(Alim_temp_GPIO_Port,Alim_temp_Pin, GPIO_PIN_SET); //ATTENTION
	HAL_Delay(100);

	// on est alimenté, on peut donc lancé une mesure
	temperature_sensor_val = GET_temperature(get_ADC_value(hadc1, ADC_CHANNEL_15), 3300);
	humidity_sensor_val = GET_humidite();
	iaq_sensor_val = GET_iaq();
	pressure_sensor_val = GET_pression() / 100;

	debug_valfloat("Temperature Sensor Value = ", temperature_sensor_val, 6);
	HAL_GPIO_WritePin(Alim_temp_GPIO_Port, Alim_temp_Pin, GPIO_PIN_RESET);
	cayenne_lpp_reset(&lpp);
	cayenne_lpp_add_temperature(&lpp, 0, temperature_sensor_val);
	cayenne_lpp_add_relative_humidity(&lpp, 1, humidity_sensor_val);
	cayenne_lpp_add_analog_output(&lpp, 2, iaq_sensor_val);
	cayenne_lpp_add_barometric_pressure(&lpp, 3, pressure_sensor_val);
	LMIC_setTxData2(1, &lpp, 15,0); //compter le nombre d'octets qu'on envoie 2 pour dire que c'est un capteur et 2 pour la valeur de température

	os_setTimedCallback(j, os_getTime()+sec2osticks(1), reportfunc); // passer à 10s

}



//////////////////////////////////////////////////
// LMIC EVENT CALLBACK
//////////////////////////////////////////////////

void onEvent (ev_t ev) {
	debug_event(ev);
	switch(ev) {
	// network joined, session established
		case EV_JOINING:
			debug_str("try joining\r\n");
			blinkfunc(&blinkjob);
			break;
		case EV_JOINED:
			// kick-off periodic sensor job
			os_clearCallback(&blinkjob);
			//debug_led(1);
			reportfunc(&reportjob);
			break;
		case EV_JOIN_FAILED:
			debug_str("join failed\r\n");
			break;
		case EV_SCAN_TIMEOUT:
			debug_str("EV_SCAN_TIMEOUT\r\n");
			break;
		case EV_BEACON_FOUND:
			debug_str("EV_BEACON_FOUND\r\n");
			break;
		case EV_BEACON_MISSED:
			debug_str("EV_BEACON_MISSED\r\n");
			break;
		case EV_BEACON_TRACKED:
			debug_str("EV_BEACON_TRACKED\r\n");
			break;
		case EV_RFU1:
			debug_str("EV_RFU1\r\n");
			break;
		case EV_REJOIN_FAILED:
			debug_str("EV_REJOIN_FAILED\r\n");
			break;
		case EV_TXCOMPLETE:
			debug_str("EV_TXCOMPLETE (includes waiting for RX windows)\r\n");
			if (LMIC.txrxFlags & TXRX_ACK)
				debug_str("Received ack\r\n");
			if (LMIC.dataLen) {
				debug_valdec("Received bytes of payload\r\n:", LMIC.dataLen);
				debug_val("Data = :",LMIC.frame[LMIC.dataBeg]);
				//debug_led(LMIC.frame[LMIC.dataBeg]);
			}
			break;
		case EV_LOST_TSYNC:
			debug_str("EV_LOST_TSYNC\r\n");
			break;
		case EV_RESET:
			debug_str("EV_RESET\r\n");
			break;
		case EV_RXCOMPLETE:
			// data received in ping slot
			debug_str("EV_RXCOMPLETE\r\n");
			break;
		case EV_LINK_DEAD:
			debug_str("EV_LINK_DEAD\r\n");
			break;
		case EV_LINK_ALIVE:
			debug_str("EV_LINK_ALIVE\r\n");
			break;
		default:
			debug_str("Unknown event\r\n");
			break;
	}
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
  MX_SPI3_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7); //NEWWWWWWWW  // <----------- change to your setup
  __HAL_SPI_ENABLE(&hspi3);    //NEWWWWWWWW    // <----------- change to your setup
  osjob_t initjob;
  // initialize runtime env
  os_init();
  // initialize debug library
  debug_init();
  // setup initial job
  os_setCallback(&initjob, initfunc); //initjob, initfunc //pour enlever le join remplacer le 2e par reportfunc
  // execute scheduled jobs and events
  os_runloop();
  // (not reached)
  return 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
