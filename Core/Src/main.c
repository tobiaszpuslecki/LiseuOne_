/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Barometrer/barometer.h"
#include "Logger/logger.h"
#include "MPU/mpu.h"

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

/* USER CODE BEGIN PV */
int16_t acc[3];
int16_t gyro[3];
int16_t pitch_;
int16_t roll_;

#define ALTITUDE_THRESHOLD 3.0
#define KALMAN_STAB_TIME 5000

float previousAltitude=0.0;

uint8_t cycle=0;

enum state_t {STATE_1_LAUNCHPAD=1, STATE_2_FLIGHTUP, STATE_3_FLIGHTDOWN, STATE_4_TOUCHDOWN};
enum state_t state=STATE_1_LAUNCHPAD;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Frequency of TIM2 is 10Hz (PSC=7199, Counter Mode=UP, ARR=999, No Div., disable)
// est. real frequency is ~11Hz
// ver.2 ARR changed to 499 -> INT_FREQ ~ 30Hz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
	  if( barometerInitialized() )
	  {
			if(cycle % 2)
			{
			  getPressure();
			  requestTemperature();
			}
			else
			{
			  getTemperature();
			  requestPressure();
			}

			calculateAltitude();

			previousAltitude = k_altitude.value;

			kalmanUpdate(&k_pressure, bt_struct.pressure); // approx. 5s to establish value (with ~10Hz freq.)
			kalmanUpdate(&k_temperature, bt_struct.temperature); // approx. 3s to establish value (with ~30Hz freq.)
			kalmanUpdate(&k_altitude, bt_struct.altitude);

			//send2UART(LOG_UART, "%d %d %d %d\n",HAL_GetTick(), bt_struct.pressure, bt_struct.temperature, bt_struct.altitude);
			//send2UART(LOG_UART, "%d %d %d %d\n",HAL_GetTick(), (int)k_pressure.value,(int)k_temperature.value,(int)(k_altitude.value*100));
	  }


	 if(MPUInited())
	 {
		 readAccelerometer(acc);
		 roll_ = atan2(acc[1] , acc[2]) * 57.3;
		 pitch_ = atan2((- acc[0]) , sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 57.3;

		 kalmanUpdate(&k_accX, acc[0]);
		 kalmanUpdate(&k_accY, acc[1]);
		 kalmanUpdate(&k_accZ, acc[2]);

		 kalmanUpdate(&k_roll, roll_);
		 kalmanUpdate(&k_pitch, pitch_);
	 }


	 if(memoryInitialized())
	 {
		 if(cycle % 1 == 0)
		 {
		 	// 13ms || 26ms (?) with 4-5 args.
			  send2Memory("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",HAL_GetTick(),
					  (int)k_pressure.value,(int)k_temperature.value,(int)(k_altitude.value*10),(int)k_accX.value,
					  (int)k_accY.value,(int)k_accZ.value,(int)k_pitch.value,(int)k_roll.value,state);
		  }
		  HAL_GPIO_TogglePin(LED_BLUEPILL_GPIO_Port,LED_BLUEPILL_Pin);
	 }

	 if(getLogEnable())
	 {
		 send2UART(LOG_UART,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",HAL_GetTick(),(int)k_pressure.value,
				 (int)k_temperature.value,(int)(k_altitude.value*10),(int)k_accX.value,(int)k_accY.value,
				 (int)k_accZ.value,(int)k_pitch.value,(int)k_roll.value,state);
	 }

	 // state machine playground
	    switch (state)
	    {
	       case STATE_1_LAUNCHPAD:
	          if (HAL_GetTick() > KALMAN_STAB_TIME && k_altitude.value > ALTITUDE_THRESHOLD) // T1
	          {
	             state = STATE_2_FLIGHTUP;
	          }
	          break;
	       case STATE_2_FLIGHTUP:
	    	  if(k_altitude.value < previousAltitude)  // T2
	    	  {
	    		  state = STATE_3_FLIGHTDOWN;
	    	  }
	          break;
	       case STATE_3_FLIGHTDOWN:
	    	  if(k_altitude.value < ALTITUDE_THRESHOLD)  // T3
	    	  {
	    		  state = STATE_4_TOUCHDOWN;
	    	  }
	          break;
	       case STATE_4_TOUCHDOWN:
	    	  //
	          break;
	       default:
	          state = STATE_1_LAUNCHPAD;
	    }
	 // end of pg

	cycle++;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(LED_BLUEPILL_GPIO_Port,LED_BLUEPILL_Pin, GPIO_PIN_SET);



	send2UART(LOG_UART,"Lisa 1 for CanSat 2020 Rocket by Tobiasz Puslecki\n");
	send2UART(LOG_UART, "Work in progress...\n");
	send2UART(LOG_UART, "Rev. : %s, %s\n",__DATE__,__TIME__);

	uint8_t sdInitCycle=5;
	while(sdInitCycle--)
	{
		if( MemoryInit() )
		{
		  send2UART(LOG_UART,"SD init error! [%d]\n",sdInitCycle+1);
		  HAL_Delay(100);

		}
		else
		{
			send2UART(LOG_UART,"SD init successed!\n");
			send2UART(LOG_UART, "File idx: %d\n", openLogFile());

			send2Memory("time,pres,temp,alt,accx,accy,accz,pitch,roll,state\n");
			break;
		}
	}
	if( !memoryInitialized() )
	{
		send2UART(LOG_UART,"SD INIT CRITICAL ERROR!\n");
		HAL_GPIO_WritePin(LED_BLUEPILL_GPIO_Port,LED_BLUEPILL_Pin, GPIO_PIN_RESET);
		return -1;
	}

	if( BarometerInit() ) // with hires param in function BarometerInit()
	{
		send2UART(LOG_UART,"Barometer init error!\n");
		HAL_GPIO_WritePin(LED_BLUEPILL_GPIO_Port,LED_BLUEPILL_Pin, GPIO_PIN_RESET);
		return -1;
	}
	else
	{
		send2UART(LOG_UART,"Barometer init successed!\n");
	}

	if( MPUInit(0x71) ) 	// 	Ascale Gscale Mscale may be configured in mpu9250.c
	{
		send2UART(LOG_UART,"MPU init error!\n");
		HAL_GPIO_WritePin(LED_BLUEPILL_GPIO_Port,LED_BLUEPILL_Pin, GPIO_PIN_RESET);
		return -1;
	}
	else
	{
		send2UART(LOG_UART,"MPU init successed!\n");
	}



	setLogEnable(1);


	// TIM2 init
	HAL_TIM_Base_Start_IT(&htim2);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
