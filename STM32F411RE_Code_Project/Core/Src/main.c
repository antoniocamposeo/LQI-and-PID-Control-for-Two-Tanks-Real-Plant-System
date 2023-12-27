/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <sensors.h>
#include "MovingAverageFilter.h"
#include <control.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T_S 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float DC;
float* ptr_DC = &DC;

float t;
float* ptr_t = &t;

Pump_t Pump1;
Tank_Param_t Tank1;
Tank_Param_t Tank2;
PID_Param_t PID1;
LQI_Param_t LQI1;

volatile Pump_t* Pump1_ptr = &Pump1;
volatile Tank_Param_t* Tank1_ptr = &Tank1;
volatile Tank_Param_t* Tank2_ptr = &Tank2;
volatile PID_Param_t* PID1_ptr = &PID1;
volatile LQI_Param_t* LQI1_ptr = &LQI1;

/* Filter Handler */
MovingAverageFilter filter1;
MovingAverageFilter filter2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    for (int DataIdx = 0; DataIdx < len; DataIdx++)
        ITM_SendChar(*ptr++);

    return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	struct sensors array_sensor; /* definition sensor struct */
	uint32_t data1,data2; /*data from sensors*/

	//struct sensors array_sensor;
	array_sensor.Dev_array[0] = &array_sensor.vl53l0x_c[0];
	array_sensor.Dev_array[1] = &array_sensor.vl53l0x_c[1];
	float level1;
	float level2;

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  Pump_Init(&Pump1);
  Tank1_Init(&Tank1);
  Tank2_Init(&Tank2);
  PID_Init(&PID1);
  LQI_Init(&LQI1);

  /*Sensors Initialization*/
  array_sensor.Dev_array[0]->I2cHandle = &hi2c1;
  array_sensor.Dev_array[1]->I2cHandle = &hi2c1;
  array_sensor.Dev_array[0]->I2cDevAddr =  array_sensor.Dev_array[1]->I2cDevAddr = original_addr;
  printf("Sensor Init:\n\r");
  Sensor_Init(array_sensor,HIGH_ACCURACY); // function from sensors.h
  MovingAverageFilter_Init(&filter1, 2);
  MovingAverageFilter_Init(&filter2, 2);
  /* For PWM */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); /* PA6 filo nero*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); /* PA7 filo arancione*/

  // Start timer 1
  HAL_TIM_Base_Start_IT(&htim1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	Tank2.level_target = 10;

	/*get data from sensors -- distance from sensors to liquid*/
	Get_Data_Sensors(array_sensor,&data1,&data2);

	/*get tanks liquid level -- difference from height of sensor and data sensors*/
	MapLevel(&data1,&data2,Tank1_ptr,Tank2_ptr,&level1,&level2);

	/*moving average filter for data get from sensors*/
	Tank1.level_current = MovingAverageFilter_Update(&filter1, level1);
	Tank2.level_current = MovingAverageFilter_Update(&filter2, level2);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		__disable_irq();
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);

		*ptr_DC = PID_Control(PID1_ptr, Tank2_ptr);
		//*ptr_DC = LQI_Control(LQI1_ptr,Tank1_ptr, Tank2_ptr);
		actuation_pump(Pump1_ptr,*ptr_DC);

		printf("%.2f,L1:%.3f,L2:%.3f,DC:%.3f\n",*(ptr_t),Tank1.level_current, Tank2.level_current,*(ptr_DC));

		*ptr_t = *ptr_t + T_S;
		__enable_irq();
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
