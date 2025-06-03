/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ACC_I2C_ADDR 0b1101000 << 1
#define ACCEL_START_REG 0x3B
#define GYRO_START_REG 0x43
#define WHO_AM_I_REG 0x75
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern void initialise_monitor_handles(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void read3dData(I2C_HandleTypeDef* hi2c, float* x, float* y, float* z, float rangeFactor, uint8_t startReg)
{
	  int16_t rawX,rawY,rawZ;
	  uint8_t i2c_receive8bit_buf[6];
	  const uint8_t bytes_to_receive = 6;

	  HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDR, startReg,1, i2c_receive8bit_buf, bytes_to_receive, 10);
	  rawX = (int16_t)(i2c_receive8bit_buf[0]<<8 | i2c_receive8bit_buf[1]);
	  rawY = (int16_t)(i2c_receive8bit_buf[2]<<8 | i2c_receive8bit_buf[3]);
	  rawZ = (int16_t)(i2c_receive8bit_buf[4]<<8 | i2c_receive8bit_buf[5]);

	  *x = (float)rawX/pow(2,15)*rangeFactor;
	  *y = (float)rawY/pow(2,15)*rangeFactor;
	  *z = (float)rawZ/pow(2,15)*rangeFactor;
}

void readAccelerometer(I2C_HandleTypeDef* hi2c, float* x, float* y, float* z)
{
	  const float wspolczynnik = 2;
	  read3dData(hi2c, x, y, z, wspolczynnik, ACCEL_START_REG);
}

void readGyroscope(I2C_HandleTypeDef* hi2c, float* x, float* y, float* z)
{
	const float wspolczynnik = 250.0f;
	read3dData(hi2c, x, y, z, wspolczynnik, GYRO_START_REG);
}

void calculatePitch(float *pitch, float ax, float ay, float az, float gx, float dt)
{
	const float alpha = 0.95;
	float acc_pitch = atan2(ax, sqrt(ay*ay+az*az)) * 180.0 / M_PI;
	*pitch = alpha * (*pitch + gx * dt) + (1.0f-alpha) * acc_pitch;
}

float PID(float y, float yzad){
	const float Tp = 0.01f; //czas próbkowania
	const float K =  2.5f;
	const float Ti = 900000.0f;
	const float Td = 0.0f;

	static float u =  0.0f;

	//bledy:
	static float e = 0.0f;
	static float e_1 = 0.0f;
	static float e_2 = 0.0f;

	const float r2 = (K*Td)/Tp;
	const float r1 = K*((Tp/(2*Ti))-(2*Td/Tp)-1);
	const float r0 = K*(1+(Tp/(2*Ti))+(Td/Tp));

	//aktualizacja bledow:
	e_2 = e_1;
	e_1 = e;
	e = yzad - y;

	u = r2*e_2 + r1*e_1 + r0*e + u;
	return u;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		const float dt = 0.01f;
		float accxf, accyf, acczf;
		float gyroxf, gyroyf, gyrozf;
		static float pitch = 0.0f, setPitch = 0.0f, u;
		const float gyroXoffset = -4.58500671;

		readAccelerometer(&hi2c1, &accxf, &accyf, &acczf);
		readGyroscope(&hi2c1, &gyroxf, &gyroyf, &gyrozf);
		calculatePitch(&pitch, accxf, accyf, acczf, gyroxf-gyroXoffset, dt);

		u = PID(pitch, setPitch);
		printf("Control signal: %3.2f \n", u);

//		printf("Pitch %3.2f degrees\n",pitch);

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  initialise_monitor_handles();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  uint8_t i2c_receive_buf[6];
  uint8_t i2c_transmit_buf[6];
  uint8_t bytes_to_receive = 1;

  HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDR, WHO_AM_I_REG, 1, i2c_receive_buf, bytes_to_receive, 50);

  printf("WHO_AM_I_A: 0x%02X \n", i2c_receive_buf[0]);

  uint8_t PWR_MGMT_1_reg = 0x6B;
  i2c_transmit_buf[0] = 0b00000000;
  HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, PWR_MGMT_1_reg, 1, i2c_transmit_buf, 1, 50);

  uint8_t SMPRT_DIV_reg = 0x19;
  i2c_transmit_buf[0] = 0x07;
  HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, SMPRT_DIV_reg, 1, i2c_transmit_buf, 1, 50);

  uint8_t GYRO_CONFIG_reg = 0x1B;
  i2c_transmit_buf[0] = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, GYRO_CONFIG_reg, 1, i2c_transmit_buf, 1, 50);

  uint8_t ACCEL_CONFIG_reg = 0x1C;
  HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, ACCEL_CONFIG_reg, 1, i2c_transmit_buf, 1, 50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);



  while (1)
  {


//	  printf("ACCEL xf: %3.2f g, yf: %3.2f g, zf: %3.2f g\n",accxf,accyf,acczf);
//	  printf("GYRO xf: %3.2f g, yf: %3.2f g, zf: %3.2f g\n",gyroxf,gyroyf,gyrozf);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
