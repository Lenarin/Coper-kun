/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "MPU9250.h"
#include "PIDController.h"
#include "sd_hal_MPU9250.h"
#include "bmp280.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t  rc_switch3 = SWITCH_OFF;
uint8_t  rc_switch4 = SWITCH_OFF;

PIDProp PIDPros[3];
uint32_t throttle = 1000;
uint32_t yaw = 1500;;
uint32_t pitch = 1500;;
uint32_t roll = 1500;

anglePoints aPoints;

float outputs[3];
int pulse_esc1 = 1000;
int pulse_esc2 = 1000;
int pulse_esc3 = 1000;
int pulse_esc4 = 1000;


uint32_t timestamp;

States currentState = STATE_LAUNCHING;

BMP280_HandleTypedef bmp280;
BMP280_Results bmp_res;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
float yawErrorFunc(float a, float b);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1) < 5000) rc_switch3 = SWITCH_ON;
				else rc_switch3 = SWITCH_OFF;
			__HAL_TIM_SET_COUNTER(&htim3, 0x0000);
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (rc_switch3 == SWITCH_OFF) {
				yaw = (float)HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
			} else {
				throttle = (float)HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
			}
		}
	}

	if(htim->Instance == TIM4)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1) < 5000) rc_switch4 = SWITCH_ON;
				else rc_switch4 = SWITCH_OFF;
			__HAL_TIM_SET_COUNTER(&htim4, 0x0000);
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (rc_switch4 == SWITCH_OFF) {
				pitch = (float)HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
			} else {
				roll = (float)HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
			}
		}
	}

}

int initAll() {
	/*
	// MPU
	uint8_t a = initMPU(&hspi1);
	if (0 == a) {
		printf("Failed code %#x \n", a);
		return -1;
	}
	printf("IMU code %#x \n", a);
	HAL_Delay(10);
	//calibrateMpu9250();
	*/

	MPU9250SetConnection(&hi2c1, SD_Device_0);

	float SelfTest[6];
	MPU9250SelfTest(&SelfTest); // Start by performing self test and reporting values
#if DEBUG == 1
	printf("x-axis self test: acceleration trim within : %f% of factory value \n", SelfTest[0]);
	printf("y-axis self test: acceleration trim within : %f% of factory value \n", SelfTest[1]);
	printf("z-axis self test: acceleration trim within : %f% of factory value \n", SelfTest[2]);
	printf("x-axis self test: gyration trim within : %f% of factory value \n", SelfTest[3]);
	printf("y-axis self test: gyration trim within : %f% of factory value \n", SelfTest[4]);
	printf("z-axis self test: gyration trim within : %f% of factory value \n", SelfTest[5]);
#endif
	HAL_Delay(10);

	if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f)
	{
		resetMPU9250();
		calibrateMPU9250();
		SD_MPUInit(SD_Accelerometer_2G, SD_Gyroscope_250s);
		initAK8963();
#if DEBUG == 1
		printf("MPU9250 initialized for active data mode....\n");
#endif

		HAL_Delay(20);
    }
	else
	{
		while (1) {
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
		}
#if DEBUG == 1
		printf("Device did not the pass self-test!\n");
#endif
	}

	HAL_Delay(100);
	// BMP280
	bmp280_init_default_params(&bmp280.params);
	bmp280.params.standby = BMP280_STANDBY_62;
	bmp280.params.filter = BMP280_FILTER_16;
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
#if DEBUG == 1
		printf("BMP280 didn't initialized\n");
#endif
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
	}

#if DEBUG == 1
	printf("BMP280 WHO AM I: %#x Should be 0x58\n", bmp280.id);
#endif


	// Regulators
	InitPID(&PIDPros[YAW],0.0f, 0.0f, 0.0f, 400.0f, -400.0f);
	InitPID(&PIDPros[ROLL], 1.0f, 0.0f, 0.0f, 400.0f, -400.0f);
	InitPID(&PIDPros[PITCH], 1.0f, 0.0f, 0.0f, 400.0f, -400.0f);

	//Change Yaw error func
	setErrorFunc(&PIDPros[YAW], yawErrorFunc);

	// Input timers
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // throttle and yaw
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);	// roll and pitch
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

    // Output times
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_4);

    if (currentState == STATE_CALIBRATING) {
    	TIM2->CCR1 = 2000;
		TIM2->CCR2 = 2000;
		TIM2->CCR3 = 2000;
		TIM2->CCR4 = 2000;
    } else {
    	TIM2->CCR1 = 1000;
		TIM2->CCR2 = 1000;
		TIM2->CCR3 = 1000;
		TIM2->CCR4 = 1000;
    }


    CalcGyroDrift();
	return 0;
}

void resetState() {
	TIM2->CCR1 = 1000;
	TIM2->CCR2 = 1000;
	TIM2->CCR3 = 1000;
	TIM2->CCR4 = 1000;

	aPoints.yaw = 0;
	aPoints.roll = 0;
	aPoints.pitch = 0;

	resetController(&PIDPros[YAW]);
	resetController(&PIDPros[ROLL]);
	resetController(&PIDPros[PITCH]);
}

void calcInputValues() {

	if (pitch > PITCH_UPPER_TRESHOLD ) {
		aPoints.pitch = -(pitch - PITCH_UPPER_TRESHOLD) / MAX_PITCH_INPUT * MAX_PITCH_POINT;
	} else if (pitch < PITCH_BOTTOM_TRESHOLD) {
		aPoints.pitch = -(pitch - PITCH_BOTTOM_TRESHOLD) / MAX_PITCH_INPUT * MAX_PITCH_POINT;
	} else {
		aPoints.pitch = 0.0f;
	}

	if (roll > ROLL_UPPER_TRESHOLD ) {
		aPoints.roll = -(roll - ROLL_UPPER_TRESHOLD) / MAX_ROLL_INPUT * MAX_ROLL_POINT;
	} else if (roll < ROLL_BOTTOM_TRESHOLD) {
		aPoints.roll = -(roll - ROLL_BOTTOM_TRESHOLD) / MAX_ROLL_INPUT * MAX_ROLL_POINT;
	} else {
		aPoints.roll = 0.0f;
	}

	/*
	if (yaw > YAW_UPPER_TRESHOLD ) {
		aPoints.yaw = -(yaw - YAW_UPPER_TRESHOLD) / MAX_YAW_INPUT * MAX_YAW_POINT;
	} else if (yaw < YAW_BOTTOM_TRESHOLD) {
		aPoints.yaw = -(yaw - YAW_BOTTOM_TRESHOLD) / MAX_YAW_INPUT * MAX_YAW_POINT;
	} else {
		aPoints.yaw = 0.0f;
	}
	*/

	float temp = 0;
	if (yaw > YAW_UPPER_TRESHOLD ) {
		temp = -(yaw - YAW_UPPER_TRESHOLD) / MAX_YAW_INPUT * MAX_YAW_POINT / 9000.0f;
	} else if (yaw < YAW_BOTTOM_TRESHOLD) {
		temp = -(yaw - YAW_BOTTOM_TRESHOLD) / MAX_YAW_INPUT * MAX_YAW_POINT / 9000.0f;
	} else {
		temp = 0.0f;
	}

	aPoints.yaw += temp;
	if (aPoints.yaw >= 180) {
		aPoints.yaw = aPoints.yaw - 360.0f;
	}
	if (aPoints.yaw < -180) {
		aPoints.yaw = aPoints.yaw + 360.0f;
	}

}

void checkState() {
	if (throttle > 1100 && throttle < 1150) { // THROTTLE
		if (roll > 1800)
		{
			resetState();
			currentState = STATE_ON;
			setPoint(&PIDPros[YAW], imuData.yaw);
			changeLEDState();
		}
		if (roll < 1200)
		{
			resetState();
			currentState = STATE_OFF;
			changeLEDState();
		}
	}

	if (currentState == STATE_LAUNCHING) {
		if (throttle > 1700 & roll > 1800) {
			resetState();
			currentState = STATE_CALIBRATING;
			changeLEDState();
		}
	}
}

void changeLEDState() {
	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // GREEN
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // ORANGE
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // RED
	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // BLUE

	if (currentState == STATE_LAUNCHING) {
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	}

	if (currentState == STATE_CALIBRATING) {
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
	}

	if (currentState == STATE_OFF) {
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	}

	if (currentState == STATE_ON) {
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	}
}

void calibrate() {
	  while (currentState == STATE_CALIBRATING) {

		  int esc = 1000 + ((throttle - 1065) / 660) * (1000);
		  if (esc < 1000) esc = 1000;
		  if (esc > 2000) esc = 2000;

		  TIM2->CCR1 = throttle;
		  TIM2->CCR2 = throttle;
		  TIM2->CCR3 = throttle;
		  TIM2->CCR4 = throttle;

		  checkState();
	  }
}

void static inline readAllData() {
	// Get gyro and magnet meters
	  if (readByte(INT_STATUS) & 0x01) {
		  MPU9250_TakeAndCalcData();

	  }
	  MPU9250_CalcYPR();

	  // Get BMP280 data
	  if (!bmp280_is_measuring(&bmp280)) {
		  bmp280_read_float_ex(&bmp280, &bmp_res);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Try to init while no errors ocured
  timestamp = HAL_GetTick();
  while(initAll() != 0) {HAL_Delay(100);};		// This must be first!!!!
  changeLEDState();

#if DEBUG == 1
  printf("All initialized\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //magcalMPU9250();

  while (currentState == STATE_LAUNCHING) {
	  checkState();
	  readAllData();
  }

  if (currentState == STATE_CALIBRATING) calibrate();

  while (1)
  {
	  readAllData();

	  if (currentState == STATE_OFF) {
		  checkState();
	  } else if (currentState == STATE_ON) {
		  calcInputValues();

		  // Set target points
		  setPoint(&PIDPros[YAW], aPoints.yaw);
		  setPoint(&PIDPros[ROLL], aPoints.roll);
		  setPoint(&PIDPros[PITCH], aPoints.pitch);

			  // Calculate regulator output next step
		  outputs[YAW] = calc(&PIDPros[YAW], imuGetPtr()->yaw);
		  outputs[ROLL] = calc(&PIDPros[ROLL], imuGetPtr()->roll);
		  outputs[PITCH] = calc(&PIDPros[PITCH], imuGetPtr()->pitch);

		  // Sum output pulses
		  pulse_esc1 = throttle + outputs[ROLL] + outputs[PITCH] + outputs[YAW];
		  pulse_esc2 = throttle + outputs[ROLL] - outputs[PITCH] - outputs[YAW];
		  pulse_esc3 = throttle - outputs[ROLL] + outputs[PITCH] - outputs[YAW];
		  pulse_esc4 = throttle - outputs[ROLL] - outputs[PITCH] + outputs[YAW];

		  checkState();
		  // Check output values
		  if (pulse_esc1 > MAX_PULSE_OUTPUT) pulse_esc1 = MAX_PULSE_OUTPUT;
		  if (pulse_esc1 < MIN_PULSE_OUTPUT) pulse_esc1 = MIN_PULSE_OUTPUT;
		  if (pulse_esc2 > MAX_PULSE_OUTPUT) pulse_esc2 = MAX_PULSE_OUTPUT;
		  if (pulse_esc2 < MIN_PULSE_OUTPUT) pulse_esc2 = MIN_PULSE_OUTPUT;
		  if (pulse_esc3 > MAX_PULSE_OUTPUT) pulse_esc3 = MAX_PULSE_OUTPUT;
		  if (pulse_esc3 < MIN_PULSE_OUTPUT) pulse_esc3 = MIN_PULSE_OUTPUT;
		  if (pulse_esc4 > MAX_PULSE_OUTPUT) pulse_esc4 = MAX_PULSE_OUTPUT;
		  if (pulse_esc4 < MIN_PULSE_OUTPUT) pulse_esc4 = MIN_PULSE_OUTPUT;

		  TIM2->CCR1 = pulse_esc1;
		  TIM2->CCR2 = pulse_esc3;
		  TIM2->CCR3 = pulse_esc4;
		  TIM2->CCR4 = pulse_esc2;

	  }
	  // Set new motor speeds


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 64999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DebugOut_GPIO_Port, DebugOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DebugOut_Pin */
  GPIO_InitStruct.Pin = DebugOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DebugOut_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float yawErrorFunc(float a, float b) {
	float temp = (a + b);
	if (a + b <= 180.0f) return temp;
	else {
		if (temp > 0) return (temp - 360.0f);
		else return (360.0f - temp);
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
