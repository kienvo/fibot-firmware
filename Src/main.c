/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <math.h>

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
	
    }

    return Status;
}

VL53L0X_Error tof_setup_start(VL53L0X_Dev_t *dev, VL53L0X_DeviceModes mode)
{
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
	
	VL53L0X_DeviceInfo_t info;
	uint16_t vl53l0x_id;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	assert_param(HAL_I2C_IsDeviceReady(&hi2c1, dev->I2cDevAddr, 2, 2) == HAL_OK);

	Status = VL53L0X_GetDeviceInfo(dev, &info);
    assert_param(Status == VL53L0X_ERROR_NONE);

	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, (uint16_t *) &vl53l0x_id);
    assert_param(Status == VL53L0X_ERROR_NONE);
    assert_param(vl53l0x_id == 0xEEAA);

	Status = VL53L0X_DataInit(dev);
	assert_param(Status == VL53L0X_ERROR_NONE);

	// StaticInit will set interrupt by default
	Status = VL53L0X_StaticInit(dev); // Device Initialization
    assert_param(Status == VL53L0X_ERROR_NONE);
    
	Status = VL53L0X_PerformRefCalibration(dev,
			&VhvSettings, &PhaseCal);
    assert_param(Status == VL53L0X_ERROR_NONE);

	Status = VL53L0X_PerformRefSpadManagement(dev,
			&refSpadCount, &isApertureSpads);
    assert_param(Status == VL53L0X_ERROR_NONE);

	Status = VL53L0X_SetDeviceMode(dev, mode); // Setup in single ranging mode
    assert_param(Status == VL53L0X_ERROR_NONE);

	Status = VL53L0X_StartMeasurement(dev);
    assert_param(Status == VL53L0X_ERROR_NONE);
}
void wait_getData(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t   *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	Status = WaitMeasurementDataReady(dev);
	assert_param(Status == VL53L0X_ERROR_NONE);

	Status = VL53L0X_GetRangingMeasurementData(dev, data);
	assert_param(Status == VL53L0X_ERROR_NONE);

	// Clear the interrupt
	VL53L0X_ClearInterruptMask(dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
}
VL53L0X_Error rangingTest(VL53L0X_Dev_t *dev) 
{
    VL53L0X_RangingMeasurementData_t    data;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	tof_setup_start(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

	uint32_t measurement;
	uint32_t no_of_measurements = 100000;

	while (1) {
		wait_getData(dev, &data);
		VL53L0X_PollingDelay(dev);
		HAL_Delay(100);
	}

	Status = VL53L0X_StopMeasurement(dev);
	assert_param(Status == VL53L0X_ERROR_NONE);
		
	Status = WaitStopCompleted(dev);
	assert_param(Status == VL53L0X_ERROR_NONE);
	
	Status = VL53L0X_ClearInterruptMask(dev,
			VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	assert_param(Status == VL53L0X_ERROR_NONE);

	return Status;
}

/*
1 0x52
2 0x54
3 0x56
4 0x58
5 0x5A
6 0x5C
*/

const uint16_t tofXpin[] = {
	TO0X_Pin, 
	TO1X_Pin, 
	TO2X_Pin, 
	TO3X_Pin, 
	TO4X_Pin, 
	TO5X_Pin
};

const GPIO_TypeDef * tofXport[] = {
	TO0X_GPIO_Port, 
	TO1X_GPIO_Port, 
	TO2X_GPIO_Port, 
	TO3X_GPIO_Port, 
	TO4X_GPIO_Port, 
	TO5X_GPIO_Port
};

const uint8_t addrs[] = {
	0x52,
	0x54,
	0x56,
	0x58,
	0x5A,
	0x5C
};

VL53L0X_Dev_t devs[] = {
	{.I2cDevAddr = addrs[0], .hi2c = &hi2c1},
	{.I2cDevAddr = addrs[1], .hi2c = &hi2c1},
	{.I2cDevAddr = addrs[2], .hi2c = &hi2c1},
	{.I2cDevAddr = addrs[3], .hi2c = &hi2c1},
	{.I2cDevAddr = addrs[4], .hi2c = &hi2c1},
	{.I2cDevAddr = addrs[5], .hi2c = &hi2c1},
};

void tofArraySetup() 
{
	VL53L0X_Dev_t dev = {
		.I2cDevAddr = addrs[0], 
		.hi2c = &hi2c1
	};
	VL53L0X_Error Status;
	for (int i=5; i>=0; i--) {
		HAL_GPIO_WritePin(tofXport[i], tofXpin[i], 1);
		HAL_Delay(2);
		
		Status = VL53L0X_DataInit(&dev);
		//assert_param(Status == VL53L0X_ERROR_NONE);
		
		Status = VL53L0X_SetDeviceAddress(&dev, addrs[i]);
		//assert_param(Status == VL53L0X_ERROR_NONE);
	}

	// confirm setup
	for (int i=0; i<6; i++)	{
		assert_param(HAL_I2C_IsDeviceReady(&hi2c1, addrs[i], 2, 2) == HAL_OK);
	}

}

MPU6050_t mpu;
int isMpuInit = 0;
void MPU6050_test() 
{
	while(MPU6050_Init(&hi2c1) == 1);
	MPU6050_EnInt(&hi2c1);
	isMpuInit = 1;
	// while(1) {
	// 	HAL_Delay(50);
	// }
}

#define PWM_MAX (100)
void M1change(int32_t speed) 
{
	assert_param(speed <= PWM_MAX && speed >= -PWM_MAX); // TODO: change to constant symbol
	// if (abs(speed)<PWM_MAX/25) speed = 0;
	HAL_GPIO_WritePin(MA1_GPIO_Port, MA1_Pin, speed < 0);
	HAL_GPIO_WritePin(MB1_GPIO_Port, MB1_Pin, speed > 0);

	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, abs(speed));
}

void M2change(int32_t speed) 
{
	assert_param(speed <= PWM_MAX && speed >= -PWM_MAX); // TODO: change to constant symbol
	// if (abs(speed)<PWM_MAX/25) speed = 0;
	HAL_GPIO_WritePin(MA2_GPIO_Port, MA2_Pin, speed < 0);
	HAL_GPIO_WritePin(MB2_GPIO_Port, MB2_Pin, speed > 0);

	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, abs(speed));
}

void test_motors() 
{
	while(1) {
		int16_t v10 = (TIM1->CNT);
		int16_t v20 = (TIM3->CNT);
		M1change(-PWM_MAX);
		M2change(-PWM_MAX);
		HAL_Delay(2000);
		M1change(-PWM_MAX/2);
		M2change(-PWM_MAX/2);
		HAL_Delay(2000);
		M1change(0);
		M2change(0);
		HAL_Delay(2000);
		M1change(PWM_MAX/2);
		M2change(PWM_MAX/2);
		HAL_Delay(2000);
		M1change(PWM_MAX);
		M2change(PWM_MAX);
		HAL_Delay(2000);
	}
}

void i2cscan() {
	int r;
	for (int i=1; i<127; i++) {
		r = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
		HAL_Delay(5);
	}
}

#define ENPOSM1() (TIM1->CNT)
#define ENPOSM2() (TIM3->CNT)
#define M_KP 3.0
#define M_KD 1.0E300
#define M_KI 0.000000

double M1_pid(double expos) { // return speed
	static double err0, i;
	double pos = (int16_t)ENPOSM1();

	double err = expos - pos;
	double PID = M_KP*err + M_KD*(err - err0) + M_KI*i;

	err0 = err;
	i += err;

	// XXX:
	if (PID > PWM_MAX) 
		return PWM_MAX;
	else if (PID < -PWM_MAX) 
		return -PWM_MAX;
	else 
		return PID;
}

double M2_pid(double expos) { // return speed
	static double err0, i;
	double pos = (int16_t)ENPOSM2();
	
	double err = expos - pos;
	double PID = M_KP*err + M_KD*(err - err0) + M_KI*i;

	err0 = err;
	i += err;

	// XXX:
	if (PID > PWM_MAX) 
		return PWM_MAX;
	else if (PID < -PWM_MAX) 
		return -PWM_MAX;
	else 
		return PID;
}

void Ms_smooth_corners() {
	// M1change(PWM_MAX/4)	;
	// M2change(PWM_MAX/4)	;
	// HAL_Delay(500);
	M1change(PWM_MAX/4+PWM_MAX/8);
	M2change(PWM_MAX/4-PWM_MAX/8);
	HAL_Delay(1000);
	// M1change(PWM_MAX/4)	;
	// M2change(PWM_MAX/4)	;
	// HAL_Delay(500);
	M1change(0)	;
	M2change(0)	;
	while(1);
}

#define en_per_cm ((5000.0/0.685)/100.0) // cm
#define center2wheelcenter (10.9/2.0)
#define circumPath (center2wheelcenter*2.0*M_PI)

double cm2en(double cm)
{
	return cm*en_per_cm;
}
double en2cm(double en)
{
	return en/en_per_cm;
}

void Mrotate(double degree) 
{
	double radian = degree/360.0;
	double path = center2wheelcenter*radian;
	int16_t en = (int16_t)cm2en(path);
	while(1) {
		M1change(M1_pid(en));
		M2change(M2_pid(-en));
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	tofArraySetup();
	// rangingTest(&devs[0]);
	// MPU6050_test();
	// test_motors();

	tof_setup_start(&devs[0], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	//test_motors();
	//Mrotate(180);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
		// VL53L0X_RangingMeasurementData_t data;
		// wait_getData(&devs[0], &data);
		// uint16_t d = data.RangeMilliMeter;
		// if (d > 8000) continue;
		// if (data.RangeStatus != 0) continue;
		// if (d <= 120) {
		int16_t v10 = (TIM1->CNT);
		int16_t v20 = (TIM3->CNT);
		// 	M1change(0);
		// 	M2change(0);
		// 	while(1);
		// }
		
		int16_t v1 = v10;
		int16_t v2 = v20;
		// M1change(5000);
		// M2change(5000);
		M1change(M1_pid(5000));
		M2change(M2_pid(5000));
		// if (v10 >= 5000) {
		// 	M1change(0);
		// 	M2change(0);
		// 	HAL_Delay(2000);
		// 	int16_t v10 = (TIM1->CNT);
		// 	int16_t v20 = (TIM3->CNT);
		// 	while(1);
		// }
		// HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
	if (GPIO_Pin == MPU6050_INT_Pin) {
		return;
		if (isMpuInit) {
			MPU6050_Read_All(&hi2c1,&mpu);
			
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
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, 1);
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
	//asm("BKPT 0");
	Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
