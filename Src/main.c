/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
/***********************************************************************2020.10.17改动说明——————高成鑫************************************************************************************
*STM32CubeMax:
*       ADC1进行了初始化---------------配置使用TIM2作为外部时钟源，具体配置如Img文件夹下20201027_023549_ADC1_Temp.png所示
                                        图中未体现开启中断
*IAR今日改动位置：*************************************************************************************************************************************************************************
*main.c:
*       USER CODE BEGIN PD---------------1.添加了两个互斥的#define，在使用时根据使用的时外部的模拟温度转换模块还是内部的时钟做出选择
*       USER CODE BEGIN PV---------------2.增添了在使用IN0或内部温度测量时使用的全局变量
*       USER CODE BEGIN 0----------------3.使用IN0和IN18时候的回调函数重写，他们的主要区别在于AD读数不同从而公式不同，还有，该公式正确性待检验
*       USER CODE BEGIN 2----------------4.开启定时器TIM2以及ADC的中断（两个IN口都会使用）
*其他说明：
*       缩进的printf表示调试过程中查看变量使用的，在正式使用中要被注释掉
************************************************************************************************************************************************************************************************/
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/***********************************************************************2020.10.26改动——————高成鑫 1 start************************************************************************************/
//采用IN0或者IN18(或者更像是IN16？),将对应的宏定义取消注释即可
#define __IN0_ADC_TRANS__
//IN18是系统自带的模拟温度传感器
//#define __IN18_TEMPERATURE_SENSOT__
/***********************************************************************2020.10.26改动——————高成鑫 1 end**************************************************************************************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/***********************************************************************2020.10.26改动——————高成鑫 2 start************************************************************************************/
#ifdef __IN0_ADC_TRANS__
    double AD_Value[100];
    double temperature;               //为了读取小数，是实际温度*100
    uint8_t index;
#endif

#ifdef __IN18_TEMPERATURE_SENSOT__
    double AD_Value[100];
    double temperature;               
    uint8_t index;
#endif
/***********************************************************************2020.10.26改动——————高成鑫 2 end**************************************************************************************/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/***********************************************************************2020.10.26改动——————高成鑫 3 start************************************************************************************/
#ifdef __IN0_ADC_TRANS__
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   AD_Value[index++] = HAL_ADC_GetValue(&hadc1);
    if (index == 100)
    {
      uint8_t i = 0;
      double tempTemp = 0;
      for (i = 0; i < 50; i++)
      {
        tempTemp += AD_Value[i];
      }
      tempTemp /= 50;
          printf("tempTemp:%lf\n",tempTemp);
      temperature = (1480 - tempTemp * 3300 / 4096) / 4.3 + 25;
      //T=12℃ tempTemp = 1890 = 0x762                   //为什么IN0的AD转换结果和IN18的结果相差一倍？
          printf("temperature:%lf°C\n",temperature);
      index = 0;
    }
}
#endif

#ifdef __IN18_TEMPERATURE_SENSOT__
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    AD_Value[index++] = HAL_ADC_GetValue(&hadc1);
    if (index == 100)
    {
      uint8_t i = 0;
      double tempTemp = 0;
      for (i = 0; i < 50; i++)
      {
        tempTemp += AD_Value[i];
      }
      tempTemp /= 50;
          printf("tempTemp:%lf\n",tempTemp);
      temperature = (760 - tempTemp*3300/4096)/2.5+25;
      //T=12℃ tempTemp = 950 = 0x3B6
          printf("temperature:%lf°C\n",temperature);

//从F746芯片参考手册 413页 温度（单位为 °C）= {(VSENSE – V25) / Avg_Slope} + 25 
//      其中：
//      – V25 = 25 °C 时的 VSENSE 值
//      – Avg_Slope = 温度与 VSENSE 曲线的平均斜率（以 mV/°C 或 μV/°C 表示）
//      有关 V25 和 Avg_Slope 实际值的相关信息，请参见数据手册中的电气特性一节。
      
// STM32F746数据手册 43页
//      The temperature sensor has to generate a voltage that varies linearly with temperature. 
//      The conversion range is between 1.7 V and 3.6 V. The temperature sensor is internally 
//        connected to the same input channel as VBAT, ADC1_IN18, which is used to convert the 
//          sensor output voltage into a digital value. When the temperature sensor and VBAT 
//            conversion are enabled at the same time, only VBAT conversion is performed.
//      As the offset of the temperature sensor varies from chip to chip due to process variation, 
//              the internal temperature sensor is mainly suitable for applications that detect 
//                temperature changes instead of absolute temperatures. If an accurate temperature
//                  reading is needed, then an external temperature sensor part should be used.
      index = 0;
    }
}
#endif
/***********************************************************************2020.10.26改动——————高成鑫 3 end**************************************************************************************/
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
/***********************************************************************2020.10.26改动——————高成鑫 4 start************************************************************************************/
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_ADC_Start_IT(&hadc1);
/***********************************************************************2020.10.26改动——————高成鑫 4 end**************************************************************************************/
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

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
