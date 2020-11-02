/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */
  
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 /***********************************************************************2020.10.30�Ķ��������������߳��� 1 start************************************************************************************/
#define SWITCH_ON 200                            //���غ��������
#define SWITCH_OFF 201
/***********************************************************************2020.10.30�Ķ��������������߳��� 1 end**************************************************************************************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/***********************************************************************2020.10.26�Ķ��������������߳��� 2 start************************************************************************************/
double AD_Value[100];
double temperature;                                    
double tempTemp;
uint16_t temp_index;                                    //�¶��źŵļ�������
/***********************************************************************2020.10.26�Ķ��������������߳��� 2 end**************************************************************************************/

/***********************************************************************2020.10.28�Ķ��������������߳��� 1 start************************************************************************************/
uint16_t ch1[120], ch2[120];
uint8_t ch1_index, ch2_index;                           //�����źŵļ�������
uint8_t ch1_decode[14];
/***********************************************************************2020.10.28�Ķ��������������߳��� 1 end**************************************************************************************/

/***********************************************************************2020.10.30�Ķ��������������߳��� 2 start************************************************************************************/
int menu = SWITCH_ON;                                           //���⿪�ص�����״̬����#define���ֵı�ʾ

extern int speed_set, temperature_set, model_set;
extern int boot_status, boot_model, boot_time_set, boot_temper_set;
/***********************************************************************2020.10.30�Ķ��������������߳��� 2 end**************************************************************************************/

/***********************************************************************2020.10.31�Ķ��������������߳��� 1 start************************************************************************************/
uint16_t ch3[120];
uint8_t ch3_index;                           //�����źŵļ�������
uint8_t ch3_decode[14];
/***********************************************************************2020.10.31�Ķ��������������߳��� 1 end**************************************************************************************/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim10;
extern void GRAPHICS_IncTick(void);

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  GRAPHICS_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/***********************************************************************2020.10.26�Ķ��������������߳��� 3 start************************************************************************************/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   AD_Value[temp_index++] = HAL_ADC_GetValue(&hadc3);
    if (temp_index == 100)
    {
      uint16_t i = 0;
      tempTemp = 0;
      for (i = 0; i < temp_index; i++)
      {
        tempTemp += AD_Value[i];
      }
      tempTemp /= temp_index;
          //printf("tempTemp:%lf\n",tempTemp);
      //temperature = (1480 - tempTemp * 3300 / 4096) / 4.3 + 25;
      //temperature = ((tempTemp - 1800) * 3300 / 4096) / 57.0958 + 12;
      temperature = 0.035 * tempTemp - 55.9715;
          //printf("temperature:%lf��C\n",temperature);
      //T�������¶ȣ������������������ط����¶ȣ�tempTemp��AD�������ص���ֵ����Ϊ���ϵ��¶ȸ��ӽ��������¶�
      //T=12�棨�賿2�㣩 tempTemp = 1890 = 0x762                  
      //T=14.5��(����10��) tempTemp:2005.400000
      //T=17.4�� tempTemp 2107.03        ���������ã���������ֱ���¶Ƚϸߣ�
      //T=17.7�� tempTemp 1795~1801       //��˼����ģ������¶�ƫ��
      //T=7�� tempTemp 1840
      //T=6�� 1414(����)
      temp_index = 0;
    }
}

//��F746оƬ�ο��ֲ� 413ҳ �¶ȣ���λΪ ��C��= {(VSENSE �C V25) / Avg_Slope} + 25 
//      ���У�
//      �C V25 = 25 ��C ʱ�� VSENSE ֵ
//      �C Avg_Slope = �¶��� VSENSE ���ߵ�ƽ��б�ʣ��� mV/��C �� ��V/��C ��ʾ��
//      �й� V25 �� Avg_Slope ʵ��ֵ�������Ϣ����μ������ֲ��еĵ�������һ�ڡ�
      
// STM32F746�����ֲ� 43ҳ
//      The temperature sensor has to generate a voltage that varies linearly with temperature. 
//      The conversion range is between 1.7 V and 3.6 V. The temperature sensor is internally 
//        connected to the same input channel as VBAT, ADC1_IN18, which is used to convert the 
//          sensor output voltage into a digital value. When the temperature sensor and VBAT 
//            conversion are enabled at the same time, only VBAT conversion is performed.
//      As the offset of the temperature sensor varies from chip to chip due to process variation, 
//              the internal temperature sensor is mainly suitable for applications that detect 
//                temperature changes instead of absolute temperatures. If an accurate temperature
//                  reading is needed, then an external temperature sensor part should be used.


/***********************************************************************2020.10.26�Ķ��������������߳��� 3 end**************************************************************************************/

/***********************************************************************2020.10.28�Ķ��������������߳��� 2 start************************************************************************************/
 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 {
    if(htim == &htim2)
    {
      //printf("I got it !\n");
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
      {
          ch1[ch1_index] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
          if (ch1[ch1_index] < 2300 && ch1[ch1_index] > 800)
          {
            if (ch1[ch1_index] < 2300 && ch1[ch1_index] > 2100)
              ch1[ch1_index] = 1;
            else 
              ch1[ch1_index] = 0;
            
            if (ch1_index == 111)
            {
              uint8_t i = 0, j = 0;
              for (i = 0; i < 14; i++)
                for (j = 0; j < 8; j++)         //10101010 01
                {
                  ch1_decode[i] = ch1_decode[i]<<1;
                  ch1_decode[i] |= ch1[8 * i + j];
                }
//              for (i = 0; i < 14; i++)
//                printf("%x\t", ch1_decode[i]);
//              printf("\n");
              if (ch1_decode[12] == 0x85)                        //���ػ�,��Ӧ���䡢���ȡ�ͨ��ģʽ�µĿ���
              {                
                //if ((ch1_decode[4] & 0xF0) != 0)
                if ((ch1_decode[4] == 0x40 && ch1_decode[7] == 0x20) |
                    (ch1_decode[4] == 0xC0 && ch1_decode[7] == 0x80) |
                    (ch1_decode[4] == 0x40 && ch1_decode[7] == 0x40))
                {
                  //                printf("����״̬�Ǵӹص�����18�桢���䡢���ٵ͵�\n");
                  //                printf("tempTemp:%lf\n",tempTemp);
                  //                printf("�¶��ǣ�%lf��\n",temperature);
                  menu = SWITCH_ON;
                }
                else
                {
                  //                printf("����״̬�Ǵӿ����أ�18�桢���䡢���ٵ͵�\n");
                  //                printf("�¶��ǣ�%lf��\n",temperature);
                  menu = SWITCH_OFF;
                }
              }
              else if (ch1_decode[12] == 0x80)                   //�¶ȼ�
              {
                if (ch1_decode[3] == 0x00)
                  temperature_set = (ch1_decode[1] >> 4) + 16;
                else if (ch1_decode[3] == 0x02)
                  if (boot_model == 0)
                    boot_time_set = (ch1_decode[1] >> 4) + 16;
                  else
                    boot_temper_set = (ch1_decode[1] >> 4) + 16;
                else 
                  printf("Error! @ stm32f7xx_it.c 385 line\n");
              }
              else if (ch1_decode[12] == 0x81)                   //�¶ȼ�
              {
                  if (ch1_decode[3] == 0x00)
                  temperature_set = (ch1_decode[1] >> 4) + 16;
                else if (ch1_decode[3] == 0x02)
                  if (boot_model == 0)
                    boot_time_set = (ch1_decode[1] >> 4) + 16;
                  else
                    boot_temper_set = (ch1_decode[1] >> 4) + 16;
                else 
                  printf("Error! @ stm32f7xx_it.c 397 line\n");
              }
              else if (ch1_decode[12] == 0x86)                   //ģʽ�ı�
              {
                if(ch1_decode[4] == 0x40 && ch1_decode[7] == 0x40) 
                {
                  model_set = 0;
                }
                else if(ch1_decode[4] == 0xC0 && ch1_decode[7] == 0x80)
                {
                  model_set = 1;
                }
                else if(ch1_decode[4] == 0x40 && ch1_decode[7] == 0x20)
                {
                  model_set = 2;
                }
              }
              else if (ch1_decode[12] == 0x84)                   //���ٵ���
              {
                if (ch1_decode[5] == 0x60)               //���ٵ�
                  speed_set = 0;
                else if (ch1_decode[5] == 0x40)
                  speed_set = 1;
                else if (ch1_decode[5] == 0x20)
                  speed_set = 2;
                else if (ch1_decode[5] == 0xa0)
                  speed_set = 3;
              }
              else if (ch1_decode[12] == 0x8B)          //���� ��������ΪԤԼ���ػ��Ŀ����ر�
              {
                boot_status++;
                boot_status %= 2;
              }
              else if (ch1_decode[12] == 0x82)          //���� ��������ΪԤԼ���ػ���ģʽ�л�
              {                                         //���������µ��¶ȼӼ� ��ΪԤԼ���ػ��ļ�������
                boot_model++;                           //�յ�ң������ֻ�ܵ�16-30���ڰ��ϰ��������Ե��ڸ���
                boot_model %= 2;
              }
              else
                printf("Error! @ stm32f7xx_it.c 436 line\n");
            }
            ch1_index++;
          }
          else 
          {
            ch1[ch1_index] = 0;
            ch1_index = 0;
          }
      }
//      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//      {
//          ch2[ch2_index++] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
//      }
              //printf("%d %d\n", ch1_index, ch2_index);
    }
    else if(htim == &htim10)
    {
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
      {
        ch3[ch3_index++] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
      }
    }
 }
/***********************************************************************2020.10.28�Ķ��������������߳��� 2 end**************************************************************************************/

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
