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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/***********************************************************************2020.10.26改动说明——————孙天一 begin************************************************************************************
* STM32CubeMax：
*       工程重建，配置了工程文件选项：主要包括基础配置以及RTC、FMC、I2C3和LTDC以及GRAPHICS
* IAR:
*       因程序走入死循环且诊断不出问题所在，重新开始工程和程序编写。
************************************************************************2020.10.26改动说明——————孙天一 end************************************************************************************/
/***********************************************************************2020.10.27改动说明——————高成鑫************************************************************************************
*STM32CubeMax:
*       ADC1进行了初始化---------------配置使用TIM2作为外部时钟源，具体配置如Img文件夹下20201027_023549_ADC1_Temp.png所示
                                        图中未体现开启中断
*IAR今日改动位置：*************************************************************************************************************************************************************************
*main.c:
*       USER CODE BEGIN PD---------------1.
*       USER CODE BEGIN PV---------------2.增添了在使用IN0测量时使用的全局变量
*       USER CODE BEGIN 0----------------3.使用IN0时候的回调函数，温度计算公式正确性待检验
*       USER CODE BEGIN 2----------------4.开启定时器TIM2以及ADC的中断
*其他说明：
*       缩进的printf表示调试过程中查看变量使用的，在正式使用中要被注释掉
************************************************************************************************************************************************************************************************/

/***********************************************************************2020.10.28改动说明——————高成鑫************************************************************************************
*STM32CubeMax:
*       ADC更改，ADC1IN0改ADC3IN8--------为了便于实时测量温度，使用了外部的模拟温度传感器，不再使用IN18的temperature传感器，删除了相关的程序
                                        将外部传感器的插口进行了更改，现在使用ADC3的IN8口作为输入，（对应扩展板上的A1位置），现在使用TIM1作为比较时钟
*       TIM1用作外部触发中断源---------External Trigger Conversion Source，配置同之前的TIM2相同
*       TIM2更改-------------------------现在将TIM2作为输入捕获使用，chanal1 chanal2分别为主从模式，用于捕获下降沿和上升沿
*       相关的配置情况如Image文件夹中20201028_161138_ADC3.png、20201028_161349_TIM1_1.png、20201028_161445_TIM1_2.png、20201028_161532_TIM2.png所示
*IAR今日改动位置：*************************************************************************************************************************************************************************
*main.c:
*       USER CODE BEGIN PV---------------1.增添了在输入捕获时候使用的全局变量
*       USER CODE BEGIN 0----------------2.输入捕获的回调函数
*       USER CODE BEGIN 2----------------3.开启定时器TIM2通道1和2输入捕获的中断（最终使用的只有TIM2 Channal1的中断）
*其他说明：
*       缩进的printf表示调试过程中查看变量使用的，在正式使用中要被注释掉
************************************************************************************************************************************************************************************************/

/**********************************************************************2020.10.29改动说明——————孙天一 begin********************************************************************************
*STM32CubeMax:
*       FMC、CRC模块功能开启------------为开启STemWin做准备，无需更多配置
*       LTDC模块功能开启---------------开启液晶屏控制器LTDC，为节省内存使用RGB565，场同步、行同步、前向纠错时间、后向纠错时间等根据手册配置相关参数，修改GPIO口，如图2020.10.29_LTDC_Parameter_Settings、20201029_125900_Layer_Setting.png、20201029_125943_GPIO_Setting.png所示
*       Graphics功能开启---------------使用STemWin绘制窗口，使用PG13，设置为Output模式作为LCD的复位管教，由STemWin来接管，其他配置如20201029_130312_Graphics.png所示
*       I2C功能开启--------------------使用I2C总线来控制屏幕触摸，修改板上I2C3对应的GPIO口，PH7为SCL，PH8为SDA，设置为快速模式
*IAR今日改动位置：*************************************************************************************************************************************************************************
*main.c:
*       USER CODE BEGIN PFP--------------1.触碰检测函数的声明，
*       USER CODE BEGIN 0----------------2.触碰检测函数的定义，向触摸屏驱动传递坐标
                                        checkTouch[]是触摸点位置的数组，ch[0]为触点个数，ch[2] ch[3]为y坐标，ch[4] ch[5]为x坐标
*GUI_App.c:
*       USER CODE BEGIN GRAPHICS_MainTask-----1.将原来的Hello World程序注释掉，将while(1)循环提前，在里面写如触摸检测程序
*WindowDLG.c:
*       USER START (Optionally insert additional static code)----1.定义的设置风速、温度、模式的全局变量
*       对三个按钮Button0~3触碰的回调函数编写
*其他说明：
*       缩进的printf表示调试过程中查看变量使用的，在正式使用中要被注释掉
*       变量名大写的表示字符串，小写的表示普通变量
***********************************************************************2020.10.29改动说明——————孙天一 end***********************************************************************************/

/**********************************************************************2020.10.30改动说明——————高成鑫 begin********************************************************************************
*STM32CubeMax:
*       TIM3_CH2、TIM12_CH1、TIM12_CH2开启-----设置为PWM输出模式，预分频器与技术周期设置为108-1与1000-1,使频率为1KHz，Pulse黄灯初始值设置为400，红蓝为0
                                            如20201030_105006_TIM3_TIM12.png所示，修改对应GPIO口
*IAR今日改动位置：*************************************************************************************************************************************************************************
*main.c:
*       USER CODE BEGIN PTD--------------1.对开关状态的宏定义，用于对于红外命令较多时候的便于识别
*       USER CODE BEGIN PV---------------2.对开关状态的定义，对温度设定、风速设定、模式设定的外部定义引入
*       USER CODE BEGIN 2----------------3.开启两个定时器的三个Channel口PWM输出
*       USER CODE BEGIN 0----------------4.在2020.10.28-2的红外中断函数中，加入了对红外解码的处理，可以使用遥控器控制开关机、风速、温度、空调工作模式
*WindowDLG.c:
*       USER START (Optionally insert additional message handling)----1.在RTC时钟的回调函数里面，添加根据风速、温度设置对LED指示灯改变的命令，实现空调变频
                                          在RTC的回调函数中，添加对空调开关状态的处理，通过改变PK3的电平状态实现，刷新温度、风速、模式显示
*其他说明：
*       缩进的printf表示调试过程中查看变量使用的，在正式使用中要被注释掉
*       变量名大写的表示字符串，小写的表示普通变量
***********************************************************************2020.10.30改动说明——————高成鑫 end***********************************************************************************/

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/***********************************************************************2020.10.30改动——————高成鑫 1 start************************************************************************************/
#define SWITCH_ON 200                            //开关红外的命令
#define SWITCH_OFF 201
/***********************************************************************2020.10.30改动——————高成鑫 1 end**************************************************************************************/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */
/***********************************************************************2020.10.26改动——————高成鑫 2 start************************************************************************************/
double AD_Value[100];
double temperature;                                    
double tempTemp;
uint16_t temp_index;                                    //温度信号的计数变量
/***********************************************************************2020.10.26改动——————高成鑫 2 end**************************************************************************************/

/***********************************************************************2020.10.28改动——————高成鑫 1 start************************************************************************************/
uint16_t ch1[120], ch2[120];
uint8_t ch1_index, ch2_index;                           //红外信号的计数变量
uint8_t ch_decode[14];
/***********************************************************************2020.10.28改动——————高成鑫 1 end**************************************************************************************/

/***********************************************************************2020.10.28改动——————孙天一 1 start************************************************************************************/
RTC_TimeTypeDef sTime;//定义时间结构体
RTC_DateTypeDef sDate;
/***********************************************************************2020.10.28改动——————孙天一 1 end************************************************************************************/
/***********************************************************************2020.10.30改动——————高成鑫 2 start************************************************************************************/
int menu = 0;                                           //红外开关的命令状态，用#define数字的表示

extern int speed_set, temperature_set, model_set;
/***********************************************************************2020.10.30改动——————高成鑫 2 end**************************************************************************************/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
extern void GRAPHICS_HW_Init(void);
extern void GRAPHICS_Init(void);
extern void GRAPHICS_MainTask(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/***********************************************************************2020.10.26改动——————高成鑫 3 start************************************************************************************/

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
          //printf("temperature:%lf°C\n",temperature);
      //T是室外温度，或许不代表传感器测量地方的温度，tempTemp是AD采样传回的数值，认为晚上的温度更接近传感器温度
      //T=12℃（凌晨2点） tempTemp = 1890 = 0x762                  
      //T=14.5℃(早上10点) tempTemp:2005.400000
      //T=17.4℃ tempTemp 2107.03        （在宿舍测得，可能阳光直射温度较高）
      //T=17.7℃ tempTemp 1795~1801       //在思西测的，可能温度偏低
      //T=7℃ tempTemp 1840
      //T=6℃ 1414(室外)
      temp_index = 0;
    }
}

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


/***********************************************************************2020.10.26改动——————高成鑫 3 end**************************************************************************************/

/***********************************************************************2020.10.28改动——————高成鑫 2 start************************************************************************************/
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
                  ch_decode[i] = ch_decode[i]<<1;
                  ch_decode[i] |= ch1[8 * i + j];
                }
//              for (i = 0; i < 14; i++)
//                printf("%x\t", ch_decode[i]);
//              printf("\n");
              //if (ch_decode[1] == 0x22 && ch_decode[4] == 0x40 && ch_decode[13] == 0x0D)
              if (ch_decode[12] == 0x85)                        //开关机,对应制冷、制热、通风模式下的开机
              {                
                //if ((ch_decode[4] & 0xF0) != 0)
                if ((ch_decode[4] == 0x40 && ch_decode[7] == 0x20) |
                    (ch_decode[4] == 0xC0 && ch_decode[7] == 0x80) |
                    (ch_decode[4] == 0x40 && ch_decode[7] == 0x40))
                {
                  //                printf("开关状态是从关到开，18℃、制冷、风速低档\n");
                  //                printf("tempTemp:%lf\n",tempTemp);
                  //                printf("温度是：%lf℃\n",temperature);
                  menu = SWITCH_ON;
                }
                else
                {
                  //                printf("开关状态是从开到关，18℃、制冷、风速低档\n");
                  //                printf("温度是：%lf℃\n",temperature);
                  menu = SWITCH_OFF;
                }
              }
              else if (ch_decode[12] == 0x80)                   //温度加
              {
                  temperature_set = (ch_decode[1] >> 4) + 16;
              }
              else if (ch_decode[12] == 0x81)                   //温度减
              {
                  temperature_set = (ch_decode[1] >> 4) + 16;
              }
              else if (ch_decode[12] == 0x86)                   //模式改变
              {
                if(ch_decode[4] == 0x40 && ch_decode[7] == 0x40) 
                {
                  model_set = 0;
                }
                else if(ch_decode[4] == 0xC0 && ch_decode[7] == 0x80)
                {
                  model_set = 1;
                }
                else if(ch_decode[4] == 0x40 && ch_decode[7] == 0x20)
                {
                  model_set = 2;
                }
              }
              else if (ch_decode[12] == 0x84)                   //风速调节
              {
                if (ch_decode[5] == 0x60)               //风速低
                  speed_set = 0;
                else if (ch_decode[5] == 0x40)
                  speed_set = 1;
                else if (ch_decode[5] == 0x20)
                  speed_set = 2;
                else if (ch_decode[5] == 0xa0)
                  speed_set = 3;
              }  
              else
                printf("解码错误\n");
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
 }
/***********************************************************************2020.10.28改动——————高成鑫 2 end**************************************************************************************/

/***********************************************************************2020.10.29改动——————孙天一 2 start************************************************************************************/

void checkTouch(void)
{
  uint8_t buf[6];
  HAL_I2C_Mem_Read(&hi2c3, 0x70, 0x02, 1, buf, 1, 1000);
  //printf("Touch: %d point\n", buf[0]);
  
  if(buf[0] > 0 && buf[0] < 6)
  {
    HAL_I2C_Mem_Read(&hi2c3, 0x70, 0x03, 1, buf, 4, 1000);
    //printf("y: %d\n", (buf[0]<<8 | buf[1]) & 0x0fff);
    //printf("x: %d\n", (buf[2]<<8 | buf[3]) & 0x0fff);
    GUI_TOUCH_StoreState((buf[2]<<8 | buf[3]) & 0x0fff, (buf[0]<<8 | buf[1]) & 0x0fff);
  }
  else
  {
    GUI_TOUCH_StoreState(-1, -1);
  } 
}
/***********************************************************************2020.10.29改动——————孙天一 2 end**************************************************************************************/
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
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_CRC_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  
/***********************************************************************2020.10.26改动——————高成鑫 4 start************************************************************************************/
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_ADC_Start_IT(&hadc3);
/***********************************************************************2020.10.26改动——————高成鑫 4 end**************************************************************************************/

/***********************************************************************2020.10.28改动——————高成鑫 3 start************************************************************************************/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
/***********************************************************************2020.10.28改动——————高成鑫 3 end**************************************************************************************/

/***********************************************************************2020.10.30改动——————高成鑫 3 start************************************************************************************/
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);                     //对应黄色风速灯
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);                     //对应蓝色制冷灯
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);                     //对应红色制热灯
/***********************************************************************2020.10.30改动——————高成鑫 3 end**************************************************************************************/
  /* USER CODE END 2 */

/* Initialise the graphical hardware */
  GRAPHICS_HW_Init();

  /* Initialise the graphical stack engine */
  GRAPHICS_Init();
  
  /* Graphic application */  
  GRAPHICS_MainTask();
    
  /* Infinite loop */
  for(;;);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 100;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */
  
  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */
  
  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */
  
  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */
  
  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */
  
  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x20404768;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  
  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  //RTC参数设置
  
  
  
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
  
  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  
  //  hrtc.Instance = RTC;
  //  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  //  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  //  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  //  {
  //    Error_Handler();
  //  }
  
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  
  
  //设置RTC定时时间1s 
  if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,1,RTC_WAKEUPCLOCK_CK_SPRE_16BITS)!=HAL_OK)
  { 
    Error_Handler();
  }
  /* USER CODE END RTC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 108-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 10;
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 108-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PK3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : PI12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

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
