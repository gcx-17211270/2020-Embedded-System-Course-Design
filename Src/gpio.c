/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
/******************************************************************************
* 2020.10.17 changes
* GPIO.c
*       USER CODE BEGIN 1------u8 DS18B20_Check(void);         //检测是否存在DS18B20
*       USER CODE BEGIN 1------void DS18B20_Start(void);       //开始温度转换
*       USER CODE BEGIN 1------short DS18B20_Get_Temp(void);    //获得温度
* GPIO.h
*       USER CODE BEGIN Prototypes---------增添了对相应函数的定义
********************************************************************************
*/
/*********************************2020.10.26 高成鑫 1*************************************/
#include <stdio.h>
/********************************2020.10.26 高成鑫 1 end**********************************/
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/*********************************************2020.10.18 高成鑫 1 begin**************************************************/
void delay_us(uint16_t i)
{
    uint32_t temp;
    //SysTick->LOAD=9*i;                          //设置重装数值, 72MHZ时
    SysTick->LOAD = 27 * i / 2;         //我们的时钟APB2为108MHz
    //SysTick->CTRL=0X01;                         //使能，减到零是无动作，采用外部时钟源
    SysTick->CTRL=0X01;
    SysTick->VAL=0;                             //清零计数器
    do
    {
        temp=SysTick->CTRL;                     //读取当前倒计数值
    }
    while((temp&0x01)&&(!(temp&(1<<16))));      //等待时间到达
    SysTick->CTRL=0;                            //关闭计数器
    SysTick->VAL=0;                             //清空计数器
}
//复位DS18B20
void DS18B20_Rst(void)     
{    
    //将PA0设置为输入模式
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);       //拉低DQ
    delay_us(750);    //拉低750us
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);         //DQ=1 
    delay_us(15);     //15US
}

uint8_t DS18B20_Check(void)             //检测是否存在DS18B20
{
  uint8_t retry = 0;
  //将PA0设置为输入模式
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //获得输入结果
  while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && retry < 200)
  {
    retry++;
    delay_us(1);
  }
  if (retry >= 200) return 1;
  else retry = 0;
  while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && retry < 240)
  {
    retry++;
    delay_us(1);
  }
  if (retry >= 240) return 1;
  return 0;
}

//从DS18B20读取一个字节
//返回值：读到的数据
uint8_t DS18B20_Read_Byte(void)    // read one byte
{        
    uint8_t i,pinState,dat;
    dat=0;
    for (i=1;i<=8;i++) 
    {
        pinState=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        dat=(pinState<<7)|(dat>>1);
    }                           
    return dat;
}
//写一个字节到DS18B20
//dat：要写入的字节
void DS18B20_Write_Byte(uint8_t dat)     
 {             
    uint8_t j;
    uint8_t testb;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    for (j=1;j<=8;j++) 
    {
        testb=dat&0x01;
        dat=dat>>1;
        if (testb) 
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // Write 1
            delay_us(2);                            
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
            delay_us(60);             
        }
        else 
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // Write 0
            delay_us(60);             
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
            delay_us(2);                          
        }
    }
}

void DS18B20_Start(void)                        //开始温度转换
{
    DS18B20_Rst();     
    int check = DS18B20_Check();   
      printf("Chenck() return = %d\n", check);
    DS18B20_Write_Byte(0xcc);                   // skip rom
    DS18B20_Write_Byte(0x44);                   // convert
}
uint16_t DS18B20_Get_Temp(void)                 //获得温度
{
    uint8_t temp;
    uint8_t TL,TH;
    uint16_t tem;
    DS18B20_Start();                    // ds1820 start convert
    DS18B20_Rst();
    int check = DS18B20_Check();   
      printf("Chenck() return = %d\n", check);     
    if (check == 0)
      return 0;
    DS18B20_Write_Byte(0xcc);// skip rom
    DS18B20_Write_Byte(0xbe);// convert     
    TL=DS18B20_Read_Byte(); // LSB   
    TH=DS18B20_Read_Byte(); // MSB  
      printf("TL = %d  TH = %d\n", (int)TL, (int)TH);

    if(TH>7)
    {
        TH=~TH;
        TL=~TL; 
        temp=0;                         //温度为负  
    }else temp=1;                       //温度为正       
    tem=TH;                             //获得高八位
    tem<<=8;    
    tem+=TL;                            //获得底八位
    //tem=(float)tem*0.625;               //转换     
    tem = tem * 5 / 8;
    if(temp)
      return tem;                 //返回温度值
    else 
      return -tem;    
}
/*********************************************2020.10.18 高成鑫 1 begin**************************************************/
/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
