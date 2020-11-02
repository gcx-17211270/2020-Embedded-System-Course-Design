#ifndef __DHT11_H__
#define __DHT11_H__

//#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal.h"
//输出
#define DHT11_OUT_LOW()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
#define DHT11_OUT_HIGH() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
//输入
#define DHT11_IN  HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8)


uint8_t DHT11_Read_Data(double *temp,double *humi);
void delay_us(volatile unsigned long nus);
#endif