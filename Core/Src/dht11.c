#include "dht11.h"
#include <stdio.h>

/*********************************
功能：配置GPIO的输出功能
参数：无
返回值：无
**********************************/
void DHT11_GPIO_OUT(void)
{
  GPIO_InitTypeDef initValue;
  
  /*1、打开GPIO时钟*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /*2、配置GPIO的输出功能*/
  initValue.Mode = GPIO_MODE_OUTPUT_PP;
  initValue.Pin = GPIO_PIN_8;
  initValue.Pull = GPIO_NOPULL;
  initValue.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &initValue);
}

/*********************************
功能：配置GPIO的输入功能
参数：无
返回值：无
**********************************/
void DHT11_GPIO_IN(void)
{
  GPIO_InitTypeDef initValue;
  
  /*1、打开GPIO时钟*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /*2、配置GPIO的输入功能*/
  initValue.Mode = GPIO_MODE_INPUT;
  initValue.Pin = GPIO_PIN_8;
  initValue.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &initValue);
}

/*********************************
功能：主机复位
参数：无
返回值：无
**********************************/
void DHT11_RST(void)
{
  /*1、配置GPIO的输出功能*/
  DHT11_GPIO_OUT();
  
  /*2、输出低电平*/
  DHT11_OUT_LOW();
  
  /*3、延时至少18ms*/
  HAL_Delay(20);
  
  /*4、输出高电平*/
  DHT11_OUT_HIGH();
  
  /*5、延时20~40us*/
  delay_us(30);
}

uint8_t retry = 0;
/*********************************
功能：DHT11应答
参数：无
返回值：
返回1-->未检测到DHT11存在
返回0-->DHT11存在
**********************************/
uint8_t DHT11_ACK(void)
{
  retry = 0;
  
  /*1、配置GPIO的输入功能*/
  DHT11_GPIO_IN();
  
  /*2、*/
  while(!DHT11_IN && retry < 100)//拉低40~80us
  {
    retry++;
    delay_us(1);
  }
  if(retry >= 100)
    return 1;
  else
    retry = 0;
  
  /*3、*/
  while(DHT11_IN && retry < 100) //拉低后再拉高40~80us
  {
    retry++;
    delay_us(1);
  }
  if(retry >= 100)
  {
    return 1;
  }
  return 0;    
  
}

/*********************************
功能：从DHT11中读取一个bit
参数：无
返回值：读取到的数据
**********************************/
uint8_t DHT11_Read_Bit(void)
{
  retry = 0;
  while(DHT11_IN && retry < 100)//等待变为低电平
  {
    retry++;
    delay_us(1);
  }
  
  retry = 0;
  
  while(!DHT11_IN && retry < 100) //等待变为高电平
  {
    retry++;
    delay_us(1);
  }
  
  //延时大于28us
  delay_us(40);
  
  //检测读取到的数据为高/低电平
  if(DHT11_IN)//高电平
  {
    return 1;
  }
  else
  {
    return 0;
  }
  
}

/*********************************
功能：从DHT11中读取一个字节
参数：无
返回值：读取到的数据
**********************************/
uint8_t DHT11_Read_Byte(void)
{
  uint8_t data = 0;
  uint8_t i = 0;
  
  for(i = 0;i<8;i++)
  {
    data <<= 1;
    data |= DHT11_Read_Bit();
  }
  
  return data;
}

/************************************************
功能：从DHT11读取一次数据（解析温度和湿度值）
参数：
temp-->温度值（0~50°）
humi-->湿度值（20~90%）
返回值：
返回1-->失败
返回0-->成功
*************************************************/

uint8_t buf[5];
uint8_t t ;
uint8_t DHT11_Read_Data(double *temp,double *humi)
{
  
  uint8_t i = 0;
  retry = 0;
  DHT11_RST();//主机复位
  //uint8_t t = DHT11_ACK();
  t = DHT11_ACK();
  //  printf("%d\n", t);
  if(t == 0)//DHT11应答
  {
    for(i=0;i<5;i++)//读取40位数据
    {
      buf[i] = DHT11_Read_Byte();
    }
    
    //printf(" %d %d %d %d %d\r\n",buf[0],buf[1],buf[2],buf[3],buf[4]);
    
    if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
    {
      *humi=buf[0] + 0.01 * buf[1]; //湿度
      *temp=buf[2] + 0.01 * buf[3]; //温度
    }
  }
  else
  {
    return 1;
  }
  return 0;
  
}



volatile unsigned long time_delay; // 延时时间，注意定义为全局变量
////延时n_ms
//void delay_ms(volatile unsigned long nms)
//{
//  extern uint32_t SystemCoreClock;
//  //SYSTICK分频--1ms的系统时钟中断
//  if (SysTick_Config(SystemCoreClock/1000))
//  {
//    
//    while (1);
//  }
//  time_delay=nms;//读取定时时间
//  while(time_delay);
//  SysTick->CTRL=0x00; //关闭计数器
//  SysTick->VAL =0X00; //清空计数器
//}
//延时nus
volatile unsigned long time_delay;
void delay_us(volatile unsigned long nus)
{
  //SYSTICK分频--1us的系统时钟中断
  if (SysTick_Config(SystemCoreClock/1000000))
  {
    while (1);
  }
  time_delay = nus;//读取定时时间
  while(time_delay);
  //  SysTick->CTRL=0x00; //关闭计数器
  SysTick->VAL =0X00; //清空计数器
}
