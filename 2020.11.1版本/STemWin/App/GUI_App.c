  /**
  ******************************************************************************
  * @file    GUI_App.c
  * @author  MCD Application Team
  * @brief   Simple demo drawing "Hello world"  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2018 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "GUI_App.h"
#include "GUI.h"

#include "DIALOG.h"
extern  WM_HWIN CreateWindow(void);  
  

void GRAPHICS_MainTask(void) {

  /* 2- Create a Window using GUIBuilder */
  CreateWindow();
 
/* USER CODE BEGIN GRAPHICS_MainTask */
  /* User can implement his graphic application here */
  /* Hello Word example */
  GUI_Clear();
  //    GUI_SetColor(GUI_WHITE);
  //    GUI_SetFont(&GUI_Font32_1);
  //    GUI_DispStringAt("Hello world!", (LCD_GetXSize()-150)/2, (LCD_GetYSize()-20)/2);
//  static const char * _apStrings[] = {
//  "Japanese:",
//  "1 - \xe3\x82\xa8\xe3\x83\xb3\xe3\x82\xb3\xe3\x83\xbc"
//    "\xe3\x83\x87\xe3\x82\xa3\xe3\x83\xb3\xe3\x82\xb0",
//  "2 - \xe3\x83\x86\xe3\x82\xad\xe3\x82\xb9\xe3\x83\x88",
//    "3 - \xe3\x82\xb5\xe3\x83\x9d\xe3\x83\xbc\xe3\x83\x88",
//    "English:",
//  "1 - encoding",
//    "2 - text",
//    "3 - support",
//    "\xe6\x88\x91\xe5\xae\xb6\xe5\xae\x9d\xe5\xae\x9d\xe6\x9c\x80\xe5\x8f\xaf\xe7\x88\xb1(\xe0\xb9\x91\xe2\x80\xa2 . \xe2\x80\xa2\xe0\xb9\x91)"
//  };
//  int i;
//  GUI_Init();
//  GUI_SetFont(&GUI_Font16_1HK);
//  GUI_UC_SetEncodeUTF8();
//  for (i = 0; i < GUI_COUNTOF(_apStrings); i++) {
//    GUI_DispString(_apStrings[i]);
//    GUI_DispNextLine();
//  }
//  while(1)
//  {
//    GUI_Delay(100);
//  }
  extern GRAPH_DATA_Handle hData;
  extern void checkTouch(void);
  extern int data;
  //  uint16_t i;
  while(1)
  {
    checkTouch();
    //    for(i=0;i<400;i++)  
    //      GRAPH_DATA_YT_AddValue(hData,data);
    GUI_Delay(100);
  }
  
  
/* USER CODE END GRAPHICS_MainTask */
  while(1)
{
      GUI_Delay(100);
}
}

/*************************** End of file ****************************/
