/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : readme.txt
* Author             : MCD Application Team
* Version            : V1.0.1
* Date               : 10/20/2008
* Description        : Description of the STM32F10xxx DSP library
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

Firmware description
===================
This firmware contains a DSP library and a demo running on the STM3210B-EVAL board.

The DSP library is s suite of common DSP functions, which are:
    - PID controller.
    - Complex 16-bit radix-4 FFT, 64, 256 and 1024 points.
    - FIR 16-bit filter.
    - IIR Direct Form I 16-bit filter.
    - IIR Canonical Form 16-bit filter.
For more information, please refer to the user manual provided with this firmware.

The demo illustrates the use of the FFT function of the DSP library. It consists on applying the 64-points FFT transformation 
to two variable frequency waves, which are a sinus and a dual sinus. The waves and the result of the FFT transformation 
are displayed on the LCD of the STM3210B-EVAL board.



Directory contents
==================
  + project : containing the project workspace and project
  
  + include : containing the user header files 
    - stm32f10x_conf.h  Library Configuration files
    - stm32f10x_it.h    Interrupt handlers header files
    - stm32f10x_lcd.h   LCD firmware driver header file
    - fonts.h           LCD fonts size definition

    
  + source  : containg the user source files 
    - stm32f10x_lcd.c   LCD driver for AM-240320LTNQW00H(LCD_HX8312) and AM-240320L8TNQW00H (LCD_ILI9320) 
                        Liquid Crystal Display Module of STM3210B-EVAL board.
    - stm32f10x_it.c    Interrupt handlers
    - main.c            Main program

Hardware environment
====================
The demo is running on the STM3210B-EVAL board, while the DSP library is fully independant from hardware.
         
How to use it
=============


+ EWARMv5:
    - Open the project.eww workspace
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 + RVMDK
    - Open the project.Uv2 project
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)

 + RIDE
    - Open the project.rprj project
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)
  

******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE******
