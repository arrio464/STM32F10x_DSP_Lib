/**
  @page FFT_Demo FFT_Demo Readme File
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file FFT_Demo/readme.txt 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    Description of the STM32F10xxx DSP library.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
   @endverbatim

@par Description 

This firmware contains a DSP library and a demo running on the STM3210B-EVAL board.

The DSP library is s suite of common DSP functions, which are:
    - PID controller.
    - Complex 16-bit radix-4 FFT, 64, 256 and 1024 points.
    - FIR 16-bit filter.
    - IIR Direct Form I 16-bit filter.
    - IIR Canonical Form 16-bit filter.
For more information, please refer to the user manual provided with this firmware.

The demo illustrates the use of the FFT function of the DSP library. 
It consists on applying the 64-points FFT transformation to two variable frequency 
waves, which are a sinus and a dual sinus. The waves and the result of the FFT 
transformation are displayed on the LCD of the STM3210B-EVAL board.

@par Directory contents 
  
  - inc: containing the user header files  
      - FFT_Demo/inc/stm32f10x_conf.h  Library Configuration files
      - FFT_Demo/inc/stm32f10x_it.h    Interrupt handlers header files
      - FFT_Demo/inc/stm3210b_lcd.h    LCD firmware driver header file
      - FFT_Demo/inc/fonts.h           LCD fonts size definition
    
  - src: containg the user source files  
      - FFT_Demo/src/stm3210b_lcd.c   LCD driver for Liquid Crystal Display Module of STM3210B-EVAL board
      - FFT_Demo/src/stm32f10x_it.c    Interrupt handlers
      - FFT_Demo/src/main.c            Main program

@par Hardware and Software environment 

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL evaluation 
    boards and can be easily tailored to any other supported device and development 
    board.
    
  - The DSP library is fully independant from hardware.
         
@par How to use it ? 

In order to load the FFT Demo code, you have do the following:

- EWARM: 
    - Open the FFT_Demo.eww workspace
    - In the workspace toolbar select the project config:
        - STM3210B-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE 
    - Open the FFT_Demo.rprj project
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210B-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)
@note 
 - When activating the "assert" function in debug mode, this application may not
   work properly with RIDE toolchain.

 - RVMDK 
    - Open the FFT_Demo.Uv2 project
    - In the build toolbar select the project config:  
        - STM3210B-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)
  
@note
 - Low-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
   
 * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
 */
