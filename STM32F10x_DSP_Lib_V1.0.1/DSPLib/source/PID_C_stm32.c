/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : PID_C_stm32.c
* Author             : MCD Application Team
* Version            : V1.0.1
* Date               : 10/20/2008
* Description        : This source file contains code PID controller
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32_dsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u16 PrevError = 0, IntTerm = 0;
u16 PrevError_C = 0, IntTerm_C = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : DoPID
* Description    : PID in C, Error computed outside the routine
* Input          : Error: difference between reference and measured value
*                  Coeff: pointer to the coefficient table
* Output         : None
* Return         : PID output (command)
*******************************************************************************/
u16 DoPID(u16 Error, u16 *Coeff)
{
  u16 Kp, Ki, Kd, Output;

  Kp = Coeff[0];
  Ki = Coeff[1];
  Kd = Coeff[2];

  IntTerm_C += Ki*Error;
  Output = Kp * Error;
  Output += IntTerm_C;
  Output += Kd * (Error - PrevError_C);

  PrevError_C = Error;

  return (Output);
}

/*******************************************************************************
* Function Name  : DoFullPID
* Description    : PID in C, Error computed inside the routine
* Input          : In: Input (measured value)
*                  Ref: reference (target value)
*                  Coeff: pointer to the coefficient table
* Output         : Computed Error
* Return         : PID output (command)
*******************************************************************************/
u16 DoFullPID(u16 In, u16 Ref, u16 *Coeff)
{
  u16 Kp, Ki, Kd, Output, Error;

  Error = Ref - In;
  Kp = Coeff[0];
  Ki = Coeff[1];
  Kd = Coeff[2];

  IntTerm_C += Ki*Error;
  Output = Kp * Error;
  Output += IntTerm_C;
  Output += Kd * (Error - PrevError_C);

  PrevError_C = Error;

  return (Output);
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
