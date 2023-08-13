/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : iir_stm32.c
* Author             : MCD Application Team
* Version            : V1.0.1
* Date               : 10/20/2008
* Description        : This source file contains IIR functions in C
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32_dsp.h"
#include "stm32f10x_lib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : IIR_biquad_stm32
* Description    : Canonique Form of 8th order IIR filter, factorized in 
*                  4 biquads sections in series.
* Input          : - y: Output array .
*                  - x: Input array 
*                  - IIRCoeff: IIR Filter Coefficients, an array of 20 shorts
*                  - ny: the number of output samples
* Output         : None
* Return         : None
*******************************************************************************/
void iir_biquad_stm32(u16 *y, u16 *x, s16 *IIRCoeff, u16 ny)
{
  u32 i;
  u32 w1_2 = 0, w1_1 = 0, w1;
  u32 w2_2 = 0, w2_1 = 0, w2;
  u32 w3_2 = 0, w3_1 = 0, w3;
  u32 w4_2 = 0, w4_1 = 0, w4;

  /** Canonic form **/
  /* 1st section */
  for (i=0; i<ny-2; i++)
  {
    w1 = x[2+i] - IIRCoeff[0]*w1_1 - IIRCoeff[1]*w1_2;
    y[2+i] = (IIRCoeff[2]*w1 + IIRCoeff[3]*w1_1 + IIRCoeff[4]*w1_2);
    w1_2 = w1_1;
    w1_1 = w1;
  }

  /* 2nd section */
  for (i=0; i<ny-2; i++)
  {
    w2 = y[2+i] - IIRCoeff[5]*w2_1 - IIRCoeff[6]*w2_2;
    y[2+i] = (IIRCoeff[7]*w2 + IIRCoeff[8]*w2_1 + IIRCoeff[9]*w2_2);
    w2_2 = w2_1;
    w2_1 = w2;
  }

  /* 3rd section */
  for (i=0; i<ny-2; i++)
  {
    w3 = y[2+i] - IIRCoeff[10]*w3_1 - IIRCoeff[11]*w3_2;
    y[2+i] = (IIRCoeff[12]*w3 + IIRCoeff[13]*w3_1 + IIRCoeff[14]*w3_2);
    w3_2 = w3_1;
    w3_1 = w3;
  }

  /* 4th section */
  for (i=0; i<ny-2; i++)
  {
    w4 = y[2+i] - IIRCoeff[15]*w4_1 - IIRCoeff[16]*w4_2;
    y[2+i] = (IIRCoeff[17]*w4 + IIRCoeff[18]*w4_1 + IIRCoeff[19]*w4_2);
    w4_2 = w4_1;
    w4_1 = w4;
  }

}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
