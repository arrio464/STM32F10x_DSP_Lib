/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V1.0.1
* Date               : 10/20/2008
* Description        : Main program body
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
#include <math.h>
#include "stm32f10x_lcd.h"
#include "stm32_dsp.h"
#include "table_fft.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PI2  6.28318530717959

#define NPT 64            /* NPT = No of FFT point*/
#define DISPLAY_RIGHT 310 /* 224 for centered, 319 for right-aligned */
#define DISPLAY_LEFT 150  /* 224 for centered, 319 for right-aligned */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern u16 TableFFT[];
long lBUFIN[NPT];         /* Complex input vector */
long lBUFOUT[NPT];        /* Complex output vector */
long lBUFMAG[NPT + NPT/2];/* Magnitude vector */

/* Private function prototypes -----------------------------------------------*/
void MyDualSweep(u32 freqinc1,u32 freqinc2);
void MygSin(long nfill, long Fs, long Freq1, long Freq2, long Ampli);
void powerMag(long nfill, char* strPara);
void In_displayWaveform(u32 DisplayPos);
void Out_displayWaveform(u32 DisplayPos);
void displayPowerMag(u32 DisplayPos, u32 scale);
void DisplayTitle(void);
void DSPDemoInit(void);

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
  #ifdef DEBUG
    debug();
  #endif

  DSPDemoInit();
  DisplayTitle();

  while (1)
  {
    MyDualSweep(30,30);
  }
}

/*******************************************************************************
* Function Name  : MygSin
* Description    : Produces a combination of two sinewaves as input signal
* Input          : Freq1: frequency increment for 1st sweep
*                  Freq2: frequency increment for 2nd sweep
* Output         : None
* Return         : None
*******************************************************************************/
void MyDualSweep(u32 freqinc1,u32 freqinc2)
{
  u32 freq;

  for (freq=40; freq <4000; freq+=freqinc1)
  {
    MygSin(NPT, 8000, freq, 0, 32767);
    GPIOC->BSRR = GPIO_Pin_7;
    
    cr4_fft_64_stm32(lBUFOUT, lBUFIN, NPT);

    GPIOC->BRR = GPIO_Pin_7;
    powerMag(NPT,"2SIDED");
    In_displayWaveform(DISPLAY_RIGHT);

    displayPowerMag(DISPLAY_RIGHT, 9);

    while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0x00);
  }

  for (freq=40; freq <4000; freq+=freqinc2)
  {
    MygSin(NPT, 8000, freq, 160, 32767/2);
    GPIOC->BSRR = GPIO_Pin_7;
    
    cr4_fft_64_stm32(lBUFOUT, lBUFIN, NPT);
   
    GPIOC->BRR = GPIO_Pin_7;
    powerMag(NPT,"2SIDED");
    In_displayWaveform(DISPLAY_LEFT);
    displayPowerMag(DISPLAY_LEFT, 8);

    while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0x00);
  }

}

/*******************************************************************************
* Function Name  : MygSin
* Description    : Produces a combination of two sinewaves as input signal
* Input          : nfill: length of the array holding input signal
*                  Fs: sampling frequency
*                  Freq1: frequency of the 1st sinewave
*                  Freq2: frequency of the 2nd sinewave
*                  Ampli: scaling factor
* Output         : None
* Return         : None
*******************************************************************************/
void MygSin(long nfill, long Fs, long Freq1, long Freq2, long Ampli)
{
 
  u32 i;
  float fFs, fFreq1, fFreq2, fAmpli;
  float fZ,fY;

  fFs = (float) Fs;
  fFreq1 = (float) Freq1;
  fFreq2 = (float) Freq2;
  fAmpli = (float) Ampli;

  for (i=0; i < nfill; i++)
  {
    fY = sin(PI2 * i * (fFreq1/fFs)) + sin(PI2 * i * (fFreq2/fFs));
    fZ = fAmpli * fY;
    lBUFIN[i]= ((short)fZ) << 16 ;  /* sine_cosine  (cos=0x0) */
  }
}


/*******************************************************************************
* Function Name  : onesided
* Description    : Removes the aliased part of the spectrum (not tested)
* Input          : nfill: length of the array holding power mag
* Output         : None
* Return         : None
*******************************************************************************/
void onesided(long nfill)
{
  u32 i;
  
  lBUFMAG[0] = lBUFMAG[0];
  lBUFMAG[nfill/2] = lBUFMAG[nfill/2];
  for (i=1; i < nfill/2; i++)
  {
    lBUFMAG[i] = lBUFMAG[i] + lBUFMAG[nfill-i];
    lBUFMAG[nfill-i] = 0x0;
  }
}


/*******************************************************************************
* Function Name  : powerMag
* Description    : Compute power magnitude of the FFT transform
* Input          : nfill: length of the array holding power mag
*                : strPara: if set to "1SIDED", removes aliases part of spectrum (not tested)
* Output         : None
* Return         : None
*******************************************************************************/
void powerMag(long nfill, char* strPara)
{
  s32 lX,lY;
  u32 i;

  for (i=0; i < nfill; i++)
  {
    lX= (lBUFOUT[i]<<16)>>16; /* sine_cosine --> cos */
    lY= (lBUFOUT[i] >> 16);   /* sine_cosine --> sin */    
    {
      float X=  64*((float)lX)/32768;
      float Y = 64*((float)lY)/32768;
      float Mag = sqrt(X*X+ Y*Y)/nfill;
      lBUFMAG[i] = (u32)(Mag*65536);
    }    
  }
  if (strPara == "1SIDED") onesided(nfill);
}

/*******************************************************************************
* Function Name  : In_displayWaveform
* Description    : Displays the input array before filtering
* Input          : DisplayPos indicates the beginning Y address on LCD
* Output         : None
* Return         : None
*******************************************************************************/
void In_displayWaveform(u32 DisplayPos)
{
  u8 aScale;
  u16 cln;

  for (cln=0; cln < 64; cln++)       /* original upper limit was 60 */
  {
    /* Clear previous line */
    LCD_SetTextColor(White);
    LCD_DrawLine(48,DisplayPos-(2*cln),72,Vertical);
    /* and go back to normal display mode */
    LCD_SetTextColor(Black);
    aScale = lBUFIN[cln]>>(10 + 16);  /* SINE IS LEFT ALIGNED */
    if (aScale > 127)         /* Negative values */
    {
      aScale = (0xFF-aScale) + 1; /* Calc absolute value */
      LCD_DrawLine(84-aScale,DisplayPos-(2*cln),aScale,Vertical);
    }
    else  /* Display positive values */
    {
      LCD_DrawLine(84,DisplayPos-(2*cln),aScale,Vertical);
    }
  }/* for */
}


/*******************************************************************************
* Function Name  : Out_displayWaveform
* Description    : Displays the output array after filtering
* Input          : DisplayPos indicates the beginning Y address on LCD
* Output         : None
* Return         : None
*******************************************************************************/
void Out_displayWaveform(u32 DisplayPos)
{
  u8 aScale;
  u16 cln;

  for (cln=0; cln < 64; cln++)    /* original upper limit was 60 */
  {
    aScale = lBUFOUT[cln]>>(10 + 16);  /* SINE IS LEFT ALIGNED */
    if (aScale > 127)         /* Negative values */
    {
      aScale = (0xFF-aScale) + 1; /* Calc absolute value */
      LCD_DrawLine(156-aScale,DisplayPos-(2*cln),aScale,Vertical);
    }
    else  /* Display positive values */
    {
      LCD_DrawLine(156,DisplayPos-(2*cln),aScale,Vertical);
    }
  }/* for */
}

/*******************************************************************************
* Function Name  : displayPowerMag
* Description    : Displays the power magnitude array following a FFT
* Input          : DisplayPos indicates the beginning Y address on LCD
*                : scale allows to normalize the result for optimal display
* Output         : None
* Return         : None
*******************************************************************************/
void displayPowerMag(u32 DisplayPos, u32 scale)
{
  u16 aScale;
  u16 cln;

  for (cln=0; cln < 64; cln++)
  {
    /* Clear previous line */
    LCD_SetTextColor(White);
    LCD_DrawLine(120,DisplayPos-(2*cln),72,Vertical);
    /* and go back to normal display mode */
    LCD_SetTextColor(Red);

    aScale =lBUFMAG[cln]>>scale ; /* spectrum is always divided by two */
    /* Power Mag contains only positive values */
    LCD_DrawLine(192-aScale,DisplayPos-(2*cln),aScale,Vertical);
  }/* for */
}

/*******************************************************************************
* Function Name  : DisplayTitle
* Description    : Displays the initial menu and then background frame
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DisplayTitle(void)
{
  u8 *ptr = "   STM32 DSP Demo   ";

  LCD_DisplayStringLine(Line0, ptr);

  ptr = " 64-pts Radix-4 FFT ";
  LCD_DisplayStringLine(Line1, ptr);

  ptr = "Press Key to freeze ";
  LCD_DisplayStringLine(Line9, ptr);

  {
    unsigned long long Delay = 0x600000;
    while (Delay--);
    while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0x00);
  }

  ptr = "   Sine   Dual sine ";
  LCD_DisplayStringLine(Line9, ptr);
}


/*******************************************************************************
* Function Name  : SysTick_Config
* Description    : Configure a SysTick Base time to 10 ms.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Config(void)
{
  /* Configure HCLK clock as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

  /* SysTick interrupt each 100 Hz with HCLK equal to 72MHz */
  SysTick_SetReload(720000);

  /* Enable the SysTick Interrupt */
  SysTick_ITConfig(ENABLE);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length (time base 10 ms).
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(u32 nCount)
{

  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);

  while (nCount--)
  {
    while (SysTick_GetFlagStatus(SysTick_FLAG_COUNT) == RESET);
  }

  /* Disable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Disable);

  /* Clear the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Clear);
}


/*******************************************************************************
* Function Name  : DSPDemoInit
* Description    : Initializes the DSP lib demo (clock, peripherals and LCD).
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DSPDemoInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ErrorStatus HSEStartUpStatus = ERROR;

  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

  /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
         | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

  /* TIM1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);


  /* SPI2 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  /* TIM2  and TIM4 clocks enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);

  /*------------------- Resources Initialization -----------------------------*/
  /* GPIO Configuration */
  /* Configure PC.06 and PC.07 as Output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure the systick */
  SysTick_Config();

  /*------------------- Drivers Initialization -------------------------------*/
  /* Initialize the LCD */
  STM3210B_LCD_Init();

  /* Clear the LCD */
  LCD_Clear(White);

  LCD_SetTextColor(White);
  LCD_SetBackColor(Black);

}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
