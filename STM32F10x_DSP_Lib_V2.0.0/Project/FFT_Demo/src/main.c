/**
  ******************************************************************************
  * @file FFT_Demo/src/main.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <math.h>
#include "stm3210b_lcd.h"
#include "stm32_dsp.h"
#include "table_fft.h"


/** @addtogroup FFT_Demo
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PI2  6.28318530717959

#define NPT 64            /* NPT = No of FFT point*/
#define DISPLAY_RIGHT 310 /* 224 for centered, 319 for right-aligned */
#define DISPLAY_LEFT 150  /* 224 for centered, 319 for right-aligned */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint16_t TableFFT[];
extern volatile uint32_t TimingDelay ;
long lBUFIN[NPT];         /* Complex input vector */
long lBUFOUT[NPT];        /* Complex output vector */
long lBUFMAG[NPT + NPT/2];/* Magnitude vector */

/* Private function prototypes -----------------------------------------------*/
void MyDualSweep(uint32_t freqinc1,uint32_t freqinc2);
void MygSin(long nfill, long Fs, long Freq1, long Freq2, long Ampli);
void powerMag(long nfill, char* strPara);
void In_displayWaveform(uint32_t DisplayPos);
void Out_displayWaveform(uint32_t DisplayPos);
void displayPowerMag(uint32_t DisplayPos, uint32_t scale);
void DisplayTitle(void);
void DSPDemoInit(void);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{

  DSPDemoInit();
  DisplayTitle();

  while (1)
  {
    MyDualSweep(30,30);
  }
}



/**
  * @brief  Produces a combination of two sinewaves as input signal
  * @param eq1: frequency increment for 1st sweep
  *   Freq2: frequency increment for 2nd sweep
  * @retval : None
  */
void MyDualSweep(uint32_t freqinc1,uint32_t freqinc2)
{
  uint32_t freq;

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



/**
  * @brief  Produces a combination of two sinewaves as input signal
  * @param ill: length of the array holding input signal
  *   Fs: sampling frequency
  *   Freq1: frequency of the 1st sinewave
  *   Freq2: frequency of the 2nd sinewave
  *   Ampli: scaling factor
  * @retval : None
  */
void MygSin(long nfill, long Fs, long Freq1, long Freq2, long Ampli)
{
 
  uint32_t i;
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




/**
  * @brief  Removes the aliased part of the spectrum (not tested)
  * @param ill: length of the array holding power mag
  * @retval : None
  */
void onesided(long nfill)
{
  uint32_t i;
  
  lBUFMAG[0] = lBUFMAG[0];
  lBUFMAG[nfill/2] = lBUFMAG[nfill/2];
  for (i=1; i < nfill/2; i++)
  {
    lBUFMAG[i] = lBUFMAG[i] + lBUFMAG[nfill-i];
    lBUFMAG[nfill-i] = 0x0;
  }
}




/**
  * @brief  Compute power magnitude of the FFT transform
  * @param ill: length of the array holding power mag
  *   : strPara: if set to "1SIDED", removes aliases part of spectrum (not tested)
  * @retval : None
  */
void powerMag(long nfill, char* strPara)
{
  int32_t lX,lY;
  uint32_t i;

  for (i=0; i < nfill; i++)
  {
    lX= (lBUFOUT[i]<<16)>>16; /* sine_cosine --> cos */
    lY= (lBUFOUT[i] >> 16);   /* sine_cosine --> sin */    
    {
      float X=  64*((float)lX)/32768;
      float Y = 64*((float)lY)/32768;
      float Mag = sqrt(X*X+ Y*Y)/nfill;
      lBUFMAG[i] = (uint32_t)(Mag*65536);
    }    
  }
  if (strPara == "1SIDED") onesided(nfill);
}



/**
  * @brief  Displays the input array before filtering
  * @param splayPos indicates the beginning Y address on LCD
  * @retval : None
  */
void In_displayWaveform(uint32_t DisplayPos)
{
  uint8_t aScale;
  uint16_t cln;

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




/**
  * @brief  Displays the output array after filtering
  * @param splayPos indicates the beginning Y address on LCD
  * @retval : None
  */
void Out_displayWaveform(uint32_t DisplayPos)
{
  uint8_t aScale;
  uint16_t cln;

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



/**
  * @brief  Displays the power magnitude array following a FFT
  * @param splayPos indicates the beginning Y address on LCD
  *   : scale allows to normalize the result for optimal display
  * @retval : None
  */
void displayPowerMag(uint32_t DisplayPos, uint32_t scale)
{
  uint16_t aScale;
  uint16_t cln;

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



/**
  * @brief  Displays the initial menu and then background frame
  * @param  None
  * @retval : None
  */
void DisplayTitle(void)
{
  uint8_t *ptr = "   STM32 DSP Demo   ";

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






/**
  * @brief  Inserts a delay time.
  * @param ount: specifies the delay time length (time base 10 ms).
  * @retval : None
  */
void Delay(uint32_t nCount)
{
  /* Configure the systick */
  __enable_irq(); 
  	
     /* Setup SysTick Timer for 10 msec interrupts */
  if (SysTick_Config(SystemFrequency /100)) /* SystemFrequency is defined in
   “system_stm32f10x.h” and equal to HCLK frequency */
   {
    /* Capture error */
     while (1);
    }     
  /* Enable the SysTick Counter */
  TimingDelay = nCount;
   while (TimingDelay )  {}	    
   TimingDelay=0;
   __disable_irq();

}




/**
  * @brief  Initializes the DSP lib demo (clock, peripherals and LCD).
  * @param  None
  * @retval : None
  */
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

 

  /*------------------- Drivers Initialization -------------------------------*/
  /* Initialize the LCD */
  STM3210B_LCD_Init();

  /* Clear the LCD */
  LCD_Clear(White);

  LCD_SetTextColor(White);
  LCD_SetBackColor(Black);

}

#ifdef  USE_FULL_ASSERT


/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 



/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
