/**
  ******************************************************************************
  * @file    system_stm32f2xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  *             
  *   This file provides two functions and one global variable to be called from 
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f2xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f2xx_system
  * @{
  */  
  
/** @addtogroup STM32F2xx_System_Private_Includes
  * @{
  */

//By TFT: was #include "stm32f4xx_hal.h", but the specific chip is #defined in
//arch_registers_impl.h
#include "interfaces/arch_registers.h"
//By TFT: was in the old stm32f4xx.h
#define HSE_STARTUP_TIMEOUT    ((uint16_t)0x0500)

//#if !defined  (HSE_VALUE)
//  #define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */
//#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

//HSE range 4-26 MHz
#ifdef HSE_VALUE
#define RCC_PLLSRC RCC_PLLCFGR_PLLSRC_HSE
    #define HS_FREQ HSE_VALUE
    #if HSE_VALUE < 4000000 || HSE_VALUE > 26000000
        #error "frequency not supported by external oscillator"
    #endif
#else
#define RCC_PLLSRC RCC_PLLCFGR_PLLSRC_HSI
#define HS_FREQ HSI_VALUE
#endif

/**
  * @}
  */

/** @addtogroup STM32F2xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F2xx_System_Private_Defines
  * @{
  */
/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM mounted
     on STM322xG_EVAL board as data memory  */
//By TFT: Miosix uses the __ENABLE_XRAM macro to tell the startup code it wants XRAM
//additionally, other boards may have the RAM connected in other ways, so
//use this code only for the STM3220G_EVAL
#if defined(__ENABLE_XRAM) && defined(_BOARD_STM3220G_EVAL)
#define DATA_IN_ExtSRAM
#endif //__ENABLE_XRAM

/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */

//By TFT -- begin
// the smartwatch has a bootloader
#ifdef _BOARD_SONY_NEWMAN
#define USER_VECT_TAB_ADDRESS
#define VECT_TAB_BASE_ADDRESS  FLASH_BASE
#define VECT_TAB_OFFSET        0x40000
#endif //_BOARD_SONY_NEWMAN
//By TFT -- end

//SYSCLK_FREQ <= 168000000
#ifdef SYSCLK_FREQ
    uint32_t SystemCoreClock = (uint32_t)SYSCLK_FREQ;
    #if SYSCLK_FREQ > 168000000
        #error "Max Frequency supported = 168MHz"
    #endif
#else
    #error "Clock not selected"
#endif

#define USB_FREQ 48000000

void setParameters(int FIN, int FOUT, int* param_m, int* param_n, int* param_q, int* param_p) {
    /* ---- declarations ----*/
    float p_choices[4][2];
    float n_choices[300][3];
    int full_conf[4] = {0};

    float ratio;
    float ratio_usb;

    /* ---- initializations ----*/
    int i,j,p,m,q;
    for (i=0; i<4; i++){
        for(j=0; j<2; j++){
            p_choices[i][j] = 0;
        }
    }

    for (i=0; i<16; i++){
        n_choices[i][0] = 0;
        n_choices[i][1] = 0;
        n_choices[i][2] = 0;
    }

    ratio = (float)FOUT/(float)FIN;
    ratio_usb = (float)USB_FREQ/(float)FIN;

    /* ---- let's find these parameters ----
    // p can be 2,4,6,8
    // p_choices(p,ratio*p)*/
    for (p=1; p<=4; p++){
        p_choices[p-1][0] = 2*p;
        p_choices[p-1][1] = ratio*2*p;
    }

    int n_idx = 0;
    float nn;
    for (m=2; m<=63; m++) {
        for (p=0; p<4; p++){
            nn = m*p_choices[p][1];
            if (nn >= 2 && nn <= 432 && nn-(int)nn == 0){
                n_choices[n_idx][0] = nn;
                n_choices[n_idx][1] = m;
                n_choices[n_idx][2] = p_choices[p][0];
                n_idx++;
            }
        }
    }
    if (n_idx){
        float qq;
        int idx_conf = 0;
        int flag = 0;
        float a;
        for (i=0; i<n_idx && !flag; i++){
            a = n_choices[i][0]/n_choices[i][1];
            qq = a/ratio_usb;

            if (qq>=2 && qq<=15 && qq-(int)qq == 0){
                full_conf[0] = (int)n_choices[i][0];
                full_conf[1] = (int)n_choices[i][1];
                full_conf[2] = (int)n_choices[i][2];
                full_conf[3] = (int)qq;
                idx_conf++;
                flag = 1;
            }
        }

        if (idx_conf == 0){
            float min_diff = (float)USB_FREQ;
            float out_usb, diff;

            for (i=0; i<n_idx; i++){
                for (q=2; q<=15; q++){
                    out_usb = (((float)FIN*n_choices[i][0]) / (n_choices[i][1]*q));
                    diff = (float)USB_FREQ - out_usb;
                    if (diff >= 0 && diff<=min_diff){
                        min_diff = diff;
                        full_conf[0] = (int)n_choices[i][0];
                        full_conf[1] = (int)n_choices[i][1];
                        full_conf[2] = (int)n_choices[i][2];
                        full_conf[3] = (int)q;
                    }
                }
            }
        }
    }
    *param_n = full_conf[0];
    *param_m = full_conf[1];
    *param_p = full_conf[2];
    *param_q = full_conf[3];
}
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F2xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F2xx_System_Private_Variables
  * @{
  */
  
  /* This variable can be updated in Three ways :
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */

  const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
  const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
/**
  * @}
  */

/** @addtogroup STM32F2xx_System_Private_FunctionPrototypes
  * @{
  */

static void SetSysClock(void);
#ifdef DATA_IN_ExtSRAM
  static void SystemInit_ExtMemCtl(void); 
#endif /* DATA_IN_ExtSRAM */

/**
  * @}
  */

/** @addtogroup STM32F2xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SystemFrequency variable.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
#ifdef DATA_IN_ExtSRAM
  SystemInit_ExtMemCtl(); 
#endif /* DATA_IN_ExtSRAM */

  /* Configure the System clock source, PLL Multiplier and Divider factors, 
     AHB/APBx prescalers and Flash settings ----------------------------------*/
  SetSysClock();

  /* Configure the Vector Table location -------------------------------------*/
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#endif /* USER_VECT_TAB_ADDRESS */
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f2xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f2xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock source */

      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */    
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  SystemCoreClock >>= tmp;
}

//By TFT -- begin
// this was backported from an older version. Now this code seems to be
// moved in a function called HAL_something...
/**
  * @brief  Configures the System clock source, PLL Multiplier and Divider factors, 
  *         AHB/APBx prescalers and Flash settings
  * @Note   This function should be called only once the RCC clock configuration  
  *         is reset to the default reset state (done in SystemInit() function).   
  * @param  None
  * @retval None
  */
static void SetSysClock(void)
{
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    #ifdef _BOARD_SONY_NEWMAN
    //By TFT: We don't know how the clock is configured by the bootloader,
    //so better switch to the HSE and disable the PLL.
    unsigned int temp=RCC->CFGR;
    temp &= ~RCC_CFGR_SW;  /* Clear SW[1:0] bits */
    temp |= RCC_CFGR_SW_0; /* Enable HSE as system clock */
    RCC->CFGR=temp;
    while((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_0) ;
    RCC->CR &= ~ RCC_CR_PLLON;
    #endif //_BOARD_SONY_NEWMAN
      
    /* HCLK = SYSCLK / 1*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    
    /* PCLK1 = HCLK / 4*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

    /* Configure the main PLL */
    int param_m, param_n, param_q, param_p;
    setParameters(HS_FREQ, SYSCLK_FREQ, &param_m, &param_n, &param_q, &param_p);

    #define PLL_M param_m
    #define PLL_N param_n
    #define PLL_Q param_q
    #define PLL_P param_p

    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLSRC) | (PLL_Q << 24);

    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till the main PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }
   
    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
#ifndef _BOARD_SONY_NEWMAN
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;
#else //_BOARD_SONY_NEWMAN
    //By TFT: Three wait states seem to make it unstable (crashing) when CPU load is high
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_7WS;
#endif //_BOARD_SONY_NEWMAN
    
    /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock
         configuration. User can add here some code to deal with this error */
  }

}
// By TFT -- end

#ifdef DATA_IN_ExtSRAM
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f2xx.s before jump to main.
  *         This function configures the external SRAM mounted on STM322xG_EVAL board
  *         This SRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
/*-- GPIOs Configuration -----------------------------------------------------*/
   /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR   |= 0x00000078;
  RCC_SYNC();

  /* Connect PDx pins to FSMC Alternate function */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FSMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FSMC Alternate function */
  GPIOF->AFR[0]  = 0x00CCCCCC;
  GPIOF->AFR[1]  = 0xCCCC0000;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA000AAA;
  /* Configure PFx pins speed to 100 MHz */ 
  GPIOF->OSPEEDR = 0xFF000FFF;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FSMC Alternate function */
  GPIOG->AFR[0]  = 0x00CCCCCC;
  GPIOG->AFR[1]  = 0x000000C0;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0x00085AAA;
  /* Configure PGx pins speed to 100 MHz */ 
  GPIOG->OSPEEDR = 0x000CAFFF;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
/*--FSMC Configuration -------------------------------------------------------*/
  /* Enable the FSMC interface clock */
  RCC->AHB3ENR         |= 0x00000001;
  RCC_SYNC();
  /* Configure and enable Bank1_SRAM2 */
  FSMC_Bank1->BTCR[2]  = 0x00001011;
  FSMC_Bank1->BTCR[3]  = 0x00000201;
  FSMC_Bank1E->BWTR[2] = 0x0FFFFFFF;
}
#endif /* DATA_IN_ExtSRAM */


/**
  * @}
  */

/**
  * @}
  */
  
/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
