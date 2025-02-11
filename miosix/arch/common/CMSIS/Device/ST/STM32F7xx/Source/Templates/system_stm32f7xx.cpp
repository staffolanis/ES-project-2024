/**
  ******************************************************************************
  * @file    system_stm32f7xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M7 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from 
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f7xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f7xx_system
  * @{
  */  
  
/** @addtogroup STM32F7xx_System_Private_Includes
  * @{
  */

//By TFT: was #include "stm32f7xx.h", but the specific chip is #defined in
//arch_registers_impl.h
#include "interfaces/arch_registers.h"

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

#define USB_FREQ 48000000

struct Parameters 
{
    int param_n;
    int param_m;
    int param_p;
    int param_q;
    int param_r;
    int state;
};

template<int FIN, int FOUT> constexpr Parameters calculateParameters() 
{
    float p_choices[4][2] = {};
    float ratio = (float)FOUT/(float)FIN;
    float ratio_usb = USB_FREQ/(float)FIN;

    float n_choices[300][3] = {};
    int full_conf[300][4] = {};

    /* ---- let's find these parameters ----
    // p can be 2,4,6,8 
    // p_choices(p,ratio*p)*/
    for (int p=0; p<4; p++)
    {
        p_choices[p][0] = 2*(p+1);
        p_choices[p][1] = ratio*2*(p+1);
    }
    int n_idx = 0;
    for (int m=2; m<=63; m++) 
    {
            for (int p=0; p<4; p++)
            {
                float nn = m*p_choices[p][1];
                float vco = FIN * (nn/m);
                if (nn >= 50 && nn <= 432 && nn-(int)nn == 0 && vco >= 100e6 && vco <= 432e6)
                { 
                    n_choices[n_idx][0] = nn;
                    n_choices[n_idx][1] = m;
                    n_choices[n_idx][2] = p_choices[p][0];
                    n_idx++;
                }
            }
    }
    
    if(n_idx == 0)
    {
        return {0,0,0,0,0,1};
    }
    else
    {
        int idx_conf = 0;
        int state = 0;
        for (int i=0; i<n_idx && idx_conf == 0; i++)
        {
            float qq = (n_choices[i][0]/n_choices[i][1])/ratio_usb;        
            if (qq>=2 && qq<=15 && qq-(int)qq == 0)
            {
                full_conf[idx_conf][0] = (int)n_choices[i][0];
                full_conf[idx_conf][1] = (int)n_choices[i][1];
                full_conf[idx_conf][2] = (int)n_choices[i][2];
                full_conf[idx_conf][3] = (int)qq;
                idx_conf++;
            }
        }

        if (idx_conf == 0)
        {
            state = 2;

            float min_diff = USB_FREQ;
            for (int i=0; i<n_idx; i++)
            {
                for (int q=2; q<=15; q++)
                {
                    float out_usb = ((FIN*n_choices[i][0]) / (n_choices[i][1]*q));
                    float diff = USB_FREQ - out_usb;
                    if (diff >= 0 && diff<=min_diff)
                    {
                        min_diff = diff;
                        full_conf[idx_conf][0] = (int)n_choices[i][0];
                        full_conf[idx_conf][1] = (int)n_choices[i][1];
                        full_conf[idx_conf][2] = (int)n_choices[i][2];
                        full_conf[idx_conf][3] = (int)q;
                        idx_conf++;
                    }
                }
            }
        }

        idx_conf --;

        return {full_conf[idx_conf][0], full_conf[idx_conf][1], full_conf[idx_conf][2], full_conf[idx_conf][3], 2, state};
    }
}

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */
/******************************************************************************/


//SYSCLK_FREQ <= 216000000
#ifdef SYSCLK_FREQ
    uint32_t SystemCoreClock = (uint32_t)SYSCLK_FREQ;
    #if SYSCLK_FREQ > 216000000
        #error "Max Frequency supported = 216MHz"
    #endif
#else
    #error "Clock not selected"
#endif

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_Variables
  * @{
  */

  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */

  //uint32_t SystemCoreClock = 16000000;
  const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
  const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_FunctionPrototypes
  * @{
  */

//By TFT: added PLL initialization
static void SetSysClk(void);

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SystemFrequency variable.
  * @param  None
  * @retval None
  */
void SystemInit() 
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #else
  #error "FPU disabled!" //By TFT: added a check to be really sure the FPU is on
  #endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;
  
  //By TFT -- begin
  SetSysClk();
  //NOTE: we don't enable caches here because different boards may want to do
  //so or not depending on whether they have external memories for code and data
  //By TFT -- end

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = RAMDTCM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
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
  *         (*) HSI_VALUE is a constant defined in stm32f7xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f7xx_hal_conf.h file (default value
  *              25 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
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

//By TFT: added PLL initialization that was not present in the CMSIS code
void SetSysClk(void) 
{
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
  
  /* Enable Power Control clock */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
 
  /* Config Voltage Scale 1 */
  PWR->CR1 |= PWR_CR1_VOS;
  
  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    tmpreg = RCC->CR & RCC_CR_HSERDY;
  } while((tmpreg != RCC_CR_HSERDY) && (timeout-- > 0));
  
  if(timeout != 0)
  {  
    /* Select regulator voltage output Scale 1 mode */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    PWR->CR1 |= PWR_CR1_VOS;
    
    /* Enable Over Drive to reach the 216MHz frequency */
    /* Enable ODEN */
    PWR->CR1 |= 0x00010000;
    timeout = 0xFFFF;
    /* Wait till ODR is ready and if Time out is reached exit */
    do
    {
      tmpreg = PWR->CSR1 & PWR_CSR1_ODRDY;
    } while((tmpreg != PWR_CSR1_ODRDY) && (timeout-- > 0));
    
    /* Enable ODSW */
    PWR->CR1 |= 0x00020000;
    timeout = 0xFFFF;
    /* Wait till ODR is ready and if Time out is reached exit */
    do
    {
      tmpreg = PWR->CSR1 & PWR_CSR1_ODSWRDY;
    } while((tmpreg != PWR_CSR1_ODSWRDY) && (timeout-- > 0));
   
    /* HCLK = SYSCLK / 1*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    
    /* PCLK2 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    
    /* PCLK1 = HCLK / 4*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

    /* Configure the main PLL */
    constexpr static Parameters param = calculateParameters<HS_FREQ,SYSCLK_FREQ>();

    static_assert(param.state != 1, "Error: no configuration found for the PLL");
    static_assert(param.state != 2, "Error: NO PERFECT configuration found for the PLL USB at 48 MHz");

    RCC->PLLCFGR = param.param_m | (param.param_n << 6) | (((param.param_p >> 1) -1) << 16) |
      (RCC_PLLSRC) | (param.param_q << 24) | (param.param_r << 28);
    
    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;
  }  
  /* Wait that PLL is ready */
  timeout = 0xFFFF;
  do
  {
    tmpreg = (RCC->CR & RCC_CR_PLLRDY);
  } while((tmpreg != RCC_CR_PLLRDY) && (timeout-- > 0));
  
  if(timeout != 0)
  {
    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    FLASH->ACR = FLASH_ACR_LATENCY_7WS;
    
    /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    
    timeout = 0xFFFF;
    do
    {
      tmpreg = (RCC->CFGR & RCC_CFGR_SWS);
    } while((tmpreg != RCC_CFGR_SWS) && (timeout-- > 0));
  }   
}

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
