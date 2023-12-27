/**
  ******************************************************************************
  * @file    system_ca32m030.c
  * @brief   CMSIS Cortex-M0 Device Peripheral Access Layer System Source File.
  *          This file contains the system clock configuration for CA32 devices
  *
  * 1.  This file provides two functions and one global variable to be called from 
  *     user application:
  *      - SystemInit(): Setups the system clock (System clock source, PLL Multiplier
  *                      and Divider factors, AHB/APBx prescalers and Flash settings),
  *                      depending on the configuration made in the clock xls tool.
  *                      This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_ca32m030.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * 2. After each device reset the HSI/2 (16MHz) is used as system clock source.
  *    Then SystemInit() function is called, in "startup_ca32m030.s" file, to
  *    configure the system clock before to branch to main program.
  *
  * 3. If the system clock source selected by user fails to startup, the SystemInit()
  *    function will do nothing and HSI still used as system clock source. User can 
  *    add some code to deal with this issue inside the SetSysClock() function.
  *
  *
  * 5. This file configures the system clock as follows:
  *=============================================================================
  *=============================================================================
  *        System Clock source                    | HSI
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 16000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 32000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 2
  *-----------------------------------------------------------------------------
  *        APB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        Flash Latency(WS)                      | 0
  *-----------------------------------------------------------------------------
  *        Prefetch Buffer                        | ON
  *-----------------------------------------------------------------------------
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "ca32.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t SystemCoreClock = 24000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t PLLVcoTable[4] = {0, 1, 2, 3};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Setup the microcontroller system.
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SystemCoreClock variable.
  * @param  None
  * @retval None
  */
void SystemInit (void)
{
	uint32_t tmpreg;
	uint16_t HSI_calValue;
	uint16_t LSI_calValue;
	uint16_t LDO_1P5_calValue;
	uint16_t LDO_1P2_calValue;
	
  /* HSI:24MHz, PLL:60MHz, HCLK:60MHz, PCLK:60MHz, Flash:8MHz, ADC:24MHz */
	FLASH->ACR = 2<<FLASH_ACR_LATENCY_Pos;
	RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE_DIV1 | RCC_CFGR_FLSPRE_DIV3 | RCC_CFGR_FLSEN | RCC_CFGR_ADCKPRE_NONE;
	
	/* HSI*M/N/OD(1:div2) = 24 * 5 / 1 / 2 = 60MHz */
	RCC->PLLR = (5<<RCC_PLLR_M_Pos) | (1<<RCC_PLLR_N_Pos) | (1<<RCC_PLLR_OD_Pos) | RCC_PLLR_PLL_OEN;
	while((RCC->PLLR & RCC_PLLR_PLL_LOCK) == 0);
	while((RCC->PLLR & RCC_PLLR_PLL_RDY) == 0);	
	RCC->CFGR |= RCC_CFGR_SW;
  while((RCC->CFGR & RCC_CFGR_SWS) == 0); 
  
  /* HSI calibration value update */
	HSI_calValue = *(__IO uint16_t *)HSI_CAL_ADDR;
	if(HSI_calValue != 0xFFFF)
	{
    tmpreg = RCC->CR;
    tmpreg &= (uint32_t)~((uint32_t)RCC_CR_HSICAL_Msk);
    tmpreg |= (uint32_t)((uint32_t)HSI_calValue << RCC_CR_HSICAL_Pos);	
    RCC->CR = tmpreg;
	}
	
	/* LSI calibration value update */
	LSI_calValue = *(__IO uint16_t *)LSI_CAL_ADDR;
	if(LSI_calValue != 0xFFFF)
	{
    tmpreg = RCC->CSR;
    tmpreg &= (uint32_t)~((uint32_t)RCC_CSR_LSICAL_Msk);
    tmpreg |= (uint32_t)((uint32_t)LSI_calValue << RCC_CSR_LSICAL_Pos) | RCC_CSR_LSION;
    RCC->CSR = tmpreg;	
	}	
	
	/* LDO 1.5v&1.2v calibration value update */
	LDO_1P5_calValue = *(__IO uint16_t *)LDO_1P5_ADDR;
	LDO_1P2_calValue = *(__IO uint16_t *)LDO_1P2_ADDR;
	if((LDO_1P5_calValue != 0xFFFF) && (LDO_1P2_calValue != 0xFFFF))
	{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    tmpreg = PWR->CFGR;
    tmpreg &= (uint32_t)~((uint32_t)(PWR_CFGR_LDO_REG_Msk | PWR_CFGR_MBGP_REG_Msk));
    tmpreg |= (uint32_t)(((uint32_t)LDO_1P5_calValue << PWR_CFGR_LDO_REG_Pos) | ((uint32_t)LDO_1P2_calValue << PWR_CFGR_MBGP_REG_Pos));
    PWR->CFGR = tmpreg;	
	}
}

/**
  * @brief  This function provides delay time (in milliseconds) by SysTick timer of CM0.
  * @param  ms: specifies the delay time (in milliseconds).
  * @note limited parameter : ms must be < ( (SysTick_LOAD_RELOAD_Msk + 1) / (SystemCoreClock/1000UL) ):
  *       for 16M SystemCoreClock, ms must be < 1048;
  *       for 24M SystemCoreClock, ms must be < 699;
  *       for 32M SystemCoreClock, ms must be < 524;
  *       for 60M SystemCoreClock, ms must be < 279;
  * @retval 0: success
  *         1: fail
  */
uint32_t Delay(uint32_t ms)
{
  uint32_t temp;
 
  ms = ms * (SystemCoreClock/1000UL);
 
  if ((ms - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);                                                   /* Reload value impossible */
  }

  SysTick->LOAD  = (uint32_t)(ms - 1UL);                            /* set reload register */
  SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick Timer */
 
  do
  {
    temp = SysTick->CTRL;
  }while( (temp & SysTick_CTRL_ENABLE_Msk) && (!(temp & SysTick_CTRL_COUNTFLAG_Msk)) ); 
 
  SysTick->CTRL  = 0UL;
  SysTick->VAL   = 0UL;
 
  return (0UL);                                                     /* Function successful */
}

/**
  * @brief  Update SystemCoreClock according to Clock Register Values
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
  *         (*) HSI_VALUE is a constant defined in ca32.h file (default value
  *             8 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in ca32.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmul = 0, plldiv = 0, pllvco = 0, pllclk = 0;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;
  
  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x01:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmul = (RCC->PLLR & RCC_PLLR_M) >> RCC_PLLR_M_Pos;
      plldiv = (RCC->PLLR & RCC_PLLR_N) >> RCC_PLLR_N_Pos;
      pllvco = (RCC->PLLR & RCC_PLLR_OD) >> RCC_PLLR_OD_Pos;
      /* HSI oscillator clock selected as PLL clock entry */
      pllclk = (((HSI_VALUE) * pllmul) / plldiv);
      pllclk >>= PLLVcoTable[pllvco];
    
      SystemCoreClock = pllclk;
      break;
    default: /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
  }

  /* Compute HCLK clock frequency ----------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;  
}



/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
