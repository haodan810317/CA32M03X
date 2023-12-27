/**
  ******************************************************************************
  * @file    ca32_pwr.c
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Power Controller (PWR) peripheral:
  *           + PVD configuration
  *           + WakeUp pins configuration
  *           + Low Power modes configuration
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ca32_pwr.h"
#include "ca32_rcc.h"

/** @addtogroup CA32_StdPeriph_Driver
  * @{
  */

/** @defgroup PWR 
  * @brief PWR driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* ------------------ PWR registers bit mask ------------------------ */

/* CR register bit mask */
#define CR_DS_MASK               ((uint32_t)0xFFFFFFFC)
#define CR_PLS_MASK              ((uint32_t)0xFFFFFF1F)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup PWR_Private_Functions
  * @{
  */

/** @defgroup PWR_Group1 Backup Domain Access function 
 *  @brief   Backup Domain Access function
 *
@verbatim
  ==============================================================================
                   ##### PWR init and deinit #####
  ==============================================================================

    [..]Deinit PWR module, using PWR_Deinit().

@endverbatim
  * @{
  */

/**
  * @brief  Deinitializes the PWR peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void PWR_DeInit(void)
{
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, DISABLE);
}


/**
  * @}
  */

/** @defgroup PWR_Group2 BOR configure and enable functions
 *  @brief   BOR configure and enable configuration functions 
 *
@verbatim
  ==============================================================================
                    ##### PVD configuration functions #####
  ==============================================================================
  [..]
  (+) The PVD is used to monitor the VDD power supply by comparing it to a threshold
      selected by the PVD Level (PLS[2:0] bits in the PWR_CR).
  (+) A PVDO flag is available to indicate if VDD/VDDA is higher or lower than the 
      PVD threshold. This event is internally connected to the EXTI line16
      and can generate an interrupt if enabled through the EXTI registers.
  (+) The PVD is stopped in Standby mode.

@endverbatim
  * @{
  */

void PWR_PVDCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the PVD */
    PWR->CR |= PWR_CR_PVDEN;
  }
  else
  {
    /* Disable the PVD */
    PWR->CR &= (uint32_t)~((uint32_t)PWR_CR_PVDEN);
  } 
}

	
void PWR_PVDConfig(uint32_t PVD_Level)
{
	PWR->CFGR &= ~PWR_CFGR_PVD_REG;
	PWR->CFGR |= PVD_Level;
	
	return;
}

FlagStatus PWR_PVDGetStatus()
{
	if(PWR->CSR & PWR_CSR_PVDO)
		return SET;
	else
		return RESET;
}

/**
  * @}
  */


/** @defgroup PWR_Group3 Low Power modes configuration functions
 *  @brief   Low Power modes configuration functions 
 *
@verbatim
  ==============================================================================
              ##### Low Power modes configuration functions #####
  ==============================================================================

    [..] The devices feature three low-power modes:
    (+) Sleep mode: Cortex-M0 core stopped, peripherals kept running.

  *** Sleep mode *** 
  ==================
  [..] 
    (+) Entry:
        (++) The Sleep mode is entered by executing the WFE() or WFI() instructions.
    (+) Exit:
        (++) Any peripheral interrupt acknowledged by the nested vectored interrupt 
             controller (NVIC) can wake up the device from Sleep mode.

@endverbatim
  * @{
  */

/**
  * @brief  Enters Sleep mode.
  * @note   In Sleep mode, all I/O pins keep the same state as in Run mode.
  * @param  PWR_SLEEPEntry: specifies if SLEEP mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *             @arg PWR_SLEEPEntry_WFI: enter SLEEP mode with WFI instruction
  *             @arg PWR_SLEEPEntry_WFE: enter SLEEP mode with WFE instruction
  * @retval None
  */
void PWR_EnterSleepMode(uint8_t PWR_SLEEPEntry)
{
  /* Check the parameters */
  assert_param(IS_PWR_SLEEP_ENTRY(PWR_SLEEPEntry));

  /* Clear SLEEPDEEP bit of Cortex-M0 System Control Register */
  SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
  
  /* Select SLEEP mode entry -------------------------------------------------*/
  if(PWR_SLEEPEntry == PWR_SLEEPEntry_WFI)
  {
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __SEV();
    __WFE(); 
    __WFE();
  }
}


/**
  * @}
  */




/** @defgroup PWR_Group5 POR configuration functions
 *  @brief   POR configuration functions 
 *
@verbatim
  ==============================================================================
              ##### POR configuration functions #####
  ==============================================================================

@endverbatim
  * @{
  */


/**
  * @brief  Enables or disables the PAD Reset.
  * @param  NewState: new state of the PAD Reset.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void PWR_PADResetCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState == DISABLE)
  {
    /* Disable the PAD */
    PWR->CFGR |= PWR_CFGR_PAD_RST_DIS;
  }
  else
  {
    /* Enable the PAD */
    PWR->CFGR &= (uint32_t)~((uint32_t)PWR_CFGR_PAD_RST_DIS);
  } 
}

void PWR_IWDGResetCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState == DISABLE)
  {
    /* Disable the PAD */
    PWR->CFGR |= PWR_CFGR_IWDG_RST_DIS;
  }
  else
  {
    /* Enable the PAD */
    PWR->CFGR &= (uint32_t)~((uint32_t)PWR_CFGR_IWDG_RST_DIS);
  } 
}

/**
  * @brief  Enables or disables the POR Reset.
  * @param  NewState: new state of the POR Reset.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void PWR_PORResetCmd(FunctionalState NewState)
{
//  /* Check the parameters */
//  assert_param(IS_FUNCTIONAL_STATE(NewState));
//  
//  if (NewState == DISABLE)
//  {
//    /* Disable the PVD */
//    PWR->CR |= PWR_CSR_PORRST_DISABLE;
//  }
//  else
//  {
//    /* Enable the PVD */
//    PWR->CR &= (uint32_t)~((uint32_t)PWR_CSR_PORRST_DISABLE);
//  } 
}


/** @defgroup PWR_Group5 BOR configuration functions
 *  @brief   BOR configuration functions 
 *
@verbatim
  ==============================================================================
              ##### BOR configuration functions #####
  ==============================================================================

@endverbatim
  * @{
  */


/**
  * @brief  Enables or disables the BOR 
  * @param  NewState: new state of the BOR.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void PWR_BORCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the PVD */
    PWR->CR |= PWR_CR_BOREN;
  }
  else
  {
    /* Disable the PVD */
    PWR->CR &= (uint32_t)~((uint32_t)PWR_CR_BOREN);
  } 
}

void PWR_BORResetCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState == DISABLE)
  {
    /* Disable the BOR */
    PWR->CFGR |= PWR_CFGR_BOR_RST_DIS;
  }
  else
  {
    /* Enable the BOR */
    PWR->CFGR &= (uint32_t)~((uint32_t)PWR_CFGR_BOR_RST_DIS);
  } 
}
	
void PWR_BORConfig(uint32_t BOR_Level)
{
	PWR->CFGR &= ~PWR_CFGR_BOR_REG;
	PWR->CFGR |= BOR_Level;
	
	return;
}

FlagStatus PWR_BORGetStatus()
{
	if(PWR->CSR & PWR_CSR_BORO)
		return SET;
	else
		return RESET;
}

/**
  * @}
  */


/** @defgroup PWR_Group6 BGP and LDO output configuration functions
 *  @brief   BGP and LDO output configuration functions 
 *
@verbatim
  ==============================================================================
              ##### BGP and LDO output configuration functions #####
  ==============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  configure BGP level.
  * @param  BGPLevel
  * @retval None
  */
void PWR_BGPLevelConfig(uint8_t BGPLevel)
{
	
	PWR->CFGR &= ~PWR_CFGR_MBGP_REG;
	PWR->CFGR |= BGPLevel<<PWR_CFGR_MBGP_REG_Pos;

	return;
}

void PWR_BGPCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Disable the BOR */
    PWR->CR |= PWR_CR_MBGPEN;
		while(!(PWR->CSR & PWR_CSR_MBGP_RDY));
  }
  else
  {
    /* Enable the BOR */
    PWR->CR &= (uint32_t)~((uint32_t)PWR_CR_MBGPEN);
  } 
}

/**
  * @brief  configure LDO level.
  * @param  BGPLevel
  * @retval None
  */
void PWR_LDOLevelConfig(uint8_t LDOLevel)
{
	
	PWR->CFGR &= ~PWR_CFGR_LDO_REG;
	PWR->CFGR |= LDOLevel<<PWR_CFGR_LDO_REG_Pos;

	return;
}


/**
  * @brief  Enters STOP mode.
  * @note   In Stop mode, all I/O pins keep the same state as in Run mode.
  * @note   When exiting Stop mode by issuing an interrupt or a wakeup event, 
  *         the HSI RC oscillator is selected as system clock.
  * @note   When the voltage regulator operates in low power mode, an additional 
  *         startup delay is incurred when waking up from Stop mode. 
  *         By keeping the internal regulator ON during Stop mode, the consumption 
  *         is higher although the startup time is reduced.
  * @param  PWR_STOPEntry: specifies if STOP mode in entered with WFI or WFE instruction.
  *         This parameter can be one of the following values:
  *             @arg PWR_STOPEntry_WFI: enter STOP mode with WFI instruction
  *             @arg PWR_STOPEntry_WFE: enter STOP mode with WFE instruction
  * @retval None
  */
void PWR_EnterSTOPMode(uint8_t PWR_STOPEntry)
{
  /* Check the parameters */
  assert_param(IS_PWR_STOP_ENTRY(PWR_STOPEntry));
	
	/* Clear Standby and deepstandby mode cfg bit */
	PWR->LPR &= ~(PWR_LPR_STB | PWR_LPR_DPSTB);
  
  /* Set SLEEPDEEP bit of Cortex-M0 System Control Register */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
  /* Select STOP mode entry --------------------------------------------------*/
  if(PWR_STOPEntry == PWR_STOPEntry_WFI)
  {
    /* Request Wait For Interrupt */
    __WFI();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk); 
  }
  else if (PWR_STOPEntry == PWR_STOPEntry_WFE)
  {
    /* Request Wait For Event */
    __WFE();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);   
  }
  else
  {
    /* Set SLEEP on exit bit of Cortex-M0 System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
  }
}


void PWR_EnterStandbyMode(uint8_t PWR_StandbyEntry)
{
  /* Check the parameters */
  assert_param(IS_PWR_STANDBY_ENTRY(PWR_StandbyEntry));
	
	/* Clear Standby and deepstandby mode cfg bit */
	PWR->LPR &= ~(PWR_LPR_STB | PWR_LPR_DPSTB);
	PWR->LPR |= PWR_LPR_STB;
  
  /* Set SLEEPDEEP bit of Cortex-M0 System Control Register */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
  /* Select STOP mode entry --------------------------------------------------*/
  if(PWR_StandbyEntry == PWR_StandbyEntry_WFI)
  {
    /* Request Wait For Interrupt */
    __WFI();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk); 
  }
  else if (PWR_StandbyEntry == PWR_StandbyEntry_WFE)
  {
    /* Request Wait For Event */
    __WFE();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);   
  }
  else
  {
    /* Set SLEEP on exit bit of Cortex-M0 System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
  }
}


/**
  * @}
  */

/**
  * @brief  Checks whether the specified PWR flag is set or not.
  * @param  PWR_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *             @arg PWR_FLAG_PINRST: Pin reset
  *             @arg PWR_FLAG_PORRST: POR/PDR reset
  *             @arg PWR_FLAG_STDBYRST: wakeup from standby Mode
  *             @arg PWR_FLAG_BORRST: BOR reset
  * @retval The new state of RCC_FLAG (SET or RESET).
  */
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_PWR_FLAG(PWR_FLAG));

  if (PWR->CSR & PWR_FLAG)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the flag status */
  return bitstatus;
	
}

/**
  * @brief  Clears the PWR reset flags.
  *         The reset flags are: PWR_FLAG_PINRST,
  *         PWR_FLAG_PORRST, 
  * @param  None
  * @retval None
  */
void PWR_ClearFlag(void)
{
  /* Set RMVF bit to clear the reset flags */
  PWR->CSR |= PWR_CSR_RMVF;
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

/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
