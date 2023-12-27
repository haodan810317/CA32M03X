/**
  ******************************************************************************
  * @file    ca32_dac.c
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Power Controller (DAC) peripheral:
  *           + DAC configuration
  *           + DAC output
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ca32_opa.h"

/** @addtogroup CA32_StdPeriph_Driver
  * @{
  */

/** @defgroup OPA
  * @brief OPA driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup OPA_Private_Functions
  * @{
  */

/** @defgroup OPA_Group1 Backup Domain Access function 
 *  @brief   Backup Domain Access function
 *
@verbatim
  ==============================================================================
                   ##### OPA init and deinit #####
  ==============================================================================


@endverbatim
  * @{
  */


/**
  * @}
  */


void OPA0_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    ANALOG->OPA0_CFGR1 |= ANALOG_OPAx_CFGR1_OPA_EN;
  }
  else
  {
    ANALOG->OPA0_CFGR1 &= ~ANALOG_OPAx_CFGR1_OPA_EN;
  } 
}

void OPA1_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    ANALOG->OPA1_CFGR1 |= ANALOG_OPAx_CFGR1_OPA_EN;
  }
  else
  {
    ANALOG->OPA1_CFGR1 &= ~ANALOG_OPAx_CFGR1_OPA_EN;
  } 
}


void OPA0_OutputCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	ANALOG->OPA_BUF &= ~(ANALOG_OPA_BUF_SEL | ANALOG_OPA_BUF_EN);
  
  if (NewState != DISABLE)
  {
    ANALOG->OPA_BUF |= ANALOG_OPA_BUF_EN | (1<<ANALOG_OPA_BUF_SEL_Pos);
  }
	
	return;
}

void OPA1_OutputCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	ANALOG->OPA_BUF &= ~(ANALOG_OPA_BUF_SEL | ANALOG_OPA_BUF_EN);
  
  if (NewState != DISABLE)
  {
    ANALOG->OPA_BUF |= ANALOG_OPA_BUF_EN | (2<<ANALOG_OPA_BUF_SEL_Pos);
  }
	
	return;
}


void OPA0_Config(OPA_InitTypeDef * OPA_Init)
{
	ANALOG->OPA0_CFGR2 &= ~(ANALOG_OPAx_CFGR2_CMVSEL | ANALOG_OPAx_CFGR2_RESEL | ANALOG_OPAx_CFGR2_ISEL | ANALOG_OPAx_CFGR2_SINGLE_OUT);
	ANALOG->OPA0_CFGR2 |= OPA_Init->OPA_CMVSEL | OPA_Init->OPA_RESEL | OPA_Init->OPA_ISEL | OPA_Init->OPA_SINGLEOUT;
	return;
}

void OPA1_Config(OPA_InitTypeDef * OPA_Init)
{
	ANALOG->OPA1_CFGR2 &= ~(ANALOG_OPAx_CFGR2_CMVSEL | ANALOG_OPAx_CFGR2_RESEL | ANALOG_OPAx_CFGR2_ISEL | ANALOG_OPAx_CFGR2_SINGLE_OUT);
	ANALOG->OPA1_CFGR2 |= OPA_Init->OPA_CMVSEL | OPA_Init->OPA_RESEL | OPA_Init->OPA_ISEL | OPA_Init->OPA_SINGLEOUT;
	return;
}


void OPA0_ShortOut(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    ANALOG->OPA0_CFGR1 |= ANALOG_OPAx_CFGR1_SHORT_OUT;
  }
  else
  {
    ANALOG->OPA0_CFGR1 &= ~ANALOG_OPAx_CFGR1_SHORT_OUT;
  } 
}

void OPA0_ShortIn(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    ANALOG->OPA0_CFGR1 |= ANALOG_OPAx_CFGR1_SHORT_IN;
  }
  else
  {
    ANALOG->OPA0_CFGR1 &= ~ANALOG_OPAx_CFGR1_SHORT_IN;
  } 
}


void OPA1_ShortOut(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    ANALOG->OPA1_CFGR1 |= ANALOG_OPAx_CFGR1_SHORT_OUT;
  }
  else
  {
    ANALOG->OPA1_CFGR1 &= ~ANALOG_OPAx_CFGR1_SHORT_OUT;
  } 
}

void OPA1_ShortIn(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    ANALOG->OPA1_CFGR1 |= ANALOG_OPAx_CFGR1_SHORT_IN;
  }
  else
  {
    ANALOG->OPA1_CFGR1 &= ~ANALOG_OPAx_CFGR1_SHORT_IN;
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

/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
