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
#include "ca32_comp.h"

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

void COMP0_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the COMP */
    ANALOG->CMP0_CSR |= ANALOG_CMPx_CSR_EN;
  }
  else
  {
    /* Disable the COMP */
    ANALOG->CMP0_CSR &= ~ANALOG_CMPx_CSR_EN;
  } 
	
	return;
}

void COMP0_Config(COMP_InitTypeDef * COMP_init)
{
	ANALOG->CMP0_CSR &= ~(ANALOG_CMPx_CSR_BLKSEL | \
	                      ANALOG_CMPx_CSR_RIE | \
	                      ANALOG_CMPx_CSR_FIE | \
	                      ANALOG_CMPx_CSR_RIF | \
	                      ANALOG_CMPx_CSR_FIF | \
	                      ANALOG_CMPx_CSR_POL | \
	                      ANALOG_CMPx_CSR_PSEL | \
	                      ANALOG_CMPx_CSR_NSEL | \
	                      ANALOG_CMPx_CSR_DLY | \
	                      ANALOG_CMPx_CSR_HYST);
	
	ANALOG->CMP0_CSR |= COMP_init->BlankWindow_Select |\
	                    COMP_init->FallingInterrupt |\
	                    COMP_init->RisingInterrupt |\
	                    COMP_init->Polarity |\
	                    COMP_init->PSEL |\
	                    COMP_init->NSEL |\
	                    COMP_init->DLY |\
	                    COMP_init->HYST;
											
	return;
}

void COMP0_Filter_Config(uint16_t filter)
{
	ANALOG->CMP_FILT &= ~ANALOG_CMP_FILT0;
	ANALOG->CMP_FILT |= filter<<ANALOG_CMP_FILT0_Pos;
}






void COMP1_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the COMP */
    ANALOG->CMP1_CSR |= ANALOG_CMPx_CSR_EN;
  }
  else
  {
    /* Disable the COMP */
    ANALOG->CMP1_CSR &= ~ANALOG_CMPx_CSR_EN;
  } 
	
	return;
}

void COMP1_Config(COMP_InitTypeDef * COMP_init)
{
	ANALOG->CMP1_CSR &= ~(ANALOG_CMPx_CSR_BLKSEL | \
	                      ANALOG_CMPx_CSR_RIE | \
	                      ANALOG_CMPx_CSR_FIE | \
	                      ANALOG_CMPx_CSR_RIF | \
	                      ANALOG_CMPx_CSR_FIF | \
	                      ANALOG_CMPx_CSR_POL | \
	                      ANALOG_CMPx_CSR_PSEL | \
	                      ANALOG_CMPx_CSR_NSEL | \
	                      ANALOG_CMPx_CSR_DLY | \
	                      ANALOG_CMPx_CSR_HYST);
	
	ANALOG->CMP1_CSR |= COMP_init->BlankWindow_Select |\
	                    COMP_init->FallingInterrupt |\
	                    COMP_init->RisingInterrupt |\
	                    COMP_init->Polarity |\
	                    COMP_init->PSEL |\
	                    COMP_init->NSEL |\
	                    COMP_init->DLY |\
	                    COMP_init->HYST;
											
	return;
}

void COMP1_Filter_Config(uint16_t filter)
{
	ANALOG->CMP_FILT &= ~ANALOG_CMP_FILT1;
	ANALOG->CMP_FILT |= filter<<ANALOG_CMP_FILT1_Pos;
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
