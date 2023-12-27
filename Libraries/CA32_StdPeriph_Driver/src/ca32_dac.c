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
#include "ca32_dac.h"

/** @addtogroup CA32_StdPeriph_Driver
  * @{
  */

/** @defgroup DAC
  * @brief DAC driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup DAC_Private_Functions
  * @{
  */

/** @defgroup DAC_Group1 Backup Domain Access function 
 *  @brief   Backup Domain Access function
 *
@verbatim
  ==============================================================================
                   ##### DAC init and deinit #####
  ==============================================================================

    [..]Deinit PWR module, using PWR_Deinit().

@endverbatim
  * @{
  */


/**
  * @}
  */

/** @defgroup PWR_Group2 BOR configure and enable functions
 *  @brief   BOR configure and enable configuration functions 
 *
*/

void DAC_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the PVD */
    ANALOG->DAC_CFGR |= ANALOG_DAC_CFGR_DACEN;
  }
  else
  {
    /* Disable the PVD */
    ANALOG->DAC_CFGR &= ~ANALOG_DAC_CFGR_DACEN;
  } 
}

void DAC_SetData(uint16_t dac_data)
{
	ANALOG->DAC_DATR = dac_data;
	return;
}


void DAC_SetVREFSelection(uint8_t VREFSelection)
{
	ANALOG->DAC_CFGR &= ~ANALOG_DAC_CFGR_REF_VSEL;
	ANALOG->DAC_CFGR |= VREFSelection;
	
	
	return;
}

void DAC_SetISelection(uint8_t ISelection)
{
	ANALOG->DAC_CFGR &= ~ANALOG_DAC_CFGR_BUFF_ISEL;
	ANALOG->DAC_CFGR |= ISelection;
	
	return;
}

void DAC_BuffCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the PVD */
    ANALOG->DAC_CFGR |= ANALOG_DAC_CFGR_BUFF_EN;
  }
  else
  {
    /* Disable the PVD */
    ANALOG->DAC_CFGR &= ~ANALOG_DAC_CFGR_BUFF_EN;
  } 
	
	return;
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
