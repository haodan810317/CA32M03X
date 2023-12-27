/**
  ******************************************************************************
  * @file    ca32_opa.h
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   This file contains all the functions prototypes for the CRC firmware 
  *          library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CA32_OPA_H
#define __CA32_OPA_H

#ifdef __cplusplus
 extern "C" {
#endif

/*!< Includes ----------------------------------------------------------------*/
#include "ca32.h"

/** @addtogroup STM32F0xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup OPA
  * @{
  */

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint32_t OPA_ISEL;
  uint32_t OPA_CMVSEL;
  uint32_t OPA_SINGLEOUT;
  uint32_t OPA_RESEL;
}OPA_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define OPA_ISEL_4UA          0
#define OPA_ISEL_5UA          (1<<ANALOG_OPAx_CFGR2_ISEL_Pos)
#define OPA_ISEL_6UA          (2<<ANALOG_OPAx_CFGR2_ISEL_Pos)
#define OPA_ISEL_7UA          (3<<ANALOG_OPAx_CFGR2_ISEL_Pos)

#define OPA_CMVSEL_1V7        0
#define OPA_CMVSEL_1V8        (1<<ANALOG_OPAx_CFGR2_CMVSEL_Pos)
#define OPA_CMVSEL_1V9        (2<<ANALOG_OPAx_CFGR2_CMVSEL_Pos)
#define OPA_CMVSEL_2V0        (3<<ANALOG_OPAx_CFGR2_CMVSEL_Pos)

#define OPA_RESEL_200K_10K    0
#define OPA_RESEL_190K_20K    (1<<ANALOG_OPAx_CFGR2_RESEL_Pos)
#define OPA_RESEL_180K_30K    (2<<ANALOG_OPAx_CFGR2_RESEL_Pos)
#define OPA_RESEL_170K_40K    (3<<ANALOG_OPAx_CFGR2_RESEL_Pos)

#define OPA_SINGLEOUT_DIS     0
#define OPA_SINGLEOUT_EN      (1<<ANALOG_OPAx_CFGR2_SINGLE_OUT_Pos)

/* Exported functions ------------------------------------------------------- */
void OPA0_Cmd(FunctionalState NewState);
void OPA0_Config(OPA_InitTypeDef * OPA_Init);
void OPA0_OutputCmd(FunctionalState NewState);
void OPA0_ShortOut(FunctionalState NewState);
void OPA0_ShortIn(FunctionalState NewState);

void OPA1_Cmd(FunctionalState NewState);
void OPA1_Config(OPA_InitTypeDef * OPA_Init);
void OPA1_OutputCmd(FunctionalState NewState);
void OPA1_ShortOut(FunctionalState NewState);
void OPA1_ShortIn(FunctionalState NewState);

#ifdef __cplusplus
}
#endif

#endif /* __CA32_OPA_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

