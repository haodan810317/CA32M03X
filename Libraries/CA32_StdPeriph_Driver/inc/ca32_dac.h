/**
  ******************************************************************************
  * @file    ca32_dac.h
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
#ifndef __CA32_DAC_H
#define __CA32_DAC_H

#ifdef __cplusplus
 extern "C" {
#endif

/*!< Includes ----------------------------------------------------------------*/
#include "ca32.h"

/** @addtogroup STM32F0xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup CRC
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define VREFSelection_AVDD    0
#define VREFSelection_1V2     (4<<ANALOG_DAC_CFGR_REF_VSEL_Pos)
#define VREFSelection_2V4     (5<<ANALOG_DAC_CFGR_REF_VSEL_Pos)
#define VREFSelection_3V6     (6<<ANALOG_DAC_CFGR_REF_VSEL_Pos)
#define VREFSelection_4V8     (7<<ANALOG_DAC_CFGR_REF_VSEL_Pos)

#define ISelection_4UA        0
#define ISelection_5UA        (1<<ANALOG_DAC_CFGR_BUFF_ISEL_Pos)
#define ISelection_6UA        (2<<ANALOG_DAC_CFGR_BUFF_ISEL_Pos)
#define ISelection_7UA        (3<<ANALOG_DAC_CFGR_BUFF_ISEL_Pos)


/* Exported functions ------------------------------------------------------- */
void DAC_Cmd(FunctionalState NewState);
void DAC_SetData(uint16_t dac_data);
void DAC_SetVREFSelection(uint8_t VREFSelection);
void DAC_SetISelection(uint8_t ISelection);
void DAC_BuffCmd(FunctionalState NewState);


#ifdef __cplusplus
}
#endif

#endif /* __CA32_DAC_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
