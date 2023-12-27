/**
  ******************************************************************************
  * @file    ca32_comp.h
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
#ifndef __CA32_COMP_H
#define __CA32_COMP_H

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
  uint32_t BlankWindow_Select;
	uint32_t FallingInterrupt;
	uint32_t RisingInterrupt;
	uint32_t Polarity;
	uint32_t PSEL;
	uint32_t NSEL;
	uint32_t DLY;
	uint32_t HYST;
}COMP_InitTypeDef;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BlankWindow_Select_None            0
#define BlankWindow_Select_TIM20_CH1       (1<<ANALOG_CMPx_CSR_BLKSEL_Pos)
#define BlankWindow_Select_TIM20_CH2       (2<<ANALOG_CMPx_CSR_BLKSEL_Pos)
#define BlankWindow_Select_TIM20_CH3       (3<<ANALOG_CMPx_CSR_BLKSEL_Pos)
#define BlankWindow_Select_TIM20_CH4       (4<<ANALOG_CMPx_CSR_BLKSEL_Pos)
#define BlankWindow_Select_TIM2_CH1        (5<<ANALOG_CMPx_CSR_BLKSEL_Pos)
#define BlankWindow_Select_TIM2_CH2        (6<<ANALOG_CMPx_CSR_BLKSEL_Pos)
#define BlankWindow_Select_TIM2_CH3        (7<<ANALOG_CMPx_CSR_BLKSEL_Pos)

#define FallingInterrupt_Enable            (1<<ANALOG_CMPx_CSR_FIE_Pos)
#define FallingInterrupt_Disable           0

#define RisingInterrupt_Enable             (1<<ANALOG_CMPx_CSR_RIE_Pos)
#define RisingInterrupt_Disable            0

#define PolarityNormal                     0
#define PolarityInvert                     (1<<ANALOG_CMPx_CSR_POL_Pos)

#define CMP0_PSEL_PB5                      0
#define CMP0_PSEL_PB10                     (1<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP0_PSEL_PB9                      (2<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP0_PSEL_PB8                      (3<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP0_PSEL_OPA0_P_OUTPUT            (4<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP0_PSEL_PC2_OPA0_IP              (5<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP0_PSEL_OPA1_P_OUTPUT            (6<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP0_PSEL_PC7_OPA1_IP              (7<<ANALOG_CMPx_CSR_PSEL_Pos)

#define CMP0_NSEL_PB12                     0
#define CMP0_NSEL_DAC                      (1<<ANALOG_CMPx_CSR_NSEL_Pos)
#define CMP0_NSEL_VREF                     (2<<ANALOG_CMPx_CSR_NSEL_Pos)
#define CMP0_NSEL_AVDD                     (3<<ANALOG_CMPx_CSR_NSEL_Pos)

#define CMP1_PSEL_PC4                      0
#define CMP1_PSEL_PC5                      (1<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP1_PSEL_PC8                      (2<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP1_PSEL_PC9                      (3<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP1_PSEL_OPA0_P_OUTPUT            (4<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP1_PSEL_PC2_OPA0_IP              (5<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP1_PSEL_OPA1_P_OUTPUT            (6<<ANALOG_CMPx_CSR_PSEL_Pos)
#define CMP1_PSEL_PC7_OPA1_IP              (7<<ANALOG_CMPx_CSR_PSEL_Pos)

#define CMP1_NSEL_PC10                     0
#define CMP1_NSEL_DAC                      (1<<ANALOG_CMPx_CSR_NSEL_Pos)
#define CMP1_NSEL_VREF                     (2<<ANALOG_CMPx_CSR_NSEL_Pos)
#define CMP1_NSEL_AVDD                     (3<<ANALOG_CMPx_CSR_NSEL_Pos)

#define DLY_0p7us                           0
#define DLY_0p26us                          (1<<ANALOG_CMPx_CSR_DLY_Pos)
#define DLY_0p16us                          (2<<ANALOG_CMPx_CSR_DLY_Pos)
#define DLY_0p11us                          (3<<ANALOG_CMPx_CSR_DLY_Pos)

#define HYST_None                           0
#define HYST_10mV                           (1<<ANALOG_CMPx_CSR_HYST_Pos)
#define HYST_15mV                           (2<<ANALOG_CMPx_CSR_HYST_Pos)
#define HYST_20mV                           (3<<ANALOG_CMPx_CSR_HYST_Pos)
/* Exported functions ------------------------------------------------------- */

void COMP0_Cmd(FunctionalState NewState);
void COMP0_Config(COMP_InitTypeDef * COMP_init);
void COMP0_Filter_Config(uint16_t filter);
void COMP1_Cmd(FunctionalState NewState);
void COMP1_Config(COMP_InitTypeDef * COMP_init);
void COMP1_Filter_Config(uint16_t filter);


#ifdef __cplusplus
}
#endif

#endif /* __CA32_COMP_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
