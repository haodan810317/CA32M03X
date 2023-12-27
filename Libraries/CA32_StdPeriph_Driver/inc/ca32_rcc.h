/**
  ******************************************************************************
  * @file    ca32_rcc.h
  * @brief   This file contains all the functions prototypes for the RCC 
  *          firmware library.
  ******************************************************************************
  */
  
/**
   \mainpage CA32 StdPeriph Library
   *
   * Introduction
   * ------------
   *
   * CA32标准库是用来配置和操作CA32系列CPU的C语言代码库。
   *
   * Copyright (C) Thingschip. All rights reserved.
   */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CA32_RCC_H
#define __CA32_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ca32.h"


/* Exported types ------------------------------------------------------------*/

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK_Frequency;
  uint32_t ADCCLK_Frequency;
  uint32_t I2C1CLK_Frequency;
	uint32_t USART1CLK_Frequency;
	uint32_t UART2CLK_Frequency;
}RCC_ClocksTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup RCC_Exported_Constants
  * @{
  */

/** @defgroup RCC_PREDIV1_division_factor
  * @{
  */
#define  RCC_PREDIV1_Div1               RCC_CFGR2_PREDIV1_DIV1
#define  RCC_PREDIV1_Div2               RCC_CFGR2_PREDIV1_DIV2
#define  RCC_PREDIV1_Div3               RCC_CFGR2_PREDIV1_DIV3
#define  RCC_PREDIV1_Div4               RCC_CFGR2_PREDIV1_DIV4
#define  RCC_PREDIV1_Div5               RCC_CFGR2_PREDIV1_DIV5
#define  RCC_PREDIV1_Div6               RCC_CFGR2_PREDIV1_DIV6
#define  RCC_PREDIV1_Div7               RCC_CFGR2_PREDIV1_DIV7
#define  RCC_PREDIV1_Div8               RCC_CFGR2_PREDIV1_DIV8
#define  RCC_PREDIV1_Div9               RCC_CFGR2_PREDIV1_DIV9
#define  RCC_PREDIV1_Div10              RCC_CFGR2_PREDIV1_DIV10
#define  RCC_PREDIV1_Div11              RCC_CFGR2_PREDIV1_DIV11
#define  RCC_PREDIV1_Div12              RCC_CFGR2_PREDIV1_DIV12
#define  RCC_PREDIV1_Div13              RCC_CFGR2_PREDIV1_DIV13
#define  RCC_PREDIV1_Div14              RCC_CFGR2_PREDIV1_DIV14
#define  RCC_PREDIV1_Div15              RCC_CFGR2_PREDIV1_DIV15
#define  RCC_PREDIV1_Div16              RCC_CFGR2_PREDIV1_DIV16

#define IS_RCC_PREDIV1(PREDIV1) (((PREDIV1) == RCC_PREDIV1_Div1) || ((PREDIV1) == RCC_PREDIV1_Div2) || \
                                 ((PREDIV1) == RCC_PREDIV1_Div3) || ((PREDIV1) == RCC_PREDIV1_Div4) || \
                                 ((PREDIV1) == RCC_PREDIV1_Div5) || ((PREDIV1) == RCC_PREDIV1_Div6) || \
                                 ((PREDIV1) == RCC_PREDIV1_Div7) || ((PREDIV1) == RCC_PREDIV1_Div8) || \
                                 ((PREDIV1) == RCC_PREDIV1_Div9) || ((PREDIV1) == RCC_PREDIV1_Div10) || \
                                 ((PREDIV1) == RCC_PREDIV1_Div11) || ((PREDIV1) == RCC_PREDIV1_Div12) || \
                                 ((PREDIV1) == RCC_PREDIV1_Div13) || ((PREDIV1) == RCC_PREDIV1_Div14) || \
                                 ((PREDIV1) == RCC_PREDIV1_Div15) || ((PREDIV1) == RCC_PREDIV1_Div16))

/** @defgroup RCC_AHB_Clock_Source
  * @{
  */

#define RCC_SYSCLK_Div1                  RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_Div2                  RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_Div4                  RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_Div8                  RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_Div16                 RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_Div64                 RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_Div128                RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_Div256                RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_Div512                RCC_CFGR_HPRE_DIV512
#define IS_RCC_HCLK(HCLK) (((HCLK) == RCC_SYSCLK_Div1) || ((HCLK) == RCC_SYSCLK_Div2) || \
                           ((HCLK) == RCC_SYSCLK_Div4) || ((HCLK) == RCC_SYSCLK_Div8) || \
                           ((HCLK) == RCC_SYSCLK_Div16) || ((HCLK) == RCC_SYSCLK_Div64) || \
                           ((HCLK) == RCC_SYSCLK_Div128) || ((HCLK) == RCC_SYSCLK_Div256) || \
                           ((HCLK) == RCC_SYSCLK_Div512))
/**
  * @}
  */ 

/** @defgroup RCC_APB_Clock_Source
  * @{
  */

#define RCC_HCLK_Div1                    RCC_CFGR_PPRE_DIV1
#define RCC_HCLK_Div2                    RCC_CFGR_PPRE_DIV2
#define RCC_HCLK_Div4                    RCC_CFGR_PPRE_DIV4
#define RCC_HCLK_Div8                    RCC_CFGR_PPRE_DIV8
#define RCC_HCLK_Div16                   RCC_CFGR_PPRE_DIV16
#define IS_RCC_PCLK(PCLK) (((PCLK) == RCC_HCLK_Div1) || ((PCLK) == RCC_HCLK_Div2) || \
                           ((PCLK) == RCC_HCLK_Div4) || ((PCLK) == RCC_HCLK_Div8) || \
                           ((PCLK) == RCC_HCLK_Div16))
/**
  * @}
  */
  
/** @defgroup RCC_ADC_clock_source 
  * @{
  */

#define RCC_ADCCLK_Disable               0
#define RCC_ADCCLK_HSI_Div2              ((uint32_t)(1<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div3              ((uint32_t)(2<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div4              ((uint32_t)(3<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div5              ((uint32_t)(4<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div6              ((uint32_t)(5<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div7              ((uint32_t)(6<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div8              ((uint32_t)(7<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div9              ((uint32_t)(8<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div10             ((uint32_t)(9<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div11             ((uint32_t)(10<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div12             ((uint32_t)(11<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div13             ((uint32_t)(12<<RCC_CFGR_ADCPRE_Pos))
#define RCC_ADCCLK_HSI_Div14             ((uint32_t)(13<<RCC_CFGR_ADCPRE_Pos))


#define IS_RCC_ADCCLK(ADCCLK) (((ADCCLK) == RCC_ADCCLK_HSI_Div2) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div3) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div4) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div5) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div6) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div7) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div8) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div9) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div10) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div11) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div12) || \
															 ((ADCCLK) == RCC_ADCCLK_HSI_Div13) || \
                               ((ADCCLK) == RCC_ADCCLK_HSI_Div14))

#define IS_RCC_ADCKCLK(ADCKCLK) (((ADCKCLK) == RCC_CFGR_ADCKPRE_DIV2) || \
                                ((ADCKCLK) == RCC_CFGR_ADCKPRE_DIV4) || \
																((ADCKCLK) == RCC_CFGR_ADCKPRE_DIV8) || \
																((ADCKCLK) == RCC_CFGR_ADCKPRE_NONE))

/**
  * @}
  */

/** @defgroup RCC_UART_clock_source 
  * @{
  */

#define RCC_UART1CLK_PCLK                  ((uint32_t)0x10000000)
#define RCC_UART1CLK_SYSCLK                ((uint32_t)0x10000001)
#define RCC_UART1CLK_LSE                   ((uint32_t)0x10000002)
#define RCC_UART1CLK_HSI                   ((uint32_t)0x10000003)


#define IS_RCC_UARTCLK(UARTCLK) (((UARTCLK) == RCC_UART1CLK_PCLK)   || \
                                   ((UARTCLK) == RCC_UART1CLK_SYSCLK) || \
                                   ((UARTCLK) == RCC_UART1CLK_LSE)    || \
                                   ((UARTCLK) == RCC_UART1CLK_HSI))

/**
  * @}
  */
         
/** @defgroup RCC_Interrupt_Source 
  * @{
  */

#define RCC_IT_LSIRDY                    ((uint8_t)0x01)
#define RCC_IT_LSERDY                    ((uint8_t)0x02)
#define RCC_IT_HSIRDY                    ((uint8_t)0x04)
#define RCC_IT_HSERDY                    ((uint8_t)0x08)
#define RCC_IT_PLLRDY                    ((uint8_t)0x10)
#define RCC_IT_HSI14RDY                  ((uint8_t)0x20)
#define RCC_IT_CSS                       ((uint8_t)0x80)

#define IS_RCC_IT(IT) ((((IT) & (uint8_t)0x80) == 0x00) && ((IT) != 0x00))

#define IS_RCC_GET_IT(IT) (((IT) == RCC_IT_LSIRDY) || ((IT) == RCC_IT_LSERDY) || \
                           ((IT) == RCC_IT_HSIRDY) || ((IT) == RCC_IT_HSERDY) || \
                           ((IT) == RCC_IT_PLLRDY) || ((IT) == RCC_IT_HSI14RDY) || \
                           ((IT) == RCC_IT_CSS)    || ((IT) == RCC_IT_HSI48RDY))

#define IS_RCC_CLEAR_IT(IT) ((IT) != 0x00)

/**
  * @}
  */
  
/** @defgroup RCC_LSE_Configuration 
  * @{
  */

#define RCC_LSE_OFF                      ((uint32_t)0x00000000)
#define RCC_LSE_ON                       RCC_BDCR_LSEON
#define RCC_LSE_Bypass                   ((uint32_t)(RCC_BDCR_LSEON | RCC_BDCR_LSEBYP))
#define IS_RCC_LSE(LSE) (((LSE) == RCC_LSE_OFF) || ((LSE) == RCC_LSE_ON) || \
                         ((LSE) == RCC_LSE_Bypass))
/**
  * @}
  */

/** @defgroup RCC_RTC_Clock_Source
  * @{
  */

#define RCC_RTCCLKSource_LSE             RCC_BDCR_RTCSEL_LSE
#define RCC_RTCCLKSource_LSI             RCC_BDCR_RTCSEL_LSI
#define RCC_RTCCLKSource_HSE_Div32       RCC_BDCR_RTCSEL_HSE

#define IS_RCC_RTCCLK_SOURCE(SOURCE) (((SOURCE) == RCC_RTCCLKSource_LSE) || \
                                      ((SOURCE) == RCC_RTCCLKSource_LSI) || \
                                      ((SOURCE) == RCC_RTCCLKSource_HSE_Div32))
/**
  * @}
  */

/** @defgroup RCC_LSE_Drive_Configuration 
  * @{
  */

#define RCC_LSEDrive_Low                 ((uint32_t)0x00000000)
#define RCC_LSEDrive_MediumLow           RCC_BDCR_LSEDRV_0
#define RCC_LSEDrive_MediumHigh          RCC_BDCR_LSEDRV_1
#define RCC_LSEDrive_High                RCC_BDCR_LSEDRV
#define IS_RCC_LSE_DRIVE(DRIVE) (((DRIVE) == RCC_LSEDrive_Low) || ((DRIVE) == RCC_LSEDrive_MediumLow) || \
                                 ((DRIVE) == RCC_LSEDrive_MediumHigh) || ((DRIVE) == RCC_LSEDrive_High))
/**
  * @}
  */
  
/** @defgroup RCC_AHB_Peripherals 
  * @{
  */
#define RCC_AHBPeriph_GPIOA               RCC_AHBENR_GPIOAEN
#define RCC_AHBPeriph_GPIOB               RCC_AHBENR_GPIOBEN
#define RCC_AHBPeriph_GPIOC               RCC_AHBENR_GPIOCEN


#define RCC_AHBPeriph_FLITF               RCC_AHBENR_FLITFEN
#define RCC_AHBPeriph_DMA1                RCC_AHBENR_DMA1EN

#define RCC_AHBPeriph_CRC                 RCC_AHBENR_CRCEN

#define RCC_AHBPeriph_CORDIC                 RCC_AHBENR_CORDICEN
#define RCC_AHBPeriph_MATH                 RCC_AHBENR_MATHEN
#define RCC_AHBPeriph_SVPWM                 RCC_AHBENR_SVPWMEN

#define IS_RCC_AHB_PERIPH(PERIPH) ((((PERIPH) & 0xFFF1F4AE) == 0x00) && ((PERIPH) != 0x00))
#define IS_RCC_AHB_RST_PERIPH(PERIPH) ((((PERIPH) & 0xFFF1F4BF) == 0x00) && ((PERIPH) != 0x00))



/**
  * @}
  */

/** @defgroup RCC_APB2_Peripherals 
  * @{
  */

#define RCC_APB2Periph_SYSCFG            RCC_APB2ENR_SYSCFGEN
#define RCC_APB2Periph_ADC1              RCC_APB2ENR_ADC1EN
#define RCC_APB2Periph_TIM20              RCC_APB2ENR_TIM20EN
#define RCC_APB2Periph_SPI1              RCC_APB2ENR_SPI1EN
#define RCC_APB2Periph_USART1            RCC_APB2ENR_USART1EN
#define RCC_APB2Periph_ANALOG            RCC_APB2ENR_ANALOGEN

#define IS_RCC_APB2_PERIPH(PERIPH) ((((PERIPH) & 0xFFFFA5FC) == 0x00) && ((PERIPH) != 0x00))

/**
  * @}
  */ 

/** @defgroup RCC_APB1_Peripherals 
  * @{
  */


#define RCC_APB1Periph_TIM2              RCC_APB1ENR_TIM2EN
#define RCC_APB1Periph_TIM3              RCC_APB1ENR_TIM3EN
#define RCC_APB1Periph_TIM6              RCC_APB1ENR_TIM6EN
#define RCC_APB1Periph_TIM7              RCC_APB1ENR_TIM7EN
#define RCC_APB1Periph_UART2             RCC_APB1ENR_UART2EN
#define RCC_APB1Periph_WWDG              RCC_APB1ENR_WWDGEN
#define RCC_APB1Periph_I2C1              RCC_APB1ENR_I2C1EN
#define RCC_APB1Periph_I2C2              RCC_APB1ENR_I2C2EN
#define RCC_APB1Periph_PWR               RCC_APB1ENR_PWREN

#define IS_RCC_APB1_PERIPH(PERIPH) ((((PERIPH) & 0xEF9DF7CC) == 0x00) && ((PERIPH) != 0x00))
/**
  * @}
  */

/** @defgroup RCC_MCO_Div
  * @{
  */


#define RCC_MCODiv_None										RCC_CFGR_MCOPRE_NONE
#define RCC_MCODiv_LSI										RCC_CFGR_MCOPRE_LSI
#define RCC_MCODiv_4										  RCC_CFGR_MCOPRE_DIV4
#define RCC_MCODiv_8										  RCC_CFGR_MCOPRE_DIV8
#define RCC_MCODiv_16										  RCC_CFGR_MCOPRE_DIV16
#define RCC_MCODiv_32										  RCC_CFGR_MCOPRE_DIV32
#define RCC_MCODiv_64										  RCC_CFGR_MCOPRE_DIV64
#define RCC_MCODiv_128									  RCC_CFGR_MCOPRE_DIV128


#define RCC_FLAG_HSIRDY                  ((uint8_t)0x01)
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x41)
#define RCC_FLAG_OBLRST                  ((uint8_t)0x59)
#define RCC_FLAG_PINRST                  ((uint8_t)0x5A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x5B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x5C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x5D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x5E)


#define IS_RCC_FLAG(FLAG) (((FLAG) == RCC_FLAG_HSIRDY)  || ((FLAG) == RCC_FLAG_OBLRST)  || \
                           ((FLAG) == RCC_FLAG_PINRST)  || ((FLAG) == RCC_FLAG_PORRST)  || \
                           ((FLAG) == RCC_FLAG_SFTRST)  || ((FLAG) == RCC_FLAG_LSIRDY)  || \
                           ((FLAG) == RCC_FLAG_IWDGRST) || (FLAG) == RCC_FLAG_WWDGRST))

#define IS_RCC_HSI_CALIBRATION_VALUE(VALUE) ((VALUE) <= 0x1F)

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void RCC_DeInit(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLKConfig(uint32_t RCC_HCLK);
void RCC_ADCCLKConfig(uint32_t RCC_ADCCLK);
//void RCC_ADCKClockCmd(FunctionalState NewState);
void RCC_ADCKClockConfig(uint32_t RCC_ADCKCLK);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
void RCC_LSICmd(FunctionalState NewState);
void RCC_MCOConfig(uint32_t RCC_MCODiv);
#ifdef __cplusplus
}
#endif

#endif /* __CA32_RCC_H */


/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
