/**
  ******************************************************************************
  * @file    ca32m030_eval.h
  * @brief   This file contains definitions for CA32M030_EVAL's Leds, push-buttons
  *          and COM ports hardware resources.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CA32M030_EVAL_H
#define __CA32M030_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ca32.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup CA32_EVAL
  * @{
  */

/** @addtogroup CA32M030_EVAL
  * @{
  */
      
/* Exported types ------------------------------------------------------------*/
typedef enum 
{
  LED1 = 0,
  LED2 = 1
} Led_TypeDef;

typedef enum 
{
  BUTTON_KEY1 = 0,
	BUTTON_KEY2 = 1,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum 
{ 
  JOY_NONE = 0,
  JOY_SEL = 1,
} JOYState_TypeDef;

typedef enum 
{
  COM1 = 0,
	COM2 = 1,
} COM_TypeDef;   

/* Exported constants --------------------------------------------------------*/
/** @defgroup CA32M030_EVAL_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** 
  * @brief  Define for CA32M030_EVAL board  
  */ 
#if !defined (USE_CA32M030_EVAL)
 #define USE_CA32M030_EVAL
#endif





/** @addtogroup CA32M030_EVAL_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             2

#define LED1_PIN                         GPIO_Pin_2
#define LED1_GPIO_PORT                   GPIOB
#define LED1_GPIO_CLK                    RCC_AHBPeriph_GPIOB
  
#define LED2_PIN                         GPIO_Pin_1
#define LED2_GPIO_PORT                   GPIOA
#define LED2_GPIO_CLK                    RCC_AHBPeriph_GPIOA

/**
  * @}
  */ 
	
	
	

/** @addtogroup CA32M030_EVAL_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                           2

#define KEY1_BUTTON_PIN                   GPIO_Pin_4
#define KEY1_BUTTON_GPIO_PORT             GPIOA
#define KEY1_BUTTON_GPIO_CLK              RCC_AHBPeriph_GPIOA
#define KEY1_BUTTON_EXTI_LINE             EXTI_Line4
#define KEY1_BUTTON_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOA
#define KEY1_BUTTON_EXTI_PIN_SOURCE       EXTI_PinSource4
#define KEY1_BUTTON_EXTI_IRQn             EXTI0_7_IRQn

#define KEY2_BUTTON_PIN                   GPIO_Pin_5
#define KEY2_BUTTON_GPIO_PORT             GPIOA
#define KEY2_BUTTON_GPIO_CLK              RCC_AHBPeriph_GPIOA
#define KEY2_BUTTON_EXTI_LINE             EXTI_Line5
#define KEY2_BUTTON_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOA
#define KEY2_BUTTON_EXTI_PIN_SOURCE       EXTI_PinSource5
#define KEY2_BUTTON_EXTI_IRQn             EXTI0_7_IRQn 

/**
  * @}
  */ 


/** @addtogroup CA32M030_EVAL_COM_PORT
  * @{
  */  

#define COMn                             2

#define EVAL_COM1                        USART1
#define EVAL_COM1_CLK                    RCC_APB2Periph_USART1

#define EVAL_COM1_TX_PIN                 GPIO_Pin_0
#define EVAL_COM1_TX_GPIO_PORT           GPIOC
#define EVAL_COM1_TX_GPIO_CLK            RCC_AHBPeriph_GPIOC
#define EVAL_COM1_TX_SOURCE              GPIO_PinSource0
#define EVAL_COM1_TX_AF                  GPIO_AF_3

#define EVAL_COM1_RX_PIN                 GPIO_Pin_1
#define EVAL_COM1_RX_GPIO_PORT           GPIOC
#define EVAL_COM1_RX_GPIO_CLK            RCC_AHBPeriph_GPIOC
#define EVAL_COM1_RX_SOURCE              GPIO_PinSource1
#define EVAL_COM1_RX_AF                  GPIO_AF_3

#define EVAL_COM1_CTS_PIN                GPIO_Pin_6
#define EVAL_COM1_CTS_GPIO_PORT          GPIOA
#define EVAL_COM1_CTS_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define EVAL_COM1_CTS_SOURCE             GPIO_PinSource6
#define EVAL_COM1_CTS_AF                 GPIO_AF_3

#define EVAL_COM1_RTS_PIN                GPIO_Pin_5
#define EVAL_COM1_RTS_GPIO_PORT          GPIOA
#define EVAL_COM1_RTS_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define EVAL_COM1_RTS_SOURCE             GPIO_PinSource5
#define EVAL_COM1_RTS_AF                 GPIO_AF_3
   
#define EVAL_COM1_IRQn                   USART1_IRQn



#define EVAL_COM2                        UART2
#define EVAL_COM2_CLK                    RCC_APB1Periph_UART2

#define EVAL_COM2_TX_PIN                 GPIO_Pin_1
#define EVAL_COM2_TX_GPIO_PORT           GPIOB
#define EVAL_COM2_TX_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define EVAL_COM2_TX_SOURCE              GPIO_PinSource1
#define EVAL_COM2_TX_AF                  GPIO_AF_3

#define EVAL_COM2_RX_PIN                 GPIO_Pin_2
#define EVAL_COM2_RX_GPIO_PORT           GPIOB
#define EVAL_COM2_RX_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define EVAL_COM2_RX_SOURCE              GPIO_PinSource2
#define EVAL_COM2_RX_AF                  GPIO_AF_3

#define EVAL_COM2_CTS_PIN                GPIO_Pin_15
#define EVAL_COM2_CTS_GPIO_PORT          GPIOC
#define EVAL_COM2_CTS_GPIO_CLK           RCC_AHBPeriph_GPIOC
#define EVAL_COM2_CTS_SOURCE             GPIO_PinSource15
#define EVAL_COM2_CTS_AF                 GPIO_AF_3

#define EVAL_COM2_RTS_PIN                GPIO_Pin_15
#define EVAL_COM2_RTS_GPIO_PORT          GPIOC
#define EVAL_COM2_RTS_GPIO_CLK           RCC_AHBPeriph_GPIOC
#define EVAL_COM2_RTS_SOURCE             GPIO_PinSource15
#define EVAL_COM2_RTS_AF                 GPIO_AF_3
   
#define EVAL_COM2_IRQn                   UART2_IRQn


/**
  * @}
  */ 
	
	

/** @addtogroup CA32M030_EVAL_LOW_LEVEL_SD_SPI
  * @{
  */
/**
  * @brief  SD SPI Interface pins
  */
#define SD_SPI                           SPI1
#define SD_SPI_CLK                       RCC_APB2Periph_SPI1

#define SD_SPI_SCK_PIN                   GPIO_Pin_11
#define SD_SPI_SCK_GPIO_PORT             GPIOA
#define SD_SPI_SCK_GPIO_CLK              RCC_AHBPeriph_GPIOA
#define SD_SPI_SCK_SOURCE                GPIO_PinSource11
#define SD_SPI_SCK_AF                    GPIO_AF_2

#define SD_SPI_MISO_PIN                  GPIO_Pin_8
#define SD_SPI_MISO_GPIO_PORT            GPIOA
#define SD_SPI_MISO_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define SD_SPI_MISO_SOURCE               GPIO_PinSource8
#define SD_SPI_MISO_AF                   GPIO_AF_2

#define SD_SPI_MOSI_PIN                  GPIO_Pin_9
#define SD_SPI_MOSI_GPIO_PORT            GPIOA
#define SD_SPI_MOSI_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define SD_SPI_MOSI_SOURCE               GPIO_PinSource9
#define SD_SPI_MOSI_AF                   GPIO_AF_2

#define SD_CS_PIN                        GPIO_Pin_10
#define SD_CS_GPIO_PORT                  GPIOA
#define SD_CS_GPIO_CLK                   RCC_AHBPeriph_GPIOA

#define SD_DETECT_PIN                    GPIO_Pin_5
#define SD_DETECT_EXTI_LINE              EXTI_Line5
#define SD_DETECT_EXTI_PIN_SOURCE        EXTI_PinSource5
#define SD_DETECT_GPIO_PORT              GPIOA
#define SD_DETECT_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define SD_DETECT_EXTI_PORT_SOURCE       EXTI_PortSourceGPIOA
#define SD_DETECT_EXTI_IRQn              EXTI0_7_IRQn

/**
  * @}
  */
  
  
/** @addtogroup CA32M030_EVAL_LOW_LEVEL_TSENSOR_I2C
  * @{
  */
/**
  * @brief  LM75 Temperature Sensor I2C Interface pins
  */
#define LM75_I2C                         I2C1
#define LM75_I2C_CLK                     RCC_APB1Periph_I2C1

#define LM75_I2C_SCL_PIN                 GPIO_Pin_3                  /* PB.3 */
#define LM75_I2C_SCL_GPIO_PORT           GPIOB                       /* GPIOB */
#define LM75_I2C_SCL_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define LM75_I2C_SCL_SOURCE              GPIO_PinSource3
#define LM75_I2C_SCL_AF                  GPIO_AF_6

#define LM75_I2C_SDA_PIN                 GPIO_Pin_4                  /* PB.4 */
#define LM75_I2C_SDA_GPIO_PORT           GPIOB                       /* GPIOB */
#define LM75_I2C_SDA_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define LM75_I2C_SDA_SOURCE              GPIO_PinSource4
#define LM75_I2C_SDA_AF                  GPIO_AF_6

#define LM75_I2C_SMBUSALERT_PIN          GPIO_Pin_6                  /* PB.6 */
#define LM75_I2C_SMBUSALERT_GPIO_PORT    GPIOB                       /* GPIOB */
#define LM75_I2C_SMBUSALERT_GPIO_CLK     RCC_AHBPeriph_GPIOB
#define LM75_I2C_SMBUSALERT_SOURCE       GPIO_PinSource6
#define LM75_I2C_SMBUSALERT_AF           GPIO_AF_6
/**
  * @}
  */
   
/** @addtogroup CA32M030_EVAL_LOW_LEVEL_I2C_EE
  * @{
  */
/**
  * @brief  I2C EEPROM Interface pins
  */
#define sEE_I2C                          I2C1
#define sEE_I2C_CLK                      RCC_APB1Periph_I2C1
   
#define sEE_I2C_SCL_PIN                  GPIO_Pin_3                  /* PB.03 */
#define sEE_I2C_SCL_GPIO_PORT            GPIOB                       /* GPIOB */
#define sEE_I2C_SCL_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define sEE_I2C_SCL_SOURCE               GPIO_PinSource3
#define sEE_I2C_SCL_AF                   GPIO_AF_6

#define sEE_I2C_SDA_PIN                  GPIO_Pin_4                  /* PB.04 */
#define sEE_I2C_SDA_GPIO_PORT            GPIOB                       /* GPIOB */
#define sEE_I2C_SDA_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define sEE_I2C_SDA_SOURCE               GPIO_PinSource4
#define sEE_I2C_SDA_AF                   GPIO_AF_6
/**
  * @}
  */
	

/**
  * @brief  Smartcard Interface pins
  */
#define SC_USART                         USART1
#define SC_DETECT_EXTI                   EXTI_Line0
#define SC_DETECT_PIN                    EXTI_PortSourceGPIOA
#define SC_DETECT_GPIO                   EXTI_PinSource0

#define SC_DETECT_IRQ                    EXTI0_7_IRQn


#define SC_PIN_OFF_GPIO                  GPIOA
#define SC_PIN_OFF                       GPIO_Pin_0
#define SC_PIN_OFF_GPIO_CLK              RCC_AHBPeriph_GPIOA

#define SC_PIN_CMDVCC_GPIO_CLK           RCC_AHBPeriph_GPIOB
#define SC_PIN_CMDVCC_GPIO               GPIOB
#define SC_PIN_CMDVCC                    GPIO_Pin_0

#define SC_PIN_RESET_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define SC_PIN_RESET_GPIO                GPIOB
#define SC_PIN_RESET                     GPIO_Pin_1

#define SC_PIN_3_5V_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define SC_PIN_3_5V_GPIO                 GPIOB
#define SC_PIN_3_5V                      GPIO_Pin_3

#define SC_USART_GPIO_CLK                RCC_AHBPeriph_GPIOB 

#define SC_USART_CLK                     RCC_APB2Periph_USART1

#define SC_USART_PIN_CK                  GPIO_Pin_5
#define SC_USART_GPIO                    GPIOB
#define SC_USART_PIN_CK_AF               GPIO_PinSource5
#define SC_USART_CK_AF                   GPIO_AF_7

#define SC_USART_PIN_TX                  GPIO_Pin_4
#define SC_USART_PIN_TX_AF               GPIO_PinSource4
#define SC_USART_TX_AF                   GPIO_AF_5


#define SC_USART_IRQn                    USART1_IRQn
/**
  * @}
  */



/** @defgroup CA32M030_EVAL_LOW_LEVEL_Exported_Functions
  * @{
  */
void TC_EVAL_LEDInit(Led_TypeDef Led);
void TC_EVAL_LEDOn(Led_TypeDef Led);
void TC_EVAL_LEDOff(Led_TypeDef Led);
void TC_EVAL_LEDToggle(Led_TypeDef Led);
void TC_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t TC_EVAL_PBGetState(Button_TypeDef Button);
#ifdef EVAL_UART
void TC_EVAL_COMInit(COM_TypeDef COM, UART_InitTypeDef* UART_InitStruct);
#endif
void SD_LowLevel_DeInit(void);
void SD_LowLevel_Init(void); 
uint8_t TC_SPI_WriteRead(uint8_t Data);
void sFLASH_LowLevel_DeInit(void);
void sFLASH_LowLevel_Init(void);
void LM75_LowLevel_DeInit(void);
void LM75_LowLevel_Init(void);
void sEE_LowLevel_DeInit(void);
void sEE_LowLevel_Init(void); 

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __CA32M030_EVAL_H */
/**
  * @}
  */ 

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
