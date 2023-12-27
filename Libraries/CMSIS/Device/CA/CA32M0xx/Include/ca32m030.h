#ifndef __CA32M030_H
#define __CA32M030_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/**
 * @brief Configuration of the Cortex-M0 Processor and Core Peripherals
 */
#define __CM0_REV                 0     /*!< Core Revision r0p0                           */
#define __MPU_PRESENT             0     /*!< CA32M030 do not provide MPU                  */
#define __NVIC_PRIO_BITS          2     /*!< CA32M030 uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0     /*!< Set to 1 if different SysTick Config is used */


 /*!< Interrupt Number Definition */
typedef enum
{
/******  Cortex-M0 Processor Exceptions Numbers **************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                       */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                             	*/
  SVC_IRQn                    = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                              	 	*/
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                             		*/
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                        			*/

/********  CA32 specific Interrupt Numbers ******************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                    	*/
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                                         */
	RCC_IRQn                    = 4,      /*!< RCC Interrupt                                                  */
  EXTI0_7_IRQn                = 5,      /*!< EXTI Line 0 to 7 Interrupt                                     */
  EXTI8_15_IRQn               = 6,      /*!< EXTI Line 8 to 15 Interrupt                                    */
  DMA1_Channel1_IRQn          = 9,      /*!< DMA1 Channel 1 Interrupt                                       */
  DMA1_Channel2_3_IRQn        = 10,     /*!< DMA1 Channel 2 and 3 Interrupt                                 */
  DMA1_Channel4_5_IRQn        = 11,     /*!< DMA1 Channel 4 and 5 Interrupt                                 */
  ADC1_IRQn                   = 12,     /*!< ADC1 Interrupt                                                 */
  TIM20_IRQn               		= 13,     /*!< TIM20 interrupt                                               	*/   
	CORDIC_IRQn                 = 15,     /*!< CORDIC Interrupt                                               */
  TIM3_IRQn                   = 16,     /*!< TIM3 global Interrupt                                          */
  I2C1_OA1_IRQn               = 17,     /*!< I2C1 OA1 Interrupt																							*/
  I2C1_OA2_IRQn               = 18,     /*!< I2C1 OA2 Interrupt																							*/	
  TIM2_IRQn                 	= 19,     /*!< TIM2 global Interrupt                                          */
  COMP0_IRQn                  = 20,     /*!< COMP0 global Interrupt                                         */
  TIM6_IRQn                 	= 21,     /*!< TIM6 global Interrupt                                          */
  TIM7_IRQn                 	= 22,     /*!< TIM7 global Interrupt                                          */
  I2C1_IRQn                   = 23,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)     */
  COMP1_IRQn                  = 24,     /*!< COMP1 global Interrupt                                         */  
  SPI1_IRQn                   = 25,     /*!< SPI1 global Interrupt                                          */
  USART1_IRQn                 = 27,     /*!< USART1 Interrupt                                        				*/
	UART2_IRQn                  = 28,     /*!< UART2 Interrupt                                        				*/
} IRQn_Type;


#include "core_cm0.h"            /* Cortex-M0 processor and core peripherals */
#include "system_ca32m030.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */



/**
  * @brief Analog to Digital Converter
  */

typedef struct
{
  __IO uint32_t ISR;              /*!< ADC Interrupt and Status Register,                 Address offset: 0x00 */
  __IO uint32_t IER;              /*!< ADC Interrupt Enable Register,                     Address offset: 0x04 */      
  __IO uint32_t CR;               /*!< ADC control register,                              Address offset: 0x08 */
  __IO uint32_t CFGR1;            /*!< ADC Configuration register1,                       Address offset: 0x0C */
  __IO uint32_t CFGR2;            /*!< ADC Configuration register2,                       Address offset: 0x10 */
  __IO uint32_t SMPR1;            /*!< ADC sample time register 1,                        Address offset: 0x14 */
  __IO uint32_t SMPR2;            /*!< ADC sample time register 2,                        Address offset: 0x18 */
  uint32_t      RESERVED1;        /*!< Reserved, 0x01C                                                         */
  __IO uint32_t TR1;              /*!< ADC watchdog threshold register 1,                 Address offset: 0x20 */
  __IO uint32_t TR2;              /*!< ADC watchdog threshold register 2,                 Address offset: 0x24 */
  __IO uint32_t TR3;              /*!< ADC watchdog threshold register 3,                 Address offset: 0x28 */
  uint32_t      RESERVED2;        /*!< Reserved, 0x02C                                                         */
  __IO uint32_t SQR1;             /*!< ADC regular sequence register 1,                   Address offset: 0x30 */
  __IO uint32_t SQR2;             /*!< ADC regular sequence register 2,                   Address offset: 0x34 */
  __IO uint32_t SQR3;             /*!< ADC regular sequence register 3,                   Address offset: 0x38 */
  __IO uint32_t SQR4;             /*!< ADC regular sequence register 4,                   Address offset: 0x3C */
  __IO uint32_t DR;               /*!< ADC regular data register,                         Address offset: 0x40 */
  uint32_t      RESERVED3;        /*!< Reserved, 0x044                                                         */
  uint32_t      RESERVED4;        /*!< Reserved, 0x048                                                         */
  __IO uint32_t JSQR;             /*!< ADC injected sequence register,                    Address offset: 0x4C */
  uint32_t      RESERVED5[4];     /*!< Reserved, 0x050 - 0x05C                                                 */
  __IO uint32_t OFR1;             /*!< ADC offset register 1,                             Address offset: 0x60 */
  __IO uint32_t OFR2;             /*!< ADC offset register 2,                             Address offset: 0x64 */
  __IO uint32_t OFR3;             /*!< ADC offset register 3,                             Address offset: 0x68 */
  __IO uint32_t OFR4;             /*!< ADC offset register 4,                             Address offset: 0x6C */
  uint32_t      RESERVED6[4];     /*!< Reserved, 0x070 - 0x07C                                                 */
  __IO uint32_t JDR1;             /*!< ADC injected data register 1,                      Address offset: 0x80 */
  __IO uint32_t JDR2;             /*!< ADC injected data register 2,                      Address offset: 0x84 */
  __IO uint32_t JDR3;             /*!< ADC injected data register 3,                      Address offset: 0x88 */
  __IO uint32_t JDR4;             /*!< ADC injected data register 4,                      Address offset: 0x8C */
  uint32_t      RESERVED7[4];     /*!< Reserved, 0x090 - 0x09C                                                 */  
  __IO uint32_t AWD2CR;           /*!< ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0 */
  __IO uint32_t AWD3CR;           /*!< ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4 */
  uint32_t      RESERVED8;        /*!< Reserved, 0x0A8                                                         */
  uint32_t      RESERVED9;        /*!< Reserved, 0x0AC                                                         */
  uint32_t      RESERVED10;       /*!< Reserved, 0x0B0                                                         */
  __IO uint32_t CALFACT;          /*!< ADC  Calibration Factors,                          Address offset: 0xB4 */
	uint32_t      RESERVED11[148];  /*!< Reserved, 0x0B8 - 0x308                                                 */
  __IO uint32_t CCR;              /*!< CCR data register,                                 Address offset:0x308 */
} ADC_TypeDef;
  


/** 
  * @brief CRC calculation unit				//????????????????????????????????????????
  */
  
typedef struct
{
  __IO uint32_t DR;          /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint8_t  IDR;         /*!< CRC Independent data register,               Address offset: 0x04 */
  uint8_t       RESERVED0;   /*!< Reserved,                                                    0x05 */
  uint16_t      RESERVED1;   /*!< Reserved,                                                    0x06 */
  __IO uint32_t CR;          /*!< CRC Control register,                        Address offset: 0x08 */ 
  uint32_t      RESERVED2;   /*!< Reserved,                                                    0x0C */
  __IO uint32_t INIT;        /*!< Initial CRC value register,                  Address offset: 0x10 */
  __IO uint32_t POL;   	     /*!< Reserved,                                                    0x14 */
} CRC_TypeDef;

/** 
  * @brief Debug MCU									//?????????????????????????????????????????
  */

typedef struct
{
  __IO uint32_t IDCODE;       /*!< MCU device ID code,                          Address offset: 0x00 */
  __IO uint32_t CR;           /*!< Debug MCU configuration register,            Address offset: 0x04 */
  __IO uint32_t APB1FZ;       /*!< Debug MCU APB1 freeze register,              Address offset: 0x08 */
  __IO uint32_t APB2FZ;       /*!< Debug MCU APB2 freeze register,              Address offset: 0x0C */
}DBGMCU_TypeDef;

/** 
  * @brief DMA Controller
  */

typedef struct
{
  __IO uint32_t CCR;          /*!< DMA channel x configuration register        Address offset: 0x08 */
  __IO uint32_t CNDTR;        /*!< DMA channel x number of data register       Address offset: 0x0C */
  __IO uint32_t CPAR;         /*!< DMA channel x peripheral address register   Address offset: 0x10 */
  __IO uint32_t CMAR;         /*!< DMA channel x memory address register       Address offset: 0x14 */
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;          /*!< DMA interrupt status register,               Address offset: 0x00 */
  __IO uint32_t IFCR;         /*!< DMA interrupt flag clear register,           Address offset: 0x04 */
} DMA_TypeDef;

/** 
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR;          /*!<EXTI Interrupt mask register,                 Address offset: 0x00 */
  __IO uint32_t EMR;          /*!<EXTI Event mask register,                     Address offset: 0x04 */
  __IO uint32_t RTSR;         /*!<EXTI Rising trigger selection register ,      Address offset: 0x08 */
  __IO uint32_t FTSR;         /*!<EXTI Falling trigger selection register,      Address offset: 0x0C */
  __IO uint32_t SWIER;        /*!<EXTI Software interrupt event register,       Address offset: 0x10 */
  __IO uint32_t PR;           /*!<EXTI Pending register,                        Address offset: 0x14 */
} EXTI_TypeDef;

typedef struct
{
	__IO uint32_t EXTI_CFGR;   
} EXTICFG_TypeDef;

/** 
  * @brief FLASH Registers
  */
typedef struct
{
  __IO uint32_t ACR;          /*!<FLASH access control register,                 Address offset: 0x00 */
  __IO uint32_t KEYR;         /*!<FLASH key register,                            Address offset: 0x04 */
  __IO uint32_t OPTKEYR;      /*!<FLASH OPT key register,                        Address offset: 0x08 */
  __IO uint32_t SR;           /*!<FLASH status register,                         Address offset: 0x0C */
  __IO uint32_t CR;           /*!<FLASH control register,                        Address offset: 0x10 */
  __IO uint32_t AR;           /*!<FLASH address register,                        Address offset: 0x14 */
  __IO uint32_t OBR;          /*!<FLASH option bytes register,                   Address offset: 0x18 */
  __IO uint32_t WRPR0;        /*!<FLASH write protect register 0,                Address offset: 0x1C */
  __IO uint32_t WRPR1;        /*!<FLASH write protect register 1,                Address offset: 0x20 */
} FLASH_TypeDef;

/** 
  * @brief Option Bytes Registers   
  */
typedef struct
{
  __IO uint16_t RDP;          /*!< FLASH option byte Read protection,             Address offset: 0x00 */
  __IO uint16_t USER;         /*!< FLASH option byte user options,                Address offset: 0x02 */
  __IO uint16_t DATA0;        /*!< User data byte 0 (stored in FLASH_OBR[23:16]), Address offset: 0x04 */
  __IO uint16_t DATA1;        /*!< User data byte 1 (stored in FLASH_OBR[31:24]), Address offset: 0x06 */
  __IO uint16_t WRP0;         /*!< FLASH option byte write protection 0,          Address offset: 0x08 */
  __IO uint16_t WRP1;         /*!< FLASH option byte write protection 1,          Address offset: 0x0A */
  __IO uint16_t WRP2;         /*!< FLASH option byte write protection 2,          Address offset: 0x0C */
  __IO uint16_t WRP3;         /*!< FLASH option byte write protection 3,          Address offset: 0x0E */
  __IO uint16_t WRP4;         /*!< FLASH option byte write protection 4,          Address offset: 0x10 */
  __IO uint16_t WRP5;         /*!< FLASH option byte write protection 5,          Address offset: 0x12 */
  __IO uint16_t WRP6;         /*!< FLASH option byte write protection 6,          Address offset: 0x14 */
  __IO uint16_t WRP7;         /*!< FLASH option byte write protection 7,          Address offset: 0x16 */
} OB_TypeDef;

/** 
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;        /*!< GPIO port mode register,                       Address offset: 0x00      */
  __IO uint32_t OTYPER;       /*!< GPIO port output type register,                Address offset: 0x04      */
  __IO uint32_t ODRIVER;      /*!< GPIO port output drive strength register,      Address offset: 0x08      */
  __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,          Address offset: 0x0C      */
  __IO uint32_t IDR;          /*!< GPIO port input data register,                 Address offset: 0x10      */
  __IO uint32_t ODR;          /*!< GPIO port output data register,                Address offset: 0x14      */
  __IO uint32_t BSRR;         /*!< GPIO port bit set/reset register,              Address offset: 0x18 	    */
  __IO uint32_t LCKR;         /*!< GPIO port configuration lock register,         Address offset: 0x1C      */
  __IO uint32_t AFR[2];       /*!< GPIO alternate function low register,          Address offset: 0x20-0x24 */
  __IO uint32_t BRR;          /*!< GPIO port bit reset register,                  Address offset: 0x28      */
  __IO uint32_t ITYPER;       /*!< GPIO port CMOS/Schmitt input model register,   Address offset: 0x2C      */
  __IO uint32_t REV;          /*!< Reserved                                   ,   Address offset: 0x30      */
  __IO uint32_t BTR;          /*!< GPIO port bit toggle register,                 Address offset: 0x34      */  
} GPIO_TypeDef;

/** 
  * @brief PI Controller
  */

typedef struct 
{
  __IO uint32_t CR;
	__IO uint32_t PI0_ARG;
	__IO uint32_t PI1_ARG;
	__IO uint32_t PI2_ARG;
	__IO uint32_t PI0_DATA1;
	__IO uint32_t PI1_DATA1;
	__IO uint32_t PI2_DATA1;
	__IO uint32_t PI0_DATA2;
	__IO uint32_t PI1_DATA2;
	__IO uint32_t PI2_DATA2;
	__IO uint32_t PI0_RESULT1;
	__IO uint32_t PI1_RESULT1;
	__IO uint32_t PI2_RESULT1;
	__IO uint32_t PI0_RESULT2;
	__IO uint32_t PI1_RESULT2;
	__IO uint32_t PI2_RESULT2;
	__IO uint32_t UMIN;
	__IO uint32_t UMAX;
}PI_TypeDef;

typedef struct
{
  __IO uint32_t CR;
	__IO uint32_t INDATA;
	__IO uint32_t OUTDATA;
	__IO uint32_t UMIN0;
	__IO uint32_t UMAX0;
	__IO uint32_t UMIN1;
	__IO uint32_t UMAX1;
	__IO uint32_t UMIN2;
	__IO uint32_t UMAX2;
	__IO uint32_t UMIN3;
	__IO uint32_t UMAX3;
}CTU_TypeDef;

typedef struct
{
	__IO uint32_t CR;
	__IO uint32_t DATA1;
	__IO uint32_t DATA2;
	__IO uint32_t RESULT1;
	__IO uint32_t RESULT2;
}MATH_TypeDef;

typedef struct
{
  __IO uint32_t CSR;
	__IO uint32_t UALPHA;
	__IO uint32_t UBETA;
	__IO uint32_t T_VALUE;
	__IO uint32_t CCR1_VALUE;
	__IO uint32_t CCR2_VALUE;
	__IO uint32_t CCR3_VALUE;
}SVPWM_TypeDef;

typedef struct
{
  __IO uint32_t CSR;
	__IO uint32_t WDATA;
	__IO uint32_t RDATA;
}CORDIC_TypeDef;

typedef struct 
{
	__IO uint32_t DAC_DATR;
	__IO uint32_t DAC_CFGR;
	__IO uint32_t OPA0_CFGR1;
	__IO uint32_t OPA0_CFGR2;
	__IO uint32_t OPA1_CFGR1;
	__IO uint32_t OPA1_CFGR2;
	__IO uint32_t OPA_BUF;
	__IO uint32_t CMP_FILT;
	__IO uint32_t CMP0_CSR;
	__IO uint32_t CMP1_CSR;
	__IO uint32_t CMP_MVR;
}ANALOG_TypeDef;
	

/** 
  * @brief SysTem Configuration   
  */
typedef struct
{
  __IO uint32_t CFGR1;         /*!< SYSCFG configuration register 1,                       Address offset: 0x00 */
  __IO uint32_t RESERVED0;     /*!< Reserved,                                              Address offset: 0x04 */  
  __IO uint32_t EXTICR[4];     /*!< SYSCFG external interrupt configuration register, Address offset: 0x08-0x14 */		 
	__IO uint32_t DBGCFG;
} SYSCFG_TypeDef;


/** 
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;          /*!< I2C Control register 1,                      				Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< I2C Control register 2,                      				Address offset: 0x04 */
  __IO uint32_t OAR1;     		/*!< I2C Own address 1 register,        									Address offset: 0x08 */
  __IO uint32_t OAR2;     		/*!< I2C Own address 2 register,        									Address offset: 0x0C */
  __IO uint32_t TIMINGR;  		/*!< I2C Timing register,               									Address offset: 0x10 */
  __IO uint32_t TIMEOUTR; 		/*!< I2C Timeout register,              									Address offset: 0x14 */
  __IO uint32_t ISR;      		/*!< I2C Interrupt and status register, 									Address offset: 0x18 */
  __IO uint32_t ICR;      		/*!< I2C Interrupt clear register,      									Address offset: 0x1C */
  __IO uint32_t PECR;     		/*!< I2C PEC register,                  									Address offset: 0x20 */
  __IO uint32_t RXDR;     		/*!< I2C Receive data register,         									Address offset: 0x24 */
  __IO uint32_t TXDR;     		/*!< I2C Transmit data register,        									Address offset: 0x28 */
} I2C_TypeDef;

/** 
  * @brief Independent WATCHDOG
  */
typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR; /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;

/** 
  * @brief Power Control
  */

typedef struct
{
  __IO uint32_t CR;
	__IO uint32_t CFGR;
	__IO uint32_t LPR;
	__IO uint32_t CSR;
	__IO uint32_t BAK0;
	__IO uint32_t BAK1;
	__IO uint32_t BAK2;
	__IO uint32_t BAK3;
	__IO uint32_t BAK4;
} PWR_TypeDef;

/** 
  * @brief Reset and Clock Control		
  */

typedef struct
{
  __IO uint32_t CR;         /*!< RCC clock control register,                            Address offset: 0x00 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                      Address offset: 0x04 */
  __IO uint32_t PLLR;  			/*!< PLL Register,                   												Address offset: 0x08 */  
  __IO uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                    Address offset: 0x0C */
  __IO uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                    Address offset: 0x10 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripheral enable clock register,              Address offset: 0x14 */
  __IO uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,             Address offset: 0x18 */
  __IO uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,             Address offset: 0x1C */
  __IO uint32_t Rev1;       /*!< Rev1,                                                  Address offset: 0x20 */
  __IO uint32_t CSR;        /*!< RCC clock control & status register,                   Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                     Address offset: 0x28 */
} RCC_TypeDef;


/** 
  * @brief Serial Peripheral Interface		
  */

typedef struct
{
  __IO uint32_t CR1;        /*!< SPI Control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI Control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI Status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI Rx CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI Tx CRC register (not used in I2S mode),         Address offset: 0x18 */
	__IO uint16_t I2SCFGR;  /*!< SPI_I2S configuration register,                      Address offset: 0x1C */
  uint16_t  RESERVED7;    /*!< Reserved, 0x1E                                                            */
  __IO uint16_t I2SPR;    /*!< SPI_I2S prescaler register,                          Address offset: 0x20 */
  uint16_t  RESERVED8;    /*!< Reserved, 0x22                                                            */    
} SPI_TypeDef;

/** 
  * @brief TIM				
  */
typedef struct
{
  __IO uint32_t CR1;          /*!< TIM control register 1,              						Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< TIM control register 2,              						Address offset: 0x04 */
  __IO uint32_t SMCR;         /*!< TIM slave Mode Control register,     						Address offset: 0x08 */
  __IO uint32_t DIER;         /*!< TIM DMA/interrupt enable register,   						Address offset: 0x0C */
  __IO uint32_t SR;           /*!< TIM status register,                 						Address offset: 0x10 */
  __IO uint32_t EGR;          /*!< TIM event generation register,       						Address offset: 0x14 */
  __IO uint32_t CCMR1;        /*!< TIM capture/compare mode register 1, 						Address offset: 0x18 */
  __IO uint32_t CCMR2;        /*!< TIM capture/compare mode register 2, 						Address offset: 0x1C */
  __IO uint32_t CCER;         /*!< TIM capture/compare enable register, 						Address offset: 0x20 */
  __IO uint32_t CNT;          /*!< TIM counter register,                						Address offset: 0x24 */
  __IO uint32_t PSC;          /*!< TIM prescaler register,              						Address offset: 0x28 */
  __IO uint32_t ARR;          /*!< TIM auto-reload register,            						Address offset: 0x2C */
  __IO uint32_t RCR;   				/*!< TIM repetition register,            	   					Address offset: 0x30 */
  __IO uint32_t CCR1;         /*!< TIM capture/compare register 1,      						Address offset: 0x34 */    
  __IO uint32_t CCR2;         /*!< TIM capture/compare register 2,      						Address offset: 0x38 */    
  __IO uint32_t CCR3;         /*!< TIM capture/compare register 3,      						Address offset: 0x3C */
  __IO uint32_t CCR4;         /*!< TIM capture/compare register 4,      						Address offset: 0x40 */
	__IO uint32_t BDTR;         /*!< TIM break and dead-time register,                Address offset: 0x44 */
	__IO uint32_t DCR;         	/*!<  0x48 */
	__IO uint32_t DMAR;         /*!<  0x4C */
	__IO uint32_t DTR2;         /*!<  0x50 */ 
  __IO uint32_t CCMR3;        /*!<  0x54 */
	__IO uint32_t CCR5;         /*!<  0x58 */
	__IO uint32_t CCR6;         /*!<  0x5C */
	__IO uint32_t CCR1N;        /*!<  0x60 */
	__IO uint32_t CCR2N;        /*!<  0x64 */
	__IO uint32_t CCR3N;        /*!<  0x68 */
	__IO uint32_t AF1;          /*!<  0x6C */
	__IO uint32_t AF2;          /*!<  0x70 */
} TIM_TypeDef;

/** 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
 
typedef struct
{
  __IO uint32_t CR1;    			/*!< UART Control register 1,                			 Address offset: 0x00 */ 
  __IO uint32_t CR2;    			/*!< UART Control register 2,                			 Address offset: 0x04 */ 
  __IO uint32_t CR3;    			/*!< UART Control register 3,                			 Address offset: 0x08 */
  __IO uint32_t BRR;    			/*!< UART Baud rate register,                			 Address offset: 0x0C */
  uint32_t RESERVED1;   			/*!< Reserved, 			 															 Address offset: 0x10 */	
  uint32_t RESERVED2;   			/*!< Reserved,        			 											 Address offset: 0x14 */      
  __IO uint16_t RQR;    			/*!< UART Request register,                  			 Address offset: 0x18 */
  uint16_t RESERVED3;         /*!< Reserved,                                     Address offset: 0x1A */	
  __IO uint32_t ISR;    			/*!< UART Interrupt and status register,     			 Address offset: 0x1C */
  __IO uint32_t ICR;    			/*!< UART Interrupt flag Clear register,     			 Address offset: 0x20 */
  __IO uint16_t RDR;    			/*!< UART Receive Data register,             			 Address offset: 0x24 */
  uint16_t RESERVED4;  			  /*!< Reserved, 																										 0x26 */
  __IO uint16_t TDR;    			/*!< UART Transmit Data register,            			 Address offset: 0x28 */
  uint16_t RESERVED5;         /*!< Reserved,                                     Address offset: 0x2A */
} UART_TypeDef;

typedef struct
{
  __IO uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */ 
  __IO uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */ 
  __IO uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint16_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
  uint16_t RESERVED1;   /*!< Reserved,                                 Address offset: 0x0E */
  __IO uint16_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  uint16_t RESERVED2;   /*!< Reserved,                                 Address offset: 0x12 */
  __IO uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */  
  __IO uint16_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
  uint16_t RESERVED3;   /*!< Reserved,                                 Address offset: 0x1A */
  __IO uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint16_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
  uint16_t RESERVED4;   /*!< Reserved,                                 Address offset: 0x26 */
  __IO uint16_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
  uint16_t RESERVED5;   /*!< Reserved,                                 Address offset: 0x2A */
} USART_TypeDef;



/** 
  * @brief Window WATCHDOG
  */
typedef struct
{
  __IO uint32_t CR;   				/*!< WWDG Control register,       									 Address offset: 0x00 */
  __IO uint32_t CFR;  				/*!< WWDG Configuration register, 									 Address offset: 0x04 */
  __IO uint32_t SR;   				/*!< WWDG Status register,        									 Address offset: 0x08 */
} WWDG_TypeDef;


/** 
  * @}
  */
  
/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            ((uint32_t)0x08000000U)              /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END       ((uint32_t)0x0800FFFFU) 						 /*!< FLASH END address of bank1 */
#define SRAM_BASE             ((uint32_t)0x20000000U)              /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000U)              /*!< Peripheral base address in the alias region */

/*!< Peripheral memory map */
#define APBPERIPH_BASE         PERIPH_BASE
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000)

/*!< APB peripherals */
#define TIM20_BASE             (APBPERIPH_BASE + 0x00012C00)
#define TIM2_BASE           	(APBPERIPH_BASE + 0x00002000)
#define TIM3_BASE           	(APBPERIPH_BASE + 0x00000400)
#define TIM6_BASE           	(APBPERIPH_BASE + 0x00014400)
#define TIM7_BASE           	(APBPERIPH_BASE + 0x00014800)
#define WWDG_BASE             (APBPERIPH_BASE + 0x00002C00)
#define IWDG_BASE             (APBPERIPH_BASE + 0x00003000)
#define I2C1_BASE             (APBPERIPH_BASE + 0x00005400)
#define I2C2_BASE             (APBPERIPH_BASE + 0x00005800)
#define PWR_BASE              (APBPERIPH_BASE + 0x00007000)
#define SYSCFG_BASE           (APBPERIPH_BASE + 0x00010000)
#define EXTI_BASE             (APBPERIPH_BASE + 0x00010400)
#define EXTICFG_BASE          (APBPERIPH_BASE + 0x000107FC)
#define ANALOG_BASE           (APBPERIPH_BASE + 0x00011000)
#define ADC1_BASE             (APBPERIPH_BASE + 0x00012400)	
#define SPI1_BASE             (APBPERIPH_BASE + 0x00013000)
#define DBGMCU_BASE           (APBPERIPH_BASE + 0x00015800)
#define USART1_BASE           (APBPERIPH_BASE + 0x00013800)
#define UART2_BASE            (APBPERIPH_BASE + 0x00013400)
/*!< AHB peripherals */
#define DMA1_BASE             (AHBPERIPH_BASE + 0x00000000)
#define DMA1_Channel1_BASE    (DMA1_BASE + 0x00000008)
#define DMA1_Channel2_BASE    (DMA1_BASE + 0x0000001C)
#define DMA1_Channel3_BASE    (DMA1_BASE + 0x00000030)
#define DMA1_Channel4_BASE    (DMA1_BASE + 0x00000044)
#define DMA1_Channel5_BASE    (DMA1_BASE + 0x00000058)

#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000) 			/*!< FLASH registers base address */
#define OB_BASE               ((uint32_t)0x1FF80000U)       			/*!< FLASH Option Bytes base address */
#define CRC_BASE              (AHBPERIPH_BASE + 0x00003000)
#define PI_BASE               (AHBPERIPH_BASE + 0x00005000)
#define MATH_BASE             (AHBPERIPH_BASE + 0x00005800)
#define SVPWM_BASE            (AHBPERIPH_BASE + 0x00005C00)
#define CORDIC_BASE           (AHBPERIPH_BASE + 0x00006000)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800)



#define SYSTEM_FLASH_RECORD				         1//1:System Record; 0:Flash Record

#if SYSTEM_FLASH_RECORD
#define HSI_CAL_ADDR                       (0x1FF005F0U)
#define HSI_CAL_SIZE                       (2U)

#define LSI_CAL_ADDR                       (0x1FF005F2U)
#define LSI_CAL_SIZE                       (2U)

#define LDO_1P2_ADDR                       (0x1FF005F4U)
#define LDO_1P2_SIZE                       (2U)
#define LDO_1P5_ADDR                       (0x1FF005F6U)
#define LDO_1P5_SIZE                       (2U)

#define ID_INFO_ADDR       					       (0x1FF005E0U) /* ID:UID + PID + MS*/
#define ID_INFO_SIZE       					       (0x0FU) 			 /* ID:12  + 2   + 1*/
#define UID_ADDR       		    			  	   (0x1FF005E0U) /* Unique device ID address */
#define UID_SIZE       					  	       (0x0CU)
#define PID_ADDR          					  	   (0x1FF005ECU) /* Product ID address */
#define PID_SIZE          					  	   (0x02U)
#define MS_ADDR           					  	   (0x1FF005EEU) /* FLASH memory size address */
#define MS_SIZE       					  	       (0x01U)
#else
#define HSI_CAL_ADDR                       (0x0800FFF0U)
#define HSI_CAL_SIZE                       (2U)

#define LSI_CAL_ADDR                       (0x0800FFF2U)
#define LSI_CAL_SIZE                       (2U)

#define LDO_1P2_ADDR                       (0x0800FFF4U)
#define LDO_1P2_SIZE                       (2U)
#define LDO_1P5_ADDR                       (0x0800FFF6U)
#define LDO_1P5_SIZE                       (2U)

#define ID_INFO_ADDR       					       (0x0800FFE0U) /* ID:UID + PID + MS*/
#define ID_INFO_SIZE       					       (0x0FU) 			 /* ID:12  + 2   + 1*/
#define UID_ADDR       					      	   (0x0800FFE0U) /* Unique device ID address */
#define UID_SIZE       					  	       (0x0CU)
#define PID_ADDR       					  	       (0x0800FFECU) /* Product ID address */
#define PID_SIZE       				    	  	   (0x02U)
#define MS_ADDR           					  	   (0x0800FFEEU) /* FLASH memory size address */
#define MS_SIZE       					  	       (0x01U)
#endif


/**
  * @}
  */
  
/** @addtogroup Peripheral_declaration
  * @{
  */  

#define TIM20                ((TIM_TypeDef *) TIM20_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define EXTICFG             ((EXTICFG_TypeDef *) EXTICFG_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE) 
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define UART2              ((UART_TypeDef *) UART2_BASE)
#define PI                 ((PI_TypeDef *)PI_BASE)
#define MATH               ((MATH_TypeDef *)MATH_BASE)
#define SVPWM              ((SVPWM_TypeDef *)SVPWM_BASE)
#define CORDIC             ((CORDIC_TypeDef *)CORDIC_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define ANALOG              ((ANALOG_TypeDef *)ANALOG_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter SAR (ADC)               */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for ADC_ISR register  ********************/
#define ADC_ISR_ADRD          ((uint32_t)0x00000001) /*!< ADC Ready (ADRDY) flag  */
#define ADC_ISR_EOSMP         ((uint32_t)0x00000002) /*!< ADC End of Sampling flag */
#define ADC_ISR_EOC           ((uint32_t)0x00000004) /*!< ADC End of Regular Conversion flag */
#define ADC_ISR_EOS           ((uint32_t)0x00000008) /*!< ADC End of Regular sequence of Conversions flag */
#define ADC_ISR_OVR           ((uint32_t)0x00000010) /*!< ADC overrun flag */
#define ADC_ISR_JEOC          ((uint32_t)0x00000020) /*!< ADC End of Injected Conversion flag */
#define ADC_ISR_JEOS          ((uint32_t)0x00000040) /*!< ADC End of Injected sequence of Conversions flag */
#define ADC_ISR_AWD1          ((uint32_t)0x00000080) /*!< ADC Analog watchdog 1 flag */
#define ADC_ISR_AWD2          ((uint32_t)0x00000100) /*!< ADC Analog watchdog 2 flag */
#define ADC_ISR_AWD3          ((uint32_t)0x00000200) /*!< ADC Analog watchdog 3 flag */
#define ADC_ISR_JQOVF         ((uint32_t)0x00000400) /*!< ADC Injected Context Queue Overflow flag */

/********************  Bit definition for ADC_IER register  ********************/
#define ADC_IER_RDY           ((uint32_t)0x00000001) /*!< ADC Ready (ADRDY) interrupt source */
#define ADC_IER_EOSMP         ((uint32_t)0x00000002) /*!< ADC End of Sampling interrupt source */
#define ADC_IER_EOC           ((uint32_t)0x00000004) /*!< ADC End of Regular Conversion interrupt source */
#define ADC_IER_EOS           ((uint32_t)0x00000008) /*!< ADC End of Regular sequence of Conversions interrupt source */
#define ADC_IER_OVR           ((uint32_t)0x00000010) /*!< ADC overrun interrupt source */
#define ADC_IER_JEOC          ((uint32_t)0x00000020) /*!< ADC End of Injected Conversion interrupt source */
#define ADC_IER_JEOS          ((uint32_t)0x00000040) /*!< ADC End of Injected sequence of Conversions interrupt source */
#define ADC_IER_AWD1          ((uint32_t)0x00000080) /*!< ADC Analog watchdog 1 interrupt source */
#define ADC_IER_AWD2          ((uint32_t)0x00000100) /*!< ADC Analog watchdog 2 interrupt source */
#define ADC_IER_AWD3          ((uint32_t)0x00000200) /*!< ADC Analog watchdog 3 interrupt source */
#define ADC_IER_JQOVF         ((uint32_t)0x00000400) /*!< ADC Injected Context Queue Overflow interrupt source */

/********************  Bit definition for ADC_CR register  ********************/
#define ADC_CR_ADEN          ((uint32_t)0x00000001) /*!< ADC Enable control */
#define ADC_CR_ADDIS         ((uint32_t)0x00000002) /*!< ADC Disable command */
#define ADC_CR_ADSTART       ((uint32_t)0x00000004) /*!< ADC Start of Regular conversion */
#define ADC_CR_JADSTART      ((uint32_t)0x00000008) /*!< ADC Start of injected conversion */
#define ADC_CR_ADSTP         ((uint32_t)0x00000010) /*!< ADC Stop of Regular conversion */
#define ADC_CR_JADSTP        ((uint32_t)0x00000020) /*!< ADC Stop of injected conversion */
#define ADC_CR_ADVREGEN      ((uint32_t)0x30000000) /*!< ADC Voltage regulator Enable */
#define ADC_CR_ADVREGEN_0    ((uint32_t)0x10000000) /*!< ADC ADVREGEN bit 0 */
#define ADC_CR_ADVREGEN_1    ((uint32_t)0x20000000) /*!< ADC ADVREGEN bit 1 */
#define ADC_CR_ADCALDIF      ((uint32_t)0x40000000) /*!< ADC Differential Mode for calibration */
#define ADC_CR_ADCAL         ((uint32_t)0x80000000) /*!< ADC Calibration */

/********************  Bit definition for ADC_CFGR1 register  ********************/
#define ADC_CFGR1_DMAEN     ((uint32_t)0x00000001) /*!< ADC DMA Enable */
#define ADC_CFGR1_JDMAEN     ((uint32_t)0x00000004) /*!< ADC DMA Enable */

#define ADC_CFGR1_RES       ((uint32_t)0x00000018) /*!< ADC Data resolution */
#define ADC_CFGR1_RES_0     ((uint32_t)0x00000008) /*!< ADC RES bit 0 */
#define ADC_CFGR1_RES_1     ((uint32_t)0x00000010) /*!< ADC RES bit 1 */

#define ADC_CFGR1_ALIGN     ((uint32_t)0x00000020) /*!< ADC Data Alignment */

#define ADC_CFGR1_EXTSEL   ((uint32_t)0x000000E0) /*!< ADC External trigger selection for regular group */
#define ADC_CFGR1_EXTSEL_0 ((uint32_t)0x00000020) /*!< ADC EXTSEL bit 0 */
#define ADC_CFGR1_EXTSEL_1 ((uint32_t)0x00000040) /*!< ADC EXTSEL bit 1 */
#define ADC_CFGR1_EXTSEL_2 ((uint32_t)0x00000080) /*!< ADC EXTSEL bit 2 */

#define ADC_CFGR1_EXTEN     ((uint32_t)0x00000C00) /*!< ADC External trigger enable and polarity selection for regular channels */
#define ADC_CFGR1_EXTEN_0   ((uint32_t)0x00000400) /*!< ADC EXTEN bit 0 */
#define ADC_CFGR1_EXTEN_1   ((uint32_t)0x00000800) /*!< ADC EXTEN bit 1 */

#define ADC_CFGR1_OVRMOD    ((uint32_t)0x00001000) /*!< ADC overrun mode */
#define ADC_CFGR1_CONT      ((uint32_t)0x00002000) /*!< ADC Single/continuous conversion mode for regular conversion */
#define ADC_CFGR1_CONT_Pos  13
#define ADC_CFGR1_AUTDLY    ((uint32_t)0x00004000) /*!< ADC Delayed conversion mode */
#define ADC_CFGR1_DISCEN    ((uint32_t)0x00010000) /*!< ADC Discontinuous mode for regular channels */

#define ADC_CFGR1_DISCNUM   ((uint32_t)0x000E0000) /*!< ADC Discontinuous mode channel count */
#define ADC_CFGR1_DISCNUM_0 ((uint32_t)0x00020000) /*!< ADC DISCNUM bit 0 */
#define ADC_CFGR1_DISCNUM_1 ((uint32_t)0x00040000) /*!< ADC DISCNUM bit 1 */
#define ADC_CFGR1_DISCNUM_2 ((uint32_t)0x00080000) /*!< ADC DISCNUM bit 2 */

#define ADC_CFGR1_JDISCEN   ((uint32_t)0x00100000) /*!< ADC Discontinuous mode on injected channels */
#define ADC_CFGR1_JQM       ((uint32_t)0x00200000) /*!< ADC JSQR Queue mode */
#define ADC_CFGR1_AWD1SGL   ((uint32_t)0x00400000) /*!< Enable the watchdog 1 on a single channel or on all channels */
#define ADC_CFGR1_AWD1EN    ((uint32_t)0x00800000) /*!< ADC Analog watchdog 1 enable on regular Channels */
#define ADC_CFGR1_JAWD1EN   ((uint32_t)0x01000000) /*!< ADC Analog watchdog 1 enable on injected Channels */
#define ADC_CFGR1_JAUTO     ((uint32_t)0x02000000) /*!< ADC Automatic injected group conversion */

#define ADC_CFGR1_AWD1CH    ((uint32_t)0x7C000000) /*!< ADC Analog watchdog 1 Channel selection */
#define ADC_CFGR1_AWD1CH_0  ((uint32_t)0x04000000) /*!< ADC AWD1CH bit 0 */
#define ADC_CFGR1_AWD1CH_1  ((uint32_t)0x08000000) /*!< ADC AWD1CH bit 1  */
#define ADC_CFGR1_AWD1CH_2  ((uint32_t)0x10000000) /*!< ADC AWD1CH bit 2  */
#define ADC_CFGR1_AWD1CH_3  ((uint32_t)0x20000000) /*!< ADC AWD1CH bit 3  */
#define ADC_CFGR1_AWD1CH_4  ((uint32_t)0x40000000) /*!< ADC AWD1CH bit 4  */

#define  ADC_CFGR1_JQDIS    ((uint32_t)0x80000000)

/********************  Bit definition for ADC_CFGR2 register  ********************/
#define ADC_CFGR2_OVSE     ((uint32_t)0x00000001)
#define ADC_CFGR2_OVSE     ((uint32_t)0x00000001)



/********************  Bit definition for ADC_SMPR1 register  ********************/
#define ADC_SMPR1_SMP0     ((uint32_t)0x00000007) /*!< ADC Channel 0 Sampling time selection  */
#define ADC_SMPR1_SMP0_0   ((uint32_t)0x00000001) /*!< ADC SMP0 bit 0 */
#define ADC_SMPR1_SMP0_1   ((uint32_t)0x00000002) /*!< ADC SMP0 bit 1 */
#define ADC_SMPR1_SMP0_2   ((uint32_t)0x00000004) /*!< ADC SMP0 bit 2 */

#define ADC_SMPR1_SMP1     ((uint32_t)0x00000038) /*!< ADC Channel 1 Sampling time selection  */
#define ADC_SMPR1_SMP1_0   ((uint32_t)0x00000008) /*!< ADC SMP1 bit 0 */
#define ADC_SMPR1_SMP1_1   ((uint32_t)0x00000010) /*!< ADC SMP1 bit 1 */
#define ADC_SMPR1_SMP1_2   ((uint32_t)0x00000020) /*!< ADC SMP1 bit 2 */

#define ADC_SMPR1_SMP2     ((uint32_t)0x000001C0) /*!< ADC Channel 2 Sampling time selection  */
#define ADC_SMPR1_SMP2_0   ((uint32_t)0x00000040) /*!< ADC SMP2 bit 0 */
#define ADC_SMPR1_SMP2_1   ((uint32_t)0x00000080) /*!< ADC SMP2 bit 1 */
#define ADC_SMPR1_SMP2_2   ((uint32_t)0x00000100) /*!< ADC SMP2 bit 2 */

#define ADC_SMPR1_SMP3     ((uint32_t)0x00000E00) /*!< ADC Channel 3 Sampling time selection  */
#define ADC_SMPR1_SMP3_0   ((uint32_t)0x00000200) /*!< ADC SMP3 bit 0 */
#define ADC_SMPR1_SMP3_1   ((uint32_t)0x00000400) /*!< ADC SMP3 bit 1 */
#define ADC_SMPR1_SMP3_2   ((uint32_t)0x00000800) /*!< ADC SMP3 bit 2 */

#define ADC_SMPR1_SMP4     ((uint32_t)0x00007000) /*!< ADC Channel 4 Sampling time selection  */
#define ADC_SMPR1_SMP4_0   ((uint32_t)0x00001000) /*!< ADC SMP4 bit 0 */
#define ADC_SMPR1_SMP4_1   ((uint32_t)0x00002000) /*!< ADC SMP4 bit 1 */
#define ADC_SMPR1_SMP4_2   ((uint32_t)0x00004000) /*!< ADC SMP4 bit 2 */

#define ADC_SMPR1_SMP5     ((uint32_t)0x00038000) /*!< ADC Channel 5 Sampling time selection  */
#define ADC_SMPR1_SMP5_0   ((uint32_t)0x00008000) /*!< ADC SMP5 bit 0 */
#define ADC_SMPR1_SMP5_1   ((uint32_t)0x00010000) /*!< ADC SMP5 bit 1 */
#define ADC_SMPR1_SMP5_2   ((uint32_t)0x00020000) /*!< ADC SMP5 bit 2 */

#define ADC_SMPR1_SMP6     ((uint32_t)0x001C0000) /*!< ADC Channel 6 Sampling time selection  */
#define ADC_SMPR1_SMP6_0   ((uint32_t)0x00040000) /*!< ADC SMP6 bit 0 */
#define ADC_SMPR1_SMP6_1   ((uint32_t)0x00080000) /*!< ADC SMP6 bit 1 */
#define ADC_SMPR1_SMP6_2   ((uint32_t)0x00100000) /*!< ADC SMP6 bit 2 */

#define ADC_SMPR1_SMP7     ((uint32_t)0x00E00000) /*!< ADC Channel 7 Sampling time selection  */
#define ADC_SMPR1_SMP7_0   ((uint32_t)0x00200000) /*!< ADC SMP7 bit 0 */
#define ADC_SMPR1_SMP7_1   ((uint32_t)0x00400000) /*!< ADC SMP7 bit 1 */
#define ADC_SMPR1_SMP7_2   ((uint32_t)0x00800000) /*!< ADC SMP7 bit 2 */

#define ADC_SMPR1_SMP8     ((uint32_t)0x07000000) /*!< ADC Channel 8 Sampling time selection  */
#define ADC_SMPR1_SMP8_0   ((uint32_t)0x01000000) /*!< ADC SMP8 bit 0 */
#define ADC_SMPR1_SMP8_1   ((uint32_t)0x02000000) /*!< ADC SMP8 bit 1 */
#define ADC_SMPR1_SMP8_2   ((uint32_t)0x04000000) /*!< ADC SMP8 bit 2 */

#define ADC_SMPR1_SMP9     ((uint32_t)0x38000000) /*!< ADC Channel 9 Sampling time selection  */
#define ADC_SMPR1_SMP9_0   ((uint32_t)0x08000000) /*!< ADC SMP9 bit 0 */
#define ADC_SMPR1_SMP9_1   ((uint32_t)0x10000000) /*!< ADC SMP9 bit 1 */
#define ADC_SMPR1_SMP9_2   ((uint32_t)0x20000000) /*!< ADC SMP9 bit 2 */

/********************  Bit definition for ADC_SMPR2 register  ********************/
#define ADC_SMPR2_SMP10     ((uint32_t)0x00000007) /*!< ADC Channel 10 Sampling time selection  */
#define ADC_SMPR2_SMP10_0   ((uint32_t)0x00000001) /*!< ADC SMP10 bit 0 */
#define ADC_SMPR2_SMP10_1   ((uint32_t)0x00000002) /*!< ADC SMP10 bit 1 */
#define ADC_SMPR2_SMP10_2   ((uint32_t)0x00000004) /*!< ADC SMP10 bit 2 */

#define ADC_SMPR2_SMP11     ((uint32_t)0x00000038) /*!< ADC Channel 11 Sampling time selection  */
#define ADC_SMPR2_SMP11_0   ((uint32_t)0x00000008) /*!< ADC SMP11 bit 0 */
#define ADC_SMPR2_SMP11_1   ((uint32_t)0x00000010) /*!< ADC SMP11 bit 1 */
#define ADC_SMPR2_SMP11_2   ((uint32_t)0x00000020) /*!< ADC SMP11 bit 2 */

#define ADC_SMPR2_SMP12     ((uint32_t)0x000001C0) /*!< ADC Channel 12 Sampling time selection  */
#define ADC_SMPR2_SMP12_0   ((uint32_t)0x00000040) /*!< ADC SMP12 bit 0 */
#define ADC_SMPR2_SMP12_1   ((uint32_t)0x00000080) /*!< ADC SMP12 bit 1 */
#define ADC_SMPR2_SMP12_2   ((uint32_t)0x00000100) /*!< ADC SMP12 bit 2 */

#define ADC_SMPR2_SMP13     ((uint32_t)0x00000E00) /*!< ADC Channel 13 Sampling time selection  */
#define ADC_SMPR2_SMP13_0   ((uint32_t)0x00000200) /*!< ADC SMP13 bit 0 */
#define ADC_SMPR2_SMP13_1   ((uint32_t)0x00000400) /*!< ADC SMP13 bit 1 */
#define ADC_SMPR2_SMP13_2   ((uint32_t)0x00000800) /*!< ADC SMP13 bit 2 */

#define ADC_SMPR2_SMP14     ((uint32_t)0x00007000) /*!< ADC Channel 14 Sampling time selection  */
#define ADC_SMPR2_SMP14_0   ((uint32_t)0x00001000) /*!< ADC SMP14 bit 0 */
#define ADC_SMPR2_SMP14_1   ((uint32_t)0x00002000) /*!< ADC SMP14 bit 1 */
#define ADC_SMPR2_SMP14_2   ((uint32_t)0x00004000) /*!< ADC SMP14 bit 2 */

#define ADC_SMPR2_SMP15     ((uint32_t)0x00038000) /*!< ADC Channel 15 Sampling time selection  */
#define ADC_SMPR2_SMP15_0   ((uint32_t)0x00008000) /*!< ADC SMP15 bit 0 */
#define ADC_SMPR2_SMP15_1   ((uint32_t)0x00010000) /*!< ADC SMP15 bit 1 */
#define ADC_SMPR2_SMP15_2   ((uint32_t)0x00020000) /*!< ADC SMP15 bit 2 */

#define ADC_SMPR2_SMP16     ((uint32_t)0x001C0000) /*!< ADC Channel 16 Sampling time selection  */
#define ADC_SMPR2_SMP16_0   ((uint32_t)0x00040000) /*!< ADC SMP16 bit 0 */
#define ADC_SMPR2_SMP16_1   ((uint32_t)0x00080000) /*!< ADC SMP16 bit 1 */
#define ADC_SMPR2_SMP16_2   ((uint32_t)0x00100000) /*!< ADC SMP16 bit 2 */

#define ADC_SMPR2_SMP17     ((uint32_t)0x00E00000) /*!< ADC Channel 17 Sampling time selection  */
#define ADC_SMPR2_SMP17_0   ((uint32_t)0x00200000) /*!< ADC SMP17 bit 0 */
#define ADC_SMPR2_SMP17_1   ((uint32_t)0x00400000) /*!< ADC SMP17 bit 1 */
#define ADC_SMPR2_SMP17_2   ((uint32_t)0x00800000) /*!< ADC SMP17 bit 2 */

#define ADC_SMPR2_SMP18     ((uint32_t)0x07000000) /*!< ADC Channel 18 Sampling time selection  */
#define ADC_SMPR2_SMP18_0   ((uint32_t)0x01000000) /*!< ADC SMP18 bit 0 */
#define ADC_SMPR2_SMP18_1   ((uint32_t)0x02000000) /*!< ADC SMP18 bit 1 */
#define ADC_SMPR2_SMP18_2   ((uint32_t)0x04000000) /*!< ADC SMP18 bit 2 */

/********************  Bit definition for ADC_TR1 register  ********************/
#define ADC_TR1_LT1         ((uint32_t)0x00000FFF) /*!< ADC Analog watchdog 1 lower threshold */
#define ADC_TR1_LT1_0       ((uint32_t)0x00000001) /*!< ADC LT1 bit 0 */
#define ADC_TR1_LT1_1       ((uint32_t)0x00000002) /*!< ADC LT1 bit 1 */
#define ADC_TR1_LT1_2       ((uint32_t)0x00000004) /*!< ADC LT1 bit 2 */
#define ADC_TR1_LT1_3       ((uint32_t)0x00000008) /*!< ADC LT1 bit 3 */
#define ADC_TR1_LT1_4       ((uint32_t)0x00000010) /*!< ADC LT1 bit 4 */
#define ADC_TR1_LT1_5       ((uint32_t)0x00000020) /*!< ADC LT1 bit 5 */
#define ADC_TR1_LT1_6       ((uint32_t)0x00000040) /*!< ADC LT1 bit 6 */
#define ADC_TR1_LT1_7       ((uint32_t)0x00000080) /*!< ADC LT1 bit 7 */
#define ADC_TR1_LT1_8       ((uint32_t)0x00000100) /*!< ADC LT1 bit 8 */
#define ADC_TR1_LT1_9       ((uint32_t)0x00000200) /*!< ADC LT1 bit 9 */
#define ADC_TR1_LT1_10      ((uint32_t)0x00000400) /*!< ADC LT1 bit 10 */
#define ADC_TR1_LT1_11      ((uint32_t)0x00000800) /*!< ADC LT1 bit 11 */

#define ADC_TR1_HT1         ((uint32_t)0x0FFF0000) /*!< ADC Analog watchdog 1 higher threshold */
#define ADC_TR1_HT1_0       ((uint32_t)0x00010000) /*!< ADC HT1 bit 0 */
#define ADC_TR1_HT1_1       ((uint32_t)0x00020000) /*!< ADC HT1 bit 1 */
#define ADC_TR1_HT1_2       ((uint32_t)0x00040000) /*!< ADC HT1 bit 2 */
#define ADC_TR1_HT1_3       ((uint32_t)0x00080000) /*!< ADC HT1 bit 3 */
#define ADC_TR1_HT1_4       ((uint32_t)0x00100000) /*!< ADC HT1 bit 4 */
#define ADC_TR1_HT1_5       ((uint32_t)0x00200000) /*!< ADC HT1 bit 5 */
#define ADC_TR1_HT1_6       ((uint32_t)0x00400000) /*!< ADC HT1 bit 6 */
#define ADC_TR1_HT1_7       ((uint32_t)0x00800000) /*!< ADC HT1 bit 7 */
#define ADC_TR1_HT1_8       ((uint32_t)0x01000000) /*!< ADC HT1 bit 8 */
#define ADC_TR1_HT1_9       ((uint32_t)0x02000000) /*!< ADC HT1 bit 9 */
#define ADC_TR1_HT1_10      ((uint32_t)0x04000000) /*!< ADC HT1 bit 10 */
#define ADC_TR1_HT1_11      ((uint32_t)0x08000000) /*!< ADC HT1 bit 11 */

/********************  Bit definition for ADC_TR2 register  ********************/
#define ADC_TR2_LT2         ((uint32_t)0x000000FF) /*!< ADC Analog watchdog 2 lower threshold */
#define ADC_TR2_LT2_0       ((uint32_t)0x00000001) /*!< ADC LT2 bit 0 */
#define ADC_TR2_LT2_1       ((uint32_t)0x00000002) /*!< ADC LT2 bit 1 */
#define ADC_TR2_LT2_2       ((uint32_t)0x00000004) /*!< ADC LT2 bit 2 */
#define ADC_TR2_LT2_3       ((uint32_t)0x00000008) /*!< ADC LT2 bit 3 */
#define ADC_TR2_LT2_4       ((uint32_t)0x00000010) /*!< ADC LT2 bit 4 */
#define ADC_TR2_LT2_5       ((uint32_t)0x00000020) /*!< ADC LT2 bit 5 */
#define ADC_TR2_LT2_6       ((uint32_t)0x00000040) /*!< ADC LT2 bit 6 */
#define ADC_TR2_LT2_7       ((uint32_t)0x00000080) /*!< ADC LT2 bit 7 */

#define ADC_TR2_HT2         ((uint32_t)0x00FF0000) /*!< ADC Analog watchdog 2 higher threshold */
#define ADC_TR2_HT2_0       ((uint32_t)0x00010000) /*!< ADC HT2 bit 0 */
#define ADC_TR2_HT2_1       ((uint32_t)0x00020000) /*!< ADC HT2 bit 1 */
#define ADC_TR2_HT2_2       ((uint32_t)0x00040000) /*!< ADC HT2 bit 2 */
#define ADC_TR2_HT2_3       ((uint32_t)0x00080000) /*!< ADC HT2 bit 3 */
#define ADC_TR2_HT2_4       ((uint32_t)0x00100000) /*!< ADC HT2 bit 4 */
#define ADC_TR2_HT2_5       ((uint32_t)0x00200000) /*!< ADC HT2 bit 5 */
#define ADC_TR2_HT2_6       ((uint32_t)0x00400000) /*!< ADC HT2 bit 6 */
#define ADC_TR2_HT2_7       ((uint32_t)0x00800000) /*!< ADC HT2 bit 7 */

/********************  Bit definition for ADC_TR3 register  ********************/
#define ADC_TR3_LT3         ((uint32_t)0x000000FF) /*!< ADC Analog watchdog 3 lower threshold */
#define ADC_TR3_LT3_0       ((uint32_t)0x00000001) /*!< ADC LT3 bit 0 */
#define ADC_TR3_LT3_1       ((uint32_t)0x00000002) /*!< ADC LT3 bit 1 */
#define ADC_TR3_LT3_2       ((uint32_t)0x00000004) /*!< ADC LT3 bit 2 */
#define ADC_TR3_LT3_3       ((uint32_t)0x00000008) /*!< ADC LT3 bit 3 */
#define ADC_TR3_LT3_4       ((uint32_t)0x00000010) /*!< ADC LT3 bit 4 */
#define ADC_TR3_LT3_5       ((uint32_t)0x00000020) /*!< ADC LT3 bit 5 */
#define ADC_TR3_LT3_6       ((uint32_t)0x00000040) /*!< ADC LT3 bit 6 */
#define ADC_TR3_LT3_7       ((uint32_t)0x00000080) /*!< ADC LT3 bit 7 */

#define ADC_TR3_HT3         ((uint32_t)0x00FF0000) /*!< ADC Analog watchdog 3 higher threshold */
#define ADC_TR3_HT3_0       ((uint32_t)0x00010000) /*!< ADC HT3 bit 0 */
#define ADC_TR3_HT3_1       ((uint32_t)0x00020000) /*!< ADC HT3 bit 1 */
#define ADC_TR3_HT3_2       ((uint32_t)0x00040000) /*!< ADC HT3 bit 2 */
#define ADC_TR3_HT3_3       ((uint32_t)0x00080000) /*!< ADC HT3 bit 3 */
#define ADC_TR3_HT3_4       ((uint32_t)0x00100000) /*!< ADC HT3 bit 4 */
#define ADC_TR3_HT3_5       ((uint32_t)0x00200000) /*!< ADC HT3 bit 5 */
#define ADC_TR3_HT3_6       ((uint32_t)0x00400000) /*!< ADC HT3 bit 6 */
#define ADC_TR3_HT3_7       ((uint32_t)0x00800000) /*!< ADC HT3 bit 7 */

/********************  Bit definition for ADC_SQR1 register  ********************/
#define ADC_SQR1_L          ((uint32_t)0x0000000F) /*!< ADC regular channel sequence length */
#define ADC_SQR1_SQ1        ((uint32_t)0x000003C0) /*!< ADC 1st conversion in regular sequence */
#define ADC_SQR1_SQ2        ((uint32_t)0x0000F000) /*!< ADC 2nd conversion in regular sequence */
#define ADC_SQR1_SQ3        ((uint32_t)0x003C0000) /*!< ADC 3rd conversion in regular sequence */
#define ADC_SQR1_SQ4        ((uint32_t)0x0F000000) /*!< ADC 4th conversion in regular sequence */

/********************  Bit definition for ADC_SQR2 register  ********************/
#define ADC_SQR2_SQ5        ((uint32_t)0x0000000F) /*!< ADC 5th conversion in regular sequence */
#define ADC_SQR2_SQ6        ((uint32_t)0x000003C0) /*!< ADC 6th conversion in regular sequence */
#define ADC_SQR2_SQ7        ((uint32_t)0x0000F000) /*!< ADC 7th conversion in regular sequence */
#define ADC_SQR2_SQ8        ((uint32_t)0x003C0000) /*!< ADC 8th conversion in regular sequence */
#define ADC_SQR2_SQ9        ((uint32_t)0x0F000000) /*!< ADC 9th conversion in regular sequence */

/********************  Bit definition for ADC_SQR3 register  ********************/
#define ADC_SQR3_SQ10       ((uint32_t)0x0000000F) /*!< ADC 10th conversion in regular sequence */
#define ADC_SQR3_SQ11       ((uint32_t)0x000003C0) /*!< ADC 11th conversion in regular sequence */
#define ADC_SQR3_SQ12       ((uint32_t)0x0000F000) /*!< ADC 12th conversion in regular sequence */
#define ADC_SQR3_SQ13       ((uint32_t)0x003C0000) /*!< ADC 13th conversion in regular sequence */
#define ADC_SQR3_SQ14       ((uint32_t)0x0F000000) /*!< ADC 14th conversion in regular sequence */

/********************  Bit definition for ADC_SQR4 register  ********************/
#define ADC_SQR4_SQ15       ((uint32_t)0x0000000F) /*!< ADC 15th conversion in regular sequence */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_RDATA        ((uint32_t)0x0000FFFF) /*!< ADC regular Data converted */
#define ADC_DR_RDATA_0      ((uint32_t)0x00000001) /*!< ADC RDATA bit 0 */
#define ADC_DR_RDATA_1      ((uint32_t)0x00000002) /*!< ADC RDATA bit 1 */
#define ADC_DR_RDATA_2      ((uint32_t)0x00000004) /*!< ADC RDATA bit 2 */
#define ADC_DR_RDATA_3      ((uint32_t)0x00000008) /*!< ADC RDATA bit 3 */
#define ADC_DR_RDATA_4      ((uint32_t)0x00000010) /*!< ADC RDATA bit 4 */
#define ADC_DR_RDATA_5      ((uint32_t)0x00000020) /*!< ADC RDATA bit 5 */
#define ADC_DR_RDATA_6      ((uint32_t)0x00000040) /*!< ADC RDATA bit 6 */
#define ADC_DR_RDATA_7      ((uint32_t)0x00000080) /*!< ADC RDATA bit 7 */
#define ADC_DR_RDATA_8      ((uint32_t)0x00000100) /*!< ADC RDATA bit 8 */
#define ADC_DR_RDATA_9      ((uint32_t)0x00000200) /*!< ADC RDATA bit 9 */
#define ADC_DR_RDATA_10     ((uint32_t)0x00000400) /*!< ADC RDATA bit 10 */
#define ADC_DR_RDATA_11     ((uint32_t)0x00000800) /*!< ADC RDATA bit 11 */
#define ADC_DR_RDATA_12     ((uint32_t)0x00001000) /*!< ADC RDATA bit 12 */
#define ADC_DR_RDATA_13     ((uint32_t)0x00002000) /*!< ADC RDATA bit 13 */
#define ADC_DR_RDATA_14     ((uint32_t)0x00004000) /*!< ADC RDATA bit 14 */
#define ADC_DR_RDATA_15     ((uint32_t)0x00008000) /*!< ADC RDATA bit 15 */

/********************  Bit definition for ADC_JSQR register  ********************/
#define ADC_JSQR_JL         ((uint32_t)0x00000003) /*!< ADC injected channel sequence length */
#define ADC_JSQR_JL_0       ((uint32_t)0x00000001) /*!< ADC JL bit 0 */
#define ADC_JSQR_JL_1       ((uint32_t)0x00000002) /*!< ADC JL bit 1 */

#define ADC_JSQR_JEXTSEL    ((uint32_t)0x0000003C) /*!< ADC external trigger selection for injected group */
#define ADC_JSQR_JEXTSEL_0  ((uint32_t)0x00000004) /*!< ADC JEXTSEL bit 0 */
#define ADC_JSQR_JEXTSEL_1  ((uint32_t)0x00000008) /*!< ADC JEXTSEL bit 1 */
#define ADC_JSQR_JEXTSEL_2  ((uint32_t)0x00000010) /*!< ADC JEXTSEL bit 2 */
#define ADC_JSQR_JEXTSEL_3  ((uint32_t)0x00000020) /*!< ADC JEXTSEL bit 3 */

#define ADC_JSQR_JEXTEN     ((uint32_t)0x000000C0) /*!< ADC external trigger enable and polarity selection for injected channels */
#define ADC_JSQR_JEXTEN_0   ((uint32_t)0x00000040) /*!< ADC JEXTEN bit 0 */
#define ADC_JSQR_JEXTEN_1   ((uint32_t)0x00000080) /*!< ADC JEXTEN bit 1 */

#define ADC_JSQR_JSQ1       ((uint32_t)0x00001F00) /*!< ADC 1st conversion in injected sequence */
#define ADC_JSQR_JSQ1_0     ((uint32_t)0x00000100) /*!< ADC JSQ1 bit 0 */
#define ADC_JSQR_JSQ1_1     ((uint32_t)0x00000200) /*!< ADC JSQ1 bit 1 */
#define ADC_JSQR_JSQ1_2     ((uint32_t)0x00000400) /*!< ADC JSQ1 bit 2 */
#define ADC_JSQR_JSQ1_3     ((uint32_t)0x00000800) /*!< ADC JSQ1 bit 3 */
#define ADC_JSQR_JSQ1_4     ((uint32_t)0x00001000) /*!< ADC JSQ1 bit 4 */

#define ADC_JSQR_JSQ2       ((uint32_t)0x0007C000) /*!< ADC 2nd conversion in injected sequence */
#define ADC_JSQR_JSQ2_0     ((uint32_t)0x00004000) /*!< ADC JSQ2 bit 0 */
#define ADC_JSQR_JSQ2_1     ((uint32_t)0x00008000) /*!< ADC JSQ2 bit 1 */
#define ADC_JSQR_JSQ2_2     ((uint32_t)0x00010000) /*!< ADC JSQ2 bit 2 */
#define ADC_JSQR_JSQ2_3     ((uint32_t)0x00020000) /*!< ADC JSQ2 bit 3 */
#define ADC_JSQR_JSQ2_4     ((uint32_t)0x00040000) /*!< ADC JSQ2 bit 4 */

#define ADC_JSQR_JSQ3       ((uint32_t)0x01F00000) /*!< ADC 3rd conversion in injected sequence */
#define ADC_JSQR_JSQ3_0     ((uint32_t)0x00100000) /*!< ADC JSQ3 bit 0 */
#define ADC_JSQR_JSQ3_1     ((uint32_t)0x00200000) /*!< ADC JSQ3 bit 1 */
#define ADC_JSQR_JSQ3_2     ((uint32_t)0x00400000) /*!< ADC JSQ3 bit 2 */
#define ADC_JSQR_JSQ3_3     ((uint32_t)0x00800000) /*!< ADC JSQ3 bit 3 */
#define ADC_JSQR_JSQ3_4     ((uint32_t)0x01000000) /*!< ADC JSQ3 bit 4 */

#define ADC_JSQR_JSQ4       ((uint32_t)0x7C000000) /*!< ADC 4th conversion in injected sequence */
#define ADC_JSQR_JSQ4_0     ((uint32_t)0x04000000) /*!< ADC JSQ4 bit 0 */
#define ADC_JSQR_JSQ4_1     ((uint32_t)0x08000000) /*!< ADC JSQ4 bit 1 */
#define ADC_JSQR_JSQ4_2     ((uint32_t)0x10000000) /*!< ADC JSQ4 bit 2 */
#define ADC_JSQR_JSQ4_3     ((uint32_t)0x20000000) /*!< ADC JSQ4 bit 3 */
#define ADC_JSQR_JSQ4_4     ((uint32_t)0x40000000) /*!< ADC JSQ4 bit 4 */

/********************  Bit definition for ADC_OFR1 register  ********************/
#define ADC_OFR1_OFFSET1    ((uint32_t)0x00000FFF) /*!< ADC data offset 1 for channel programmed into bits OFFSET1_CH[4:0] */
#define ADC_OFR1_OFFSET1_0  ((uint32_t)0x00000001) /*!< ADC OFFSET1 bit 0 */
#define ADC_OFR1_OFFSET1_1  ((uint32_t)0x00000002) /*!< ADC OFFSET1 bit 1 */
#define ADC_OFR1_OFFSET1_2  ((uint32_t)0x00000004) /*!< ADC OFFSET1 bit 2 */
#define ADC_OFR1_OFFSET1_3  ((uint32_t)0x00000008) /*!< ADC OFFSET1 bit 3 */
#define ADC_OFR1_OFFSET1_4  ((uint32_t)0x00000010) /*!< ADC OFFSET1 bit 4 */
#define ADC_OFR1_OFFSET1_5  ((uint32_t)0x00000020) /*!< ADC OFFSET1 bit 5 */
#define ADC_OFR1_OFFSET1_6  ((uint32_t)0x00000040) /*!< ADC OFFSET1 bit 6 */
#define ADC_OFR1_OFFSET1_7  ((uint32_t)0x00000080) /*!< ADC OFFSET1 bit 7 */
#define ADC_OFR1_OFFSET1_8  ((uint32_t)0x00000100) /*!< ADC OFFSET1 bit 8 */
#define ADC_OFR1_OFFSET1_9  ((uint32_t)0x00000200) /*!< ADC OFFSET1 bit 9 */
#define ADC_OFR1_OFFSET1_10 ((uint32_t)0x00000400) /*!< ADC OFFSET1 bit 10 */
#define ADC_OFR1_OFFSET1_11 ((uint32_t)0x00000800) /*!< ADC OFFSET1 bit 11 */

#define ADC_OFR1_OFFSET1_CH     ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 1 */
#define ADC_OFR1_OFFSET1_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET1_CH bit 0 */
#define ADC_OFR1_OFFSET1_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET1_CH bit 1 */
#define ADC_OFR1_OFFSET1_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET1_CH bit 2 */
#define ADC_OFR1_OFFSET1_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET1_CH bit 3 */
#define ADC_OFR1_OFFSET1_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET1_CH bit 4 */

#define ADC_OFR1_OFFSET1_EN ((uint32_t)0x80000000) /*!< ADC offset 1 enable */

/********************  Bit definition for ADC_OFR2 register  ********************/
#define ADC_OFR2_OFFSET2    ((uint32_t)0x00000FFF) /*!< ADC data offset 2 for channel programmed into bits OFFSET2_CH[4:0] */
#define ADC_OFR2_OFFSET2_0  ((uint32_t)0x00000001) /*!< ADC OFFSET2 bit 0 */
#define ADC_OFR2_OFFSET2_1  ((uint32_t)0x00000002) /*!< ADC OFFSET2 bit 1 */
#define ADC_OFR2_OFFSET2_2  ((uint32_t)0x00000004) /*!< ADC OFFSET2 bit 2 */
#define ADC_OFR2_OFFSET2_3  ((uint32_t)0x00000008) /*!< ADC OFFSET2 bit 3 */
#define ADC_OFR2_OFFSET2_4  ((uint32_t)0x00000010) /*!< ADC OFFSET2 bit 4 */
#define ADC_OFR2_OFFSET2_5  ((uint32_t)0x00000020) /*!< ADC OFFSET2 bit 5 */
#define ADC_OFR2_OFFSET2_6  ((uint32_t)0x00000040) /*!< ADC OFFSET2 bit 6 */
#define ADC_OFR2_OFFSET2_7  ((uint32_t)0x00000080) /*!< ADC OFFSET2 bit 7 */
#define ADC_OFR2_OFFSET2_8  ((uint32_t)0x00000100) /*!< ADC OFFSET2 bit 8 */
#define ADC_OFR2_OFFSET2_9  ((uint32_t)0x00000200) /*!< ADC OFFSET2 bit 9 */
#define ADC_OFR2_OFFSET2_10 ((uint32_t)0x00000400) /*!< ADC OFFSET2 bit 10 */
#define ADC_OFR2_OFFSET2_11 ((uint32_t)0x00000800) /*!< ADC OFFSET2 bit 11 */

#define ADC_OFR2_OFFSET2_CH     ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 2 */
#define ADC_OFR2_OFFSET2_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET2_CH bit 0 */
#define ADC_OFR2_OFFSET2_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET2_CH bit 1 */
#define ADC_OFR2_OFFSET2_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET2_CH bit 2 */
#define ADC_OFR2_OFFSET2_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET2_CH bit 3 */
#define ADC_OFR2_OFFSET2_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET2_CH bit 4 */

#define ADC_OFR2_OFFSET2_EN ((uint32_t)0x80000000) /*!< ADC offset 2 enable */

/********************  Bit definition for ADC_OFR3 register  ********************/
#define ADC_OFR3_OFFSET3    ((uint32_t)0x00000FFF) /*!< ADC data offset 3 for channel programmed into bits OFFSET3_CH[4:0] */
#define ADC_OFR3_OFFSET3_0  ((uint32_t)0x00000001) /*!< ADC OFFSET3 bit 0 */
#define ADC_OFR3_OFFSET3_1  ((uint32_t)0x00000002) /*!< ADC OFFSET3 bit 1 */
#define ADC_OFR3_OFFSET3_2  ((uint32_t)0x00000004) /*!< ADC OFFSET3 bit 2 */
#define ADC_OFR3_OFFSET3_3  ((uint32_t)0x00000008) /*!< ADC OFFSET3 bit 3 */
#define ADC_OFR3_OFFSET3_4  ((uint32_t)0x00000010) /*!< ADC OFFSET3 bit 4 */
#define ADC_OFR3_OFFSET3_5  ((uint32_t)0x00000020) /*!< ADC OFFSET3 bit 5 */
#define ADC_OFR3_OFFSET3_6  ((uint32_t)0x00000040) /*!< ADC OFFSET3 bit 6 */
#define ADC_OFR3_OFFSET3_7  ((uint32_t)0x00000080) /*!< ADC OFFSET3 bit 7 */
#define ADC_OFR3_OFFSET3_8  ((uint32_t)0x00000100) /*!< ADC OFFSET3 bit 8 */
#define ADC_OFR3_OFFSET3_9  ((uint32_t)0x00000200) /*!< ADC OFFSET3 bit 9 */
#define ADC_OFR3_OFFSET3_10 ((uint32_t)0x00000400) /*!< ADC OFFSET3 bit 10 */
#define ADC_OFR3_OFFSET3_11 ((uint32_t)0x00000800) /*!< ADC OFFSET3 bit 11 */

#define ADC_OFR3_OFFSET3_CH     ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 3 */
#define ADC_OFR3_OFFSET3_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET3_CH bit 0 */
#define ADC_OFR3_OFFSET3_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET3_CH bit 1 */
#define ADC_OFR3_OFFSET3_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET3_CH bit 2 */
#define ADC_OFR3_OFFSET3_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET3_CH bit 3 */
#define ADC_OFR3_OFFSET3_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET3_CH bit 4 */

#define ADC_OFR3_OFFSET3_EN ((uint32_t)0x80000000) /*!< ADC offset 3 enable */

/********************  Bit definition for ADC_OFR4 register  ********************/
#define ADC_OFR4_OFFSET4    ((uint32_t)0x00000FFF) /*!< ADC data offset 4 for channel programmed into bits OFFSET4_CH[4:0] */
#define ADC_OFR4_OFFSET4_0  ((uint32_t)0x00000001) /*!< ADC OFFSET4 bit 0 */
#define ADC_OFR4_OFFSET4_1  ((uint32_t)0x00000002) /*!< ADC OFFSET4 bit 1 */
#define ADC_OFR4_OFFSET4_2  ((uint32_t)0x00000004) /*!< ADC OFFSET4 bit 2 */
#define ADC_OFR4_OFFSET4_3  ((uint32_t)0x00000008) /*!< ADC OFFSET4 bit 3 */
#define ADC_OFR4_OFFSET4_4  ((uint32_t)0x00000010) /*!< ADC OFFSET4 bit 4 */
#define ADC_OFR4_OFFSET4_5  ((uint32_t)0x00000020) /*!< ADC OFFSET4 bit 5 */
#define ADC_OFR4_OFFSET4_6  ((uint32_t)0x00000040) /*!< ADC OFFSET4 bit 6 */
#define ADC_OFR4_OFFSET4_7  ((uint32_t)0x00000080) /*!< ADC OFFSET4 bit 7 */
#define ADC_OFR4_OFFSET4_8  ((uint32_t)0x00000100) /*!< ADC OFFSET4 bit 8 */
#define ADC_OFR4_OFFSET4_9  ((uint32_t)0x00000200) /*!< ADC OFFSET4 bit 9 */
#define ADC_OFR4_OFFSET4_10 ((uint32_t)0x00000400) /*!< ADC OFFSET4 bit 10 */
#define ADC_OFR4_OFFSET4_11 ((uint32_t)0x00000800) /*!< ADC OFFSET4 bit 11 */

#define ADC_OFR4_OFFSET4_CH     ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 4 */
#define ADC_OFR4_OFFSET4_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET4_CH bit 0 */
#define ADC_OFR4_OFFSET4_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET4_CH bit 1 */
#define ADC_OFR4_OFFSET4_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET4_CH bit 2 */
#define ADC_OFR4_OFFSET4_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET4_CH bit 3 */
#define ADC_OFR4_OFFSET4_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET4_CH bit 4 */

#define ADC_OFR4_OFFSET4_EN ((uint32_t)0x80000000) /*!< ADC offset 4 enable */

/********************  Bit definition for ADC_JDR1 register  ********************/
#define ADC_JDR1_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR1_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR1_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR1_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR1_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR1_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR1_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR1_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR1_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR1_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR1_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR1_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR1_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR1_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR1_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR1_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR1_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR2 register  ********************/
#define ADC_JDR2_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR2_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR2_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR2_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR2_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR2_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR2_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR2_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR2_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR2_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR2_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR2_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR2_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR2_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR2_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR2_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR2_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR3 register  ********************/
#define ADC_JDR3_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR3_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR3_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR3_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR3_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR3_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR3_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR3_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR3_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR3_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR3_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR3_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR3_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR3_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR3_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR3_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR3_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR4 register  ********************/
#define ADC_JDR4_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR4_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR4_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR4_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR4_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR4_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR4_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR4_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR4_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR4_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR4_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR4_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR4_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR4_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR4_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR4_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR4_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_AWD2CR register  ********************/
#define ADC_AWD2CR_AWD2CH    ((uint32_t)0x0007FFFE) /*!< ADC Analog watchdog 2 channel selection */
#define ADC_AWD2CR_AWD2CH_0  ((uint32_t)0x00000002) /*!< ADC AWD2CH bit 0 */
#define ADC_AWD2CR_AWD2CH_1  ((uint32_t)0x00000004) /*!< ADC AWD2CH bit 1 */
#define ADC_AWD2CR_AWD2CH_2  ((uint32_t)0x00000008) /*!< ADC AWD2CH bit 2 */
#define ADC_AWD2CR_AWD2CH_3  ((uint32_t)0x00000010) /*!< ADC AWD2CH bit 3 */
#define ADC_AWD2CR_AWD2CH_4  ((uint32_t)0x00000020) /*!< ADC AWD2CH bit 4 */
#define ADC_AWD2CR_AWD2CH_5  ((uint32_t)0x00000040) /*!< ADC AWD2CH bit 5 */
#define ADC_AWD2CR_AWD2CH_6  ((uint32_t)0x00000080) /*!< ADC AWD2CH bit 6 */
#define ADC_AWD2CR_AWD2CH_7  ((uint32_t)0x00000100) /*!< ADC AWD2CH bit 7 */
#define ADC_AWD2CR_AWD2CH_8  ((uint32_t)0x00000200) /*!< ADC AWD2CH bit 8 */
#define ADC_AWD2CR_AWD2CH_9  ((uint32_t)0x00000400) /*!< ADC AWD2CH bit 9 */
#define ADC_AWD2CR_AWD2CH_10 ((uint32_t)0x00000800) /*!< ADC AWD2CH bit 10 */
#define ADC_AWD2CR_AWD2CH_11 ((uint32_t)0x00001000) /*!< ADC AWD2CH bit 11 */
#define ADC_AWD2CR_AWD2CH_12 ((uint32_t)0x00002000) /*!< ADC AWD2CH bit 12 */
#define ADC_AWD2CR_AWD2CH_13 ((uint32_t)0x00004000) /*!< ADC AWD2CH bit 13 */
#define ADC_AWD2CR_AWD2CH_14 ((uint32_t)0x00008000) /*!< ADC AWD2CH bit 14 */
#define ADC_AWD2CR_AWD2CH_15 ((uint32_t)0x00010000) /*!< ADC AWD2CH bit 15 */
#define ADC_AWD2CR_AWD2CH_16 ((uint32_t)0x00020000) /*!< ADC AWD2CH bit 16 */
#define ADC_AWD2CR_AWD2CH_17 ((uint32_t)0x00030000) /*!< ADC AWD2CH bit 17 */

/********************  Bit definition for ADC_AWD3CR register  ********************/
#define ADC_AWD3CR_AWD3CH    ((uint32_t)0x0007FFFE) /*!< ADC Analog watchdog 2 channel selection */
#define ADC_AWD3CR_AWD3CH_0  ((uint32_t)0x00000002) /*!< ADC AWD3CH bit 0 */
#define ADC_AWD3CR_AWD3CH_1  ((uint32_t)0x00000004) /*!< ADC AWD3CH bit 1 */
#define ADC_AWD3CR_AWD3CH_2  ((uint32_t)0x00000008) /*!< ADC AWD3CH bit 2 */
#define ADC_AWD3CR_AWD3CH_3  ((uint32_t)0x00000010) /*!< ADC AWD3CH bit 3 */
#define ADC_AWD3CR_AWD3CH_4  ((uint32_t)0x00000020) /*!< ADC AWD3CH bit 4 */
#define ADC_AWD3CR_AWD3CH_5  ((uint32_t)0x00000040) /*!< ADC AWD3CH bit 5 */
#define ADC_AWD3CR_AWD3CH_6  ((uint32_t)0x00000080) /*!< ADC AWD3CH bit 6 */
#define ADC_AWD3CR_AWD3CH_7  ((uint32_t)0x00000100) /*!< ADC AWD3CH bit 7 */
#define ADC_AWD3CR_AWD3CH_8  ((uint32_t)0x00000200) /*!< ADC AWD3CH bit 8 */
#define ADC_AWD3CR_AWD3CH_9  ((uint32_t)0x00000400) /*!< ADC AWD3CH bit 9 */
#define ADC_AWD3CR_AWD3CH_10 ((uint32_t)0x00000800) /*!< ADC AWD3CH bit 10 */
#define ADC_AWD3CR_AWD3CH_11 ((uint32_t)0x00001000) /*!< ADC AWD3CH bit 11 */
#define ADC_AWD3CR_AWD3CH_12 ((uint32_t)0x00002000) /*!< ADC AWD3CH bit 12 */
#define ADC_AWD3CR_AWD3CH_13 ((uint32_t)0x00004000) /*!< ADC AWD3CH bit 13 */
#define ADC_AWD3CR_AWD3CH_14 ((uint32_t)0x00008000) /*!< ADC AWD3CH bit 14 */
#define ADC_AWD3CR_AWD3CH_15 ((uint32_t)0x00010000) /*!< ADC AWD3CH bit 15 */
#define ADC_AWD3CR_AWD3CH_16 ((uint32_t)0x00020000) /*!< ADC AWD3CH bit 16 */
#define ADC_AWD3CR_AWD3CH_17 ((uint32_t)0x00030000) /*!< ADC AWD3CH bit 17 */

/********************  Bit definition for ADC_CALFACT register  ********************/
#define ADC_CALFACT_CH0_Pos        (0U)
#define ADC_CALFACT_CH0_Msk        (0x7FU << ADC_CALFACT_CH0_Pos)        /*!< 0x0000007F */
#define ADC_CALFACT_CH0            ADC_CALFACT_CH0_Msk 

#define ADC_CALFACT_CH1_Pos        (16U)
#define ADC_CALFACT_CH1_Msk        (0x7FU << ADC_CALFACT_CH1_Pos)        /*!< 0x007F0000 */
#define ADC_CALFACT_CH1            ADC_CALFACT_CH1_Msk 

#define ADC_CCR_REFSEL_Pos                     (16U)                      
#define ADC_CCR_REFSEL_Msk                     (0x3U << ADC_CCR_REFSEL_Pos)
#define ADC_CCR_REFSEL                         ADC_CCR_REFSEL_Msk
#define ADC_CCR_REFSEL_VDD                     (0x0U << ADC_CCR_REFSEL_Pos)
#define ADC_CCR_REFSEL_V24                     (0x2U << ADC_CCR_REFSEL_Pos)
#define ADC_CCR_REFSEL_V36                     (0x3U << ADC_CCR_REFSEL_Pos)

/******************************************************************************/
/*                                                                            */
/*                           Debug MCU (DBGMCU)                               */			//??????????????????????
/*                                                                            */
/******************************************************************************/

/****************  Bit definition for DBGMCU_IDCODE register  *****************/
#define DBGMCU_IDCODE_DEV_ID_Pos                     (0U)                      
#define DBGMCU_IDCODE_DEV_ID_Msk                     (0xFFFU << DBGMCU_IDCODE_DEV_ID_Pos) /*!< 0x00000FFF */
#define DBGMCU_IDCODE_DEV_ID                         DBGMCU_IDCODE_DEV_ID_Msk  /*!< Device Identifier */

#define DBGMCU_IDCODE_REV_ID_Pos                     (16U)                     
#define DBGMCU_IDCODE_REV_ID_Msk                     (0xFFFFU << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0xFFFF0000 */
#define DBGMCU_IDCODE_REV_ID                         DBGMCU_IDCODE_REV_ID_Msk  /*!< REV_ID[15:0] bits (Revision Identifier) */
#define DBGMCU_IDCODE_REV_ID_0                       (0x0001U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00010000 */
#define DBGMCU_IDCODE_REV_ID_1                       (0x0002U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00020000 */
#define DBGMCU_IDCODE_REV_ID_2                       (0x0004U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00040000 */
#define DBGMCU_IDCODE_REV_ID_3                       (0x0008U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00080000 */
#define DBGMCU_IDCODE_REV_ID_4                       (0x0010U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00100000 */
#define DBGMCU_IDCODE_REV_ID_5                       (0x0020U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00200000 */
#define DBGMCU_IDCODE_REV_ID_6                       (0x0040U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00400000 */
#define DBGMCU_IDCODE_REV_ID_7                       (0x0080U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00800000 */
#define DBGMCU_IDCODE_REV_ID_8                       (0x0100U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x01000000 */
#define DBGMCU_IDCODE_REV_ID_9                       (0x0200U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x02000000 */
#define DBGMCU_IDCODE_REV_ID_10                      (0x0400U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x04000000 */
#define DBGMCU_IDCODE_REV_ID_11                      (0x0800U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x08000000 */
#define DBGMCU_IDCODE_REV_ID_12                      (0x1000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x10000000 */
#define DBGMCU_IDCODE_REV_ID_13                      (0x2000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x20000000 */
#define DBGMCU_IDCODE_REV_ID_14                      (0x4000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x40000000 */
#define DBGMCU_IDCODE_REV_ID_15                      (0x8000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x80000000 */

/******************  Bit definition for DBGMCU_CR register  *******************/
#define DBGMCU_CR_DBG_STOP_Pos                       (1U)                      
#define DBGMCU_CR_DBG_STOP_Msk                       (0x1U << DBGMCU_CR_DBG_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_CR_DBG_STOP                           DBGMCU_CR_DBG_STOP_Msk    /*!< Debug Stop Mode */
#define DBGMCU_CR_DBG_STANDBY_Pos                    (2U)                      
#define DBGMCU_CR_DBG_STANDBY_Msk                    (0x1U << DBGMCU_CR_DBG_STANDBY_Pos) /*!< 0x00000004 */
#define DBGMCU_CR_DBG_STANDBY                        DBGMCU_CR_DBG_STANDBY_Msk /*!< Debug Standby mode */

/******************  Bit definition for DBGMCU_APB1_FZ register  **************/
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos             (0U)                      
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos) /*!< 0x00000001 */
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP                 DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk /*!< TIM2 counter stopped when core is halted */
#define DBGMCU_APB1_FZ_DBG_TIM0_STOP_Pos             (1U)                      
#define DBGMCU_APB1_FZ_DBG_TIM0_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM0_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_APB1_FZ_DBG_TIM0_STOP                 DBGMCU_APB1_FZ_DBG_TIM0_STOP_Msk /*!< TIM0 counter stopped when core is halted */
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos            (8U)                      
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk            (0x1U << DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos) /*!< 0x00000100 */
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP                DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk /*!< TIM14 counter stopped when core is halted */
#define DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos              (10U)                     
#define DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk              (0x1U << DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos) /*!< 0x00000400 */
#define DBGMCU_APB1_FZ_DBG_RTC_STOP                  DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk /*!< RTC Calendar frozen when core is halted */
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos             (11U)                     
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP                 DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk /*!< Debug Window Watchdog stopped when Core is halted */
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos             (12U)                     
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP                 DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk /*!< Debug Independent Watchdog stopped when Core is halted */
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos    (21U)                     
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk    (0x1U << DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos) /*!< 0x00200000 */
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk /*!< I2C1 SMBUS timeout mode stopped when Core is halted */

/******************  Bit definition for DBGMCU_APB2_FZ register  **************/
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos             (11U)                     
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk             (0x1U << DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP                 DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk /*!< TIM1 counter stopped when core is halted */
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos            (17U)                     
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk            (0x1U << DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos) /*!< 0x00020000 */
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP                DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk /*!< TIM16 counter stopped when core is halted */
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos            (18U)                     
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk            (0x1U << DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos) /*!< 0x00040000 */
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP                DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk /*!< TIM17 counter stopped when core is halted */

/******************************************************************************/
/*                                                                            */
/*                           DMA Controller (DMA)                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for DMA_ISR register  ********************/
#define DMA_ISR_GIF1_Pos       (0U)                                            
#define DMA_ISR_GIF1_Msk       (0x1U << DMA_ISR_GIF1_Pos)                      /*!< 0x00000001 */
#define DMA_ISR_GIF1           DMA_ISR_GIF1_Msk                                /*!< Channel 1 Global interrupt flag    */
#define DMA_ISR_TCIF1_Pos      (1U)                                            
#define DMA_ISR_TCIF1_Msk      (0x1U << DMA_ISR_TCIF1_Pos)                     /*!< 0x00000002 */
#define DMA_ISR_TCIF1          DMA_ISR_TCIF1_Msk                               /*!< Channel 1 Transfer Complete flag   */
#define DMA_ISR_HTIF1_Pos      (2U)                                            
#define DMA_ISR_HTIF1_Msk      (0x1U << DMA_ISR_HTIF1_Pos)                     /*!< 0x00000004 */
#define DMA_ISR_HTIF1          DMA_ISR_HTIF1_Msk                               /*!< Channel 1 Half Transfer flag       */
#define DMA_ISR_TEIF1_Pos      (3U)                                            
#define DMA_ISR_TEIF1_Msk      (0x1U << DMA_ISR_TEIF1_Pos)                     /*!< 0x00000008 */
#define DMA_ISR_TEIF1          DMA_ISR_TEIF1_Msk                               /*!< Channel 1 Transfer Error flag      */
#define DMA_ISR_GIF2_Pos       (4U)                                            
#define DMA_ISR_GIF2_Msk       (0x1U << DMA_ISR_GIF2_Pos)                      /*!< 0x00000010 */
#define DMA_ISR_GIF2           DMA_ISR_GIF2_Msk                                /*!< Channel 2 Global interrupt flag    */
#define DMA_ISR_TCIF2_Pos      (5U)                                            
#define DMA_ISR_TCIF2_Msk      (0x1U << DMA_ISR_TCIF2_Pos)                     /*!< 0x00000020 */
#define DMA_ISR_TCIF2          DMA_ISR_TCIF2_Msk                               /*!< Channel 2 Transfer Complete flag   */
#define DMA_ISR_HTIF2_Pos      (6U)                                            
#define DMA_ISR_HTIF2_Msk      (0x1U << DMA_ISR_HTIF2_Pos)                     /*!< 0x00000040 */
#define DMA_ISR_HTIF2          DMA_ISR_HTIF2_Msk                               /*!< Channel 2 Half Transfer flag       */
#define DMA_ISR_TEIF2_Pos      (7U)                                            
#define DMA_ISR_TEIF2_Msk      (0x1U << DMA_ISR_TEIF2_Pos)                     /*!< 0x00000080 */
#define DMA_ISR_TEIF2          DMA_ISR_TEIF2_Msk                               /*!< Channel 2 Transfer Error flag      */
#define DMA_ISR_GIF3_Pos       (8U)                                            
#define DMA_ISR_GIF3_Msk       (0x1U << DMA_ISR_GIF3_Pos)                      /*!< 0x00000100 */
#define DMA_ISR_GIF3           DMA_ISR_GIF3_Msk                                /*!< Channel 3 Global interrupt flag    */
#define DMA_ISR_TCIF3_Pos      (9U)                                            
#define DMA_ISR_TCIF3_Msk      (0x1U << DMA_ISR_TCIF3_Pos)                     /*!< 0x00000200 */
#define DMA_ISR_TCIF3          DMA_ISR_TCIF3_Msk                               /*!< Channel 3 Transfer Complete flag   */
#define DMA_ISR_HTIF3_Pos      (10U)                                           
#define DMA_ISR_HTIF3_Msk      (0x1U << DMA_ISR_HTIF3_Pos)                     /*!< 0x00000400 */
#define DMA_ISR_HTIF3          DMA_ISR_HTIF3_Msk                               /*!< Channel 3 Half Transfer flag       */
#define DMA_ISR_TEIF3_Pos      (11U)                                           
#define DMA_ISR_TEIF3_Msk      (0x1U << DMA_ISR_TEIF3_Pos)                     /*!< 0x00000800 */
#define DMA_ISR_TEIF3          DMA_ISR_TEIF3_Msk                               /*!< Channel 3 Transfer Error flag      */
#define DMA_ISR_GIF4_Pos       (12U)                                           
#define DMA_ISR_GIF4_Msk       (0x1U << DMA_ISR_GIF4_Pos)                      /*!< 0x00001000 */
#define DMA_ISR_GIF4           DMA_ISR_GIF4_Msk                                /*!< Channel 4 Global interrupt flag    */
#define DMA_ISR_TCIF4_Pos      (13U)                                           
#define DMA_ISR_TCIF4_Msk      (0x1U << DMA_ISR_TCIF4_Pos)                     /*!< 0x00002000 */
#define DMA_ISR_TCIF4          DMA_ISR_TCIF4_Msk                               /*!< Channel 4 Transfer Complete flag   */
#define DMA_ISR_HTIF4_Pos      (14U)                                           
#define DMA_ISR_HTIF4_Msk      (0x1U << DMA_ISR_HTIF4_Pos)                     /*!< 0x00004000 */
#define DMA_ISR_HTIF4          DMA_ISR_HTIF4_Msk                               /*!< Channel 4 Half Transfer flag       */
#define DMA_ISR_TEIF4_Pos      (15U)                                           
#define DMA_ISR_TEIF4_Msk      (0x1U << DMA_ISR_TEIF4_Pos)                     /*!< 0x00008000 */
#define DMA_ISR_TEIF4          DMA_ISR_TEIF4_Msk                               /*!< Channel 4 Transfer Error flag      */
#define DMA_ISR_GIF5_Pos       (16U)                                           
#define DMA_ISR_GIF5_Msk       (0x1U << DMA_ISR_GIF5_Pos)                      /*!< 0x00010000 */
#define DMA_ISR_GIF5           DMA_ISR_GIF5_Msk                                /*!< Channel 5 Global interrupt flag    */
#define DMA_ISR_TCIF5_Pos      (17U)                                           
#define DMA_ISR_TCIF5_Msk      (0x1U << DMA_ISR_TCIF5_Pos)                     /*!< 0x00020000 */
#define DMA_ISR_TCIF5          DMA_ISR_TCIF5_Msk                               /*!< Channel 5 Transfer Complete flag   */
#define DMA_ISR_HTIF5_Pos      (18U)                                           
#define DMA_ISR_HTIF5_Msk      (0x1U << DMA_ISR_HTIF5_Pos)                     /*!< 0x00040000 */
#define DMA_ISR_HTIF5          DMA_ISR_HTIF5_Msk                               /*!< Channel 5 Half Transfer flag       */
#define DMA_ISR_TEIF5_Pos      (19U)                                           
#define DMA_ISR_TEIF5_Msk      (0x1U << DMA_ISR_TEIF5_Pos)                     /*!< 0x00080000 */
#define DMA_ISR_TEIF5          DMA_ISR_TEIF5_Msk                               /*!< Channel 5 Transfer Error flag      */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define DMA_IFCR_CGIF1_Pos     (0U)                                            
#define DMA_IFCR_CGIF1_Msk     (0x1U << DMA_IFCR_CGIF1_Pos)                    /*!< 0x00000001 */
#define DMA_IFCR_CGIF1         DMA_IFCR_CGIF1_Msk                              /*!< Channel 1 Global interrupt clear    */
#define DMA_IFCR_CTCIF1_Pos    (1U)                                            
#define DMA_IFCR_CTCIF1_Msk    (0x1U << DMA_IFCR_CTCIF1_Pos)                   /*!< 0x00000002 */
#define DMA_IFCR_CTCIF1        DMA_IFCR_CTCIF1_Msk                             /*!< Channel 1 Transfer Complete clear   */
#define DMA_IFCR_CHTIF1_Pos    (2U)                                            
#define DMA_IFCR_CHTIF1_Msk    (0x1U << DMA_IFCR_CHTIF1_Pos)                   /*!< 0x00000004 */
#define DMA_IFCR_CHTIF1        DMA_IFCR_CHTIF1_Msk                             /*!< Channel 1 Half Transfer clear       */
#define DMA_IFCR_CTEIF1_Pos    (3U)                                            
#define DMA_IFCR_CTEIF1_Msk    (0x1U << DMA_IFCR_CTEIF1_Pos)                   /*!< 0x00000008 */
#define DMA_IFCR_CTEIF1        DMA_IFCR_CTEIF1_Msk                             /*!< Channel 1 Transfer Error clear      */
#define DMA_IFCR_CGIF2_Pos     (4U)                                            
#define DMA_IFCR_CGIF2_Msk     (0x1U << DMA_IFCR_CGIF2_Pos)                    /*!< 0x00000010 */
#define DMA_IFCR_CGIF2         DMA_IFCR_CGIF2_Msk                              /*!< Channel 2 Global interrupt clear    */
#define DMA_IFCR_CTCIF2_Pos    (5U)                                            
#define DMA_IFCR_CTCIF2_Msk    (0x1U << DMA_IFCR_CTCIF2_Pos)                   /*!< 0x00000020 */
#define DMA_IFCR_CTCIF2        DMA_IFCR_CTCIF2_Msk                             /*!< Channel 2 Transfer Complete clear   */
#define DMA_IFCR_CHTIF2_Pos    (6U)                                            
#define DMA_IFCR_CHTIF2_Msk    (0x1U << DMA_IFCR_CHTIF2_Pos)                   /*!< 0x00000040 */
#define DMA_IFCR_CHTIF2        DMA_IFCR_CHTIF2_Msk                             /*!< Channel 2 Half Transfer clear       */
#define DMA_IFCR_CTEIF2_Pos    (7U)                                            
#define DMA_IFCR_CTEIF2_Msk    (0x1U << DMA_IFCR_CTEIF2_Pos)                   /*!< 0x00000080 */
#define DMA_IFCR_CTEIF2        DMA_IFCR_CTEIF2_Msk                             /*!< Channel 2 Transfer Error clear      */
#define DMA_IFCR_CGIF3_Pos     (8U)                                            
#define DMA_IFCR_CGIF3_Msk     (0x1U << DMA_IFCR_CGIF3_Pos)                    /*!< 0x00000100 */
#define DMA_IFCR_CGIF3         DMA_IFCR_CGIF3_Msk                              /*!< Channel 3 Global interrupt clear    */
#define DMA_IFCR_CTCIF3_Pos    (9U)                                            
#define DMA_IFCR_CTCIF3_Msk    (0x1U << DMA_IFCR_CTCIF3_Pos)                   /*!< 0x00000200 */
#define DMA_IFCR_CTCIF3        DMA_IFCR_CTCIF3_Msk                             /*!< Channel 3 Transfer Complete clear   */
#define DMA_IFCR_CHTIF3_Pos    (10U)                                           
#define DMA_IFCR_CHTIF3_Msk    (0x1U << DMA_IFCR_CHTIF3_Pos)                   /*!< 0x00000400 */
#define DMA_IFCR_CHTIF3        DMA_IFCR_CHTIF3_Msk                             /*!< Channel 3 Half Transfer clear       */
#define DMA_IFCR_CTEIF3_Pos    (11U)                                           
#define DMA_IFCR_CTEIF3_Msk    (0x1U << DMA_IFCR_CTEIF3_Pos)                   /*!< 0x00000800 */
#define DMA_IFCR_CTEIF3        DMA_IFCR_CTEIF3_Msk                             /*!< Channel 3 Transfer Error clear      */
#define DMA_IFCR_CGIF4_Pos     (12U)                                           
#define DMA_IFCR_CGIF4_Msk     (0x1U << DMA_IFCR_CGIF4_Pos)                    /*!< 0x00001000 */
#define DMA_IFCR_CGIF4         DMA_IFCR_CGIF4_Msk                              /*!< Channel 4 Global interrupt clear    */
#define DMA_IFCR_CTCIF4_Pos    (13U)                                           
#define DMA_IFCR_CTCIF4_Msk    (0x1U << DMA_IFCR_CTCIF4_Pos)                   /*!< 0x00002000 */
#define DMA_IFCR_CTCIF4        DMA_IFCR_CTCIF4_Msk                             /*!< Channel 4 Transfer Complete clear   */
#define DMA_IFCR_CHTIF4_Pos    (14U)                                           
#define DMA_IFCR_CHTIF4_Msk    (0x1U << DMA_IFCR_CHTIF4_Pos)                   /*!< 0x00004000 */
#define DMA_IFCR_CHTIF4        DMA_IFCR_CHTIF4_Msk                             /*!< Channel 4 Half Transfer clear       */
#define DMA_IFCR_CTEIF4_Pos    (15U)                                           
#define DMA_IFCR_CTEIF4_Msk    (0x1U << DMA_IFCR_CTEIF4_Pos)                   /*!< 0x00008000 */
#define DMA_IFCR_CTEIF4        DMA_IFCR_CTEIF4_Msk                             /*!< Channel 4 Transfer Error clear      */
#define DMA_IFCR_CGIF5_Pos     (16U)                                           
#define DMA_IFCR_CGIF5_Msk     (0x1U << DMA_IFCR_CGIF5_Pos)                    /*!< 0x00010000 */
#define DMA_IFCR_CGIF5         DMA_IFCR_CGIF5_Msk                              /*!< Channel 5 Global interrupt clear    */
#define DMA_IFCR_CTCIF5_Pos    (17U)                                           
#define DMA_IFCR_CTCIF5_Msk    (0x1U << DMA_IFCR_CTCIF5_Pos)                   /*!< 0x00020000 */
#define DMA_IFCR_CTCIF5        DMA_IFCR_CTCIF5_Msk                             /*!< Channel 5 Transfer Complete clear   */
#define DMA_IFCR_CHTIF5_Pos    (18U)                                           
#define DMA_IFCR_CHTIF5_Msk    (0x1U << DMA_IFCR_CHTIF5_Pos)                   /*!< 0x00040000 */
#define DMA_IFCR_CHTIF5        DMA_IFCR_CHTIF5_Msk                             /*!< Channel 5 Half Transfer clear       */
#define DMA_IFCR_CTEIF5_Pos    (19U)                                           
#define DMA_IFCR_CTEIF5_Msk    (0x1U << DMA_IFCR_CTEIF5_Pos)                   /*!< 0x00080000 */
#define DMA_IFCR_CTEIF5        DMA_IFCR_CTEIF5_Msk                             /*!< Channel 5 Transfer Error clear      */

/*******************  Bit definition for DMA_CCR register  ********************/
#define DMA_CCR_EN_Pos         (0U)                                            
#define DMA_CCR_EN_Msk         (0x1U << DMA_CCR_EN_Pos)                        /*!< 0x00000001 */
#define DMA_CCR_EN             DMA_CCR_EN_Msk                                  /*!< Channel enable                      */
#define DMA_CCR_TCIE_Pos       (1U)                                            
#define DMA_CCR_TCIE_Msk       (0x1U << DMA_CCR_TCIE_Pos)                      /*!< 0x00000002 */
#define DMA_CCR_TCIE           DMA_CCR_TCIE_Msk                                /*!< Transfer complete interrupt enable  */
#define DMA_CCR_HTIE_Pos       (2U)                                            
#define DMA_CCR_HTIE_Msk       (0x1U << DMA_CCR_HTIE_Pos)                      /*!< 0x00000004 */
#define DMA_CCR_HTIE           DMA_CCR_HTIE_Msk                                /*!< Half Transfer interrupt enable      */
#define DMA_CCR_TEIE_Pos       (3U)                                            
#define DMA_CCR_TEIE_Msk       (0x1U << DMA_CCR_TEIE_Pos)                      /*!< 0x00000008 */
#define DMA_CCR_TEIE           DMA_CCR_TEIE_Msk                                /*!< Transfer error interrupt enable     */
#define DMA_CCR_DIR_Pos        (4U)                                            
#define DMA_CCR_DIR_Msk        (0x1U << DMA_CCR_DIR_Pos)                       /*!< 0x00000010 */
#define DMA_CCR_DIR            DMA_CCR_DIR_Msk                                 /*!< Data transfer direction             */
#define DMA_CCR_CIRC_Pos       (5U)                                            
#define DMA_CCR_CIRC_Msk       (0x1U << DMA_CCR_CIRC_Pos)                      /*!< 0x00000020 */
#define DMA_CCR_CIRC           DMA_CCR_CIRC_Msk                                /*!< Circular mode                       */
#define DMA_CCR_PINC_Pos       (6U)                                            
#define DMA_CCR_PINC_Msk       (0x1U << DMA_CCR_PINC_Pos)                      /*!< 0x00000040 */
#define DMA_CCR_PINC           DMA_CCR_PINC_Msk                                /*!< Peripheral increment mode           */
#define DMA_CCR_MINC_Pos       (7U)                                            
#define DMA_CCR_MINC_Msk       (0x1U << DMA_CCR_MINC_Pos)                      /*!< 0x00000080 */
#define DMA_CCR_MINC           DMA_CCR_MINC_Msk                                /*!< Memory increment mode               */
#define DMA_CCR_PSIZE_Pos      (8U)                                            
#define DMA_CCR_PSIZE_Msk      (0x3U << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000300 */
#define DMA_CCR_PSIZE          DMA_CCR_PSIZE_Msk                               /*!< PSIZE[1:0] bits (Peripheral size)   */
#define DMA_CCR_PSIZE_0        (0x0U << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000000 */
#define DMA_CCR_PSIZE_1        (0x1U << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000100 */
#define DMA_CCR_PSIZE_2        (0x2U << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000200 */
#define DMA_CCR_MSIZE_Pos      (10U)                                           
#define DMA_CCR_MSIZE_Msk      (0x3U << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000C00 */
#define DMA_CCR_MSIZE          DMA_CCR_MSIZE_Msk                               /*!< MSIZE[1:0] bits (Memory size)       */
#define DMA_CCR_MSIZE_0        (0x0U << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000000 */
#define DMA_CCR_MSIZE_1        (0x1U << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000400 */
#define DMA_CCR_MSIZE_2        (0x2U << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000800 */
#define DMA_CCR_PL_Pos         (12U)                                           
#define DMA_CCR_PL_Msk         (0x3U << DMA_CCR_PL_Pos)                        /*!< 0x00003000 */
#define DMA_CCR_PL             DMA_CCR_PL_Msk                                  /*!< PL[1:0] bits(Channel Priority level)*/
#define DMA_CCR_PL_0           (0x0U << DMA_CCR_PL_Pos)                        /*!< 0x00000000 */
#define DMA_CCR_PL_1           (0x1U << DMA_CCR_PL_Pos)                        /*!< 0x00001000 */
#define DMA_CCR_PL_2           (0x2U << DMA_CCR_PL_Pos)                        /*!< 0x00002000 */
#define DMA_CCR_PL_3           (0x3U << DMA_CCR_PL_Pos)                        /*!< 0x00003000 */
#define DMA_CCR_MEM2MEM_Pos    (14U)                                           
#define DMA_CCR_MEM2MEM_Msk    (0x1U << DMA_CCR_MEM2MEM_Pos)                   /*!< 0x00004000 */
#define DMA_CCR_MEM2MEM        DMA_CCR_MEM2MEM_Msk                             /*!< Memory to memory mode               */

/******************  Bit definition for DMA_CNDTR register  *******************/
#define DMA_CNDTR_NDT_Pos      (0U)                                            
#define DMA_CNDTR_NDT_Msk      (0xFFFFU << DMA_CNDTR_NDT_Pos)                  /*!< 0x0000FFFF */
#define DMA_CNDTR_NDT          DMA_CNDTR_NDT_Msk                               /*!< Number of data to Transfer          */

/******************  Bit definition for DMA_CPAR register  ********************/
#define DMA_CPAR_PA_Pos        (0U)                                            
#define DMA_CPAR_PA_Msk        (0xFFFFFFFFU << DMA_CPAR_PA_Pos)                /*!< 0xFFFFFFFF */
#define DMA_CPAR_PA            DMA_CPAR_PA_Msk                                 /*!< Peripheral Address                  */

/******************  Bit definition for DMA_CMAR register  ********************/
#define DMA_CMAR_MA_Pos        (0U)                                            
#define DMA_CMAR_MA_Msk        (0xFFFFFFFFU << DMA_CMAR_MA_Pos)                /*!< 0xFFFFFFFF */
#define DMA_CMAR_MA            DMA_CMAR_MA_Msk                                 /*!< Memory Address                      */

/******************************************************************************/
/*                                                                            */
/*                 External Interrupt/Event Controller (EXTI)                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos          (0U)                                         
#define EXTI_IMR_MR0_Msk          (0x1U << EXTI_IMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_IMR_MR0              EXTI_IMR_MR0_Msk                             /*!< Interrupt Mask on line 0  */
#define EXTI_IMR_MR1_Pos          (1U)                                         
#define EXTI_IMR_MR1_Msk          (0x1U << EXTI_IMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_IMR_MR1              EXTI_IMR_MR1_Msk                             /*!< Interrupt Mask on line 1  */
#define EXTI_IMR_MR2_Pos          (2U)                                         
#define EXTI_IMR_MR2_Msk          (0x1U << EXTI_IMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_IMR_MR2              EXTI_IMR_MR2_Msk                             /*!< Interrupt Mask on line 2  */
#define EXTI_IMR_MR3_Pos          (3U)                                         
#define EXTI_IMR_MR3_Msk          (0x1U << EXTI_IMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_IMR_MR3              EXTI_IMR_MR3_Msk                             /*!< Interrupt Mask on line 3  */
#define EXTI_IMR_MR4_Pos          (4U)                                         
#define EXTI_IMR_MR4_Msk          (0x1U << EXTI_IMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_IMR_MR4              EXTI_IMR_MR4_Msk                             /*!< Interrupt Mask on line 4  */
#define EXTI_IMR_MR5_Pos          (5U)                                         
#define EXTI_IMR_MR5_Msk          (0x1U << EXTI_IMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_IMR_MR5              EXTI_IMR_MR5_Msk                             /*!< Interrupt Mask on line 5  */
#define EXTI_IMR_MR6_Pos          (6U)                                         
#define EXTI_IMR_MR6_Msk          (0x1U << EXTI_IMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_IMR_MR6              EXTI_IMR_MR6_Msk                             /*!< Interrupt Mask on line 6  */
#define EXTI_IMR_MR7_Pos          (7U)                                         
#define EXTI_IMR_MR7_Msk          (0x1U << EXTI_IMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_IMR_MR7              EXTI_IMR_MR7_Msk                             /*!< Interrupt Mask on line 7  */
#define EXTI_IMR_MR8_Pos          (8U)                                         
#define EXTI_IMR_MR8_Msk          (0x1U << EXTI_IMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_IMR_MR8              EXTI_IMR_MR8_Msk                             /*!< Interrupt Mask on line 8  */
#define EXTI_IMR_MR9_Pos          (9U)                                         
#define EXTI_IMR_MR9_Msk          (0x1U << EXTI_IMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_IMR_MR9              EXTI_IMR_MR9_Msk                             /*!< Interrupt Mask on line 9  */
#define EXTI_IMR_MR10_Pos         (10U)                                        
#define EXTI_IMR_MR10_Msk         (0x1U << EXTI_IMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_IMR_MR10             EXTI_IMR_MR10_Msk                            /*!< Interrupt Mask on line 10 */
#define EXTI_IMR_MR11_Pos         (11U)                                        
#define EXTI_IMR_MR11_Msk         (0x1U << EXTI_IMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_IMR_MR11             EXTI_IMR_MR11_Msk                            /*!< Interrupt Mask on line 11 */
#define EXTI_IMR_MR12_Pos         (12U)                                        
#define EXTI_IMR_MR12_Msk         (0x1U << EXTI_IMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_IMR_MR12             EXTI_IMR_MR12_Msk                            /*!< Interrupt Mask on line 12 */
#define EXTI_IMR_MR13_Pos         (13U)                                        
#define EXTI_IMR_MR13_Msk         (0x1U << EXTI_IMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_IMR_MR13             EXTI_IMR_MR13_Msk                            /*!< Interrupt Mask on line 13 */
#define EXTI_IMR_MR14_Pos         (14U)                                        
#define EXTI_IMR_MR14_Msk         (0x1U << EXTI_IMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_IMR_MR14             EXTI_IMR_MR14_Msk                            /*!< Interrupt Mask on line 14 */
#define EXTI_IMR_MR15_Pos         (15U)                                        
#define EXTI_IMR_MR15_Msk         (0x1U << EXTI_IMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_IMR_MR15             EXTI_IMR_MR15_Msk                            /*!< Interrupt Mask on line 15 */
#define EXTI_IMR_MR16_Pos         (16U)                                        
#define EXTI_IMR_MR16_Msk         (0x1U << EXTI_IMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_IMR_MR16             EXTI_IMR_MR16_Msk                            /*!< Interrupt Mask on line 16 */
#define EXTI_IMR_MR17_Pos         (17U)                                        
#define EXTI_IMR_MR17_Msk         (0x1U << EXTI_IMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_IMR_MR17             EXTI_IMR_MR17_Msk                            /*!< Interrupt Mask on line 17 */
#define EXTI_IMR_MR18_Pos         (18U)                                        
#define EXTI_IMR_MR18_Msk         (0x1U << EXTI_IMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_IMR_MR18             EXTI_IMR_MR18_Msk                            /*!< Interrupt Mask on line 18 */
#define EXTI_IMR_MR19_Pos         (19U)                                        
#define EXTI_IMR_MR19_Msk         (0x1U << EXTI_IMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_IMR_MR19             EXTI_IMR_MR19_Msk                            /*!< Interrupt Mask on line 19 */
#define EXTI_IMR_MR20_Pos         (20U)                                        
#define EXTI_IMR_MR20_Msk         (0x1U << EXTI_IMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_IMR_MR20             EXTI_IMR_MR20_Msk                            /*!< Interrupt Mask on line 20 */
#define EXTI_IMR_MR21_Pos         (21U)                                        
#define EXTI_IMR_MR21_Msk         (0x1U << EXTI_IMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_IMR_MR21             EXTI_IMR_MR21_Msk                            /*!< Interrupt Mask on line 21 */
#define EXTI_IMR_MR22_Pos         (22U)                                        
#define EXTI_IMR_MR22_Msk         (0x1U << EXTI_IMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_IMR_MR22             EXTI_IMR_MR22_Msk                            /*!< Interrupt Mask on line 22 */
#define EXTI_IMR_MR23_Pos         (23U)                                        
#define EXTI_IMR_MR23_Msk         (0x1U << EXTI_IMR_MR23_Pos)                  /*!< 0x00800000 */
#define EXTI_IMR_MR23             EXTI_IMR_MR23_Msk                            /*!< Interrupt Mask on line 23 */
#define EXTI_IMR_MR24_Pos         (24U)                                        
#define EXTI_IMR_MR24_Msk         (0x1U << EXTI_IMR_MR24_Pos)                  /*!< 0x01000000 */
#define EXTI_IMR_MR24             EXTI_IMR_MR24_Msk                            /*!< Interrupt Mask on line 24 */ 
#define EXTI_IMR_MR25_Pos         (25U)                                        
#define EXTI_IMR_MR25_Msk         (0x1U << EXTI_IMR_MR25_Pos)                  /*!< 0x02000000 */
#define EXTI_IMR_MR25             EXTI_IMR_MR25_Msk                            /*!< Interrupt Mask on line 25 */
#define EXTI_IMR_MR26_Pos         (26U)                                        
#define EXTI_IMR_MR26_Msk         (0x1U << EXTI_IMR_MR26_Pos)                  /*!< 0x04000000 */
#define EXTI_IMR_MR26             EXTI_IMR_MR26_Msk                            /*!< Interrupt Mask on line 26 */ 
#define EXTI_IMR_MR27_Pos         (27U)                                        
#define EXTI_IMR_MR27_Msk         (0x1U << EXTI_IMR_MR27_Pos)                  /*!< 0x08000000 */
#define EXTI_IMR_MR27             EXTI_IMR_MR27_Msk                            /*!< Interrupt Mask on line 27 */

/* References Defines */
#define  EXTI_IMR_IM0 EXTI_IMR_MR0
#define  EXTI_IMR_IM1 EXTI_IMR_MR1
#define  EXTI_IMR_IM2 EXTI_IMR_MR2
#define  EXTI_IMR_IM3 EXTI_IMR_MR3
#define  EXTI_IMR_IM4 EXTI_IMR_MR4
#define  EXTI_IMR_IM5 EXTI_IMR_MR5
#define  EXTI_IMR_IM6 EXTI_IMR_MR6
#define  EXTI_IMR_IM7 EXTI_IMR_MR7
#define  EXTI_IMR_IM8 EXTI_IMR_MR8
#define  EXTI_IMR_IM9 EXTI_IMR_MR9
#define  EXTI_IMR_IM10 EXTI_IMR_MR10
#define  EXTI_IMR_IM11 EXTI_IMR_MR11
#define  EXTI_IMR_IM12 EXTI_IMR_MR12
#define  EXTI_IMR_IM13 EXTI_IMR_MR13
#define  EXTI_IMR_IM14 EXTI_IMR_MR14
#define  EXTI_IMR_IM15 EXTI_IMR_MR15
#define  EXTI_IMR_IM16 EXTI_IMR_MR16
#define  EXTI_IMR_IM17 EXTI_IMR_MR17
#define  EXTI_IMR_IM18 EXTI_IMR_MR18
#define  EXTI_IMR_IM19 EXTI_IMR_MR19
#define  EXTI_IMR_IM21 EXTI_IMR_MR21
#define  EXTI_IMR_IM22 EXTI_IMR_MR22
#define  EXTI_IMR_IM23 EXTI_IMR_MR23
#define  EXTI_IMR_IM24 EXTI_IMR_MR24
#define  EXTI_IMR_IM25 EXTI_IMR_MR25
#define  EXTI_IMR_IM26 EXTI_IMR_MR26
#define  EXTI_IMR_IM27 EXTI_IMR_MR27

#define EXTI_IMR_IM_Pos           (0U)                                         
#define EXTI_IMR_IM_Msk           (0xFFFFFFFU << EXTI_IMR_IM_Pos)              /*!< 0x0AEFFFFF */
#define EXTI_IMR_IM               EXTI_IMR_IM_Msk                              /*!< Interrupt Mask All */


/******************  Bit definition for EXTI_EMR register  ********************/
#define EXTI_EMR_MR0_Pos          (0U)                                         
#define EXTI_EMR_MR0_Msk          (0x1U << EXTI_EMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_EMR_MR0              EXTI_EMR_MR0_Msk                             /*!< Event Mask on line 0  */
#define EXTI_EMR_MR1_Pos          (1U)                                         
#define EXTI_EMR_MR1_Msk          (0x1U << EXTI_EMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_EMR_MR1              EXTI_EMR_MR1_Msk                             /*!< Event Mask on line 1  */
#define EXTI_EMR_MR2_Pos          (2U)                                         
#define EXTI_EMR_MR2_Msk          (0x1U << EXTI_EMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_EMR_MR2              EXTI_EMR_MR2_Msk                             /*!< Event Mask on line 2  */
#define EXTI_EMR_MR3_Pos          (3U)                                         
#define EXTI_EMR_MR3_Msk          (0x1U << EXTI_EMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_EMR_MR3              EXTI_EMR_MR3_Msk                             /*!< Event Mask on line 3  */
#define EXTI_EMR_MR4_Pos          (4U)                                         
#define EXTI_EMR_MR4_Msk          (0x1U << EXTI_EMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_EMR_MR4              EXTI_EMR_MR4_Msk                             /*!< Event Mask on line 4  */
#define EXTI_EMR_MR5_Pos          (5U)                                         
#define EXTI_EMR_MR5_Msk          (0x1U << EXTI_EMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_EMR_MR5              EXTI_EMR_MR5_Msk                             /*!< Event Mask on line 5  */
#define EXTI_EMR_MR6_Pos          (6U)                                         
#define EXTI_EMR_MR6_Msk          (0x1U << EXTI_EMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_EMR_MR6              EXTI_EMR_MR6_Msk                             /*!< Event Mask on line 6  */
#define EXTI_EMR_MR7_Pos          (7U)                                         
#define EXTI_EMR_MR7_Msk          (0x1U << EXTI_EMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_EMR_MR7              EXTI_EMR_MR7_Msk                             /*!< Event Mask on line 7  */
#define EXTI_EMR_MR8_Pos          (8U)                                         
#define EXTI_EMR_MR8_Msk          (0x1U << EXTI_EMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_EMR_MR8              EXTI_EMR_MR8_Msk                             /*!< Event Mask on line 8  */
#define EXTI_EMR_MR9_Pos          (9U)                                         
#define EXTI_EMR_MR9_Msk          (0x1U << EXTI_EMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_EMR_MR9              EXTI_EMR_MR9_Msk                             /*!< Event Mask on line 9  */
#define EXTI_EMR_MR10_Pos         (10U)                                        
#define EXTI_EMR_MR10_Msk         (0x1U << EXTI_EMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_EMR_MR10             EXTI_EMR_MR10_Msk                            /*!< Event Mask on line 10 */
#define EXTI_EMR_MR11_Pos         (11U)                                        
#define EXTI_EMR_MR11_Msk         (0x1U << EXTI_EMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_EMR_MR11             EXTI_EMR_MR11_Msk                            /*!< Event Mask on line 11 */
#define EXTI_EMR_MR12_Pos         (12U)                                        
#define EXTI_EMR_MR12_Msk         (0x1U << EXTI_EMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_EMR_MR12             EXTI_EMR_MR12_Msk                            /*!< Event Mask on line 12 */
#define EXTI_EMR_MR13_Pos         (13U)                                        
#define EXTI_EMR_MR13_Msk         (0x1U << EXTI_EMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_EMR_MR13             EXTI_EMR_MR13_Msk                            /*!< Event Mask on line 13 */
#define EXTI_EMR_MR14_Pos         (14U)                                        
#define EXTI_EMR_MR14_Msk         (0x1U << EXTI_EMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_EMR_MR14             EXTI_EMR_MR14_Msk                            /*!< Event Mask on line 14 */
#define EXTI_EMR_MR15_Pos         (15U)                                        
#define EXTI_EMR_MR15_Msk         (0x1U << EXTI_EMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_EMR_MR15             EXTI_EMR_MR15_Msk                            /*!< Event Mask on line 15 */
#define EXTI_EMR_MR16_Pos         (16U)                                        
#define EXTI_EMR_MR16_Msk         (0x1U << EXTI_EMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_EMR_MR16             EXTI_EMR_MR16_Msk                            /*!< Event Mask on line 16 */
#define EXTI_EMR_MR17_Pos         (17U)                                        
#define EXTI_EMR_MR17_Msk         (0x1U << EXTI_EMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_EMR_MR17             EXTI_EMR_MR17_Msk                            /*!< Event Mask on line 17 */
#define EXTI_EMR_MR18_Pos         (18U)                                        
#define EXTI_EMR_MR18_Msk         (0x1U << EXTI_EMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_EMR_MR18             EXTI_EMR_MR18_Msk                            /*!< Event Mask on line 18 */
#define EXTI_EMR_MR19_Pos         (19U)                                        
#define EXTI_EMR_MR19_Msk         (0x1U << EXTI_EMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_EMR_MR19             EXTI_EMR_MR19_Msk                            /*!< Event Mask on line 19 */
#define EXTI_EMR_MR20_Pos         (20U)                                        
#define EXTI_EMR_MR20_Msk         (0x1U << EXTI_EMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_EMR_MR20             EXTI_EMR_MR20_Msk                            /*!< Event Mask on line 20 */
#define EXTI_EMR_MR21_Pos         (21U)                                        
#define EXTI_EMR_MR21_Msk         (0x1U << EXTI_EMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_EMR_MR21             EXTI_EMR_MR21_Msk                            /*!< Event Mask on line 21 */
#define EXTI_EMR_MR22_Pos         (22U)                                        
#define EXTI_EMR_MR22_Msk         (0x1U << EXTI_EMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_EMR_MR22             EXTI_EMR_MR22_Msk                            /*!< Event Mask on line 22 */
#define EXTI_EMR_MR23_Pos         (23U)                                        
#define EXTI_EMR_MR23_Msk         (0x1U << EXTI_EMR_MR23_Pos)                  /*!< 0x00800000 */
#define EXTI_EMR_MR23             EXTI_EMR_MR23_Msk                            /*!< Event Mask on line 23 */
#define EXTI_EMR_MR24_Pos         (24U)                                        
#define EXTI_EMR_MR24_Msk         (0x1U << EXTI_EMR_MR24_Pos)                  /*!< 0x01000000 */
#define EXTI_EMR_MR24             EXTI_EMR_MR24_Msk                            /*!< Event Mask on line 24 */
#define EXTI_EMR_MR25_Pos         (25U)                                        
#define EXTI_EMR_MR25_Msk         (0x1U << EXTI_EMR_MR25_Pos)                  /*!< 0x02000000 */
#define EXTI_EMR_MR25             EXTI_EMR_MR25_Msk                            /*!< Event Mask on line 25 */
#define EXTI_EMR_MR26_Pos         (25U)                                        
#define EXTI_EMR_MR26_Msk         (0x1U << EXTI_EMR_MR26_Pos)                  /*!< 0x04000000 */
#define EXTI_EMR_MR26             EXTI_EMR_MR26_Msk                            /*!< Event Mask on line 26 */
#define EXTI_EMR_MR27_Pos         (27U)                                        
#define EXTI_EMR_MR27_Msk         (0x1U << EXTI_EMR_MR27_Pos)                  /*!< 0x08000000 */
#define EXTI_EMR_MR27             EXTI_EMR_MR27_Msk                            /*!< Event Mask on line 27 */

/* References Defines */
#define  EXTI_EMR_EM0 EXTI_EMR_MR0
#define  EXTI_EMR_EM1 EXTI_EMR_MR1
#define  EXTI_EMR_EM2 EXTI_EMR_MR2
#define  EXTI_EMR_EM3 EXTI_EMR_MR3
#define  EXTI_EMR_EM4 EXTI_EMR_MR4
#define  EXTI_EMR_EM5 EXTI_EMR_MR5
#define  EXTI_EMR_EM6 EXTI_EMR_MR6
#define  EXTI_EMR_EM7 EXTI_EMR_MR7
#define  EXTI_EMR_EM8 EXTI_EMR_MR8
#define  EXTI_EMR_EM9 EXTI_EMR_MR9
#define  EXTI_EMR_EM10 EXTI_EMR_MR10
#define  EXTI_EMR_EM11 EXTI_EMR_MR11
#define  EXTI_EMR_EM12 EXTI_EMR_MR12
#define  EXTI_EMR_EM13 EXTI_EMR_MR13
#define  EXTI_EMR_EM14 EXTI_EMR_MR14
#define  EXTI_EMR_EM15 EXTI_EMR_MR15
#define  EXTI_EMR_EM16 EXTI_EMR_MR16
#define  EXTI_EMR_EM17 EXTI_EMR_MR17
#define  EXTI_EMR_EM18 EXTI_EMR_MR18
#define  EXTI_EMR_EM19 EXTI_EMR_MR19
#define  EXTI_EMR_EM20 EXTI_EMR_MR20
#define  EXTI_EMR_EM21 EXTI_EMR_MR21
#define  EXTI_EMR_EM22 EXTI_EMR_MR22
#define  EXTI_EMR_EM23 EXTI_EMR_MR23
#define  EXTI_EMR_EM24 EXTI_EMR_MR24
#define  EXTI_EMR_EM25 EXTI_EMR_MR25
#define  EXTI_EMR_EM26 EXTI_EMR_MR26
#define  EXTI_EMR_EM27 EXTI_EMR_MR27

/*******************  Bit definition for EXTI_RTSR register  ******************/
#define EXTI_RTSR_TR0_Pos         (0U)                                         
#define EXTI_RTSR_TR0_Msk         (0x1U << EXTI_RTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_RTSR_TR0             EXTI_RTSR_TR0_Msk                            /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR_TR1_Pos         (1U)                                         
#define EXTI_RTSR_TR1_Msk         (0x1U << EXTI_RTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_RTSR_TR1             EXTI_RTSR_TR1_Msk                            /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR_TR2_Pos         (2U)                                         
#define EXTI_RTSR_TR2_Msk         (0x1U << EXTI_RTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_RTSR_TR2             EXTI_RTSR_TR2_Msk                            /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR_TR3_Pos         (3U)                                         
#define EXTI_RTSR_TR3_Msk         (0x1U << EXTI_RTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_RTSR_TR3             EXTI_RTSR_TR3_Msk                            /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR_TR4_Pos         (4U)                                         
#define EXTI_RTSR_TR4_Msk         (0x1U << EXTI_RTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_RTSR_TR4             EXTI_RTSR_TR4_Msk                            /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR_TR5_Pos         (5U)                                         
#define EXTI_RTSR_TR5_Msk         (0x1U << EXTI_RTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_RTSR_TR5             EXTI_RTSR_TR5_Msk                            /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR_TR6_Pos         (6U)                                         
#define EXTI_RTSR_TR6_Msk         (0x1U << EXTI_RTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_RTSR_TR6             EXTI_RTSR_TR6_Msk                            /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR_TR7_Pos         (7U)                                         
#define EXTI_RTSR_TR7_Msk         (0x1U << EXTI_RTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_RTSR_TR7             EXTI_RTSR_TR7_Msk                            /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR_TR8_Pos         (8U)                                         
#define EXTI_RTSR_TR8_Msk         (0x1U << EXTI_RTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_RTSR_TR8             EXTI_RTSR_TR8_Msk                            /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR_TR9_Pos         (9U)                                         
#define EXTI_RTSR_TR9_Msk         (0x1U << EXTI_RTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_RTSR_TR9             EXTI_RTSR_TR9_Msk                            /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR_TR10_Pos        (10U)                                        
#define EXTI_RTSR_TR10_Msk        (0x1U << EXTI_RTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_RTSR_TR10            EXTI_RTSR_TR10_Msk                           /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR_TR11_Pos        (11U)                                        
#define EXTI_RTSR_TR11_Msk        (0x1U << EXTI_RTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_RTSR_TR11            EXTI_RTSR_TR11_Msk                           /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR_TR12_Pos        (12U)                                        
#define EXTI_RTSR_TR12_Msk        (0x1U << EXTI_RTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_RTSR_TR12            EXTI_RTSR_TR12_Msk                           /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR_TR13_Pos        (13U)                                        
#define EXTI_RTSR_TR13_Msk        (0x1U << EXTI_RTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_RTSR_TR13            EXTI_RTSR_TR13_Msk                           /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR_TR14_Pos        (14U)                                        
#define EXTI_RTSR_TR14_Msk        (0x1U << EXTI_RTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_RTSR_TR14            EXTI_RTSR_TR14_Msk                           /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR_TR15_Pos        (15U)                                        
#define EXTI_RTSR_TR15_Msk        (0x1U << EXTI_RTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_RTSR_TR15            EXTI_RTSR_TR15_Msk                           /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR_TR16_Pos        (16U)                                        
#define EXTI_RTSR_TR16_Msk        (0x1U << EXTI_RTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_RTSR_TR16            EXTI_RTSR_TR16_Msk                           /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR_TR17_Pos        (17U)                                        
#define EXTI_RTSR_TR17_Msk        (0x1U << EXTI_RTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_RTSR_TR17            EXTI_RTSR_TR17_Msk                           /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR_TR18_Pos        (18U)                                        
#define EXTI_RTSR_TR18_Msk        (0x1U << EXTI_RTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_RTSR_TR18            EXTI_RTSR_TR18_Msk                           /*!< Rising trigger event configuration bit of line 18 */
#define EXTI_RTSR_TR19_Pos        (19U)                                        
#define EXTI_RTSR_TR19_Msk        (0x1U << EXTI_RTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_RTSR_TR19            EXTI_RTSR_TR19_Msk                           /*!< Rising trigger event configuration bit of line 19 */


/* References Defines */
#define EXTI_RTSR_RT0 EXTI_RTSR_TR0
#define EXTI_RTSR_RT1 EXTI_RTSR_TR1
#define EXTI_RTSR_RT2 EXTI_RTSR_TR2
#define EXTI_RTSR_RT3 EXTI_RTSR_TR3
#define EXTI_RTSR_RT4 EXTI_RTSR_TR4
#define EXTI_RTSR_RT5 EXTI_RTSR_TR5
#define EXTI_RTSR_RT6 EXTI_RTSR_TR6
#define EXTI_RTSR_RT7 EXTI_RTSR_TR7
#define EXTI_RTSR_RT8 EXTI_RTSR_TR8
#define EXTI_RTSR_RT9 EXTI_RTSR_TR9
#define EXTI_RTSR_RT10 EXTI_RTSR_TR10
#define EXTI_RTSR_RT11 EXTI_RTSR_TR11
#define EXTI_RTSR_RT12 EXTI_RTSR_TR12
#define EXTI_RTSR_RT13 EXTI_RTSR_TR13
#define EXTI_RTSR_RT14 EXTI_RTSR_TR14
#define EXTI_RTSR_RT15 EXTI_RTSR_TR15
#define EXTI_RTSR_RT16 EXTI_RTSR_TR16
#define EXTI_RTSR_RT17 EXTI_RTSR_TR17
#define EXTI_RTSR_RT18 EXTI_RTSR_TR18
#define EXTI_RTSR_RT19 EXTI_RTSR_TR19


/*******************  Bit definition for EXTI_FTSR register *******************/
#define EXTI_FTSR_TR0_Pos         (0U)                                         
#define EXTI_FTSR_TR0_Msk         (0x1U << EXTI_FTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_FTSR_TR0             EXTI_FTSR_TR0_Msk                            /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR_TR1_Pos         (1U)                                         
#define EXTI_FTSR_TR1_Msk         (0x1U << EXTI_FTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_FTSR_TR1             EXTI_FTSR_TR1_Msk                            /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR_TR2_Pos         (2U)                                         
#define EXTI_FTSR_TR2_Msk         (0x1U << EXTI_FTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_FTSR_TR2             EXTI_FTSR_TR2_Msk                            /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR_TR3_Pos         (3U)                                         
#define EXTI_FTSR_TR3_Msk         (0x1U << EXTI_FTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_FTSR_TR3             EXTI_FTSR_TR3_Msk                            /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR_TR4_Pos         (4U)                                         
#define EXTI_FTSR_TR4_Msk         (0x1U << EXTI_FTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_FTSR_TR4             EXTI_FTSR_TR4_Msk                            /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR_TR5_Pos         (5U)                                         
#define EXTI_FTSR_TR5_Msk         (0x1U << EXTI_FTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_FTSR_TR5             EXTI_FTSR_TR5_Msk                            /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR_TR6_Pos         (6U)                                         
#define EXTI_FTSR_TR6_Msk         (0x1U << EXTI_FTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_FTSR_TR6             EXTI_FTSR_TR6_Msk                            /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR_TR7_Pos         (7U)                                         
#define EXTI_FTSR_TR7_Msk         (0x1U << EXTI_FTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_FTSR_TR7             EXTI_FTSR_TR7_Msk                            /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR_TR8_Pos         (8U)                                         
#define EXTI_FTSR_TR8_Msk         (0x1U << EXTI_FTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_FTSR_TR8             EXTI_FTSR_TR8_Msk                            /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR_TR9_Pos         (9U)                                         
#define EXTI_FTSR_TR9_Msk         (0x1U << EXTI_FTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_FTSR_TR9             EXTI_FTSR_TR9_Msk                            /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR_TR10_Pos        (10U)                                        
#define EXTI_FTSR_TR10_Msk        (0x1U << EXTI_FTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_FTSR_TR10            EXTI_FTSR_TR10_Msk                           /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR_TR11_Pos        (11U)                                        
#define EXTI_FTSR_TR11_Msk        (0x1U << EXTI_FTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_FTSR_TR11            EXTI_FTSR_TR11_Msk                           /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR_TR12_Pos        (12U)                                        
#define EXTI_FTSR_TR12_Msk        (0x1U << EXTI_FTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_FTSR_TR12            EXTI_FTSR_TR12_Msk                           /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR_TR13_Pos        (13U)                                        
#define EXTI_FTSR_TR13_Msk        (0x1U << EXTI_FTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_FTSR_TR13            EXTI_FTSR_TR13_Msk                           /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR_TR14_Pos        (14U)                                        
#define EXTI_FTSR_TR14_Msk        (0x1U << EXTI_FTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_FTSR_TR14            EXTI_FTSR_TR14_Msk                           /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR_TR15_Pos        (15U)                                        
#define EXTI_FTSR_TR15_Msk        (0x1U << EXTI_FTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_FTSR_TR15            EXTI_FTSR_TR15_Msk                           /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR_TR16_Pos        (16U)                                        
#define EXTI_FTSR_TR16_Msk        (0x1U << EXTI_FTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_FTSR_TR16            EXTI_FTSR_TR16_Msk                           /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR_TR17_Pos        (17U)                                        
#define EXTI_FTSR_TR17_Msk        (0x1U << EXTI_FTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_FTSR_TR17            EXTI_FTSR_TR17_Msk                           /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR_TR18_Pos        (18U)                                        
#define EXTI_FTSR_TR18_Msk        (0x1U << EXTI_FTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_FTSR_TR18            EXTI_FTSR_TR18_Msk                           /*!< Falling trigger event configuration bit of line 18 */
#define EXTI_FTSR_TR19_Pos        (19U)                                        
#define EXTI_FTSR_TR19_Msk        (0x1U << EXTI_FTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_FTSR_TR19            EXTI_FTSR_TR19_Msk                           /*!< Falling trigger event configuration bit of line 19 */


/* References Defines */
#define EXTI_FTSR_FT0 EXTI_FTSR_TR0
#define EXTI_FTSR_FT1 EXTI_FTSR_TR1
#define EXTI_FTSR_FT2 EXTI_FTSR_TR2
#define EXTI_FTSR_FT3 EXTI_FTSR_TR3
#define EXTI_FTSR_FT4 EXTI_FTSR_TR4
#define EXTI_FTSR_FT5 EXTI_FTSR_TR5
#define EXTI_FTSR_FT6 EXTI_FTSR_TR6
#define EXTI_FTSR_FT7 EXTI_FTSR_TR7
#define EXTI_FTSR_FT8 EXTI_FTSR_TR8
#define EXTI_FTSR_FT9 EXTI_FTSR_TR9
#define EXTI_FTSR_FT10 EXTI_FTSR_TR10
#define EXTI_FTSR_FT11 EXTI_FTSR_TR11
#define EXTI_FTSR_FT12 EXTI_FTSR_TR12
#define EXTI_FTSR_FT13 EXTI_FTSR_TR13
#define EXTI_FTSR_FT14 EXTI_FTSR_TR14
#define EXTI_FTSR_FT15 EXTI_FTSR_TR15
#define EXTI_FTSR_FT16 EXTI_FTSR_TR16
#define EXTI_FTSR_FT17 EXTI_FTSR_TR17
#define EXTI_FTSR_FT18 EXTI_FTSR_TR18
#define EXTI_FTSR_FT19 EXTI_FTSR_TR19


/******************* Bit definition for EXTI_SWIER register *******************/
#define EXTI_SWIER_SWIER0_Pos     (0U)                                         
#define EXTI_SWIER_SWIER0_Msk     (0x1U << EXTI_SWIER_SWIER0_Pos)              /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0         EXTI_SWIER_SWIER0_Msk                        /*!< Software Interrupt on line 0  */
#define EXTI_SWIER_SWIER1_Pos     (1U)                                         
#define EXTI_SWIER_SWIER1_Msk     (0x1U << EXTI_SWIER_SWIER1_Pos)              /*!< 0x00000002 */
#define EXTI_SWIER_SWIER1         EXTI_SWIER_SWIER1_Msk                        /*!< Software Interrupt on line 1  */
#define EXTI_SWIER_SWIER2_Pos     (2U)                                         
#define EXTI_SWIER_SWIER2_Msk     (0x1U << EXTI_SWIER_SWIER2_Pos)              /*!< 0x00000004 */
#define EXTI_SWIER_SWIER2         EXTI_SWIER_SWIER2_Msk                        /*!< Software Interrupt on line 2  */
#define EXTI_SWIER_SWIER3_Pos     (3U)                                         
#define EXTI_SWIER_SWIER3_Msk     (0x1U << EXTI_SWIER_SWIER3_Pos)              /*!< 0x00000008 */
#define EXTI_SWIER_SWIER3         EXTI_SWIER_SWIER3_Msk                        /*!< Software Interrupt on line 3  */
#define EXTI_SWIER_SWIER4_Pos     (4U)                                         
#define EXTI_SWIER_SWIER4_Msk     (0x1U << EXTI_SWIER_SWIER4_Pos)              /*!< 0x00000010 */
#define EXTI_SWIER_SWIER4         EXTI_SWIER_SWIER4_Msk                        /*!< Software Interrupt on line 4  */
#define EXTI_SWIER_SWIER5_Pos     (5U)                                         
#define EXTI_SWIER_SWIER5_Msk     (0x1U << EXTI_SWIER_SWIER5_Pos)              /*!< 0x00000020 */
#define EXTI_SWIER_SWIER5         EXTI_SWIER_SWIER5_Msk                        /*!< Software Interrupt on line 5  */
#define EXTI_SWIER_SWIER6_Pos     (6U)                                         
#define EXTI_SWIER_SWIER6_Msk     (0x1U << EXTI_SWIER_SWIER6_Pos)              /*!< 0x00000040 */
#define EXTI_SWIER_SWIER6         EXTI_SWIER_SWIER6_Msk                        /*!< Software Interrupt on line 6  */
#define EXTI_SWIER_SWIER7_Pos     (7U)                                         
#define EXTI_SWIER_SWIER7_Msk     (0x1U << EXTI_SWIER_SWIER7_Pos)              /*!< 0x00000080 */
#define EXTI_SWIER_SWIER7         EXTI_SWIER_SWIER7_Msk                        /*!< Software Interrupt on line 7  */
#define EXTI_SWIER_SWIER8_Pos     (8U)                                         
#define EXTI_SWIER_SWIER8_Msk     (0x1U << EXTI_SWIER_SWIER8_Pos)              /*!< 0x00000100 */
#define EXTI_SWIER_SWIER8         EXTI_SWIER_SWIER8_Msk                        /*!< Software Interrupt on line 8  */
#define EXTI_SWIER_SWIER9_Pos     (9U)                                         
#define EXTI_SWIER_SWIER9_Msk     (0x1U << EXTI_SWIER_SWIER9_Pos)              /*!< 0x00000200 */
#define EXTI_SWIER_SWIER9         EXTI_SWIER_SWIER9_Msk                        /*!< Software Interrupt on line 9  */
#define EXTI_SWIER_SWIER10_Pos    (10U)                                        
#define EXTI_SWIER_SWIER10_Msk    (0x1U << EXTI_SWIER_SWIER10_Pos)             /*!< 0x00000400 */
#define EXTI_SWIER_SWIER10        EXTI_SWIER_SWIER10_Msk                       /*!< Software Interrupt on line 10 */
#define EXTI_SWIER_SWIER11_Pos    (11U)                                        
#define EXTI_SWIER_SWIER11_Msk    (0x1U << EXTI_SWIER_SWIER11_Pos)             /*!< 0x00000800 */
#define EXTI_SWIER_SWIER11        EXTI_SWIER_SWIER11_Msk                       /*!< Software Interrupt on line 11 */
#define EXTI_SWIER_SWIER12_Pos    (12U)                                        
#define EXTI_SWIER_SWIER12_Msk    (0x1U << EXTI_SWIER_SWIER12_Pos)             /*!< 0x00001000 */
#define EXTI_SWIER_SWIER12        EXTI_SWIER_SWIER12_Msk                       /*!< Software Interrupt on line 12 */
#define EXTI_SWIER_SWIER13_Pos    (13U)                                        
#define EXTI_SWIER_SWIER13_Msk    (0x1U << EXTI_SWIER_SWIER13_Pos)             /*!< 0x00002000 */
#define EXTI_SWIER_SWIER13        EXTI_SWIER_SWIER13_Msk                       /*!< Software Interrupt on line 13 */
#define EXTI_SWIER_SWIER14_Pos    (14U)                                        
#define EXTI_SWIER_SWIER14_Msk    (0x1U << EXTI_SWIER_SWIER14_Pos)             /*!< 0x00004000 */
#define EXTI_SWIER_SWIER14        EXTI_SWIER_SWIER14_Msk                       /*!< Software Interrupt on line 14 */
#define EXTI_SWIER_SWIER15_Pos    (15U)                                        
#define EXTI_SWIER_SWIER15_Msk    (0x1U << EXTI_SWIER_SWIER15_Pos)             /*!< 0x00008000 */
#define EXTI_SWIER_SWIER15        EXTI_SWIER_SWIER15_Msk                       /*!< Software Interrupt on line 15 */
#define EXTI_SWIER_SWIER16_Pos    (16U)                                        
#define EXTI_SWIER_SWIER16_Msk    (0x1U << EXTI_SWIER_SWIER16_Pos)             /*!< 0x00010000 */
#define EXTI_SWIER_SWIER16        EXTI_SWIER_SWIER16_Msk                       /*!< Software Interrupt on line 16 */
#define EXTI_SWIER_SWIER17_Pos    (17U)                                        
#define EXTI_SWIER_SWIER17_Msk    (0x1U << EXTI_SWIER_SWIER17_Pos)             /*!< 0x00020000 */
#define EXTI_SWIER_SWIER17        EXTI_SWIER_SWIER17_Msk                       /*!< Software Interrupt on line 17 */
#define EXTI_SWIER_SWIER18_Pos    (18U)                                        
#define EXTI_SWIER_SWIER18_Msk    (0x1U << EXTI_SWIER_SWIER18_Pos)             /*!< 0x00040000 */
#define EXTI_SWIER_SWIER18        EXTI_SWIER_SWIER18_Msk                       /*!< Software Interrupt on line 18 */
#define EXTI_SWIER_SWIER19_Pos    (19U)                                        
#define EXTI_SWIER_SWIER19_Msk    (0x1U << EXTI_SWIER_SWIER19_Pos)             /*!< 0x00080000 */
#define EXTI_SWIER_SWIER19        EXTI_SWIER_SWIER19_Msk                       /*!< Software Interrupt on line 19 */


/* References Defines */
#define EXTI_SWIER_SWI0 EXTI_SWIER_SWIER0
#define EXTI_SWIER_SWI1 EXTI_SWIER_SWIER1
#define EXTI_SWIER_SWI2 EXTI_SWIER_SWIER2
#define EXTI_SWIER_SWI3 EXTI_SWIER_SWIER3
#define EXTI_SWIER_SWI4 EXTI_SWIER_SWIER4
#define EXTI_SWIER_SWI5 EXTI_SWIER_SWIER5
#define EXTI_SWIER_SWI6 EXTI_SWIER_SWIER6
#define EXTI_SWIER_SWI7 EXTI_SWIER_SWIER7
#define EXTI_SWIER_SWI8 EXTI_SWIER_SWIER8
#define EXTI_SWIER_SWI9 EXTI_SWIER_SWIER9
#define EXTI_SWIER_SWI10 EXTI_SWIER_SWIER10
#define EXTI_SWIER_SWI11 EXTI_SWIER_SWIER11
#define EXTI_SWIER_SWI12 EXTI_SWIER_SWIER12
#define EXTI_SWIER_SWI13 EXTI_SWIER_SWIER13
#define EXTI_SWIER_SWI14 EXTI_SWIER_SWIER14
#define EXTI_SWIER_SWI15 EXTI_SWIER_SWIER15
#define EXTI_SWIER_SWI16 EXTI_SWIER_SWIER16
#define EXTI_SWIER_SWI17 EXTI_SWIER_SWIER17
#define EXTI_SWIER_SWI18 EXTI_SWIER_SWIER18
#define EXTI_SWIER_SWI19 EXTI_SWIER_SWIER19


/******************  Bit definition for EXTI_PR register  *********************/
#define EXTI_PR_PR0_Pos           (0U)                                         
#define EXTI_PR_PR0_Msk           (0x1U << EXTI_PR_PR0_Pos)                    /*!< 0x00000001 */
#define EXTI_PR_PR0               EXTI_PR_PR0_Msk                              /*!< Pending bit 0  */
#define EXTI_PR_PR1_Pos           (1U)                                         
#define EXTI_PR_PR1_Msk           (0x1U << EXTI_PR_PR1_Pos)                    /*!< 0x00000002 */
#define EXTI_PR_PR1               EXTI_PR_PR1_Msk                              /*!< Pending bit 1  */
#define EXTI_PR_PR2_Pos           (2U)                                         
#define EXTI_PR_PR2_Msk           (0x1U << EXTI_PR_PR2_Pos)                    /*!< 0x00000004 */
#define EXTI_PR_PR2               EXTI_PR_PR2_Msk                              /*!< Pending bit 2  */
#define EXTI_PR_PR3_Pos           (3U)                                         
#define EXTI_PR_PR3_Msk           (0x1U << EXTI_PR_PR3_Pos)                    /*!< 0x00000008 */
#define EXTI_PR_PR3               EXTI_PR_PR3_Msk                              /*!< Pending bit 3  */
#define EXTI_PR_PR4_Pos           (4U)                                         
#define EXTI_PR_PR4_Msk           (0x1U << EXTI_PR_PR4_Pos)                    /*!< 0x00000010 */
#define EXTI_PR_PR4               EXTI_PR_PR4_Msk                              /*!< Pending bit 4  */
#define EXTI_PR_PR5_Pos           (5U)                                         
#define EXTI_PR_PR5_Msk           (0x1U << EXTI_PR_PR5_Pos)                    /*!< 0x00000020 */
#define EXTI_PR_PR5               EXTI_PR_PR5_Msk                              /*!< Pending bit 5  */
#define EXTI_PR_PR6_Pos           (6U)                                         
#define EXTI_PR_PR6_Msk           (0x1U << EXTI_PR_PR6_Pos)                    /*!< 0x00000040 */
#define EXTI_PR_PR6               EXTI_PR_PR6_Msk                              /*!< Pending bit 6  */
#define EXTI_PR_PR7_Pos           (7U)                                         
#define EXTI_PR_PR7_Msk           (0x1U << EXTI_PR_PR7_Pos)                    /*!< 0x00000080 */
#define EXTI_PR_PR7               EXTI_PR_PR7_Msk                              /*!< Pending bit 7  */
#define EXTI_PR_PR8_Pos           (8U)                                         
#define EXTI_PR_PR8_Msk           (0x1U << EXTI_PR_PR8_Pos)                    /*!< 0x00000100 */
#define EXTI_PR_PR8               EXTI_PR_PR8_Msk                              /*!< Pending bit 8  */
#define EXTI_PR_PR9_Pos           (9U)                                         
#define EXTI_PR_PR9_Msk           (0x1U << EXTI_PR_PR9_Pos)                    /*!< 0x00000200 */
#define EXTI_PR_PR9               EXTI_PR_PR9_Msk                              /*!< Pending bit 9  */
#define EXTI_PR_PR10_Pos          (10U)                                        
#define EXTI_PR_PR10_Msk          (0x1U << EXTI_PR_PR10_Pos)                   /*!< 0x00000400 */
#define EXTI_PR_PR10              EXTI_PR_PR10_Msk                             /*!< Pending bit 10 */
#define EXTI_PR_PR11_Pos          (11U)                                        
#define EXTI_PR_PR11_Msk          (0x1U << EXTI_PR_PR11_Pos)                   /*!< 0x00000800 */
#define EXTI_PR_PR11              EXTI_PR_PR11_Msk                             /*!< Pending bit 11 */
#define EXTI_PR_PR12_Pos          (12U)                                        
#define EXTI_PR_PR12_Msk          (0x1U << EXTI_PR_PR12_Pos)                   /*!< 0x00001000 */
#define EXTI_PR_PR12              EXTI_PR_PR12_Msk                             /*!< Pending bit 12 */
#define EXTI_PR_PR13_Pos          (13U)                                        
#define EXTI_PR_PR13_Msk          (0x1U << EXTI_PR_PR13_Pos)                   /*!< 0x00002000 */
#define EXTI_PR_PR13              EXTI_PR_PR13_Msk                             /*!< Pending bit 13 */
#define EXTI_PR_PR14_Pos          (14U)                                        
#define EXTI_PR_PR14_Msk          (0x1U << EXTI_PR_PR14_Pos)                   /*!< 0x00004000 */
#define EXTI_PR_PR14              EXTI_PR_PR14_Msk                             /*!< Pending bit 14 */
#define EXTI_PR_PR15_Pos          (15U)                                        
#define EXTI_PR_PR15_Msk          (0x1U << EXTI_PR_PR15_Pos)                   /*!< 0x00008000 */
#define EXTI_PR_PR15              EXTI_PR_PR15_Msk                             /*!< Pending bit 15 */
#define EXTI_PR_PR16_Pos          (16U)                                        
#define EXTI_PR_PR16_Msk          (0x1U << EXTI_PR_PR16_Pos)                   /*!< 0x00010000 */
#define EXTI_PR_PR16              EXTI_PR_PR16_Msk                             /*!< Pending bit 16 */
#define EXTI_PR_PR17_Pos          (17U)                                        
#define EXTI_PR_PR17_Msk          (0x1U << EXTI_PR_PR17_Pos)                   /*!< 0x00020000 */
#define EXTI_PR_PR17              EXTI_PR_PR17_Msk                             /*!< Pending bit 17 */
#define EXTI_PR_PR18_Pos          (18U)                                        
#define EXTI_PR_PR18_Msk          (0x1U << EXTI_PR_PR18_Pos)                   /*!< 0x00040000 */
#define EXTI_PR_PR18              EXTI_PR_PR18_Msk                             /*!< Pending bit 18 */
#define EXTI_PR_PR19_Pos          (19U)                                        
#define EXTI_PR_PR19_Msk          (0x1U << EXTI_PR_PR19_Pos)                   /*!< 0x00080000 */
#define EXTI_PR_PR19              EXTI_PR_PR19_Msk                             /*!< Pending bit 19 */

/* References Defines */
#define EXTI_PR_PIF0 EXTI_PR_PR0
#define EXTI_PR_PIF1 EXTI_PR_PR1
#define EXTI_PR_PIF2 EXTI_PR_PR2
#define EXTI_PR_PIF3 EXTI_PR_PR3
#define EXTI_PR_PIF4 EXTI_PR_PR4
#define EXTI_PR_PIF5 EXTI_PR_PR5
#define EXTI_PR_PIF6 EXTI_PR_PR6
#define EXTI_PR_PIF7 EXTI_PR_PR7
#define EXTI_PR_PIF8 EXTI_PR_PR8
#define EXTI_PR_PIF9 EXTI_PR_PR9
#define EXTI_PR_PIF10 EXTI_PR_PR10
#define EXTI_PR_PIF11 EXTI_PR_PR11
#define EXTI_PR_PIF12 EXTI_PR_PR12
#define EXTI_PR_PIF13 EXTI_PR_PR13
#define EXTI_PR_PIF14 EXTI_PR_PR14
#define EXTI_PR_PIF15 EXTI_PR_PR15
#define EXTI_PR_PIF16 EXTI_PR_PR16
#define EXTI_PR_PIF17 EXTI_PR_PR17
#define EXTI_PR_PIF18 EXTI_PR_PR18
#define EXTI_PR_PIF19 EXTI_PR_PR19

#define EXTICFG_WKUP_SEL_Pos          (2U)                                        
#define EXTICFG_WKUP_SEL_Msk          (0x3U << EXTICFG_WKUP_SEL_Pos)
#define EXTICFG_WKUP_SEL              EXTICFG_WKUP_SEL_Msk

#define EXTICFG_DLY_SEL_Pos          (0U)                                        
#define EXTICFG_DLY_SEL_Msk          (0x3U << EXTICFG_DLY_SEL_Pos)
#define EXTICFG_DLY_SEL              EXTICFG_DLY_SEL_Msk


/******************************************************************************/
/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for FLASH_ACR register  ******************/
#define FLASH_ACR_LATENCY_Pos             (0U)                                 
#define FLASH_ACR_LATENCY_Msk             (0x7U << FLASH_ACR_LATENCY_Pos)      /*!< 0x00000001 */
#define FLASH_ACR_LATENCY                 FLASH_ACR_LATENCY_Msk                /*!< LATENCY bit (Latency) */

//#define FLASH_ACR_PRFTBE_Pos              (4U)                                 
//#define FLASH_ACR_PRFTBE_Msk              (0x1U << FLASH_ACR_PRFTBE_Pos)       /*!< 0x00000010 */
//#define FLASH_ACR_PRFTBE                  FLASH_ACR_PRFTBE_Msk                 /*!< Prefetch Buffer Enable */
//#define FLASH_ACR_PRFTBS_Pos              (5U)                                 
//#define FLASH_ACR_PRFTBS_Msk              (0x1U << FLASH_ACR_PRFTBS_Pos)       /*!< 0x00000020 */
//#define FLASH_ACR_PRFTBS                  FLASH_ACR_PRFTBS_Msk                 /*!< Prefetch Buffer Status */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define FLASH_KEYR_FKEYR_Pos              (0U)                                 
#define FLASH_KEYR_FKEYR_Msk              (0xFFFFFFFFU << FLASH_KEYR_FKEYR_Pos) /*!< 0xFFFFFFFF */
#define FLASH_KEYR_FKEYR                  FLASH_KEYR_FKEYR_Msk                 /*!< FPEC Key */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define FLASH_OPTKEYR_OPTKEYR_Pos         (0U)                                 
#define FLASH_OPTKEYR_OPTKEYR_Msk         (0xFFFFFFFFU << FLASH_OPTKEYR_OPTKEYR_Pos) /*!< 0xFFFFFFFF */
#define FLASH_OPTKEYR_OPTKEYR             FLASH_OPTKEYR_OPTKEYR_Msk            /*!< Option Byte Key */


/******************  FLASH Keys  **********************************************/
#define FLASH_KEY1_Pos                    (0U)                                 
#define FLASH_KEY1_Msk                    (0x45670123U << FLASH_KEY1_Pos)      /*!< 0x45670123 */
#define FLASH_KEY1                        FLASH_KEY1_Msk                       /*!< Flash program erase key1 */
#define FLASH_KEY2_Pos                    (0U)                                 
#define FLASH_KEY2_Msk                    (0xCDEF89ABU << FLASH_KEY2_Pos)      /*!< 0xCDEF89AB */
#define FLASH_KEY2                        FLASH_KEY2_Msk                       /*!< Flash program erase key2: used with FLASH_PEKEY1
                                                                                to unlock the write access to the FPEC. */
                                                               
#define FLASH_OPTKEY1_Pos                 (0U)                                 
#define FLASH_OPTKEY1_Msk                 (0x45670123U << FLASH_OPTKEY1_Pos)   /*!< 0x45670123 */
#define FLASH_OPTKEY1                     FLASH_OPTKEY1_Msk                    /*!< Flash option key1 */
#define FLASH_OPTKEY2_Pos                 (0U)                                 
#define FLASH_OPTKEY2_Msk                 (0xCDEF89ABU << FLASH_OPTKEY2_Pos)   /*!< 0xCDEF89AB */
#define FLASH_OPTKEY2                     FLASH_OPTKEY2_Msk                    /*!< Flash option key2: used with FLASH_OPTKEY1 to
                                                                                unlock the write access to the option byte block */

/******************  Bit definition for FLASH_SR register  *******************/
#define FLASH_SR_BSY_Pos                  (0U)                                 
#define FLASH_SR_BSY_Msk                  (0x1U << FLASH_SR_BSY_Pos)           /*!< 0x00000001 */
#define FLASH_SR_BSY                      FLASH_SR_BSY_Msk                     /*!< Busy */
#define FLASH_SR_PGERR_Pos                (2U)                                 
#define FLASH_SR_PGERR_Msk                (0x1U << FLASH_SR_PGERR_Pos)         /*!< 0x00000004 */
#define FLASH_SR_PGERR                    FLASH_SR_PGERR_Msk                   /*!< Programming Error */
#define FLASH_SR_WRPRTERR_Pos             (4U)                                 
#define FLASH_SR_WRPRTERR_Msk             (0x1U << FLASH_SR_WRPRTERR_Pos)      /*!< 0x00000010 */
#define FLASH_SR_WRPRTERR                 FLASH_SR_WRPRTERR_Msk                /*!< Write Protection Error */
#define FLASH_SR_EOP_Pos                  (5U)                                 
#define FLASH_SR_EOP_Msk                  (0x1U << FLASH_SR_EOP_Pos)           /*!< 0x00000020 */
#define FLASH_SR_EOP                      FLASH_SR_EOP_Msk                     /*!< End of operation */
#define  FLASH_SR_WRPERR                  FLASH_SR_WRPRTERR             		   /*!< Legacy of Write Protection Error */

/*******************  Bit definition for FLASH_CR register  *******************/
#define FLASH_CR_PG_Pos                   (0U)                                 
#define FLASH_CR_PG_Msk                   (0x1U << FLASH_CR_PG_Pos)            /*!< 0x00000001 */
#define FLASH_CR_PG                       FLASH_CR_PG_Msk                      /*!< Programming */
#define FLASH_CR_PER_Pos                  (1U)                                 
#define FLASH_CR_PER_Msk                  (0x1U << FLASH_CR_PER_Pos)           /*!< 0x00000002 */
#define FLASH_CR_PER                      FLASH_CR_PER_Msk                     /*!< Page Erase */
#define FLASH_CR_MER_Pos                  (2U)                                 
#define FLASH_CR_MER_Msk                  (0x1U << FLASH_CR_MER_Pos)           /*!< 0x00000004 */
#define FLASH_CR_MER                      FLASH_CR_MER_Msk                     /*!< Mass Erase */
#define FLASH_CR_OPTPG_Pos                (4U)                                 
#define FLASH_CR_OPTPG_Msk                (0x1U << FLASH_CR_OPTPG_Pos)         /*!< 0x00000010 */
#define FLASH_CR_OPTPG                    FLASH_CR_OPTPG_Msk                   /*!< Option Byte Programming */
#define FLASH_CR_OPTER_Pos                (5U)                                 
#define FLASH_CR_OPTER_Msk                (0x1U << FLASH_CR_OPTER_Pos)         /*!< 0x00000020 */
#define FLASH_CR_OPTER                    FLASH_CR_OPTER_Msk                   /*!< Option Byte Erase */
#define FLASH_CR_STRT_Pos                 (6U)                                 
#define FLASH_CR_STRT_Msk                 (0x1U << FLASH_CR_STRT_Pos)          /*!< 0x00000040 */
#define FLASH_CR_STRT                     FLASH_CR_STRT_Msk                    /*!< Start */
#define FLASH_CR_LOCK_Pos                 (7U)                                 
#define FLASH_CR_LOCK_Msk                 (0x1U << FLASH_CR_LOCK_Pos)          /*!< 0x00000080 */
#define FLASH_CR_LOCK                     FLASH_CR_LOCK_Msk                    /*!< Lock */
#define FLASH_CR_OPTWRE_Pos               (9U)                                 
#define FLASH_CR_OPTWRE_Msk               (0x1U << FLASH_CR_OPTWRE_Pos)        /*!< 0x00000200 */
#define FLASH_CR_OPTWRE                   FLASH_CR_OPTWRE_Msk                  /*!< Option Bytes Write Enable */
#define FLASH_CR_ERRIE_Pos                (10U)                                
#define FLASH_CR_ERRIE_Msk                (0x1U << FLASH_CR_ERRIE_Pos)         /*!< 0x00000400 */
#define FLASH_CR_ERRIE                    FLASH_CR_ERRIE_Msk                   /*!< Error Interrupt Enable */
#define FLASH_CR_EOPIE_Pos                (12U)                                
#define FLASH_CR_EOPIE_Msk                (0x1U << FLASH_CR_EOPIE_Pos)         /*!< 0x00001000 */
#define FLASH_CR_EOPIE                    FLASH_CR_EOPIE_Msk                   /*!< End of operation interrupt enable */
#define FLASH_CR_OBL_LAUNCH_Pos           (13U)                                
#define FLASH_CR_OBL_LAUNCH_Msk           (0x1U << FLASH_CR_OBL_LAUNCH_Pos)    /*!< 0x00002000 */
#define FLASH_CR_OBL_LAUNCH               FLASH_CR_OBL_LAUNCH_Msk              /*!< Option Bytes Loader Launch */

/*******************  Bit definition for FLASH_AR register  *******************/
#define FLASH_AR_FAR_Pos                  (0U)                                 
#define FLASH_AR_FAR_Msk                  (0xFFFFFFFFU << FLASH_AR_FAR_Pos)    /*!< 0xFFFFFFFF */
#define FLASH_AR_FAR                      FLASH_AR_FAR_Msk                     /*!< Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define FLASH_OBR_RDP_Pos                 (0U)     
#define FLASH_OBR_RDP_Msk                 (0xFFUL << FLASH_OBR_RDP_Pos)        /*!< 0x000000FF */
#define FLASH_OBR_RDP                     FLASH_OBR_RDP_Msk                    /*!< Read Protection */
#define FLASH_OBR_USER_Pos                (8U)
#define FLASH_OBR_USER_Msk                (0xFFU << FLASH_OBR_USER_Pos)        /*!< 0x0000FF00 */
#define FLASH_OBR_USER                    FLASH_OBR_USER_Msk                   /*!< User Option Bytes */
#define FLASH_OBR_IWDG_SW_Pos             (8U)                                 
#define FLASH_OBR_IWDG_SW_Msk             (0x1U << FLASH_OBR_IWDG_SW_Pos)      /*!< 0x00000100 */
#define FLASH_OBR_IWDG_SW                 FLASH_OBR_IWDG_SW_Msk                /*!< IWDG SW */
//#define FLASH_OBR_nRST_STOP_Pos           (9U)                                 
//#define FLASH_OBR_nRST_STOP_Msk           (0x1U << FLASH_OBR_nRST_STOP_Pos)    /*!< 0x00000200 */
//#define FLASH_OBR_nRST_STOP               FLASH_OBR_nRST_STOP_Msk              /*!< nRST_STOP */
//#define FLASH_OBR_nRST_STDBY_Pos          (10U)                                
//#define FLASH_OBR_nRST_STDBY_Msk          (0x1U << FLASH_OBR_nRST_STDBY_Pos)   /*!< 0x00000400 */
//#define FLASH_OBR_nRST_STDBY              FLASH_OBR_nRST_STDBY_Msk             /*!< nRST_STDBY */
#define FLASH_OBR_BOOT0_Pos               (11U)                                
#define FLASH_OBR_BOOT0_Msk               (0x1U << FLASH_OBR_BOOT0_Pos)        /*!< 0x00000800 */
#define FLASH_OBR_BOOT0                   FLASH_OBR_BOOT0_Msk                  /*!< BOOT0 */
#define FLASH_OBR_BOOT1_Pos               (12U)                                
#define FLASH_OBR_BOOT1_Msk               (0x1U << FLASH_OBR_BOOT1_Pos)        /*!< 0x00001000 */
#define FLASH_OBR_BOOT1                   FLASH_OBR_BOOT1_Msk                  /*!< BOOT1 */
//#define FLASH_OBR_VDDA_MONITOR_Pos        (13U)                                
//#define FLASH_OBR_VDDA_MONITOR_Msk        (0x1U << FLASH_OBR_VDDA_MONITOR_Pos) /*!< 0x00002000 */
//#define FLASH_OBR_VDDA_MONITOR            FLASH_OBR_VDDA_MONITOR_Msk           /*!< VDDA power supply supervisor */
//#define FLASH_OBR_RAM_PARITY_CHECK_Pos    (14U)                                
//#define FLASH_OBR_RAM_PARITY_CHECK_Msk    (0x1U << FLASH_OBR_RAM_PARITY_CHECK_Pos) /*!< 0x00004000 */
//#define FLASH_OBR_RAM_PARITY_CHECK        FLASH_OBR_RAM_PARITY_CHECK_Msk       /*!< RAM parity check */
#define FLASH_OBR_BOOT0SEL_Pos            (15U)                                
#define FLASH_OBR_BOOT0SEL_Msk            (0x1U << FLASH_OBR_BOOT0SEL_Pos)     /*!< 0x00008000 */
#define FLASH_OBR_BOOT0SEL                FLASH_OBR_BOOT0SEL_Msk               /*!< BOOT0 Selection */
#define FLASH_OBR_DATA0_Pos               (16U)                                
#define FLASH_OBR_DATA0_Msk               (0xFFU << FLASH_OBR_DATA0_Pos)       /*!< 0x00FF0000 */
#define FLASH_OBR_DATA0                   FLASH_OBR_DATA0_Msk                  /*!< Data0 */
#define FLASH_OBR_DATA1_Pos               (24U)                                
#define FLASH_OBR_DATA1_Msk               (0xFFU << FLASH_OBR_DATA1_Pos)       /*!< 0xFF000000 */
#define FLASH_OBR_DATA1                   FLASH_OBR_DATA1_Msk                  /*!< Data1 */

/******************  Bit definition for FLASH_WRPR0 register  ******************/
#define FLASH_WRPR0_WRP_Pos                (0U)                                 
#define FLASH_WRPR0_WRP_Msk                (0xFFFFFFFFUL << FLASH_WRPR0_WRP_Pos)      /*!< 0xFFFFFFFF */
#define FLASH_WRPR0_WRP                    FLASH_WRPR0_WRP_Msk                        /*!< Write Protect */

/******************  Bit definition for FLASH_WRPR1 register  ******************/
#define FLASH_WRPR1_WRP_Pos                (0U)                                 
#define FLASH_WRPR1_WRP_Msk                (0xFFFFFFFFUL << FLASH_WRPR1_WRP_Pos)      /*!< 0xFFFFFFFF */
#define FLASH_WRPR1_WRP                    FLASH_WRPR1_WRP_Msk                        /*!< Write Protect */

/******************  Bit definition for FLASH_WRPR2 register  ******************/
#define FLASH_WRPR2_WRP_Pos                (0U)                                 
#define FLASH_WRPR2_WRP_Msk                (0xFFFFFFFFUL << FLASH_WRPR2_WRP_Pos)      /*!< 0xFFFFFFFF */
#define FLASH_WRPR2_WRP                    FLASH_WRPR2_WRP_Msk                        /*!< Write Protect */

/******************  Bit definition for FLASH_WRPR3 register  ******************/
#define FLASH_WRPR3_WRP_Pos                (0U)                                 
#define FLASH_WRPR3_WRP_Msk                (0xFFFFFFFFUL << FLASH_WRPR3_WRP_Pos)      /*!< 0xFFFFFFFF */
#define FLASH_WRPR3_WRP                    FLASH_WRPR3_WRP_Msk                        /*!< Write Protect */

/******************************************************************************/
/*                                                                            */
/*                       General Purpose IOs (GPIO)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0_Pos           (0U)                                   
#define GPIO_MODER_MODER0_Msk           (0x3U << GPIO_MODER_MODER0_Pos)        /*!< 0x00000003 */
#define GPIO_MODER_MODER0               GPIO_MODER_MODER0_Msk                  
#define GPIO_MODER_MODER0_0             (0x0U << GPIO_MODER_MODER0_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER0_1             (0x1U << GPIO_MODER_MODER0_Pos)        /*!< 0x00000001 */
#define GPIO_MODER_MODER0_2             (0x2U << GPIO_MODER_MODER0_Pos)        /*!< 0x00000002 */
#define GPIO_MODER_MODER0_3             (0x3U << GPIO_MODER_MODER0_Pos)        /*!< 0x00000003 */
#define GPIO_MODER_MODER1_Pos           (2U)                                   
#define GPIO_MODER_MODER1_Msk           (0x3U << GPIO_MODER_MODER1_Pos)        /*!< 0x0000000C */
#define GPIO_MODER_MODER1               GPIO_MODER_MODER1_Msk                  
#define GPIO_MODER_MODER1_0             (0x0U << GPIO_MODER_MODER1_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER1_1             (0x1U << GPIO_MODER_MODER1_Pos)        /*!< 0x00000004 */
#define GPIO_MODER_MODER1_2             (0x2U << GPIO_MODER_MODER1_Pos)        /*!< 0x00000008 */
#define GPIO_MODER_MODER1_3             (0x3U << GPIO_MODER_MODER1_Pos)        /*!< 0x0000000C */
#define GPIO_MODER_MODER2_Pos           (4U)                                   
#define GPIO_MODER_MODER2_Msk           (0x3U << GPIO_MODER_MODER2_Pos)        /*!< 0x00000030 */
#define GPIO_MODER_MODER2               GPIO_MODER_MODER2_Msk                  
#define GPIO_MODER_MODER2_0             (0x0U << GPIO_MODER_MODER2_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER2_1             (0x1U << GPIO_MODER_MODER2_Pos)        /*!< 0x00000010 */
#define GPIO_MODER_MODER2_2             (0x2U << GPIO_MODER_MODER2_Pos)        /*!< 0x00000020 */
#define GPIO_MODER_MODER2_3             (0x3U << GPIO_MODER_MODER2_Pos)        /*!< 0x00000030 */
#define GPIO_MODER_MODER3_Pos           (6U)                                   
#define GPIO_MODER_MODER3_Msk           (0x3U << GPIO_MODER_MODER3_Pos)        /*!< 0x000000C0 */
#define GPIO_MODER_MODER3               GPIO_MODER_MODER3_Msk                  
#define GPIO_MODER_MODER3_0             (0x0U << GPIO_MODER_MODER3_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER3_1             (0x1U << GPIO_MODER_MODER3_Pos)        /*!< 0x00000040 */
#define GPIO_MODER_MODER3_2             (0x2U << GPIO_MODER_MODER3_Pos)        /*!< 0x00000080 */
#define GPIO_MODER_MODER3_3             (0x3U << GPIO_MODER_MODER3_Pos)        /*!< 0x000000C0 */
#define GPIO_MODER_MODER4_Pos           (8U)                                   
#define GPIO_MODER_MODER4_Msk           (0x3U << GPIO_MODER_MODER4_Pos)        /*!< 0x00000300 */
#define GPIO_MODER_MODER4               GPIO_MODER_MODER4_Msk                  
#define GPIO_MODER_MODER4_0             (0x0U << GPIO_MODER_MODER4_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER4_1             (0x1U << GPIO_MODER_MODER4_Pos)        /*!< 0x00000100 */
#define GPIO_MODER_MODER4_2             (0x2U << GPIO_MODER_MODER4_Pos)        /*!< 0x00000200 */
#define GPIO_MODER_MODER4_3             (0x3U << GPIO_MODER_MODER4_Pos)        /*!< 0x00000300 */
#define GPIO_MODER_MODER5_Pos           (10U)                                  
#define GPIO_MODER_MODER5_Msk           (0x3U << GPIO_MODER_MODER5_Pos)        /*!< 0x00000C00 */
#define GPIO_MODER_MODER5               GPIO_MODER_MODER5_Msk                  
#define GPIO_MODER_MODER5_0             (0x0U << GPIO_MODER_MODER5_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER5_1             (0x1U << GPIO_MODER_MODER5_Pos)        /*!< 0x00000400 */
#define GPIO_MODER_MODER5_2             (0x2U << GPIO_MODER_MODER5_Pos)        /*!< 0x00000800 */
#define GPIO_MODER_MODER5_3             (0x3U << GPIO_MODER_MODER5_Pos)        /*!< 0x00000C00 */
#define GPIO_MODER_MODER6_Pos           (12U)                                  
#define GPIO_MODER_MODER6_Msk           (0x3U << GPIO_MODER_MODER6_Pos)        /*!< 0x00003000 */
#define GPIO_MODER_MODER6               GPIO_MODER_MODER6_Msk                  
#define GPIO_MODER_MODER6_0             (0x0U << GPIO_MODER_MODER6_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER6_1             (0x1U << GPIO_MODER_MODER6_Pos)        /*!< 0x00001000 */
#define GPIO_MODER_MODER6_2             (0x2U << GPIO_MODER_MODER6_Pos)        /*!< 0x00002000 */
#define GPIO_MODER_MODER6_3             (0x3U << GPIO_MODER_MODER6_Pos)        /*!< 0x00003000 */
#define GPIO_MODER_MODER7_Pos           (14U)                                  
#define GPIO_MODER_MODER7_Msk           (0x3U << GPIO_MODER_MODER7_Pos)        /*!< 0x0000C000 */
#define GPIO_MODER_MODER7               GPIO_MODER_MODER7_Msk                  
#define GPIO_MODER_MODER7_0             (0x0U << GPIO_MODER_MODER7_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER7_1             (0x1U << GPIO_MODER_MODER7_Pos)        /*!< 0x00004000 */
#define GPIO_MODER_MODER7_2             (0x2U << GPIO_MODER_MODER7_Pos)        /*!< 0x00008000 */
#define GPIO_MODER_MODER7_3             (0x3U << GPIO_MODER_MODER7_Pos)        /*!< 0x0000C000 */
#define GPIO_MODER_MODER8_Pos           (16U)                                  
#define GPIO_MODER_MODER8_Msk           (0x3U << GPIO_MODER_MODER8_Pos)        /*!< 0x00030000 */
#define GPIO_MODER_MODER8               GPIO_MODER_MODER8_Msk                  
#define GPIO_MODER_MODER8_0             (0x0U << GPIO_MODER_MODER8_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER8_1             (0x1U << GPIO_MODER_MODER8_Pos)        /*!< 0x00010000 */
#define GPIO_MODER_MODER8_2             (0x2U << GPIO_MODER_MODER8_Pos)        /*!< 0x00020000 */
#define GPIO_MODER_MODER8_3             (0x3U << GPIO_MODER_MODER8_Pos)        /*!< 0x00030000 */
#define GPIO_MODER_MODER9_Pos           (18U)                                  
#define GPIO_MODER_MODER9_Msk           (0x3U << GPIO_MODER_MODER9_Pos)        /*!< 0x000C0000 */
#define GPIO_MODER_MODER9               GPIO_MODER_MODER9_Msk                  
#define GPIO_MODER_MODER9_0             (0x0U << GPIO_MODER_MODER9_Pos)        /*!< 0x00000000 */
#define GPIO_MODER_MODER9_1             (0x1U << GPIO_MODER_MODER9_Pos)        /*!< 0x00040000 */
#define GPIO_MODER_MODER9_2             (0x2U << GPIO_MODER_MODER9_Pos)        /*!< 0x00080000 */
#define GPIO_MODER_MODER9_3             (0x3U << GPIO_MODER_MODER9_Pos)        /*!< 0x000C0000 */
#define GPIO_MODER_MODER10_Pos          (20U)                                  
#define GPIO_MODER_MODER10_Msk          (0x3U << GPIO_MODER_MODER10_Pos)       /*!< 0x00300000 */
#define GPIO_MODER_MODER10              GPIO_MODER_MODER10_Msk                 
#define GPIO_MODER_MODER10_0            (0x0U << GPIO_MODER_MODER10_Pos)       /*!< 0x00000000 */
#define GPIO_MODER_MODER10_1            (0x1U << GPIO_MODER_MODER10_Pos)       /*!< 0x00100000 */
#define GPIO_MODER_MODER10_2            (0x2U << GPIO_MODER_MODER10_Pos)       /*!< 0x00200000 */
#define GPIO_MODER_MODER10_3            (0x3U << GPIO_MODER_MODER10_Pos)       /*!< 0x00300000 */
#define GPIO_MODER_MODER11_Pos          (22U)                                  
#define GPIO_MODER_MODER11_Msk          (0x3U << GPIO_MODER_MODER11_Pos)       /*!< 0x00C00000 */
#define GPIO_MODER_MODER11              GPIO_MODER_MODER11_Msk                 
#define GPIO_MODER_MODER11_0            (0x0U << GPIO_MODER_MODER11_Pos)       /*!< 0x00000000 */
#define GPIO_MODER_MODER11_1            (0x1U << GPIO_MODER_MODER11_Pos)       /*!< 0x00400000 */
#define GPIO_MODER_MODER11_2            (0x2U << GPIO_MODER_MODER11_Pos)       /*!< 0x00800000 */
#define GPIO_MODER_MODER11_3            (0x3U << GPIO_MODER_MODER11_Pos)       /*!< 0x00C00000 */
#define GPIO_MODER_MODER12_Pos          (24U)                                  
#define GPIO_MODER_MODER12_Msk          (0x3U << GPIO_MODER_MODER12_Pos)       /*!< 0x03000000 */
#define GPIO_MODER_MODER12              GPIO_MODER_MODER12_Msk                 
#define GPIO_MODER_MODER12_0            (0x0U << GPIO_MODER_MODER12_Pos)       /*!< 0x00000000 */
#define GPIO_MODER_MODER12_1            (0x1U << GPIO_MODER_MODER12_Pos)       /*!< 0x01000000 */
#define GPIO_MODER_MODER12_2            (0x2U << GPIO_MODER_MODER12_Pos)       /*!< 0x02000000 */
#define GPIO_MODER_MODER12_3            (0x3U << GPIO_MODER_MODER12_Pos)       /*!< 0x03000000 */
#define GPIO_MODER_MODER13_Pos          (26U)                                  
#define GPIO_MODER_MODER13_Msk          (0x3U << GPIO_MODER_MODER13_Pos)       /*!< 0x0C000000 */
#define GPIO_MODER_MODER13              GPIO_MODER_MODER13_Msk                 
#define GPIO_MODER_MODER13_0            (0x0U << GPIO_MODER_MODER13_Pos)       /*!< 0x00000000 */
#define GPIO_MODER_MODER13_1            (0x1U << GPIO_MODER_MODER13_Pos)       /*!< 0x04000000 */
#define GPIO_MODER_MODER13_2            (0x2U << GPIO_MODER_MODER13_Pos)       /*!< 0x08000000 */
#define GPIO_MODER_MODER13_3            (0x3U << GPIO_MODER_MODER13_Pos)       /*!< 0x0C000000 */
#define GPIO_MODER_MODER14_Pos          (28U)                                  
#define GPIO_MODER_MODER14_Msk          (0x3U << GPIO_MODER_MODER14_Pos)       /*!< 0x30000000 */
#define GPIO_MODER_MODER14              GPIO_MODER_MODER14_Msk                 
#define GPIO_MODER_MODER14_0            (0x0U << GPIO_MODER_MODER14_Pos)       /*!< 0x00000000 */
#define GPIO_MODER_MODER14_1            (0x1U << GPIO_MODER_MODER14_Pos)       /*!< 0x10000000 */
#define GPIO_MODER_MODER14_2            (0x2U << GPIO_MODER_MODER14_Pos)       /*!< 0x20000000 */
#define GPIO_MODER_MODER14_3            (0x3U << GPIO_MODER_MODER14_Pos)       /*!< 0x30000000 */
#define GPIO_MODER_MODER15_Pos          (30U)                                  
#define GPIO_MODER_MODER15_Msk          (0x3U << GPIO_MODER_MODER15_Pos)       /*!< 0xC0000000 */
#define GPIO_MODER_MODER15              GPIO_MODER_MODER15_Msk                 
#define GPIO_MODER_MODER15_0            (0x0U << GPIO_MODER_MODER15_Pos)       /*!< 0x00000000 */
#define GPIO_MODER_MODER15_1            (0x1U << GPIO_MODER_MODER15_Pos)       /*!< 0x40000000 */
#define GPIO_MODER_MODER15_2            (0x2U << GPIO_MODER_MODER15_Pos)       /*!< 0x80000000 */
#define GPIO_MODER_MODER15_3            (0x3U << GPIO_MODER_MODER15_Pos)       /*!< 0xC0000000 */



/******************  Bit definition for GPIO_OTYPER register  *****************/
#define GPIO_OTYPER_OD_0                (0x00000001U)                          
#define GPIO_OTYPER_OD_1                (0x00000002U)                          
#define GPIO_OTYPER_OD_2                (0x00000004U)                          
#define GPIO_OTYPER_OD_3                (0x00000008U)                          
#define GPIO_OTYPER_OD_4                (0x00000010U)                          
#define GPIO_OTYPER_OD_5                (0x00000020U)                          
#define GPIO_OTYPER_OD_6                (0x00000040U)                          
#define GPIO_OTYPER_OD_7                (0x00000080U)                          
#define GPIO_OTYPER_OD_8                (0x00000100U)                          
#define GPIO_OTYPER_OD_9                (0x00000200U)                          
#define GPIO_OTYPER_OD_10               (0x00000400U)                          
#define GPIO_OTYPER_OD_11               (0x00000800U)                          
#define GPIO_OTYPER_OD_12               (0x00001000U)                          
#define GPIO_OTYPER_OD_13               (0x00002000U)                          
#define GPIO_OTYPER_OD_14               (0x00004000U)                          
#define GPIO_OTYPER_OD_15               (0x00008000U)  
#define GPIO_OTYPER_OS_0                (0x00010000U)                          
#define GPIO_OTYPER_OS_1                (0x00020000U)                          
#define GPIO_OTYPER_OS_2                (0x00040000U)                          
#define GPIO_OTYPER_OS_3                (0x00080000U)                          
#define GPIO_OTYPER_OS_4                (0x00100000U)                          
#define GPIO_OTYPER_OS_5                (0x00200000U)                          
#define GPIO_OTYPER_OS_6                (0x00400000U)                          
#define GPIO_OTYPER_OS_7                (0x00800000U)                          
#define GPIO_OTYPER_OS_8                (0x01000000U)                          
#define GPIO_OTYPER_OS_9                (0x02000000U)                          
#define GPIO_OTYPER_OS_10               (0x04000000U)                          
#define GPIO_OTYPER_OS_11               (0x08000000U)                          
#define GPIO_OTYPER_OS_12               (0x10000000U)                          
#define GPIO_OTYPER_OS_13               (0x20000000U)                          
#define GPIO_OTYPER_OS_14               (0x40000000U)                          
#define GPIO_OTYPER_OS_15               (0x80000000U)                         



/******************  Bit definition for GPIO_ODRIVER register  *****************/
#define GPIO_ODRIVER_0                  (0x00000003U)
#define GPIO_ODRIVER_1                  (0x0000000CU)                          
#define GPIO_ODRIVER_2                  (0x00000030U)                          
#define GPIO_ODRIVER_3                  (0x000000C0U)                          
#define GPIO_ODRIVER_4                  (0x00000300U)                          
#define GPIO_ODRIVER_5                  (0x00000C00U)                          
#define GPIO_ODRIVER_6                  (0x00003000U)                          
#define GPIO_ODRIVER_7                  (0x0000C000U)                          
#define GPIO_ODRIVER_8                  (0x00030000U)                          
#define GPIO_ODRIVER_9                  (0x000C0000U)                          
#define GPIO_ODRIVER_10                 (0x00300000U)                          
#define GPIO_ODRIVER_11                 (0x00C00000U)                          
#define GPIO_ODRIVER_12                 (0x03000000U)                          
#define GPIO_ODRIVER_13                 (0x0C000000U)                          
#define GPIO_ODRIVER_14                 (0x30000000U)                          
#define GPIO_ODRIVER_15                 (0xC0000000U)



/****************  Bit definition for GPIO_ITYPER register  ******************/
#define GPIO_ITYPER_0                  (0x00000001U)                          
#define GPIO_ITYPER_1                  (0x00000002U)                          
#define GPIO_ITYPER_2                  (0x00000004U)                          
#define GPIO_ITYPER_3                  (0x00000008U)                          
#define GPIO_ITYPER_4                  (0x00000010U)                          
#define GPIO_ITYPER_5                  (0x00000020U)                          
#define GPIO_ITYPER_6                  (0x00000040U)                          
#define GPIO_ITYPER_7                  (0x00000080U)                          
#define GPIO_ITYPER_8                  (0x00000100U)                          
#define GPIO_ITYPER_9                  (0x00000200U)                          
#define GPIO_ITYPER_10                 (0x00000400U)                          
#define GPIO_ITYPER_11                 (0x00000800U)                          
#define GPIO_ITYPER_12                 (0x00001000U)                          
#define GPIO_ITYPER_13                 (0x00002000U)                          
#define GPIO_ITYPER_14                 (0x00004000U)                          
#define GPIO_ITYPER_15                 (0x00008000U)


/*******************  Bit definition for GPIO_PUPDR register ******************/
#define GPIO_PUPDR_PUPDR0_Pos           (0U)                                   
#define GPIO_PUPDR_PUPDR0_Msk           (0x3U << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPDR0               GPIO_PUPDR_PUPDR0_Msk                  
#define GPIO_PUPDR_PUPDR0_0             (0x0U << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR0_1             (0x1U << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000001 */
#define GPIO_PUPDR_PUPDR0_2             (0x2U << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000002 */
#define GPIO_PUPDR_PUPDR0_3             (0x3U << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000002 */
#define GPIO_PUPDR_PUPDR1_Pos           (2U)                                   
#define GPIO_PUPDR_PUPDR1_Msk           (0x3U << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x0000000C */
#define GPIO_PUPDR_PUPDR1               GPIO_PUPDR_PUPDR1_Msk                  
#define GPIO_PUPDR_PUPDR1_0             (0x0U << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR1_1             (0x1U << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x00000004 */
#define GPIO_PUPDR_PUPDR1_2             (0x2U << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x00000008 */
#define GPIO_PUPDR_PUPDR1_3             (0x3U << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x00000008 */
#define GPIO_PUPDR_PUPDR2_Pos           (4U)                                   
#define GPIO_PUPDR_PUPDR2_Msk           (0x3U << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000030 */
#define GPIO_PUPDR_PUPDR2               GPIO_PUPDR_PUPDR2_Msk                  
#define GPIO_PUPDR_PUPDR2_0             (0x0U << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR2_1             (0x1U << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000010 */
#define GPIO_PUPDR_PUPDR2_2             (0x2U << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000020 */
#define GPIO_PUPDR_PUPDR2_3             (0x3U << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000020 */
#define GPIO_PUPDR_PUPDR3_Pos           (6U)                                   
#define GPIO_PUPDR_PUPDR3_Msk           (0x3U << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPDR3               GPIO_PUPDR_PUPDR3_Msk                  
#define GPIO_PUPDR_PUPDR3_0             (0x0U << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR3_1             (0x1U << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x00000040 */
#define GPIO_PUPDR_PUPDR3_2             (0x2U << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x00000080 */
#define GPIO_PUPDR_PUPDR3_3             (0x3U << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x00000080 */
#define GPIO_PUPDR_PUPDR4_Pos           (8U)                                   
#define GPIO_PUPDR_PUPDR4_Msk           (0x3U << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000300 */
#define GPIO_PUPDR_PUPDR4               GPIO_PUPDR_PUPDR4_Msk                  
#define GPIO_PUPDR_PUPDR4_0             (0x0U << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR4_1             (0x1U << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000100 */
#define GPIO_PUPDR_PUPDR4_2             (0x2U << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000200 */
#define GPIO_PUPDR_PUPDR4_3             (0x3U << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000200 */
#define GPIO_PUPDR_PUPDR5_Pos           (10U)                                  
#define GPIO_PUPDR_PUPDR5_Msk           (0x3U << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPDR5               GPIO_PUPDR_PUPDR5_Msk                  
#define GPIO_PUPDR_PUPDR5_0             (0x0U << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR5_1             (0x1U << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000400 */
#define GPIO_PUPDR_PUPDR5_2             (0x2U << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000800 */
#define GPIO_PUPDR_PUPDR5_3             (0x3U << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000800 */
#define GPIO_PUPDR_PUPDR6_Pos           (12U)                                  
#define GPIO_PUPDR_PUPDR6_Msk           (0x3U << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00003000 */
#define GPIO_PUPDR_PUPDR6               GPIO_PUPDR_PUPDR6_Msk                  
#define GPIO_PUPDR_PUPDR6_0             (0x0U << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR6_1             (0x1U << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00001000 */
#define GPIO_PUPDR_PUPDR6_2             (0x2U << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00002000 */
#define GPIO_PUPDR_PUPDR6_3             (0x3U << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00002000 */
#define GPIO_PUPDR_PUPDR7_Pos           (14U)                                  
#define GPIO_PUPDR_PUPDR7_Msk           (0x3U << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPDR7               GPIO_PUPDR_PUPDR7_Msk                  
#define GPIO_PUPDR_PUPDR7_0             (0x0U << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR7_1             (0x1U << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x00004000 */
#define GPIO_PUPDR_PUPDR7_2             (0x2U << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x00008000 */
#define GPIO_PUPDR_PUPDR7_3             (0x3U << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x00008000 */
#define GPIO_PUPDR_PUPDR8_Pos           (16U)                                  
#define GPIO_PUPDR_PUPDR8_Msk           (0x3U << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00030000 */
#define GPIO_PUPDR_PUPDR8               GPIO_PUPDR_PUPDR8_Msk                  
#define GPIO_PUPDR_PUPDR8_0             (0x0U << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR8_1             (0x1U << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00010000 */
#define GPIO_PUPDR_PUPDR8_2             (0x2U << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00020000 */
#define GPIO_PUPDR_PUPDR8_3             (0x3U << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00020000 */
#define GPIO_PUPDR_PUPDR9_Pos           (18U)                                  
#define GPIO_PUPDR_PUPDR9_Msk           (0x3U << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPDR9               GPIO_PUPDR_PUPDR9_Msk                  
#define GPIO_PUPDR_PUPDR9_0             (0x0U << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR9_1             (0x1U << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x00040000 */
#define GPIO_PUPDR_PUPDR9_2             (0x2U << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x00080000 */
#define GPIO_PUPDR_PUPDR9_3             (0x3U << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x00080000 */
#define GPIO_PUPDR_PUPDR10_Pos          (20U)                                  
#define GPIO_PUPDR_PUPDR10_Msk          (0x3U << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00300000 */
#define GPIO_PUPDR_PUPDR10              GPIO_PUPDR_PUPDR10_Msk                 
#define GPIO_PUPDR_PUPDR10_0            (0x0U << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR10_1            (0x1U << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00100000 */
#define GPIO_PUPDR_PUPDR10_2            (0x2U << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00200000 */
#define GPIO_PUPDR_PUPDR10_3            (0x3U << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00200000 */
#define GPIO_PUPDR_PUPDR11_Pos          (22U)                                  
#define GPIO_PUPDR_PUPDR11_Msk          (0x3U << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPDR11              GPIO_PUPDR_PUPDR11_Msk                 
#define GPIO_PUPDR_PUPDR11_0            (0x0U << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR11_1            (0x1U << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00400000 */
#define GPIO_PUPDR_PUPDR11_2            (0x2U << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00800000 */
#define GPIO_PUPDR_PUPDR11_3            (0x3U << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00800000 */
#define GPIO_PUPDR_PUPDR12_Pos          (24U)                                  
#define GPIO_PUPDR_PUPDR12_Msk          (0x3U << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x03000000 */
#define GPIO_PUPDR_PUPDR12              GPIO_PUPDR_PUPDR12_Msk                 
#define GPIO_PUPDR_PUPDR12_0            (0x0U << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR12_1            (0x1U << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x01000000 */
#define GPIO_PUPDR_PUPDR12_2            (0x2U << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x02000000 */
#define GPIO_PUPDR_PUPDR12_3            (0x3U << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x02000000 */
#define GPIO_PUPDR_PUPDR13_Pos          (26U)                                  
#define GPIO_PUPDR_PUPDR13_Msk          (0x3U << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPDR13              GPIO_PUPDR_PUPDR13_Msk                 
#define GPIO_PUPDR_PUPDR13_0            (0x0U << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR13_1            (0x1U << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x04000000 */
#define GPIO_PUPDR_PUPDR13_2            (0x2U << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x08000000 */
#define GPIO_PUPDR_PUPDR13_3            (0x3U << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x08000000 */
#define GPIO_PUPDR_PUPDR14_Pos          (28U)                                  
#define GPIO_PUPDR_PUPDR14_Msk          (0x3U << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x30000000 */
#define GPIO_PUPDR_PUPDR14              GPIO_PUPDR_PUPDR14_Msk                 
#define GPIO_PUPDR_PUPDR14_0            (0x0U << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR14_1            (0x1U << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x10000000 */
#define GPIO_PUPDR_PUPDR14_2            (0x2U << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x20000000 */
#define GPIO_PUPDR_PUPDR14_3            (0x3U << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x20000000 */
#define GPIO_PUPDR_PUPDR15_Pos          (30U)                                  
#define GPIO_PUPDR_PUPDR15_Msk          (0x3U << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPDR15              GPIO_PUPDR_PUPDR15_Msk                 
#define GPIO_PUPDR_PUPDR15_0            (0x0U << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0x00000000 */
#define GPIO_PUPDR_PUPDR15_1            (0x1U << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0x40000000 */
#define GPIO_PUPDR_PUPDR15_2            (0x2U << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0x80000000 */
#define GPIO_PUPDR_PUPDR15_3            (0x3U << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0x80000000 */

/*******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_0                      (0x00000001U)                          
#define GPIO_IDR_1                      (0x00000002U)                          
#define GPIO_IDR_2                      (0x00000004U)                          
#define GPIO_IDR_3                      (0x00000008U)                          
#define GPIO_IDR_4                      (0x00000010U)                          
#define GPIO_IDR_5                      (0x00000020U)                          
#define GPIO_IDR_6                      (0x00000040U)                          
#define GPIO_IDR_7                      (0x00000080U)                          
#define GPIO_IDR_8                      (0x00000100U)                          
#define GPIO_IDR_9                      (0x00000200U)                          
#define GPIO_IDR_10                     (0x00000400U)                          
#define GPIO_IDR_11                     (0x00000800U)                          
#define GPIO_IDR_12                     (0x00001000U)                          
#define GPIO_IDR_13                     (0x00002000U)                          
#define GPIO_IDR_14                     (0x00004000U)                          
#define GPIO_IDR_15                     (0x00008000U)                          

/******************  Bit definition for GPIO_ODR register  ********************/
#define GPIO_ODR_0                      (0x00000001U)                          
#define GPIO_ODR_1                      (0x00000002U)                          
#define GPIO_ODR_2                      (0x00000004U)                          
#define GPIO_ODR_3                      (0x00000008U)                          
#define GPIO_ODR_4                      (0x00000010U)                          
#define GPIO_ODR_5                      (0x00000020U)                          
#define GPIO_ODR_6                      (0x00000040U)                          
#define GPIO_ODR_7                      (0x00000080U)                          
#define GPIO_ODR_8                      (0x00000100U)                          
#define GPIO_ODR_9                      (0x00000200U)                          
#define GPIO_ODR_10                     (0x00000400U)                          
#define GPIO_ODR_11                     (0x00000800U)                          
#define GPIO_ODR_12                     (0x00001000U)                          
#define GPIO_ODR_13                     (0x00002000U)                          
#define GPIO_ODR_14                     (0x00004000U)                          
#define GPIO_ODR_15                     (0x00008000U)                          

/****************** Bit definition for GPIO_BSRR register  ********************/
#define GPIO_BSRR_BS_0                  (0x00000001U)                          
#define GPIO_BSRR_BS_1                  (0x00000002U)                          
#define GPIO_BSRR_BS_2                  (0x00000004U)                          
#define GPIO_BSRR_BS_3                  (0x00000008U)                          
#define GPIO_BSRR_BS_4                  (0x00000010U)                          
#define GPIO_BSRR_BS_5                  (0x00000020U)                          
#define GPIO_BSRR_BS_6                  (0x00000040U)                          
#define GPIO_BSRR_BS_7                  (0x00000080U)                          
#define GPIO_BSRR_BS_8                  (0x00000100U)                          
#define GPIO_BSRR_BS_9                  (0x00000200U)                          
#define GPIO_BSRR_BS_10                 (0x00000400U)                          
#define GPIO_BSRR_BS_11                 (0x00000800U)                          
#define GPIO_BSRR_BS_12                 (0x00001000U)                          
#define GPIO_BSRR_BS_13                 (0x00002000U)                          
#define GPIO_BSRR_BS_14                 (0x00004000U)                          
#define GPIO_BSRR_BS_15                 (0x00008000U)                          
#define GPIO_BSRR_BR_0                  (0x00010000U)                          
#define GPIO_BSRR_BR_1                  (0x00020000U)                          
#define GPIO_BSRR_BR_2                  (0x00040000U)                          
#define GPIO_BSRR_BR_3                  (0x00080000U)                          
#define GPIO_BSRR_BR_4                  (0x00100000U)                          
#define GPIO_BSRR_BR_5                  (0x00200000U)                          
#define GPIO_BSRR_BR_6                  (0x00400000U)                          
#define GPIO_BSRR_BR_7                  (0x00800000U)                          
#define GPIO_BSRR_BR_8                  (0x01000000U)                          
#define GPIO_BSRR_BR_9                  (0x02000000U)                          
#define GPIO_BSRR_BR_10                 (0x04000000U)                          
#define GPIO_BSRR_BR_11                 (0x08000000U)                          
#define GPIO_BSRR_BR_12                 (0x10000000U)                          
#define GPIO_BSRR_BR_13                 (0x20000000U)                          
#define GPIO_BSRR_BR_14                 (0x40000000U)                          
#define GPIO_BSRR_BR_15                 (0x80000000U)                          

/****************** Bit definition for GPIO_LCKR register  ********************/
#define GPIO_LCKR_LCK0_Pos              (0U)                                   
#define GPIO_LCKR_LCK0_Msk              (0x1U << GPIO_LCKR_LCK0_Pos)           /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                  GPIO_LCKR_LCK0_Msk                     
#define GPIO_LCKR_LCK1_Pos              (1U)                                   
#define GPIO_LCKR_LCK1_Msk              (0x1U << GPIO_LCKR_LCK1_Pos)           /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                  GPIO_LCKR_LCK1_Msk                     
#define GPIO_LCKR_LCK2_Pos              (2U)                                   
#define GPIO_LCKR_LCK2_Msk              (0x1U << GPIO_LCKR_LCK2_Pos)           /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                  GPIO_LCKR_LCK2_Msk                     
#define GPIO_LCKR_LCK3_Pos              (3U)                                   
#define GPIO_LCKR_LCK3_Msk              (0x1U << GPIO_LCKR_LCK3_Pos)           /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                  GPIO_LCKR_LCK3_Msk                     
#define GPIO_LCKR_LCK4_Pos              (4U)                                   
#define GPIO_LCKR_LCK4_Msk              (0x1U << GPIO_LCKR_LCK4_Pos)           /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                  GPIO_LCKR_LCK4_Msk                     
#define GPIO_LCKR_LCK5_Pos              (5U)                                   
#define GPIO_LCKR_LCK5_Msk              (0x1U << GPIO_LCKR_LCK5_Pos)           /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                  GPIO_LCKR_LCK5_Msk                     
#define GPIO_LCKR_LCK6_Pos              (6U)                                   
#define GPIO_LCKR_LCK6_Msk              (0x1U << GPIO_LCKR_LCK6_Pos)           /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                  GPIO_LCKR_LCK6_Msk                     
#define GPIO_LCKR_LCK7_Pos              (7U)                                   
#define GPIO_LCKR_LCK7_Msk              (0x1U << GPIO_LCKR_LCK7_Pos)           /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                  GPIO_LCKR_LCK7_Msk                     
#define GPIO_LCKR_LCK8_Pos              (8U)                                   
#define GPIO_LCKR_LCK8_Msk              (0x1U << GPIO_LCKR_LCK8_Pos)           /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                  GPIO_LCKR_LCK8_Msk                     
#define GPIO_LCKR_LCK9_Pos              (9U)                                   
#define GPIO_LCKR_LCK9_Msk              (0x1U << GPIO_LCKR_LCK9_Pos)           /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                  GPIO_LCKR_LCK9_Msk                     
#define GPIO_LCKR_LCK10_Pos             (10U)                                  
#define GPIO_LCKR_LCK10_Msk             (0x1U << GPIO_LCKR_LCK10_Pos)          /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                 GPIO_LCKR_LCK10_Msk                    
#define GPIO_LCKR_LCK11_Pos             (11U)                                  
#define GPIO_LCKR_LCK11_Msk             (0x1U << GPIO_LCKR_LCK11_Pos)          /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                 GPIO_LCKR_LCK11_Msk                    
#define GPIO_LCKR_LCK12_Pos             (12U)                                  
#define GPIO_LCKR_LCK12_Msk             (0x1U << GPIO_LCKR_LCK12_Pos)          /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                 GPIO_LCKR_LCK12_Msk                    
#define GPIO_LCKR_LCK13_Pos             (13U)                                  
#define GPIO_LCKR_LCK13_Msk             (0x1U << GPIO_LCKR_LCK13_Pos)          /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                 GPIO_LCKR_LCK13_Msk                    
#define GPIO_LCKR_LCK14_Pos             (14U)                                  
#define GPIO_LCKR_LCK14_Msk             (0x1U << GPIO_LCKR_LCK14_Pos)          /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                 GPIO_LCKR_LCK14_Msk                    
#define GPIO_LCKR_LCK15_Pos             (15U)                                  
#define GPIO_LCKR_LCK15_Msk             (0x1U << GPIO_LCKR_LCK15_Pos)          /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                 GPIO_LCKR_LCK15_Msk                    
#define GPIO_LCKR_LCKK_Pos              (16U)                                  
#define GPIO_LCKR_LCKK_Msk              (0x1U << GPIO_LCKR_LCKK_Pos)           /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                  GPIO_LCKR_LCKK_Msk                     

/****************** Bit definition for GPIO_AFRL register  ********************/
#define GPIO_AFRL_AFSEL0_Pos            (0U)                                   
#define GPIO_AFRL_AFSEL0_Msk            (0xFU << GPIO_AFRL_AFSEL0_Pos)         /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0                GPIO_AFRL_AFSEL0_Msk                    
#define GPIO_AFRL_AFSEL1_Pos            (4U)                                   
#define GPIO_AFRL_AFSEL1_Msk            (0xFU << GPIO_AFRL_AFSEL1_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1                GPIO_AFRL_AFSEL1_Msk                    
#define GPIO_AFRL_AFSEL2_Pos            (8U)                                   
#define GPIO_AFRL_AFSEL2_Msk            (0xFU << GPIO_AFRL_AFSEL2_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2                GPIO_AFRL_AFSEL2_Msk                    
#define GPIO_AFRL_AFSEL3_Pos            (12U)                                  
#define GPIO_AFRL_AFSEL3_Msk            (0xFU << GPIO_AFRL_AFSEL3_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3                GPIO_AFRL_AFSEL3_Msk                    
#define GPIO_AFRL_AFSEL4_Pos            (16U)                                  
#define GPIO_AFRL_AFSEL4_Msk            (0xFU << GPIO_AFRL_AFSEL4_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4                GPIO_AFRL_AFSEL4_Msk                    
#define GPIO_AFRL_AFSEL5_Pos            (20U)                                  
#define GPIO_AFRL_AFSEL5_Msk            (0xFU << GPIO_AFRL_AFSEL5_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5                GPIO_AFRL_AFSEL5_Msk                    
#define GPIO_AFRL_AFSEL6_Pos            (24U)                                  
#define GPIO_AFRL_AFSEL6_Msk            (0xFU << GPIO_AFRL_AFSEL6_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6                GPIO_AFRL_AFSEL6_Msk                    
#define GPIO_AFRL_AFSEL7_Pos            (28U)                                  
#define GPIO_AFRL_AFSEL7_Msk            (0xFU << GPIO_AFRL_AFSEL7_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRL_AFSEL7                GPIO_AFRL_AFSEL7_Msk  

/* Legacy aliases */                  
#define GPIO_AFRL_AFRL0_Pos             GPIO_AFRL_AFSEL0_Pos                                  
#define GPIO_AFRL_AFRL0_Msk             GPIO_AFRL_AFSEL0_Msk
#define GPIO_AFRL_AFRL0                 GPIO_AFRL_AFSEL0
#define GPIO_AFRL_AFRL1_Pos             GPIO_AFRL_AFSEL1_Pos
#define GPIO_AFRL_AFRL1_Msk             GPIO_AFRL_AFSEL1_Msk
#define GPIO_AFRL_AFRL1                 GPIO_AFRL_AFSEL1
#define GPIO_AFRL_AFRL2_Pos             GPIO_AFRL_AFSEL2_Pos
#define GPIO_AFRL_AFRL2_Msk             GPIO_AFRL_AFSEL2_Msk
#define GPIO_AFRL_AFRL2                 GPIO_AFRL_AFSEL2
#define GPIO_AFRL_AFRL3_Pos             GPIO_AFRL_AFSEL3_Pos
#define GPIO_AFRL_AFRL3_Msk             GPIO_AFRL_AFSEL3_Msk
#define GPIO_AFRL_AFRL3                 GPIO_AFRL_AFSEL3
#define GPIO_AFRL_AFRL4_Pos             GPIO_AFRL_AFSEL4_Pos
#define GPIO_AFRL_AFRL4_Msk             GPIO_AFRL_AFSEL4_Msk
#define GPIO_AFRL_AFRL4                 GPIO_AFRL_AFSEL4
#define GPIO_AFRL_AFRL5_Pos             GPIO_AFRL_AFSEL5_Pos
#define GPIO_AFRL_AFRL5_Msk             GPIO_AFRL_AFSEL5_Msk
#define GPIO_AFRL_AFRL5                 GPIO_AFRL_AFSEL5
#define GPIO_AFRL_AFRL6_Pos             GPIO_AFRL_AFSEL6_Pos
#define GPIO_AFRL_AFRL6_Msk             GPIO_AFRL_AFSEL6_Msk
#define GPIO_AFRL_AFRL6                 GPIO_AFRL_AFSEL6
#define GPIO_AFRL_AFRL7_Pos             GPIO_AFRL_AFSEL7_Pos
#define GPIO_AFRL_AFRL7_Msk             GPIO_AFRL_AFSEL7_Msk
#define GPIO_AFRL_AFRL7                 GPIO_AFRL_AFSEL7
 
/****************** Bit definition for GPIO_AFRH register  ********************/
#define GPIO_AFRH_AFSEL8_Pos            (0U)                                   
#define GPIO_AFRH_AFSEL8_Msk            (0xFU << GPIO_AFRH_AFSEL8_Pos)         /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8                GPIO_AFRH_AFSEL8_Msk                    
#define GPIO_AFRH_AFSEL9_Pos            (4U)                                   
#define GPIO_AFRH_AFSEL9_Msk            (0xFU << GPIO_AFRH_AFSEL9_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9                GPIO_AFRH_AFSEL9_Msk                    
#define GPIO_AFRH_AFSEL10_Pos           (8U)                                   
#define GPIO_AFRH_AFSEL10_Msk           (0xFU << GPIO_AFRH_AFSEL10_Pos)        /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10               GPIO_AFRH_AFSEL10_Msk                    
#define GPIO_AFRH_AFSEL11_Pos           (12U)                                  
#define GPIO_AFRH_AFSEL11_Msk           (0xFU << GPIO_AFRH_AFSEL11_Pos)        /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11               GPIO_AFRH_AFSEL11_Msk                    
#define GPIO_AFRH_AFSEL12_Pos           (16U)                                  
#define GPIO_AFRH_AFSEL12_Msk           (0xFU << GPIO_AFRH_AFSEL12_Pos)        /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12               GPIO_AFRH_AFSEL12_Msk                    
#define GPIO_AFRH_AFSEL13_Pos           (20U)                                  
#define GPIO_AFRH_AFSEL13_Msk           (0xFU << GPIO_AFRH_AFSEL13_Pos)        /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13               GPIO_AFRH_AFSEL13_Msk                    
#define GPIO_AFRH_AFSEL14_Pos           (24U)                                  
#define GPIO_AFRH_AFSEL14_Msk           (0xFU << GPIO_AFRH_AFSEL14_Pos)        /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14               GPIO_AFRH_AFSEL14_Msk                    
#define GPIO_AFRH_AFSEL15_Pos           (28U)                                  
#define GPIO_AFRH_AFSEL15_Msk           (0xFU << GPIO_AFRH_AFSEL15_Pos)        /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15               GPIO_AFRH_AFSEL15_Msk                    

/* Legacy aliases */                  
#define GPIO_AFRH_AFRH0_Pos             GPIO_AFRH_AFSEL8_Pos
#define GPIO_AFRH_AFRH0_Msk             GPIO_AFRH_AFSEL8_Msk
#define GPIO_AFRH_AFRH0                 GPIO_AFRH_AFSEL8
#define GPIO_AFRH_AFRH1_Pos             GPIO_AFRH_AFSEL9_Pos
#define GPIO_AFRH_AFRH1_Msk             GPIO_AFRH_AFSEL9_Msk
#define GPIO_AFRH_AFRH1                 GPIO_AFRH_AFSEL9
#define GPIO_AFRH_AFRH2_Pos             GPIO_AFRH_AFSEL10_Pos
#define GPIO_AFRH_AFRH2_Msk             GPIO_AFRH_AFSEL10_Msk
#define GPIO_AFRH_AFRH2                 GPIO_AFRH_AFSEL10
#define GPIO_AFRH_AFRH3_Pos             GPIO_AFRH_AFSEL11_Pos
#define GPIO_AFRH_AFRH3_Msk             GPIO_AFRH_AFSEL11_Msk
#define GPIO_AFRH_AFRH3                 GPIO_AFRH_AFSEL11
#define GPIO_AFRH_AFRH4_Pos             GPIO_AFRH_AFSEL12_Pos
#define GPIO_AFRH_AFRH4_Msk             GPIO_AFRH_AFSEL12_Msk
#define GPIO_AFRH_AFRH4                 GPIO_AFRH_AFSEL12
#define GPIO_AFRH_AFRH5_Pos             GPIO_AFRH_AFSEL13_Pos
#define GPIO_AFRH_AFRH5_Msk             GPIO_AFRH_AFSEL13_Msk
#define GPIO_AFRH_AFRH5                 GPIO_AFRH_AFSEL13
#define GPIO_AFRH_AFRH6_Pos             GPIO_AFRH_AFSEL14_Pos
#define GPIO_AFRH_AFRH6_Msk             GPIO_AFRH_AFSEL14_Msk
#define GPIO_AFRH_AFRH6                 GPIO_AFRH_AFSEL14
#define GPIO_AFRH_AFRH7_Pos             GPIO_AFRH_AFSEL15_Pos
#define GPIO_AFRH_AFRH7_Msk             GPIO_AFRH_AFSEL15_Msk
#define GPIO_AFRH_AFRH7                 GPIO_AFRH_AFSEL15

/****************** Bit definition for GPIO_BRR register  *********************/
#define GPIO_BRR_BR_0                   (0x00000001U)                          
#define GPIO_BRR_BR_1                   (0x00000002U)                          
#define GPIO_BRR_BR_2                   (0x00000004U)                          
#define GPIO_BRR_BR_3                   (0x00000008U)                          
#define GPIO_BRR_BR_4                   (0x00000010U)                          
#define GPIO_BRR_BR_5                   (0x00000020U)                          
#define GPIO_BRR_BR_6                   (0x00000040U)                          
#define GPIO_BRR_BR_7                   (0x00000080U)                          
#define GPIO_BRR_BR_8                   (0x00000100U)                          
#define GPIO_BRR_BR_9                   (0x00000200U)                          
#define GPIO_BRR_BR_10                  (0x00000400U)                          
#define GPIO_BRR_BR_11                  (0x00000800U)                          
#define GPIO_BRR_BR_12                  (0x00001000U)                          
#define GPIO_BRR_BR_13                  (0x00002000U)                          
#define GPIO_BRR_BR_14                  (0x00004000U)                          
#define GPIO_BRR_BR_15                  (0x00008000U) 

/****************** Bit definition for GPIO_BTR register  *********************/
#define GPIO_BTR_BT_0                   (0x00000001U)                          
#define GPIO_BTR_BT_1                   (0x00000002U)                          
#define GPIO_BTR_BT_2                   (0x00000004U)                          
#define GPIO_BTR_BT_3                   (0x00000008U)                          
#define GPIO_BTR_BT_4                   (0x00000010U)                          
#define GPIO_BTR_BT_5                   (0x00000020U)                          
#define GPIO_BTR_BT_6                   (0x00000040U)                          
#define GPIO_BTR_BT_7                   (0x00000080U)                          
#define GPIO_BTR_BT_8                   (0x00000100U)                          
#define GPIO_BTR_BT_9                   (0x00000200U)                          
#define GPIO_BTR_BT_10                  (0x00000400U)                          
#define GPIO_BTR_BT_11                  (0x00000800U)                          
#define GPIO_BTR_BT_12                  (0x00001000U)                          
#define GPIO_BTR_BT_13                  (0x00002000U)                          
#define GPIO_BTR_BT_14                  (0x00004000U)                          
#define GPIO_BTR_BT_15                  (0x00008000U)


/****************** Bit definition for GPIOA_DBGDISR register  ********************/
#define GPIOA_DBGDISR_DBGDIS_Pos        (1U)                                   
#define GPIOA_DBGDISR_DBGDIS_Msk        (0x1U << GPIOA_DBGDISR_DBGDIS_Pos)         /*!< 0x00000002 */
#define GPIOA_DBGDISR_DBGDIS            GPIOA_DBGDISR_DBGDIS_Msk  

/******************************************************************************/
/*                                                                            */
/*                   Inter-integrated Circuit Interface (I2C)                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  *******************/
#define I2C_CR1_PE_Pos               (0U)                                      
#define I2C_CR1_PE_Msk               (0x1U << I2C_CR1_PE_Pos)                  /*!< 0x00000001 */
#define I2C_CR1_PE                   I2C_CR1_PE_Msk                            /*!< Peripheral enable */
#define I2C_CR1_TXIE_Pos             (1U)                                      
#define I2C_CR1_TXIE_Msk             (0x1U << I2C_CR1_TXIE_Pos)                /*!< 0x00000002 */
#define I2C_CR1_TXIE                 I2C_CR1_TXIE_Msk                          /*!< TX interrupt enable */
#define I2C_CR1_RXIE_Pos             (2U)                                      
#define I2C_CR1_RXIE_Msk             (0x1U << I2C_CR1_RXIE_Pos)                /*!< 0x00000004 */
#define I2C_CR1_RXIE                 I2C_CR1_RXIE_Msk                          /*!< RX interrupt enable */
#define I2C_CR1_ADDRIE_Pos           (3U)                                      
#define I2C_CR1_ADDRIE_Msk           (0x1U << I2C_CR1_ADDRIE_Pos)              /*!< 0x00000008 */
#define I2C_CR1_ADDRIE               I2C_CR1_ADDRIE_Msk                        /*!< Address match interrupt enable */
#define I2C_CR1_NACKIE_Pos           (4U)                                      
#define I2C_CR1_NACKIE_Msk           (0x1U << I2C_CR1_NACKIE_Pos)              /*!< 0x00000010 */
#define I2C_CR1_NACKIE               I2C_CR1_NACKIE_Msk                        /*!< NACK received interrupt enable */
#define I2C_CR1_STOPIE_Pos           (5U)                                      
#define I2C_CR1_STOPIE_Msk           (0x1U << I2C_CR1_STOPIE_Pos)              /*!< 0x00000020 */
#define I2C_CR1_STOPIE               I2C_CR1_STOPIE_Msk                        /*!< STOP detection interrupt enable */
#define I2C_CR1_TCIE_Pos             (6U)                                      
#define I2C_CR1_TCIE_Msk             (0x1U << I2C_CR1_TCIE_Pos)                /*!< 0x00000040 */
#define I2C_CR1_TCIE                 I2C_CR1_TCIE_Msk                          /*!< Transfer complete interrupt enable */
#define I2C_CR1_ERRIE_Pos            (7U)                                      
#define I2C_CR1_ERRIE_Msk            (0x1U << I2C_CR1_ERRIE_Pos)               /*!< 0x00000080 */
#define I2C_CR1_ERRIE                I2C_CR1_ERRIE_Msk                         /*!< Errors interrupt enable */
#define I2C_CR1_DNF_Pos              (8U)                                      
#define I2C_CR1_DNF_Msk              (0xFU << I2C_CR1_DNF_Pos)                 /*!< 0x00000F00 */
#define I2C_CR1_DNF                  I2C_CR1_DNF_Msk                           /*!< Digital noise filter */
#define I2C_CR1_ANFOFF_Pos           (12U)                                     
#define I2C_CR1_ANFOFF_Msk           (0x1U << I2C_CR1_ANFOFF_Pos)              /*!< 0x00001000 */
#define I2C_CR1_ANFOFF               I2C_CR1_ANFOFF_Msk                        /*!< Analog noise filter OFF */
#define I2C_CR1_ADDRNOSTRETCH_Pos    (13U)                                     
#define I2C_CR1_ADDRNOSTRETCH_Msk    (0x1U << I2C_CR1_ADDRNOSTRETCH_Pos)       /*!< 0x00002000 */
#define I2C_CR1_ADDRNOSTRETCH        I2C_CR1_ADDRNOSTRETCH_Msk                 /*!< Address stretch disable in stretch mode */
#define I2C_CR1_TXDMAEN_Pos          (14U)                                     
#define I2C_CR1_TXDMAEN_Msk          (0x1U << I2C_CR1_TXDMAEN_Pos)             /*!< 0x00004000 */
#define I2C_CR1_TXDMAEN              I2C_CR1_TXDMAEN_Msk                       /*!< DMA transmission requests enable */
#define I2C_CR1_RXDMAEN_Pos          (15U)                                     
#define I2C_CR1_RXDMAEN_Msk          (0x1U << I2C_CR1_RXDMAEN_Pos)             /*!< 0x00008000 */
#define I2C_CR1_RXDMAEN              I2C_CR1_RXDMAEN_Msk                       /*!< DMA reception requests enable */
#define I2C_CR1_SBC_Pos              (16U)                                     
#define I2C_CR1_SBC_Msk              (0x1U << I2C_CR1_SBC_Pos)                 /*!< 0x00010000 */
#define I2C_CR1_SBC                  I2C_CR1_SBC_Msk                           /*!< Slave byte control */
#define I2C_CR1_NOSTRETCH_Pos        (17U)                                     
#define I2C_CR1_NOSTRETCH_Msk        (0x1U << I2C_CR1_NOSTRETCH_Pos)           /*!< 0x00020000 */
#define I2C_CR1_NOSTRETCH            I2C_CR1_NOSTRETCH_Msk                     /*!< Clock stretching disable */
#define I2C_CR1_GCEN_Pos             (19U)                                     
#define I2C_CR1_GCEN_Msk             (0x1U << I2C_CR1_GCEN_Pos)                /*!< 0x00080000 */
#define I2C_CR1_GCEN                 I2C_CR1_GCEN_Msk                          /*!< General call enable */
#define I2C_CR1_SMBHEN_Pos           (20U)                                     
#define I2C_CR1_SMBHEN_Msk           (0x1U << I2C_CR1_SMBHEN_Pos)              /*!< 0x00100000 */
#define I2C_CR1_SMBHEN               I2C_CR1_SMBHEN_Msk                        /*!< SMBus host address enable */
#define I2C_CR1_SMBDEN_Pos           (21U)                                     
#define I2C_CR1_SMBDEN_Msk           (0x1U << I2C_CR1_SMBDEN_Pos)              /*!< 0x00200000 */
#define I2C_CR1_SMBDEN               I2C_CR1_SMBDEN_Msk                        /*!< SMBus device default address enable */
#define I2C_CR1_ALERTEN_Pos          (22U)                                     
#define I2C_CR1_ALERTEN_Msk          (0x1U << I2C_CR1_ALERTEN_Pos)             /*!< 0x00400000 */
#define I2C_CR1_ALERTEN              I2C_CR1_ALERTEN_Msk                       /*!< SMBus alert enable */
#define I2C_CR1_PECEN_Pos            (23U)                                     
#define I2C_CR1_PECEN_Msk            (0x1U << I2C_CR1_PECEN_Pos)               /*!< 0x00800000 */
#define I2C_CR1_PECEN                I2C_CR1_PECEN_Msk                         /*!< PEC enable */
#define I2C_CR1_OA2IE_Pos            (30U)                                     
#define I2C_CR1_OA2IE_Msk            (0x1U << I2C_CR1_OA2IE_Pos)               /*!< 0x4000000 */
#define I2C_CR1_OA2IE                I2C_CR1_OA2IE_Msk                         /*!< OA2 enable */
#define I2C_CR1_OA1IE_Pos            (31U)                                     
#define I2C_CR1_OA1IE_Msk            (0x1U << I2C_CR1_OA1IE_Pos)               /*!< 0x8000000 */
#define I2C_CR1_OA1IE                I2C_CR1_OA1IE_Msk                         /*!< OA1 enable */

/******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_SADD_Pos             (0U)                                      
#define I2C_CR2_SADD_Msk             (0x3FFU << I2C_CR2_SADD_Pos)              /*!< 0x000003FF */
#define I2C_CR2_SADD                 I2C_CR2_SADD_Msk                          /*!< Slave address (master mode) */
#define I2C_CR2_RD_WRN_Pos           (10U)                                     
#define I2C_CR2_RD_WRN_Msk           (0x1U << I2C_CR2_RD_WRN_Pos)              /*!< 0x00000400 */
#define I2C_CR2_RD_WRN               I2C_CR2_RD_WRN_Msk                        /*!< Transfer direction (master mode) */
#define I2C_CR2_ADD10_Pos            (11U)                                     
#define I2C_CR2_ADD10_Msk            (0x1U << I2C_CR2_ADD10_Pos)               /*!< 0x00000800 */
#define I2C_CR2_ADD10                I2C_CR2_ADD10_Msk                         /*!< 10-bit addressing mode (master mode) */
#define I2C_CR2_HEAD10R_Pos          (12U)                                     
#define I2C_CR2_HEAD10R_Msk          (0x1U << I2C_CR2_HEAD10R_Pos)             /*!< 0x00001000 */
#define I2C_CR2_HEAD10R              I2C_CR2_HEAD10R_Msk                       /*!< 10-bit address header only read direction (master mode) */
#define I2C_CR2_START_Pos            (13U)                                     
#define I2C_CR2_START_Msk            (0x1U << I2C_CR2_START_Pos)               /*!< 0x00002000 */
#define I2C_CR2_START                I2C_CR2_START_Msk                         /*!< START generation */
#define I2C_CR2_STOP_Pos             (14U)                                     
#define I2C_CR2_STOP_Msk             (0x1U << I2C_CR2_STOP_Pos)                /*!< 0x00004000 */
#define I2C_CR2_STOP                 I2C_CR2_STOP_Msk                          /*!< STOP generation (master mode) */
#define I2C_CR2_NACK_Pos             (15U)                                     
#define I2C_CR2_NACK_Msk             (0x1U << I2C_CR2_NACK_Pos)                /*!< 0x00008000 */
#define I2C_CR2_NACK                 I2C_CR2_NACK_Msk                          /*!< NACK generation (slave mode) */
#define I2C_CR2_NBYTES_Pos           (16U)                                     
#define I2C_CR2_NBYTES_Msk           (0xFFU << I2C_CR2_NBYTES_Pos)             /*!< 0x00FF0000 */
#define I2C_CR2_NBYTES               I2C_CR2_NBYTES_Msk                        /*!< Number of bytes */
#define I2C_CR2_RELOAD_Pos           (24U)                                     
#define I2C_CR2_RELOAD_Msk           (0x1U << I2C_CR2_RELOAD_Pos)              /*!< 0x01000000 */
#define I2C_CR2_RELOAD               I2C_CR2_RELOAD_Msk                        /*!< NBYTES reload mode */
#define I2C_CR2_AUTOEND_Pos          (25U)                                     
#define I2C_CR2_AUTOEND_Msk          (0x1U << I2C_CR2_AUTOEND_Pos)             /*!< 0x02000000 */
#define I2C_CR2_AUTOEND              I2C_CR2_AUTOEND_Msk                       /*!< Automatic end mode (master mode) */
#define I2C_CR2_PECBYTE_Pos          (26U)                                     
#define I2C_CR2_PECBYTE_Msk          (0x1U << I2C_CR2_PECBYTE_Pos)             /*!< 0x04000000 */
#define I2C_CR2_PECBYTE              I2C_CR2_PECBYTE_Msk                       /*!< Packet error checking byte */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define I2C_OAR1_OA1_Pos             (0U)                                      
#define I2C_OAR1_OA1_Msk             (0x3FFU << I2C_OAR1_OA1_Pos)              /*!< 0x000003FF */
#define I2C_OAR1_OA1                 I2C_OAR1_OA1_Msk                          /*!< Interface own address 1 */
#define I2C_OAR1_OA1MODE_Pos         (10U)                                     
#define I2C_OAR1_OA1MODE_Msk         (0x1U << I2C_OAR1_OA1MODE_Pos)            /*!< 0x00000400 */
#define I2C_OAR1_OA1MODE             I2C_OAR1_OA1MODE_Msk                      /*!< Own address 1 10-bit mode */
#define I2C_OAR1_OA1EN_Pos           (15U)                                     
#define I2C_OAR1_OA1EN_Msk           (0x1U << I2C_OAR1_OA1EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR1_OA1EN               I2C_OAR1_OA1EN_Msk                        /*!< Own address 1 enable */

/*******************  Bit definition for I2C_OAR2 register  ******************/
#define I2C_OAR2_OA2_Pos             (1U)                                      
#define I2C_OAR2_OA2_Msk             (0x7FU << I2C_OAR2_OA2_Pos)               /*!< 0x000000FE */
#define I2C_OAR2_OA2                 I2C_OAR2_OA2_Msk                          /*!< Interface own address 2 */
#define I2C_OAR2_OA2MSK_Pos          (8U)                                      
#define I2C_OAR2_OA2MSK_Msk          (0x7FU << I2C_OAR2_OA2MSK_Pos)            /*!< 0x00007F00 */
#define I2C_OAR2_OA2MSK              I2C_OAR2_OA2MSK_Msk                       /*!< Own address 2 masks */
#define I2C_OAR2_OA2NOMASK           (0x00000000U)                             /*!< No mask                                        */
#define I2C_OAR2_OA2MASK01_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK01_Msk       (0x1U << I2C_OAR2_OA2MASK01_Pos)          /*!< 0x00000100 */
#define I2C_OAR2_OA2MASK01           I2C_OAR2_OA2MASK01_Msk                    /*!< OA2[1] is masked, OA2[7:2] are compared        */
#define I2C_OAR2_OA2MASK02_Pos       (9U)                                      
#define I2C_OAR2_OA2MASK02_Msk       (0x1U << I2C_OAR2_OA2MASK02_Pos)          /*!< 0x00000200 */
#define I2C_OAR2_OA2MASK02           I2C_OAR2_OA2MASK02_Msk                    /*!< OA2[2] is masked, OA2[7:3, 1] are compared     */
#define I2C_OAR2_OA2MASK03_Pos       (10U)                                      
#define I2C_OAR2_OA2MASK03_Msk       (0x1U << I2C_OAR2_OA2MASK03_Pos)          /*!< 0x00000400 */
#define I2C_OAR2_OA2MASK03           I2C_OAR2_OA2MASK03_Msk                    /*!< OA2[3] is masked, OA2[7:4, 2:1] are compared   */
#define I2C_OAR2_OA2MASK04_Pos       (11U)                                     
#define I2C_OAR2_OA2MASK04_Msk       (0x1U << I2C_OAR2_OA2MASK04_Pos)          /*!< 0x00000800 */
#define I2C_OAR2_OA2MASK04           I2C_OAR2_OA2MASK04_Msk                    /*!< OA2[4] is masked, OA2[7:5, 3:1] are compared   */
#define I2C_OAR2_OA2MASK05_Pos       (12U)                                      
#define I2C_OAR2_OA2MASK05_Msk       (0x1U << I2C_OAR2_OA2MASK05_Pos)          /*!< 0x00001000 */
#define I2C_OAR2_OA2MASK05           I2C_OAR2_OA2MASK05_Msk                    /*!< OA2[5] is masked, OA2[7:6, 4:1] are compared   */
#define I2C_OAR2_OA2MASK06_Pos       (13U)                                      
#define I2C_OAR2_OA2MASK06_Msk       (0x1U << I2C_OAR2_OA2MASK06_Pos)          /*!< 0x00002000 */
#define I2C_OAR2_OA2MASK06           I2C_OAR2_OA2MASK06_Msk                    /*!< OA2[6] is masked, OA2[7, 5:1] are compared     */
#define I2C_OAR2_OA2MASK07_Pos       (14U)                                      
#define I2C_OAR2_OA2MASK07_Msk       (0x1U << I2C_OAR2_OA2MASK07_Pos)          /*!< 0x00004000 */
#define I2C_OAR2_OA2MASK07           I2C_OAR2_OA2MASK07_Msk                    /*!< OA2[7] is masked, OA2[6:1] are compared        */
#define I2C_OAR2_OA2EN_Pos           (15U)                                     
#define I2C_OAR2_OA2EN_Msk           (0x1U << I2C_OAR2_OA2EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR2_OA2EN               I2C_OAR2_OA2EN_Msk                        /*!< Own address 2 enable */

/*******************  Bit definition for I2C_TIMINGR register ****************/
#define I2C_TIMINGR_SCLL_Pos         (0U)                                      
#define I2C_TIMINGR_SCLL_Msk         (0xFFU << I2C_TIMINGR_SCLL_Pos)           /*!< 0x000000FF */
#define I2C_TIMINGR_SCLL             I2C_TIMINGR_SCLL_Msk                      /*!< SCL low period (master mode) */
#define I2C_TIMINGR_SCLH_Pos         (8U)                                      
#define I2C_TIMINGR_SCLH_Msk         (0xFFU << I2C_TIMINGR_SCLH_Pos)           /*!< 0x0000FF00 */
#define I2C_TIMINGR_SCLH             I2C_TIMINGR_SCLH_Msk                      /*!< SCL high period (master mode) */
#define I2C_TIMINGR_SDADEL_Pos       (16U)                                     
#define I2C_TIMINGR_SDADEL_Msk       (0xFU << I2C_TIMINGR_SDADEL_Pos)          /*!< 0x000F0000 */
#define I2C_TIMINGR_SDADEL           I2C_TIMINGR_SDADEL_Msk                    /*!< Data hold time */
#define I2C_TIMINGR_SCLDEL_Pos       (20U)                                     
#define I2C_TIMINGR_SCLDEL_Msk       (0xFU << I2C_TIMINGR_SCLDEL_Pos)          /*!< 0x00F00000 */
#define I2C_TIMINGR_SCLDEL           I2C_TIMINGR_SCLDEL_Msk                    /*!< Data setup time */
#define I2C_TIMINGR_PRESC_Pos        (28U)                                     
#define I2C_TIMINGR_PRESC_Msk        (0xFU << I2C_TIMINGR_PRESC_Pos)           /*!< 0xF0000000 */
#define I2C_TIMINGR_PRESC            I2C_TIMINGR_PRESC_Msk                     /*!< Timings prescaler */

/******************* Bit definition for I2C_TIMEOUTR register ****************/
#define I2C_TIMEOUTR_TIMEOUTA_Pos    (0U)                                      
#define I2C_TIMEOUTR_TIMEOUTA_Msk    (0xFFFU << I2C_TIMEOUTR_TIMEOUTA_Pos)     /*!< 0x00000FFF */
#define I2C_TIMEOUTR_TIMEOUTA        I2C_TIMEOUTR_TIMEOUTA_Msk                 /*!< Bus timeout A */
#define I2C_TIMEOUTR_TIDLE_Pos       (12U)                                     
#define I2C_TIMEOUTR_TIDLE_Msk       (0x1U << I2C_TIMEOUTR_TIDLE_Pos)          /*!< 0x00001000 */
#define I2C_TIMEOUTR_TIDLE           I2C_TIMEOUTR_TIDLE_Msk                    /*!< Idle clock timeout detection */
#define I2C_TIMEOUTR_TIMOUTEN_Pos    (15U)                                     
#define I2C_TIMEOUTR_TIMOUTEN_Msk    (0x1U << I2C_TIMEOUTR_TIMOUTEN_Pos)       /*!< 0x00008000 */
#define I2C_TIMEOUTR_TIMOUTEN        I2C_TIMEOUTR_TIMOUTEN_Msk                 /*!< Clock timeout enable */
#define I2C_TIMEOUTR_TIMEOUTB_Pos    (16U)                                     
#define I2C_TIMEOUTR_TIMEOUTB_Msk    (0xFFFU << I2C_TIMEOUTR_TIMEOUTB_Pos)     /*!< 0x0FFF0000 */
#define I2C_TIMEOUTR_TIMEOUTB        I2C_TIMEOUTR_TIMEOUTB_Msk                 /*!< Bus timeout B*/
#define I2C_TIMEOUTR_TEXTEN_Pos      (31U)                                     
#define I2C_TIMEOUTR_TEXTEN_Msk      (0x1U << I2C_TIMEOUTR_TEXTEN_Pos)         /*!< 0x80000000 */
#define I2C_TIMEOUTR_TEXTEN          I2C_TIMEOUTR_TEXTEN_Msk                   /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  ********************/
#define I2C_ISR_TXE_Pos              (0U)                                      
#define I2C_ISR_TXE_Msk              (0x1U << I2C_ISR_TXE_Pos)                 /*!< 0x00000001 */
#define I2C_ISR_TXE                  I2C_ISR_TXE_Msk                           /*!< Transmit data register empty */
#define I2C_ISR_TXIS_Pos             (1U)                                      
#define I2C_ISR_TXIS_Msk             (0x1U << I2C_ISR_TXIS_Pos)                /*!< 0x00000002 */
#define I2C_ISR_TXIS                 I2C_ISR_TXIS_Msk                          /*!< Transmit interrupt status */
#define I2C_ISR_RXNE_Pos             (2U)                                      
#define I2C_ISR_RXNE_Msk             (0x1U << I2C_ISR_RXNE_Pos)                /*!< 0x00000004 */
#define I2C_ISR_RXNE                 I2C_ISR_RXNE_Msk                          /*!< Receive data register not empty */
#define I2C_ISR_ADDR_Pos             (3U)                                      
#define I2C_ISR_ADDR_Msk             (0x1U << I2C_ISR_ADDR_Pos)                /*!< 0x00000008 */
#define I2C_ISR_ADDR                 I2C_ISR_ADDR_Msk                          /*!< Address matched (slave mode)*/
#define I2C_ISR_NACKF_Pos            (4U)                                      
#define I2C_ISR_NACKF_Msk            (0x1U << I2C_ISR_NACKF_Pos)               /*!< 0x00000010 */
#define I2C_ISR_NACKF                I2C_ISR_NACKF_Msk                         /*!< NACK received flag */
#define I2C_ISR_STOPF_Pos            (5U)                                      
#define I2C_ISR_STOPF_Msk            (0x1U << I2C_ISR_STOPF_Pos)               /*!< 0x00000020 */
#define I2C_ISR_STOPF                I2C_ISR_STOPF_Msk                         /*!< STOP detection flag */
#define I2C_ISR_TC_Pos               (6U)                                      
#define I2C_ISR_TC_Msk               (0x1U << I2C_ISR_TC_Pos)                  /*!< 0x00000040 */
#define I2C_ISR_TC                   I2C_ISR_TC_Msk                            /*!< Transfer complete (master mode) */
#define I2C_ISR_TCR_Pos              (7U)                                      
#define I2C_ISR_TCR_Msk              (0x1U << I2C_ISR_TCR_Pos)                 /*!< 0x00000080 */
#define I2C_ISR_TCR                  I2C_ISR_TCR_Msk                           /*!< Transfer complete reload */
#define I2C_ISR_BERR_Pos             (8U)                                      
#define I2C_ISR_BERR_Msk             (0x1U << I2C_ISR_BERR_Pos)                /*!< 0x00000100 */
#define I2C_ISR_BERR                 I2C_ISR_BERR_Msk                          /*!< Bus error */
#define I2C_ISR_ARLO_Pos             (9U)                                      
#define I2C_ISR_ARLO_Msk             (0x1U << I2C_ISR_ARLO_Pos)                /*!< 0x00000200 */
#define I2C_ISR_ARLO                 I2C_ISR_ARLO_Msk                          /*!< Arbitration lost */
#define I2C_ISR_OVR_Pos              (10U)                                     
#define I2C_ISR_OVR_Msk              (0x1U << I2C_ISR_OVR_Pos)                 /*!< 0x00000400 */
#define I2C_ISR_OVR                  I2C_ISR_OVR_Msk                           /*!< Overrun/Underrun */
#define I2C_ISR_PECERR_Pos           (11U)                                     
#define I2C_ISR_PECERR_Msk           (0x1U << I2C_ISR_PECERR_Pos)              /*!< 0x00000800 */
#define I2C_ISR_PECERR               I2C_ISR_PECERR_Msk                        /*!< PEC error in reception */
#define I2C_ISR_TIMEOUT_Pos          (12U)                                     
#define I2C_ISR_TIMEOUT_Msk          (0x1U << I2C_ISR_TIMEOUT_Pos)             /*!< 0x00001000 */
#define I2C_ISR_TIMEOUT              I2C_ISR_TIMEOUT_Msk                       /*!< Timeout or Tlow detection flag */
#define I2C_ISR_ALERT_Pos            (13U)                                     
#define I2C_ISR_ALERT_Msk            (0x1U << I2C_ISR_ALERT_Pos)               /*!< 0x00002000 */
#define I2C_ISR_ALERT                I2C_ISR_ALERT_Msk                         /*!< SMBus alert */
#define I2C_ISR_BUSY_Pos             (15U)                                     
#define I2C_ISR_BUSY_Msk             (0x1U << I2C_ISR_BUSY_Pos)                /*!< 0x00008000 */
#define I2C_ISR_BUSY                 I2C_ISR_BUSY_Msk                          /*!< Bus busy */
#define I2C_ISR_DIR_Pos              (16U)                                     
#define I2C_ISR_DIR_Msk              (0x1U << I2C_ISR_DIR_Pos)                 /*!< 0x00010000 */
#define I2C_ISR_DIR                  I2C_ISR_DIR_Msk                           /*!< Transfer direction (slave mode) */
#define I2C_ISR_ADDCODE_Pos          (17U)                                     
#define I2C_ISR_ADDCODE_Msk          (0x7FU << I2C_ISR_ADDCODE_Pos)            /*!< 0x00FE0000 */
#define I2C_ISR_ADDCODE              I2C_ISR_ADDCODE_Msk                       /*!< Address match code (slave mode) */
#define I2C_ISR_OA2_Pos              (30U)                                      
#define I2C_ISR_OA2_Msk              (0x1U << I2C_ISR_OA2_Pos)                /*!< 0x40000000 */
#define I2C_ISR_OA2                  I2C_ISR_OA2_Msk                          /*!< OA2 matched (slave mode)*/
#define I2C_ISR_OA1_Pos              (31U)                                      
#define I2C_ISR_OA1_Msk              (0x1U << I2C_ISR_OA1_Pos)                /*!< 0x80000000 */
#define I2C_ISR_OA1                  I2C_ISR_OA1_Msk                          /*!< OA1 matched (slave mode)*/

/******************  Bit definition for I2C_ICR register  ********************/
#define I2C_ICR_ADDRCF_Pos           (3U)                                      
#define I2C_ICR_ADDRCF_Msk           (0x1U << I2C_ICR_ADDRCF_Pos)              /*!< 0x00000008 */
#define I2C_ICR_ADDRCF               I2C_ICR_ADDRCF_Msk                        /*!< Address matched clear flag */
#define I2C_ICR_NACKCF_Pos           (4U)                                      
#define I2C_ICR_NACKCF_Msk           (0x1U << I2C_ICR_NACKCF_Pos)              /*!< 0x00000010 */
#define I2C_ICR_NACKCF               I2C_ICR_NACKCF_Msk                        /*!< NACK clear flag */
#define I2C_ICR_STOPCF_Pos           (5U)                                      
#define I2C_ICR_STOPCF_Msk           (0x1U << I2C_ICR_STOPCF_Pos)              /*!< 0x00000020 */
#define I2C_ICR_STOPCF               I2C_ICR_STOPCF_Msk                        /*!< STOP detection clear flag */
#define I2C_ICR_BERRCF_Pos           (8U)                                      
#define I2C_ICR_BERRCF_Msk           (0x1U << I2C_ICR_BERRCF_Pos)              /*!< 0x00000100 */
#define I2C_ICR_BERRCF               I2C_ICR_BERRCF_Msk                        /*!< Bus error clear flag */
#define I2C_ICR_ARLOCF_Pos           (9U)                                      
#define I2C_ICR_ARLOCF_Msk           (0x1U << I2C_ICR_ARLOCF_Pos)              /*!< 0x00000200 */
#define I2C_ICR_ARLOCF               I2C_ICR_ARLOCF_Msk                        /*!< Arbitration lost clear flag */
#define I2C_ICR_OVRCF_Pos            (10U)                                     
#define I2C_ICR_OVRCF_Msk            (0x1U << I2C_ICR_OVRCF_Pos)               /*!< 0x00000400 */
#define I2C_ICR_OVRCF                I2C_ICR_OVRCF_Msk                         /*!< Overrun/Underrun clear flag */
#define I2C_ICR_PECCF_Pos            (11U)                                     
#define I2C_ICR_PECCF_Msk            (0x1U << I2C_ICR_PECCF_Pos)               /*!< 0x00000800 */
#define I2C_ICR_PECCF                I2C_ICR_PECCF_Msk                         /*!< PAC error clear flag */
#define I2C_ICR_TIMOUTCF_Pos         (12U)                                     
#define I2C_ICR_TIMOUTCF_Msk         (0x1U << I2C_ICR_TIMOUTCF_Pos)            /*!< 0x00001000 */
#define I2C_ICR_TIMOUTCF             I2C_ICR_TIMOUTCF_Msk                      /*!< Timeout clear flag */
#define I2C_ICR_ALERTCF_Pos          (13U)                                     
#define I2C_ICR_ALERTCF_Msk          (0x1U << I2C_ICR_ALERTCF_Pos)             /*!< 0x00002000 */
#define I2C_ICR_ALERTCF              I2C_ICR_ALERTCF_Msk                       /*!< Alert clear flag */
#define I2C_ICR_OA2CF_Pos            (30U)                                      
#define I2C_ICR_OA2CF_Msk            (0x1U << I2C_ICR_OA2CF_Pos)               /*!< 0x40000000*/
#define I2C_ICR_OA2CF                I2C_ICR_OA2CF_Msk                         /*!< OA2 matched clear flag */
#define I2C_ICR_OA1CF_Pos            (31U)                                      
#define I2C_ICR_OA1CF_Msk            (0x1U << I2C_ICR_OA1CF_Pos)               /*!< 0x80000000*/
#define I2C_ICR_OA1CF                I2C_ICR_OA1CF_Msk                         /*!< OA1 matched clear flag */

/******************  Bit definition for I2C_PECR register  *******************/
#define I2C_PECR_PEC_Pos             (0U)                                      
#define I2C_PECR_PEC_Msk             (0xFFU << I2C_PECR_PEC_Pos)               /*!< 0x000000FF */
#define I2C_PECR_PEC                 I2C_PECR_PEC_Msk                          /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define I2C_RXDR_RXDATA_Pos          (0U)                                      
#define I2C_RXDR_RXDATA_Msk          (0xFFU << I2C_RXDR_RXDATA_Pos)            /*!< 0x000000FF */
#define I2C_RXDR_RXDATA              I2C_RXDR_RXDATA_Msk                       /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *******************/
#define I2C_TXDR_TXDATA_Pos          (0U)                                      
#define I2C_TXDR_TXDATA_Msk          (0xFFU << I2C_TXDR_TXDATA_Pos)            /*!< 0x000000FF */
#define I2C_TXDR_TXDATA              I2C_TXDR_TXDATA_Msk                       /*!< 8-bit transmit data */


/*****************************************************************************/
/*                                                                           */
/*                        Independent WATCHDOG (IWDG)                        */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for IWDG_KR register  *******************/
#define IWDG_KR_KEY_Pos      (0U)                                              
#define IWDG_KR_KEY_Msk      (0xFFFFU << IWDG_KR_KEY_Pos)                      /*!< 0x0000FFFF */
#define IWDG_KR_KEY          IWDG_KR_KEY_Msk                                   /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PR register  *******************/
#define IWDG_PR_PR_Pos       (0U)                                              
#define IWDG_PR_PR_Msk       (0x7U << IWDG_PR_PR_Pos)                          /*!< 0x00000007 */
#define IWDG_PR_PR           IWDG_PR_PR_Msk                                    /*!< PR[2:0] (Prescaler divider) */
#define IWDG_PR_PR_0         (0x1U << IWDG_PR_PR_Pos)                          /*!< 0x01 */
#define IWDG_PR_PR_1         (0x2U << IWDG_PR_PR_Pos)                          /*!< 0x02 */
#define IWDG_PR_PR_2         (0x4U << IWDG_PR_PR_Pos)                          /*!< 0x04 */

/*******************  Bit definition for IWDG_RLR register  ******************/
#define IWDG_RLR_RL_Pos      (0U)                                              
#define IWDG_RLR_RL_Msk      (0xFFFU << IWDG_RLR_RL_Pos)                       /*!< 0x00000FFF */
#define IWDG_RLR_RL          IWDG_RLR_RL_Msk                                   /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  *******************/
#define IWDG_SR_PVU_Pos      (0U)                                              
#define IWDG_SR_PVU_Msk      (0x1U << IWDG_SR_PVU_Pos)                         /*!< 0x00000001 */
#define IWDG_SR_PVU          IWDG_SR_PVU_Msk                                   /*!< Watchdog prescaler value update */
#define IWDG_SR_RVU_Pos      (1U)                                              
#define IWDG_SR_RVU_Msk      (0x1U << IWDG_SR_RVU_Pos)                         /*!< 0x00000002 */
#define IWDG_SR_RVU          IWDG_SR_RVU_Msk                                   /*!< Watchdog counter reload value update */
#define IWDG_SR_WVU_Pos      (2U)                                              
#define IWDG_SR_WVU_Msk      (0x1U << IWDG_SR_WVU_Pos)                         /*!< 0x00000004 */
#define IWDG_SR_WVU          IWDG_SR_WVU_Msk                                   /*!< Watchdog counter window value update */

/*******************  Bit definition for IWDG_WINR register  *****************/
#define IWDG_WINR_WIN_Pos    (0U)                                              
#define IWDG_WINR_WIN_Msk    (0xFFFU << IWDG_WINR_WIN_Pos)                     /*!< 0x00000FFF */
#define IWDG_WINR_WIN        IWDG_WINR_WIN_Msk                                 /*!< Watchdog counter window value */


/*****************************************************************************/
/*                                                                           */
/*                          Power Control (PWR)                              */
/*                                                                           */
/*****************************************************************************/

/*******************  Bit definition for PWR_CSR register  *******************/
#define PWR_CR_MBGPEN_Pos           (0U)
#define PWR_CR_MBGPEN_Msk           (0x1U << PWR_CR_MBGPEN_Pos)
#define PWR_CR_MBGPEN               PWR_CR_MBGPEN_Msk
#define PWR_CR_BOREN_Pos            (1U)
#define PWR_CR_BOREN_Msk            (0x1U << PWR_CR_BOREN_Pos)
#define PWR_CR_BOREN                PWR_CR_BOREN_Msk
#define PWR_CR_PVDEN_Pos            (2U)
#define PWR_CR_PVDEN_Msk            (0x1U << PWR_CR_PVDEN_Pos)
#define PWR_CR_PVDEN                PWR_CR_PVDEN_Msk
#define PWR_CR_PVDSEL_Pos           (3U)
#define PWR_CR_PVDSEL_Msk           (0x1U << PWR_CR_PVDSEL_Pos)
#define PWR_CR_PVDSEL               PWR_CR_PVDSEL_Msk

/*******************  Bit definition for PWR_CFGR register  *******************/
#define PWR_CFGR_LDO_REG_Pos        (0U)
#define PWR_CFGR_LDO_REG_Msk        (0xFU << PWR_CFGR_LDO_REG_Pos)
#define PWR_CFGR_LDO_REG            PWR_CFGR_LDO_REG_Msk
#define PWR_CFGR_PVD_REG_Pos        (4U)
#define PWR_CFGR_PVD_REG_Msk        (0xFU << PWR_CFGR_PVD_REG_Pos)
#define PWR_CFGR_PVD_REG            PWR_CFGR_PVD_REG_Msk
#define PWR_CFGR_BOR_REG_Pos        (8U)
#define PWR_CFGR_BOR_REG_Msk        (0xFU << PWR_CFGR_BOR_REG_Pos)
#define PWR_CFGR_BOR_REG            PWR_CFGR_BOR_REG_Msk
#define PWR_CFGR_PAD_RST_DIS_Pos    (12U)
#define PWR_CFGR_PAD_RST_DIS_Msk    (0x1U << PWR_CFGR_PAD_RST_DIS_Pos)
#define PWR_CFGR_PAD_RST_DIS        PWR_CFGR_PAD_RST_DIS_Msk
#define PWR_CFGR_BOR_RST_DIS_Pos    (13U)
#define PWR_CFGR_BOR_RST_DIS_Msk    (0x1U << PWR_CFGR_BOR_RST_DIS_Pos)
#define PWR_CFGR_BOR_RST_DIS        PWR_CFGR_BOR_RST_DIS_Msk
#define PWR_CFGR_IWDG_RST_DIS_Pos   (14U)
#define PWR_CFGR_IWDG_RST_DIS_Msk   (0x1U << PWR_CFGR_IWDG_RST_DIS_Pos)
#define PWR_CFGR_IWDG_RST_DIS       PWR_CFGR_IWDG_RST_DIS_Msk
#define PWR_CFGR_MBGP_REG_Pos       (16U)
#define PWR_CFGR_MBGP_REG_Msk       (0xFFU << PWR_CFGR_MBGP_REG_Pos)
#define PWR_CFGR_MBGP_REG           PWR_CFGR_MBGP_REG_Msk
#define PWR_CFGR_TEST_SEL_Pos       (24U)
#define PWR_CFGR_TEST_SEL_Msk       (0x3U << PWR_CFGR_TEST_SEL_Pos)
#define PWR_CFGR_TEST_SEL           PWR_CFGR_TEST_SEL_Msk

/*******************  Bit definition for PWR_LPR register  *******************/
#define PWR_LPR_STB_Pos             (0U)
#define PWR_LPR_STB_Msk             (0x1U << PWR_LPR_STB_Pos)
#define PWR_LPR_STB                 PWR_LPR_STB_Msk
#define PWR_LPR_DPSTB_Pos           (1U)
#define PWR_LPR_DPSTB_Msk           (0x1U << PWR_LPR_DPSTB_Pos)
#define PWR_LPR_DPSTB               PWR_LPR_DPSTB_Msk
#define PWR_LPR_FLSDS_Pos           (2U)
#define PWR_LPR_FLSDS_Msk           (0x1U << PWR_LPR_FLSDS_Pos)
#define PWR_LPR_FLSDS               PWR_LPR_FLSDS_Msk
#define PWR_LPR_MBGPPD_Pos          (3U)
#define PWR_LPR_MBGPPD_Msk          (0x1U << PWR_LPR_MBGPPD_Pos)
#define PWR_LPR_MBGPPD              PWR_LPR_MBGPPD_Msk

/***************  Bit definition for PWR_CSR register   **********************/
#define PWR_CSR_PORRSTF_Pos         (0U)
#define PWR_CSR_PORRSTF_Msk         (0x1U << PWR_CSR_PORRSTF_Pos)
#define PWR_CSR_PORRSTF             PWR_CSR_PORRSTF_Msk
#define PWR_CSR_BORRSTF_Pos         (1U)
#define PWR_CSR_BORRSTF_Msk         (0x1U << PWR_CSR_BORRSTF_Pos)
#define PWR_CSR_BORRSTF             PWR_CSR_BORRSTF_Msk
#define PWR_CSR_PADRSTF_Pos         (2U)
#define PWR_CSR_PADRSTF_Msk         (0x1U << PWR_CSR_PADRSTF_Pos)
#define PWR_CSR_PADRSTF             PWR_CSR_PADRSTF_Msk
#define PWR_CSR_PSORSTF_Pos         (3U)
#define PWR_CSR_PSORSTF_Msk         (0x1U << PWR_CSR_PSORSTF_Pos)
#define PWR_CSR_PSORSTF             PWR_CSR_PSORSTF_Msk
#define PWR_CSR_PVDO_Pos            (4U)
#define PWR_CSR_PVDO_Msk            (0x1U << PWR_CSR_PVDO_Pos)
#define PWR_CSR_PVDO                PWR_CSR_PVDO_Msk
#define PWR_CSR_BORO_Pos            (5U)
#define PWR_CSR_BORO_Msk            (0x1U << PWR_CSR_BORO_Pos)
#define PWR_CSR_BORO                PWR_CSR_BORO_Msk
#define PWR_CSR_MBGP_RDY_Pos        (6U)
#define PWR_CSR_MBGP_RDY_Msk        (0x1U << PWR_CSR_MBGP_RDY_Pos)
#define PWR_CSR_MBGP_RDY            PWR_CSR_MBGP_RDY_Msk
#define PWR_CSR_RMVF_Pos            (7U)
#define PWR_CSR_RMVF_Msk            (0x1U << PWR_CSR_RMVF_Pos)
#define PWR_CSR_RMVF                PWR_CSR_RMVF_Msk

/*****************************************************************************/
/*                                                                           */
/*                         Reset and Clock Control                           */
/*                                                                           */
/*****************************************************************************/
/*
* @brief Specific device feature definitions.
*/

/********************  Bit definition for RCC_CR register  *******************/
#define RCC_CR_HSION_Pos                         (0U)                          
#define RCC_CR_HSION_Msk                         (0x1U << RCC_CR_HSION_Pos)    /*!< 0x00000001 */
#define RCC_CR_HSION                             RCC_CR_HSION_Msk              /*!< Internal High Speed clock enable */
#define RCC_CR_HSIREFRDY_Pos                     (1U)                          
#define RCC_CR_HSIREFRDY_Msk                     (0x1U << RCC_CR_HSIREFRDY_Pos)/*!< 0x00000002 */
#define RCC_CR_HSIREFRDY                         RCC_CR_HSIRDY_Msk             /*!< Internal High Speed Reference clock ready flag */
#define RCC_CR_HSIRDY_Pos                        (2U)                          
#define RCC_CR_HSIRDY_Msk                        (0x1U << RCC_CR_HSIRDY_Pos)   /*!< 0x00000004 */
#define RCC_CR_HSIRDY                            RCC_CR_HSIRDY_Msk             /*!< Internal High Speed clock ready flag */
#define RCC_CR_HSICALDONE_Pos                    (3)                          
#define RCC_CR_HSICALDONE_Msk                    (0x1U << RCC_CR_HSICALDONE_Pos)/*!< 0x00000008 */
#define RCC_CR_HSICALDONE                        RCC_CR_HSICALDONE_Msk          /*!< Internal High Speed clock Calibration Done*/
#define RCC_CR_HSICAL_Pos                        (8U)                          
#define RCC_CR_HSICAL_Msk                        (0x1FU << RCC_CR_HSICAL_Pos)  /*!< 0x00001F00 */
#define RCC_CR_HSICAL                            RCC_CR_HSICAL_Msk             /*!< Internal High Speed clock Calibration Value*/
#define RCC_CR_HSICALEN_Pos                      (15U)
#define RCC_CR_HSICALEN_Msk                      (0x01U << RCC_CR_HSICALEN_Pos)/*!< 0x00008000 */ 
#define RCC_CR_HSICALEN                          RCC_CR_HSICALEN_Msk           /*!< Internal High Speed clock Calibration Enable*/

/********************  Bit definition for RCC_CFGR register  *****************/
/*!< HPRE configuration */
#define RCC_CFGR_SW_Pos                          (0U)                          
#define RCC_CFGR_SW_Msk                          (0x1U << RCC_CFGR_SW_Pos)   /*!< 0x000000F0 */
#define RCC_CFGR_SW                              RCC_CFGR_SW_Msk             /*!< HPRE[3:0] bits (AHB prescaler) */                      

#define RCC_CFGR_ADCSW_Pos                       (1U)
#define RCC_CFGR_ADCSW_Msk                       (0x1U << RCC_CFGR_ADCSW_Pos)   /*!< 0x000000F0 */
#define RCC_CFGR_ADCSW                           RCC_CFGR_ADCSW_Msk             /*!< HPRE[3:0] bits (AHB prescaler) */       

#define RCC_CFGR_SWS_Pos                         (2U)
#define RCC_CFGR_SWS_Msk                         (0x1U << RCC_CFGR_SWS_Pos)   /*!< 0x000000F0 */
#define RCC_CFGR_SWS                             RCC_CFGR_SWS_Msk             /*!< HPRE[3:0] bits (AHB prescaler) */       

#define RCC_CFGR_ADCSWS_Pos                       (3U)
#define RCC_CFGR_ADCSWS_Msk                       (0x1U << RCC_CFGR_ADCSWS_Pos)   /*!< 0x000000F0 */
#define RCC_CFGR_ADCSWS                           RCC_CFGR_ADCSWS_Msk             /*!< HPRE[3:0] bits (AHB prescaler) */       

#define RCC_CFGR_HPRE_Pos                        (4U)                          
#define RCC_CFGR_HPRE_Msk                        (0xFU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                            RCC_CFGR_HPRE_Msk             /*!< HPRE[3:0] bits (AHB prescaler) */                      
#define RCC_CFGR_HPRE_DIV1                       (0x0U << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2                       (0x8U << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                       (0x9U << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                       (0xAU << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                      (0xBU << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                      (0xCU << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128                     (0xDU << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256                     (0xEU << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512                     (0xFU << RCC_CFGR_HPRE_Pos)   /*!< SYSCLK divided by 512 */

#define RCC_CFGR_HPRE_0                          (0x0U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000000 */
#define RCC_CFGR_HPRE_1                          (0x8U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000080 */
#define RCC_CFGR_HPRE_2                          (0x9U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000090 */
#define RCC_CFGR_HPRE_3                          (0xAU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000A0 */
#define RCC_CFGR_HPRE_4                          (0xBU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000B0 */
#define RCC_CFGR_HPRE_5                          (0xCU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000C0 */
#define RCC_CFGR_HPRE_6                          (0xDU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000D0 */
#define RCC_CFGR_HPRE_7                          (0xEU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000E0 */
#define RCC_CFGR_HPRE_8                          (0xFU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000F0 */
/*!< PPRE configuration */
#define RCC_CFGR_PPRE_Pos                        (8U)                          
#define RCC_CFGR_PPRE_Msk                        (0x7U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000700 */
#define RCC_CFGR_PPRE                            RCC_CFGR_PPRE_Msk             /*!< PRE[2:0] bits (APB prescaler) */
#define RCC_CFGR_PPRE_DIV1                       (0x0U << RCC_CFGR_PPRE_Pos)   /*!< HCLK not divided */
#define RCC_CFGR_PPRE_DIV2                       (0x4U << RCC_CFGR_PPRE_Pos)   /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE_DIV4                       (0x5U << RCC_CFGR_PPRE_Pos)   /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE_DIV8                       (0x6U << RCC_CFGR_PPRE_Pos)   /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE_DIV16                      (0x7U << RCC_CFGR_PPRE_Pos)   /*!< HCLK divided by 16 */

#define RCC_CFGR_PPRE_0                          (0x0U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000000 */
#define RCC_CFGR_PPRE_1                          (0x4U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000400 */
#define RCC_CFGR_PPRE_2                          (0x5U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000500 */
#define RCC_CFGR_PPRE_3                          (0x6U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000600 */
#define RCC_CFGR_PPRE_4                          (0x7U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000700 */
/*!< FLSPRE configuration */
#define RCC_CFGR_FLSPRE_Pos                      (11U)                         
#define RCC_CFGR_FLSPRE_Msk                      (0xFU << RCC_CFGR_FLSPRE_Pos) /*!< 0x00007800 */
#define RCC_CFGR_FLSPRE                          RCC_CFGR_FLSPRE_Msk           /*!< Flash Internal Clock division */
#define RCC_CFGR_FLSPRE_FLSCLK_DISABLE           0
#define RCC_CFGR_FLSPRE_DIV2										 (1<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV3										 (2<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV4										 (3<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV5										 (4<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV6										 (5<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV7										 (6<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV8										 (7<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV9										 (8<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV10										 (9<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV11										 (10<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV12										 (11<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV13										 (12<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV14										 (13<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV15										 (14<<RCC_CFGR_FLSPRE_Pos)
#define RCC_CFGR_FLSPRE_DIV16										 (15<<RCC_CFGR_FLSPRE_Pos)

#define RCC_CFGR_FLSEN_Pos                       (15U)                         
#define RCC_CFGR_FLSEN_Msk                       (0xFU << RCC_CFGR_FLSEN_Pos)    	 /*!< 0x00008000 */
#define RCC_CFGR_FLSEN                           RCC_CFGR_FLSEN_Msk                /*!< Flash Internal Clock division enable */
/*!< MCOPRE configuration */
#define RCC_CFGR_MCOPRE_Pos                      (21U)
#define RCC_CFGR_MCOPRE_Msk                      (0x7U << RCC_CFGR_MCOPRE_Pos)     /*!< 0x00E00000 */
#define RCC_CFGR_MCOPRE                          RCC_CFGR_MCOPRE_Msk               /*!< MCO Divider */
#define RCC_CFGR_MCOPRE_NONE                     0U
#define RCC_CFGR_MCOPRE_LSI                      (0x1U << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV4                     (0x2U << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV8                     (0x3U << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV16                    (0x4U << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV32                    (0x5U << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV64                    (0x6U << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV128                   (0x7U << RCC_CFGR_MCOPRE_Pos)
/*!< ADCKPRE configuration */
#define RCC_CFGR_ADCKPRE_Pos                     (29U)                         
#define RCC_CFGR_ADCKPRE_Msk                     (0x3U << RCC_CFGR_ADCKPRE_Pos)    /*!< 0x60000000 */
#define RCC_CFGR_ADCKPRE                         RCC_CFGR_ADCKPRE_Msk              /*!< ADC clock */
#define RCC_CFGR_ADCKPRE_DIV2										 (0<<RCC_CFGR_ADCKPRE_Pos)
#define RCC_CFGR_ADCKPRE_DIV4										 (1<<RCC_CFGR_ADCKPRE_Pos)
#define RCC_CFGR_ADCKPRE_DIV8										 (2<<RCC_CFGR_ADCKPRE_Pos)
#define RCC_CFGR_ADCKPRE_NONE										 (3<<RCC_CFGR_ADCKPRE_Pos)

/*****************  Bit definition for RCC_APB2RSTR register  ****************/
#define RCC_APB2RSTR_SYSCFGRST_Pos               (0U)                          
#define RCC_APB2RSTR_SYSCFGRST_Msk               (0x1U << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00000001 */
#define RCC_APB2RSTR_SYSCFGRST                   RCC_APB2RSTR_SYSCFGRST_Msk    				/*!< SYSCFG reset */
#define RCC_APB2RSTR_ANALOGRST_Pos               (1U)                          
#define RCC_APB2RSTR_ANALOGRST_Msk               (0x1U << RCC_APB2RSTR_ANALOGRST_Pos) /*!< 0x00000002 */
#define RCC_APB2RSTR_ANALOGRST                   RCC_APB2RSTR_ANALOGRST_Msk    				/*!< ANALOG reset */
#define RCC_APB2RSTR_ADCRST_Pos                  (9U)                          
#define RCC_APB2RSTR_ADCRST_Msk                  (0x1U << RCC_APB2RSTR_ADCRST_Pos) 		/*!< 0x00000200 */
#define RCC_APB2RSTR_ADCRST                      RCC_APB2RSTR_ADCRST_Msk       				/*!< ADC reset */
#define RCC_APB2RSTR_TIM20RST_Pos                 (11U)                         
#define RCC_APB2RSTR_TIM20RST_Msk                 (0x1U << RCC_APB2RSTR_TIM20RST_Pos) 	/*!< 0x00000800 */
#define RCC_APB2RSTR_TIM20RST                     RCC_APB2RSTR_TIM20RST_Msk      				/*!< TIM20 reset */
#define RCC_APB2RSTR_SPI1RST_Pos                 (12U)                         															//????????????
#define RCC_APB2RSTR_SPI1RST_Msk                 (0x1U << RCC_APB2RSTR_SPI1RST_Pos) 	/*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST                     RCC_APB2RSTR_SPI1RST_Msk      				/*!< SPI1 reset */
#define RCC_APB2RSTR_USART1RST_Pos               (14U)                         
#define RCC_APB2RSTR_USART1RST_Msk               (0x1U << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_USART1RST                   RCC_APB2RSTR_USART1RST_Msk    				/*!< USART1 reset */

/*!< Old ADC1 reset bit definition maintained for legacy purpose */
#define  RCC_APB2RSTR_ADC1RST                RCC_APB2RSTR_ADCRST          

/*****************  Bit definition for RCC_APB1RSTR register  ****************/
#define RCC_APB1RSTR_TIM2RST_Pos                 (0U)                          
#define RCC_APB1RSTR_TIM2RST_Msk                 (0x1U << RCC_APB1RSTR_TIM2RST_Pos) 	/*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST                     RCC_APB1RSTR_TIM2RST_Msk      				/*!< Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST_Pos                 (1U)                          
#define RCC_APB1RSTR_TIM3RST_Msk                 (0x1U << RCC_APB1RSTR_TIM3RST_Pos) 	/*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST                     RCC_APB1RSTR_TIM3RST_Msk      				/*!< Timer 3 reset */
#define RCC_APB1RSTR_TIM6RST_Pos                 (4U)                          
#define RCC_APB1RSTR_TIM6RST_Msk                 (0x1U << RCC_APB1RSTR_TIM6RST_Pos) 	/*!< 0x00000010 */
#define RCC_APB1RSTR_TIM6RST                     RCC_APB1RSTR_TIM6RST_Msk     				/*!< Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST_Pos                 (5U)                          
#define RCC_APB1RSTR_TIM7RST_Msk                 (0x1U << RCC_APB1RSTR_TIM7RST_Pos) 	/*!< 0x00000020 */
#define RCC_APB1RSTR_TIM7RST                     RCC_APB1RSTR_TIM7RST_Msk     				/*!< Timer 7 reset */
#define RCC_APB1RSTR_WWDGRST_Pos                 (11U)                         
#define RCC_APB1RSTR_WWDGRST_Msk                 (0x1U << RCC_APB1RSTR_WWDGRST_Pos) 	/*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST                     RCC_APB1RSTR_WWDGRST_Msk      				/*!< Window Watchdog reset */
#define RCC_APB1RSTR_UART2RST_Pos                (17U)                         
#define RCC_APB1RSTR_UART2RST_Msk                (0x1U << RCC_APB1RSTR_UART2RST_Pos) 	/*!< 0x00020000 */
#define RCC_APB1RSTR_UART2RST                    RCC_APB1RSTR_UART2RST_Msk      			/*!< UART2 reset */
#define RCC_APB1RSTR_I2C1RST_Pos                 (21U)                         
#define RCC_APB1RSTR_I2C1RST_Msk                 (0x1U << RCC_APB1RSTR_I2C1RST_Pos) 	/*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST                     RCC_APB1RSTR_I2C1RST_Msk      				/*!< I2C 1 reset */
#define RCC_APB1RSTR_I2C2RST_Pos                 (22U)                         
#define RCC_APB1RSTR_I2C2RST_Msk                 (0x1U << RCC_APB1RSTR_I2C2RST_Pos) 	/*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST                     RCC_APB1RSTR_I2C2RST_Msk      				/*!< I2C 2 reset */
#define RCC_APB1RSTR_PWRRST_Pos                  (28U)                         
#define RCC_APB1RSTR_PWRRST_Msk                  (0x1U << RCC_APB1RSTR_PWRRST_Pos) 		/*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                      RCC_APB1RSTR_PWRRST_Msk       				/*!< PWR reset */

/******************  Bit definition for RCC_AHBENR register  *****************/
#define RCC_AHBENR_DMAEN_Pos                     (0U)                          
#define RCC_AHBENR_DMAEN_Msk                     (0x1U << RCC_AHBENR_DMAEN_Pos) 			/*!< 0x00000001 */
#define RCC_AHBENR_DMAEN                         RCC_AHBENR_DMAEN_Msk          				/*!< DMA1 clock enable */
//#define RCC_AHBENR_SRAMEN_Pos                    (2U)                          
//#define RCC_AHBENR_SRAMEN_Msk                    (0x1U << RCC_AHBENR_SRAMEN_Pos) 		/*!< 0x00000004 */
//#define RCC_AHBENR_SRAMEN                        RCC_AHBENR_SRAMEN_Msk         			/*!< SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN_Pos                   (4U)                          
#define RCC_AHBENR_FLITFEN_Msk                   (0x1U << RCC_AHBENR_FLITFEN_Pos) 		/*!< 0x00000010 */
#define RCC_AHBENR_FLITFEN                       RCC_AHBENR_FLITFEN_Msk        				/*!< FLITF clock enable */
#define RCC_AHBENR_CRCEN_Pos                     (6U)                          
#define RCC_AHBENR_CRCEN_Msk                     (0x1U << RCC_AHBENR_CRCEN_Pos) 		/*!< 0x00000040 */
#define RCC_AHBENR_CRCEN                         RCC_AHBENR_CRCEN_Msk          			/*!< CRC clock enable */
#define RCC_AHBENR_CORDIC_Pos                    (8U)                          
#define RCC_AHBENR_CORDIC_Msk                    (0x1U << RCC_AHBENR_CORDIC_Pos) 		/*!< 0x00000040 */
#define RCC_AHBENR_CORDICEN                       RCC_AHBENR_CORDIC_Msk          			/*!< CRC clock enable */
#define RCC_AHBENR_MATH_Pos                     (9U)                          
#define RCC_AHBENR_MATH_Msk                     (0x1U << RCC_AHBENR_MATH_Pos) 		/*!< 0x00000040 */
#define RCC_AHBENR_MATHEN                         RCC_AHBENR_MATH_Msk          			/*!< CRC clock enable */
#define RCC_AHBENR_SVPWM_Pos                     (11U)                          
#define RCC_AHBENR_SVPWM_Msk                     (0x1U << RCC_AHBENR_SVPWM_Pos) 		/*!< 0x00000040 */
#define RCC_AHBENR_SVPWMEN                         RCC_AHBENR_SVPWM_Msk          			/*!< CRC clock enable */
#define RCC_AHBENR_GPIOAEN_Pos                   (17U)                         
#define RCC_AHBENR_GPIOAEN_Msk                   (0x1U << RCC_AHBENR_GPIOAEN_Pos) 		/*!< 0x00020000 */
#define RCC_AHBENR_GPIOAEN                       RCC_AHBENR_GPIOAEN_Msk        				/*!< GPIOA clock enable */
#define RCC_AHBENR_GPIOBEN_Pos                   (18U)                         
#define RCC_AHBENR_GPIOBEN_Msk                   (0x1U << RCC_AHBENR_GPIOBEN_Pos) 		/*!< 0x00040000 */
#define RCC_AHBENR_GPIOBEN                       RCC_AHBENR_GPIOBEN_Msk        				/*!< GPIOB clock enable */
#define RCC_AHBENR_GPIOCEN_Pos                   (19U)                         
#define RCC_AHBENR_GPIOCEN_Msk                   (0x1U << RCC_AHBENR_GPIOCEN_Pos) 	/*!< 0x00080000 */
#define RCC_AHBENR_GPIOCEN                       RCC_AHBENR_GPIOCEN_Msk        			/*!< GPIOC clock enable */
//#define RCC_AHBENR_GPIOFEN_Pos                   (22U)                         
//#define RCC_AHBENR_GPIOFEN_Msk                   (0x1U << RCC_AHBENR_GPIOFEN_Pos) 	/*!< 0x00400000 */
//#define RCC_AHBENR_GPIOFEN                       RCC_AHBENR_GPIOFEN_Msk        			/*!< GPIOF clock enable */

/* Old Bit definition maintained for legacy purpose */
#define  RCC_AHBENR_DMA1EN                   RCC_AHBENR_DMAEN        /*!< DMA1 clock enable */
#define  RCC_AHBENR_TSEN                     RCC_AHBENR_TSCEN        /*!< TS clock enable */

/*****************  Bit definition for RCC_APB2ENR register  *****************/
#define RCC_APB2ENR_SYSCFGCOMPEN_Pos             (0U)                          
#define RCC_APB2ENR_SYSCFGCOMPEN_Msk             (0x1U << RCC_APB2ENR_SYSCFGCOMPEN_Pos) 	/*!< 0x00000001 */
#define RCC_APB2ENR_SYSCFGCOMPEN                 RCC_APB2ENR_SYSCFGCOMPEN_Msk  						/*!< SYSCFG and comparator clock enable */
#define RCC_APB2ENR_ANALOGEN_Pos                 (1U)                          
#define RCC_APB2ENR_ANALOGEN_Msk                 (0x1U << RCC_APB2ENR_ANALOGEN_Pos) 	/*!< 0x00000001 */
#define RCC_APB2ENR_ANALOGEN                     RCC_APB2ENR_ANALOGEN_Msk  						/*!< ANALOG and comparator clock enable */
#define RCC_APB2ENR_ADCEN_Pos                    (9U)                          
#define RCC_APB2ENR_ADCEN_Msk                    (0x1U << RCC_APB2ENR_ADCEN_Pos) 					/*!< 0x00000200 */
#define RCC_APB2ENR_ADCEN                        RCC_APB2ENR_ADCEN_Msk         						/*!< ADC1 clock enable */
#define RCC_APB2ENR_TIM20EN_Pos                   (11U)                         
#define RCC_APB2ENR_TIM20EN_Msk                   (0x1U << RCC_APB2ENR_TIM20EN_Pos) 				/*!< 0x00000800 */
#define RCC_APB2ENR_TIM20EN                       RCC_APB2ENR_TIM20EN_Msk        						/*!< TIM20 clock enable */
#define RCC_APB2ENR_SPI1EN_Pos                   (12U)                         																					//????????????????
#define RCC_APB2ENR_SPI1EN_Msk                   (0x1U << RCC_APB2ENR_SPI1EN_Pos) 				/*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                       RCC_APB2ENR_SPI1EN_Msk        						/*!< SPI1 clock enable */
#define RCC_APB2ENR_USART1EN_Pos                 (14U)                         
#define RCC_APB2ENR_USART1EN_Msk                 (0x1U << RCC_APB2ENR_USART1EN_Pos) 			/*!< 0x00004000 */
#define RCC_APB2ENR_USART1EN                     RCC_APB2ENR_USART1EN_Msk      						/*!< USART1 clock enable */

/* Old Bit definition maintained for legacy purpose */
#define  RCC_APB2ENR_SYSCFGEN                RCC_APB2ENR_SYSCFGCOMPEN        /*!< SYSCFG clock enable */
#define  RCC_APB2ENR_ADC1EN                  RCC_APB2ENR_ADCEN               /*!< ADC1 clock enable */

/*****************  Bit definition for RCC_APB1ENR register  *****************/
#define RCC_APB1ENR_TIM2EN_Pos                   (0U)                          
#define RCC_APB1ENR_TIM2EN_Msk                   (0x1U << RCC_APB1ENR_TIM2EN_Pos) /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                       RCC_APB1ENR_TIM2EN_Msk        		/*!< Timer 2 clock enable */
#define RCC_APB1ENR_TIM3EN_Pos                   (1U)                          
#define RCC_APB1ENR_TIM3EN_Msk                   (0x1U << RCC_APB1ENR_TIM3EN_Pos) /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN                       RCC_APB1ENR_TIM3EN_Msk        		/*!< Timer 3 clock enable */
#define RCC_APB1ENR_TIM6EN_Pos                   (4U)                          
#define RCC_APB1ENR_TIM6EN_Msk                   (0x1U << RCC_APB1ENR_TIM6EN_Pos) /*!< 0x00000010 */
#define RCC_APB1ENR_TIM6EN                       RCC_APB1ENR_TIM6EN_Msk       		/*!< Timer 6 clock enable */
#define RCC_APB1ENR_TIM7EN_Pos                   (5U)                          
#define RCC_APB1ENR_TIM7EN_Msk                   (0x1U << RCC_APB1ENR_TIM7EN_Pos) /*!< 0x00000020 */
#define RCC_APB1ENR_TIM7EN                       RCC_APB1ENR_TIM7EN_Msk       		/*!< Timer 7 clock enable */
#define RCC_APB1ENR_WWDGEN_Pos                   (11U)                         
#define RCC_APB1ENR_WWDGEN_Msk                   (0x1U << RCC_APB1ENR_WWDGEN_Pos) /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                       RCC_APB1ENR_WWDGEN_Msk        		/*!< Window Watchdog clock enable */
#define RCC_APB1ENR_UART2EN_Pos                  (17U)                         
#define RCC_APB1ENR_UART2EN_Msk                  (0x1U << RCC_APB1ENR_UART2EN_Pos) /*!< 0x00020000 */
#define RCC_APB1ENR_UART2EN                      RCC_APB1ENR_UART2EN_Msk        	 /*!< UART2 clock enable */
#define RCC_APB1ENR_I2C1EN_Pos                   (21U)                         
#define RCC_APB1ENR_I2C1EN_Msk                   (0x1U << RCC_APB1ENR_I2C1EN_Pos) /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                       RCC_APB1ENR_I2C1EN_Msk        		/*!< I2C1 clock enable */
#define RCC_APB1ENR_I2C2EN_Pos                   (22U)                         
#define RCC_APB1ENR_I2C2EN_Msk                   (0x1U << RCC_APB1ENR_I2C2EN_Pos) /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN                       RCC_APB1ENR_I2C2EN_Msk        		/*!< I2C2 clock enable */
#define RCC_APB1ENR_PWREN_Pos                    (28U)                         
#define RCC_APB1ENR_PWREN_Msk                    (0x1U << RCC_APB1ENR_PWREN_Pos) 	/*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                        RCC_APB1ENR_PWREN_Msk         		/*!< PWR clock enable */

/*******************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                        (0U) 
#define RCC_CSR_LSION_Msk                        (0x1UL << RCC_CSR_LSION_Pos)  /*!< 0x00000001 */
#define RCC_CSR_LSION                            RCC_CSR_LSION_Msk             /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSICAL_Pos                       (2U) 
#define RCC_CSR_LSICAL_Msk                       (0x1FUL << RCC_CSR_LSICAL_Pos)  /*!< 0x0000007C */
#define RCC_CSR_LSICAL                           RCC_CSR_LSICAL_Msk              /*!< Internal Low Speed oscillator calibration */
#define RCC_CSR_LSIUPDATEREQ_Pos                 (7U) 
#define RCC_CSR_LSIUPDATEREQ_Msk                 (0x1UL << RCC_CSR_LSIUPDATEREQ_Pos) /*!< 0x00000080 */
#define RCC_CSR_LSIUPDATEREQ                     RCC_CSR_LSIUPDATEREQ_Msk            /*!< Internal Low Speed oscillator update request */
#define RCC_CSR_LSICALEN_Pos                     (8U) 
#define RCC_CSR_LSICALEN_Msk                     (0x1UL << RCC_CSR_LSICALEN_Pos) /*!< 0x00000100 */
#define RCC_CSR_LSICALEN                         RCC_CSR_LSICALEN_Msk            /*!< Internal Low Speed oscillator calibration enable */
#define RCC_CSR_LSICALDONE_Pos                   (9U) 
#define RCC_CSR_LSICALDONE_Msk                   (0x1UL << RCC_CSR_LSICALDONE_Pos) /*!< 0x00000200 */
#define RCC_CSR_LSICALDONE                       RCC_CSR_LSICALDONE_Msk            /*!< Internal Low Speed oscillator calibration done */
#define RCC_CSR_RMVF_Pos                         (24U)                         
#define RCC_CSR_RMVF_Msk                         (0x1U << RCC_CSR_RMVF_Pos)    /*!< 0x01000000 */
#define RCC_CSR_RMVF                             RCC_CSR_RMVF_Msk              /*!< Remove reset flag */
#define RCC_CSR_OBLRSTF_Pos                      (25U)                         
#define RCC_CSR_OBLRSTF_Msk                      (0x1U << RCC_CSR_OBLRSTF_Pos) /*!< 0x02000000 */
#define RCC_CSR_OBLRSTF                          RCC_CSR_OBLRSTF_Msk           /*!< OBL reset flag */
#define RCC_CSR_PINRSTF_Pos                      (26U)                         
#define RCC_CSR_PINRSTF_Msk                      (0x1U << RCC_CSR_PINRSTF_Pos) /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                          RCC_CSR_PINRSTF_Msk           /*!< PIN reset flag */
#define RCC_CSR_PORRSTF_Pos                      (27U)                         
#define RCC_CSR_PORRSTF_Msk                      (0x1U << RCC_CSR_PORRSTF_Pos) /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                          RCC_CSR_PORRSTF_Msk           /*!< POR/PDR reset flag */
#define RCC_CSR_SFTRSTF_Pos                      (28U)                         
#define RCC_CSR_SFTRSTF_Msk                      (0x1U << RCC_CSR_SFTRSTF_Pos) /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                          RCC_CSR_SFTRSTF_Msk           /*!< Software Reset flag */
#define RCC_CSR_IWDGRSTF_Pos                     (29U)                         
#define RCC_CSR_IWDGRSTF_Msk                     (0x1U << RCC_CSR_IWDGRSTF_Pos) /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                         RCC_CSR_IWDGRSTF_Msk          	/*!< Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF_Pos                     (30U)                         
#define RCC_CSR_WWDGRSTF_Msk                     (0x1U << RCC_CSR_WWDGRSTF_Pos) /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                         RCC_CSR_WWDGRSTF_Msk          	/*!< Window watchdog reset flag */

/* Old Bit definition maintained for legacy purpose */
#define  RCC_CSR_OBL                         RCC_CSR_OBLRSTF        /*!< OBL reset flag */

/*******************  Bit definition for RCC_AHBRSTR register  ***************/
#define RCC_AHBRSTR_GPIOARST_Pos                 (17U)                         
#define RCC_AHBRSTR_GPIOARST_Msk                 (0x1U << RCC_AHBRSTR_GPIOARST_Pos) 	/*!< 0x00020000 */
#define RCC_AHBRSTR_GPIOARST                     RCC_AHBRSTR_GPIOARST_Msk      				/*!< GPIOA reset */
#define RCC_AHBRSTR_GPIOBRST_Pos                 (18U)                         
#define RCC_AHBRSTR_GPIOBRST_Msk                 (0x1U << RCC_AHBRSTR_GPIOBRST_Pos) 	/*!< 0x00040000 */
#define RCC_AHBRSTR_GPIOBRST                     RCC_AHBRSTR_GPIOBRST_Msk      				/*!< GPIOB reset */
//#define RCC_AHBRSTR_GPIOCRST_Pos                 (19U)                         
//#define RCC_AHBRSTR_GPIOCRST_Msk                 (0x1U << RCC_AHBRSTR_GPIOCRST_Pos) /*!< 0x00080000 */
//#define RCC_AHBRSTR_GPIOCRST                     RCC_AHBRSTR_GPIOCRST_Msk      /*!< GPIOC reset */
//#define RCC_AHBRSTR_GPIOFRST_Pos                 (22U)                         
//#define RCC_AHBRSTR_GPIOFRST_Msk                 (0x1U << RCC_AHBRSTR_GPIOFRST_Pos) /*!< 0x00400000 */
//#define RCC_AHBRSTR_GPIOFRST                     RCC_AHBRSTR_GPIOFRST_Msk      /*!< GPIOF reset */

/*******************  Bit definition for RCC_CFGR2 register  *****************/
/*!< PREDIV configuration */
//#define RCC_CFGR2_UART1SW_Pos                    (0U)                          
//#define RCC_CFGR2_UART1SW_Msk                    (0x03U << RCC_CFGR2_UART1SW_Pos) /*!< 0x0000000F */
//#define RCC_CFGR2_UART1SW                        RCC_CFGR2_UART1SW_Msk          /*!< PREDIV[3:0] bits */
//
//#define RCC_CFGR2_UART1SW_PCLK                  (0x00000000U)                 /*!< PCLK clock used as UART1 clock source */
//#define RCC_CFGR2_UART1SW_SYSCLK                (0x00000001U)                 /*!< System clock selected as UART1 clock source */
//#define RCC_CFGR2_UART1SW_HSI                   (0x00000003U)                 /*!< HSI oscillator clock used as UART1 clock source */
//
//
//#define RCC_CFGR2_I2C1SW_Pos                     (4U)                          
//#define RCC_CFGR2_I2C1SW_Msk                     (0x01U << RCC_CFGR2_I2C1SW_Pos)
//#define RCC_CFGR2_I2C1SW                         RCC_CFGR2_I2C1SW_Msk
//
//#define RCC_CFGR2_I2C1SW_PCLK                    (0x00000000U)                 /*!< PCLK used as I2C1 clock source */
//#define RCC_CFGR2_I2C1SW_SYSCLK_Pos              (4U)                          
//#define RCC_CFGR2_I2C1SW_SYSCLK_Msk              (0x1U << RCC_CFGR2_I2C1SW_SYSCLK_Pos) /*!< 0x00000010 */
//#define RCC_CFGR2_I2C1SW_SYSCLK                  RCC_CFGR2_I2C1SW_SYSCLK_Msk   /*!< System clock selected as I2C1 clock source */
//

/******************************  PLLR  ****************************************/
#define RCC_PLLR_M_Pos                           (16U)                         
#define RCC_PLLR_M_Msk                           (0x7FU << RCC_PLLR_M_Pos)
#define RCC_PLLR_M                               RCC_PLLR_M_Msk  

#define RCC_PLLR_N_Pos                           (8U)                         
#define RCC_PLLR_N_Msk                           (0x3FU << RCC_PLLR_N_Pos)
#define RCC_PLLR_N                               RCC_PLLR_N_Msk  

#define RCC_PLLR_OD_Pos                           (6U)                         
#define RCC_PLLR_OD_Msk                           (0x3U << RCC_PLLR_OD_Pos)
#define RCC_PLLR_OD                               RCC_PLLR_OD_Msk  

#define RCC_PLLR_PLL_LOCK_Pos                     (5U)                         
#define RCC_PLLR_PLL_LOCK_Msk                     (0x1U << RCC_PLLR_PLL_LOCK_Pos)
#define RCC_PLLR_PLL_LOCK                         RCC_PLLR_PLL_LOCK_Msk  

#define RCC_PLLR_PLL_RDY_Pos                     (4U)                         
#define RCC_PLLR_PLL_RDY_Msk                     (0x1U << RCC_PLLR_PLL_RDY_Pos)
#define RCC_PLLR_PLL_RDY                         RCC_PLLR_PLL_RDY_Msk  

#define RCC_PLLR_PLL_OEN_Pos                     (2U)
#define RCC_PLLR_PLL_OEN_Msk                     (0x1U << RCC_PLLR_PLL_OEN_Pos)
#define RCC_PLLR_PLL_OEN                         RCC_PLLR_PLL_OEN_Msk  

#define RCC_PLLR_PLL_BYPASSEN_Pos                (1U)                         
#define RCC_PLLR_PLL_BYPASSEN_Msk                (0x1U << RCC_PLLR_PLL_BYPASSEN_Pos)
#define RCC_PLLR_PLL_BYPASSEN                    RCC_PLLR_PLL_BYPASSEN_Msk  

#define RCC_PLLR_PLL_POWER_DOWN_Pos                (0U)                         
#define RCC_PLLR_PLL_POWER_DOWN_Msk                (0x1U << RCC_PLLR_PLL_POWER_DOWN_Pos)
#define RCC_PLLR_PLL_POWER_DOWN                    RCC_PLLR_PLL_POWER_DOWN_Msk  



/*****************************************************************************/
/*                                                                           */
/*                        Serial Peripheral Interface (SPI)                  */				//???????????????
/*                                                                           */
/*****************************************************************************/

/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos            (0U)      
#define SPI_CR1_CPHA_Msk            (0x1UL << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!< Clock Phase */
#define SPI_CR1_CPOL_Pos            (1U)      
#define SPI_CR1_CPOL_Msk            (0x1UL << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!< Clock Polarity */
#define SPI_CR1_MSTR_Pos            (2U)      
#define SPI_CR1_MSTR_Msk            (0x1UL << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!< Master Selection */
#define SPI_CR1_BR_Pos              (3U)      
#define SPI_CR1_BR_Msk              (0x7UL << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1UL << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2UL << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4UL << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */
#define SPI_CR1_SPE_Pos             (6U)      
#define SPI_CR1_SPE_Msk             (0x1UL << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!< SPI Enable */
#define SPI_CR1_LSBFIRST_Pos        (7U)      
#define SPI_CR1_LSBFIRST_Msk        (0x1UL << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!< Frame Format */
#define SPI_CR1_SSI_Pos             (8U)      
#define SPI_CR1_SSI_Msk             (0x1UL << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!< Internal slave select */
#define SPI_CR1_SSM_Pos             (9U)      
#define SPI_CR1_SSM_Msk             (0x1UL << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!< Software slave management */
#define SPI_CR1_RXONLY_Pos          (10U)     
#define SPI_CR1_RXONLY_Msk          (0x1UL << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!< Receive only */
#define SPI_CR1_DFF_Pos             (11U)     
#define SPI_CR1_DFF_Msk             (0x1UL << SPI_CR1_DFF_Pos)                  /*!< 0x00000800 */
#define SPI_CR1_DFF                 SPI_CR1_DFF_Msk                            /*!< Data Frame Format */
#define SPI_CR1_CRCNEXT_Pos         (12U)     
#define SPI_CR1_CRCNEXT_Msk         (0x1UL << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!< Transmit CRC next */
#define SPI_CR1_CRCEN_Pos           (13U)     
#define SPI_CR1_CRCEN_Msk           (0x1UL << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!< Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE_Pos          (14U)     
#define SPI_CR1_BIDIOE_Msk          (0x1UL << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15U)     
#define SPI_CR1_BIDIMODE_Msk        (0x1UL << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN_Pos         (0U)      
#define SPI_CR2_RXDMAEN_Msk         (0x1UL << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk                        /*!< Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN_Pos         (1U)      
#define SPI_CR2_TXDMAEN_Msk         (0x1UL << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk                        /*!< Tx Buffer DMA Enable */
#define SPI_CR2_SSOE_Pos            (2U)      
#define SPI_CR2_SSOE_Msk            (0x1UL << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!< SS Output Enable */
#define SPI_CR2_FRF_Pos             (4U)      
#define SPI_CR2_FRF_Msk             (0x1UL << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!< Frame Format Enable */
#define SPI_CR2_ERRIE_Pos           (5U)      
#define SPI_CR2_ERRIE_Msk           (0x1UL << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE_Pos          (6U)      
#define SPI_CR2_RXNEIE_Msk          (0x1UL << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7U)      
#define SPI_CR2_TXEIE_Msk           (0x1UL << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!< Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos             (0U)      
#define SPI_SR_RXNE_Msk             (0x1UL << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!< Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1U)      
#define SPI_SR_TXE_Msk              (0x1UL << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!< Transmit buffer Empty */
#define SPI_SR_CRCERR_Pos           (4U)      
#define SPI_SR_CRCERR_Msk           (0x1UL << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!< CRC Error flag */
#define SPI_SR_MODF_Pos             (5U)      
#define SPI_SR_MODF_Msk             (0x1UL << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!< Mode fault */
#define SPI_SR_OVR_Pos              (6U)      
#define SPI_SR_OVR_Msk              (0x1UL << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!< Overrun flag */
#define SPI_SR_BSY_Pos              (7U)      
#define SPI_SR_BSY_Msk              (0x1UL << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!< Busy flag */
#define SPI_SR_FRE_Pos              (8U)      
#define SPI_SR_FRE_Msk              (0x1UL << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!< TI frame format error */  

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos               (0U)      
#define SPI_DR_DR_Msk               (0xFFFFUL << SPI_DR_DR_Pos)                 /*!< 0x0000FFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)      
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFUL << SPI_CRCPR_CRCPOLY_Pos)         /*!< 0x0000FFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)      
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFUL << SPI_RXCRCR_RXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)      
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFUL << SPI_TXCRCR_TXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   ((uint16_t)0x0001)            /*!<Channel length (number of bits per audio channel) */
#define  SPI_I2SCFGR_DATLEN                  ((uint16_t)0x0006)            /*!<DATLEN[1:0] bits (Data length to be transferred) */
#define  SPI_I2SCFGR_DATLEN_0                ((uint16_t)0x0002)            /*!<Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                ((uint16_t)0x0004)            /*!<Bit 1 */
#define  SPI_I2SCFGR_CKPOL                   ((uint16_t)0x0008)            /*!<steady state clock polarity */
#define  SPI_I2SCFGR_I2SSTD                  ((uint16_t)0x0030)            /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                ((uint16_t)0x0010)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                ((uint16_t)0x0020)            /*!<Bit 1 */
#define  SPI_I2SCFGR_PCMSYNC                 ((uint16_t)0x0080)            /*!<PCM frame synchronization */
#define  SPI_I2SCFGR_I2SCFG                  ((uint16_t)0x0300)            /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                ((uint16_t)0x0100)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                ((uint16_t)0x0200)            /*!<Bit 1 */
#define  SPI_I2SCFGR_I2SE                    ((uint16_t)0x0400)            /*!<I2S Enable */
#define  SPI_I2SCFGR_I2SMOD                  ((uint16_t)0x0800)            /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    ((uint16_t)0x00FF)            /*!<I2S Linear prescaler */
#define  SPI_I2SPR_ODD                       ((uint16_t)0x0100)            /*!<Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     ((uint16_t)0x0200)            /*!<Master Clock Output Enable */


/*****************************************************************************/
/*                                                                           */
/*                       System Configuration (SYSCFG)                       */
/*                                                                           */
/*****************************************************************************/

/*****************  Bit definition for SYSCFG_CFGR1 register  ****************/
#define SYSCFG_CFGR1_MEM_MODE_Pos                (0U)                          
#define SYSCFG_CFGR1_MEM_MODE_Msk                (0x3UL << SYSCFG_CFGR1_MEM_MODE_Pos) 		/*!< 0x00000003 */
#define SYSCFG_CFGR1_MEM_MODE                    SYSCFG_CFGR1_MEM_MODE_Msk     						/*!< SYSCFG_Memory Remap Config */
#define SYSCFG_CFGR1_MEM_MODE_1                  (0x1UL << SYSCFG_CFGR1_MEM_MODE_Pos) 		/*!< 0x00000001 */
#define SYSCFG_CFGR1_MEM_MODE_3                  (0x3UL << SYSCFG_CFGR1_MEM_MODE_Pos) 		/*!< 0x00000003 */				//?????????????????????????


/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0_Pos                 (0U)                          
#define SYSCFG_EXTICR1_EXTI0_Msk                 (0xFUL << SYSCFG_EXTICR1_EXTI0_Pos) 	/*!< 0x0000000F */
#define SYSCFG_EXTICR1_EXTI0                     SYSCFG_EXTICR1_EXTI0_Msk      				/*!< EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1_Pos                 (4U)                          
#define SYSCFG_EXTICR1_EXTI1_Msk                 (0xFUL << SYSCFG_EXTICR1_EXTI1_Pos) 	/*!< 0x000000F0 */
#define SYSCFG_EXTICR1_EXTI1                     SYSCFG_EXTICR1_EXTI1_Msk      				/*!< EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2_Pos                 (8U)                          
#define SYSCFG_EXTICR1_EXTI2_Msk                 (0xFUL << SYSCFG_EXTICR1_EXTI2_Pos) 	/*!< 0x00000F00 */
#define SYSCFG_EXTICR1_EXTI2                     SYSCFG_EXTICR1_EXTI2_Msk      				/*!< EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3_Pos                 (12U)                         
#define SYSCFG_EXTICR1_EXTI3_Msk                 (0xFUL << SYSCFG_EXTICR1_EXTI3_Pos) 	/*!< 0x0000F000 */
#define SYSCFG_EXTICR1_EXTI3                     SYSCFG_EXTICR1_EXTI3_Msk      				/*!< EXTI 3 configuration */

/** 
  * @brief  EXTI0 configuration  
  */
#define SYSCFG_EXTICR1_EXTI0_PA                  (0x00000000U)                 /*!< PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB                  (0x00000001U)                 /*!< PB[0] pin */

/** 
  * @brief  EXTI1 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI1_PA                  (0x00000000U)                 /*!< PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB                  (0x00000010U)                 /*!< PB[1] pin */

/** 
  * @brief  EXTI2 configuration  
  */
#define SYSCFG_EXTICR1_EXTI2_PA                  (0x00000000U)                 /*!< PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB                  (0x00000100U)                 /*!< PB[2] pin */

/** 
  * @brief  EXTI3 configuration  
  */
#define SYSCFG_EXTICR1_EXTI3_PA                  (0x00000000U)                 /*!< PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB                  (0x00001000U)                 /*!< PB[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  *****************/
#define SYSCFG_EXTICR2_EXTI4_Pos                 (0U)                          
#define SYSCFG_EXTICR2_EXTI4_Msk                 (0xFUL << SYSCFG_EXTICR2_EXTI4_Pos) 	/*!< 0x0000000F */
#define SYSCFG_EXTICR2_EXTI4                     SYSCFG_EXTICR2_EXTI4_Msk      				/*!< EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5_Pos                 (4U)                          
#define SYSCFG_EXTICR2_EXTI5_Msk                 (0xFUL << SYSCFG_EXTICR2_EXTI5_Pos) 	/*!< 0x000000F0 */
#define SYSCFG_EXTICR2_EXTI5                     SYSCFG_EXTICR2_EXTI5_Msk      				/*!< EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6_Pos                 (8U)                          
#define SYSCFG_EXTICR2_EXTI6_Msk                 (0xFUL << SYSCFG_EXTICR2_EXTI6_Pos) 	/*!< 0x00000F00 */
#define SYSCFG_EXTICR2_EXTI6                     SYSCFG_EXTICR2_EXTI6_Msk      				/*!< EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7_Pos                 (12U)                         
#define SYSCFG_EXTICR2_EXTI7_Msk                 (0xFUL << SYSCFG_EXTICR2_EXTI7_Pos) 	/*!< 0x0000F000 */
#define SYSCFG_EXTICR2_EXTI7                     SYSCFG_EXTICR2_EXTI7_Msk      				/*!< EXTI 7 configuration */

/** 
  * @brief  EXTI4 configuration  
  */
#define SYSCFG_EXTICR2_EXTI4_PA                  (0x00000000U)                 /*!< PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB                  (0x00000001U)                 /*!< PB[4] pin */

/** 
  * @brief  EXTI5 configuration  
  */
#define SYSCFG_EXTICR2_EXTI5_PA                  (0x00000000U)                 /*!< PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB                  (0x00000010U)                 /*!< PB[5] pin */

/** 
  * @brief  EXTI6 configuration  
  */
#define SYSCFG_EXTICR2_EXTI6_PA                  (0x00000000U)                 /*!< PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB                  (0x00000100U)                 /*!< PB[6] pin */

/** 
  * @brief  EXTI7 configuration  
  */
#define SYSCFG_EXTICR2_EXTI7_PA                  (0x00000000U)                 /*!< PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB                  (0x00001000U)                 /*!< PB[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  *****************/
#define SYSCFG_EXTICR3_EXTI8_Pos                 (0U)                          
#define SYSCFG_EXTICR3_EXTI8_Msk                 (0xFUL << SYSCFG_EXTICR3_EXTI8_Pos) 	/*!< 0x0000000F */
#define SYSCFG_EXTICR3_EXTI8                     SYSCFG_EXTICR3_EXTI8_Msk      				/*!< EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9_Pos                 (4U)                          
#define SYSCFG_EXTICR3_EXTI9_Msk                 (0xFUL << SYSCFG_EXTICR3_EXTI9_Pos) 	/*!< 0x000000F0 */
#define SYSCFG_EXTICR3_EXTI9                     SYSCFG_EXTICR3_EXTI9_Msk      				/*!< EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10_Pos                (8U)                          
#define SYSCFG_EXTICR3_EXTI10_Msk                (0xFUL << SYSCFG_EXTICR3_EXTI10_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR3_EXTI10                    SYSCFG_EXTICR3_EXTI10_Msk     				/*!< EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11_Pos                (12U)                         
#define SYSCFG_EXTICR3_EXTI11_Msk                (0xFUL << SYSCFG_EXTICR3_EXTI11_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR3_EXTI11                    SYSCFG_EXTICR3_EXTI11_Msk     				/*!< EXTI 11 configuration */

/** 
  * @brief  EXTI8 configuration  
  */
#define SYSCFG_EXTICR3_EXTI8_PA                  (0x00000000U)                 /*!< PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB                  (0x00000001U)                 /*!< PB[8] pin */

/** 
  * @brief  EXTI9 configuration  
  */
#define SYSCFG_EXTICR3_EXTI9_PA                  (0x00000000U)                 /*!< PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB                  (0x00000010U)                 /*!< PB[9] pin */

/** 
  * @brief  EXTI10 configuration  
  */
#define SYSCFG_EXTICR3_EXTI10_PA                 (0x00000000U)                 /*!< PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB                 (0x00000100U)                 /*!< PB[10] pin */

/** 
  * @brief  EXTI11 configuration  
  */
#define SYSCFG_EXTICR3_EXTI11_PA                 (0x00000000U)                 /*!< PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB                 (0x00001000U)                 /*!< PB[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  *****************/
#define SYSCFG_EXTICR4_EXTI12_Pos                (0U)                          
#define SYSCFG_EXTICR4_EXTI12_Msk                (0xFUL << SYSCFG_EXTICR4_EXTI12_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR4_EXTI12                    SYSCFG_EXTICR4_EXTI12_Msk     				/*!< EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13_Pos                (4U)                          
#define SYSCFG_EXTICR4_EXTI13_Msk                (0xFUL << SYSCFG_EXTICR4_EXTI13_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR4_EXTI13                    SYSCFG_EXTICR4_EXTI13_Msk     				/*!< EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14_Pos                (8U)                          
#define SYSCFG_EXTICR4_EXTI14_Msk                (0xFUL << SYSCFG_EXTICR4_EXTI14_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR4_EXTI14                    SYSCFG_EXTICR4_EXTI14_Msk     				/*!< EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15_Pos                (12U)                         
#define SYSCFG_EXTICR4_EXTI15_Msk                (0xFUL << SYSCFG_EXTICR4_EXTI15_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR4_EXTI15                    SYSCFG_EXTICR4_EXTI15_Msk     				/*!< EXTI 15 configuration */

/** 
  * @brief  EXTI12 configuration  
  */
#define SYSCFG_EXTICR4_EXTI12_PA                 (0x00000000U)                 /*!< PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB                 (0x00000001U)                 /*!< PB[12] pin */

/** 
  * @brief  EXTI13 configuration  
  */
#define SYSCFG_EXTICR4_EXTI13_PA                 (0x00000000U)                 /*!< PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB                 (0x00000010U)                 /*!< PB[13] pin */

/** 
  * @brief  EXTI14 configuration  
  */
#define SYSCFG_EXTICR4_EXTI14_PA                 (0x00000000U)                 /*!< PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB                 (0x00000100U)                 /*!< PB[14] pin */

/** 
  * @brief  EXTI15 configuration  
  */
#define SYSCFG_EXTICR4_EXTI15_PA                 (0x00000000U)                 /*!< PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB                 (0x00001000U)                 /*!< PB[15] pin */

#define SYSCFG_DBGCFG_IWDG_Pos       (0U)                                         
#define SYSCFG_DBGCFG_IWDG_Msk       (0x1U << SYSCFG_DBGCFG_IWDG_Pos)                    
#define SYSCFG_DBGCFG_IWDG           SYSCFG_DBGCFG_IWDG_Msk                              
#define SYSCFG_DBGCFG_WWDG_Pos       (1U)                                         
#define SYSCFG_DBGCFG_WWDG_Msk       (0x1U << SYSCFG_DBGCFG_WWDG_Pos)                    
#define SYSCFG_DBGCFG_WWDG           SYSCFG_DBGCFG_WWDG_Msk


/*****************************************************************************/
/*                                                                           */
/*                               Timers (TIM)                                */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for TIM_CR1 register  *******************/
#define TIM_CR1_CEN_Pos           (0U)                                         
#define TIM_CR1_CEN_Msk           (0x1U << TIM_CR1_CEN_Pos)                    /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                              /*!<Counter enable */
#define TIM_CR1_UDIS_Pos          (1U)                                         
#define TIM_CR1_UDIS_Msk          (0x1U << TIM_CR1_UDIS_Pos)                   /*!< 0x00000002 */
#define TIM_CR1_UDIS              TIM_CR1_UDIS_Msk                             /*!<Update disable */
#define TIM_CR1_URS_Pos           (2U)                                         
#define TIM_CR1_URS_Msk           (0x1U << TIM_CR1_URS_Pos)                    /*!< 0x00000004 */
#define TIM_CR1_URS               TIM_CR1_URS_Msk                              /*!<Update request source */
#define TIM_CR1_OPM_Pos           (3U)                                         
#define TIM_CR1_OPM_Msk           (0x1U << TIM_CR1_OPM_Pos)                    /*!< 0x00000008 */
#define TIM_CR1_OPM               TIM_CR1_OPM_Msk                              /*!<One pulse mode */
#define TIM_CR1_DIR_Pos           (4U)                                         
#define TIM_CR1_DIR_Msk           (0x1U << TIM_CR1_DIR_Pos)                    /*!< 0x00000010 */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk                              /*!<Direction */

#define TIM_CR1_CMS_Pos           (5U)                                         
#define TIM_CR1_CMS_Msk           (0x3U << TIM_CR1_CMS_Pos)                    /*!< 0x00000060 */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk                              /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0             (0x0U << TIM_CR1_CMS_Pos)                    /*!< 0x00000000 */
#define TIM_CR1_CMS_1             (0x1U << TIM_CR1_CMS_Pos)                    /*!< 0x00000020 */
#define TIM_CR1_CMS_2             (0x2U << TIM_CR1_CMS_Pos)                    /*!< 0x00000040 */
#define TIM_CR1_CMS_3             (0x3U << TIM_CR1_CMS_Pos)                    /*!< 0x00000060 */

#define TIM_CR1_ARPE_Pos          (7U)                                         
#define TIM_CR1_ARPE_Msk          (0x1U << TIM_CR1_ARPE_Pos)                   /*!< 0x00000080 */
#define TIM_CR1_ARPE              TIM_CR1_ARPE_Msk                             /*!<Auto-reload preload enable */

#define TIM_CR1_CKD_Pos           (8U)                                         
#define TIM_CR1_CKD_Msk           (0x3U << TIM_CR1_CKD_Pos)                    /*!< 0x00000300 */
#define TIM_CR1_CKD               TIM_CR1_CKD_Msk                              /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0             (0x0U << TIM_CR1_CKD_Pos)                    /*!< 0x00000000 */
#define TIM_CR1_CKD_1             (0x1U << TIM_CR1_CKD_Pos)                    /*!< 0x00000100 */
#define TIM_CR1_CKD_2             (0x2U << TIM_CR1_CKD_Pos)                    /*!< 0x00000200 */


/*******************  Bit definition for TIM_CR2 register  *******************/
#define TIM_CR2_CCPC_Pos          (0U)                                         
#define TIM_CR2_CCPC_Msk          (0x1U << TIM_CR2_CCPC_Pos)                   /*!< 0x00000001 */
#define TIM_CR2_CCPC              TIM_CR2_CCPC_Msk                             /*!<Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS_Pos          (2U)                                         
#define TIM_CR2_CCUS_Msk          (0x1U << TIM_CR2_CCUS_Pos)                   /*!< 0x00000004 */
#define TIM_CR2_CCUS              TIM_CR2_CCUS_Msk                             /*!<Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS_Pos          (3U)                                         
#define TIM_CR2_CCDS_Msk          (0x1U << TIM_CR2_CCDS_Pos)                   /*!< 0x00000008 */
#define TIM_CR2_CCDS              TIM_CR2_CCDS_Msk                             /*!<Capture/Compare DMA Selection */

#define TIM_CR2_MMS_Pos           (4U)                                         
#define TIM_CR2_MMS_Msk           (0x7U << TIM_CR2_MMS_Pos)                    /*!< 0x00000070 */
#define TIM_CR2_MMS               TIM_CR2_MMS_Msk                              /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0             (0x0U << TIM_CR2_MMS_Pos)                    /*!< 0x00000000 */
#define TIM_CR2_MMS_1             (0x1U << TIM_CR2_MMS_Pos)                    /*!< 0x00000010 */
#define TIM_CR2_MMS_2             (0x2U << TIM_CR2_MMS_Pos)                    /*!< 0x00000020 */

#define TIM_CR2_TI1S_Pos          (7U)                                         
#define TIM_CR2_TI1S_Msk          (0x1U << TIM_CR2_TI1S_Pos)                   /*!< 0x00000080 */
#define TIM_CR2_TI1S              TIM_CR2_TI1S_Msk                             /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos          (8U)                                         
#define TIM_CR2_OIS1_Msk          (0x1U << TIM_CR2_OIS1_Pos)                   /*!< 0x00000100 */
#define TIM_CR2_OIS1              TIM_CR2_OIS1_Msk                             /*!<Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N_Pos         (9U)                                         
#define TIM_CR2_OIS1N_Msk         (0x1U << TIM_CR2_OIS1N_Pos)                  /*!< 0x00000200 */
#define TIM_CR2_OIS1N             TIM_CR2_OIS1N_Msk                            /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos          (10U)                                        
#define TIM_CR2_OIS2_Msk          (0x1U << TIM_CR2_OIS2_Pos)                   /*!< 0x00000400 */
#define TIM_CR2_OIS2              TIM_CR2_OIS2_Msk                             /*!<Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N_Pos         (11U)                                        
#define TIM_CR2_OIS2N_Msk         (0x1U << TIM_CR2_OIS2N_Pos)                  /*!< 0x00000800 */
#define TIM_CR2_OIS2N             TIM_CR2_OIS2N_Msk                            /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos          (12U)                                        
#define TIM_CR2_OIS3_Msk          (0x1U << TIM_CR2_OIS3_Pos)                   /*!< 0x00001000 */
#define TIM_CR2_OIS3              TIM_CR2_OIS3_Msk                             /*!<Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N_Pos         (13U)                                        
#define TIM_CR2_OIS3N_Msk         (0x1U << TIM_CR2_OIS3N_Pos)                  /*!< 0x00002000 */
#define TIM_CR2_OIS3N             TIM_CR2_OIS3N_Msk                            /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos          (14U)                                        
#define TIM_CR2_OIS4_Msk          (0x1U << TIM_CR2_OIS4_Pos)                   /*!< 0x00004000 */
#define TIM_CR2_OIS4              TIM_CR2_OIS4_Msk                             /*!<Output Idle state 4 (OC4 output) */

#define TIM_CR2_MMS2_Pos           (20U)                                         
#define TIM_CR2_MMS2_Msk           (0x7U << TIM_CR2_MMS2_Pos)                    /*!< 0x00000070 */
#define TIM_CR2_MMS2               TIM_CR2_MMS2_Msk                              /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS2_0             (0x0U << TIM_CR2_MMS2_Pos)                    /*!< 0x00000000 */
#define TIM_CR2_MMS2_1             (0x1U << TIM_CR2_MMS2_Pos)                    /*!< 0x00000010 */
#define TIM_CR2_MMS2_2             (0x2U << TIM_CR2_MMS2_Pos)                    /*!< 0x00000020 */

/*******************  Bit definition for TIM_SMCR register  ******************/
#define TIM_SMCR_SMS_Pos          (0U)                                         
#define TIM_SMCR_SMS_Msk          (0x7U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000007 */
#define TIM_SMCR_SMS              TIM_SMCR_SMS_Msk                             /*!<SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0            (0x0U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000000 */					//?????????????????????????????
#define TIM_SMCR_SMS_1            (0x1U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000001 */
#define TIM_SMCR_SMS_2            (0x2U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000002 */   
#define TIM_SMCR_SMS_3            (0x3U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000003 */
#define TIM_SMCR_SMS_4            (0x4U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000004 */
#define TIM_SMCR_SMS_5            (0x5U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000005 */
#define TIM_SMCR_SMS_6            (0x6U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000006 */
#define TIM_SMCR_SMS_7            (0x7U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000007 */


#define TIM_SMCR_OCCS_Pos         (3U)                                         
#define TIM_SMCR_OCCS_Msk         (0x1U << TIM_SMCR_OCCS_Pos)                  /*!< 0x00000008 */
#define TIM_SMCR_OCCS             TIM_SMCR_OCCS_Msk                            /*!< OCREF clear selection */

#define TIM_SMCR_TS_Pos           (4U)                                         
#define TIM_SMCR_TS_Msk           (0x7U << TIM_SMCR_TS_Pos)                    /*!< 0x00000070 */
#define TIM_SMCR_TS               TIM_SMCR_TS_Msk                              /*!<TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0             (0x0U << TIM_SMCR_TS_Pos)                    /*!< 0x00000000 */
#define TIM_SMCR_TS_1             (0x1U << TIM_SMCR_TS_Pos)                    /*!< 0x00000010 */
#define TIM_SMCR_TS_2             (0x2U << TIM_SMCR_TS_Pos)                    /*!< 0x00000020 */
#define TIM_SMCR_TS_3             (0x3U << TIM_SMCR_TS_Pos)                    /*!< 0x00000030 */
#define TIM_SMCR_TS_4             (0x4U << TIM_SMCR_TS_Pos)                    /*!< 0x00000040 */
#define TIM_SMCR_TS_5             (0x5U << TIM_SMCR_TS_Pos)                    /*!< 0x00000050 */
#define TIM_SMCR_TS_6             (0x6U << TIM_SMCR_TS_Pos)                    /*!< 0x00000060 */
#define TIM_SMCR_TS_7             (0x7U << TIM_SMCR_TS_Pos)                    /*!< 0x00000070 */

#define TIM_SMCR_MSM_Pos          (7U)                                         
#define TIM_SMCR_MSM_Msk          (0x1U << TIM_SMCR_MSM_Pos)                   /*!< 0x00000080 */
#define TIM_SMCR_MSM              TIM_SMCR_MSM_Msk                             /*!<Master/slave mode */

#define TIM_SMCR_ETF_Pos          (8U)                                         
#define TIM_SMCR_ETF_Msk          (0xFU << TIM_SMCR_ETF_Pos)                   /*!< 0x00000F00 */
#define TIM_SMCR_ETF              TIM_SMCR_ETF_Msk                             /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0            (0x0U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000000 */
#define TIM_SMCR_ETF_1            (0x1U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000100 */
#define TIM_SMCR_ETF_2            (0x2U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000200 */
#define TIM_SMCR_ETF_3            (0x3U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000300 */

#define TIM_SMCR_ETPS_Pos         (12U)                                        
#define TIM_SMCR_ETPS_Msk         (0x3U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00003000 */
#define TIM_SMCR_ETPS             TIM_SMCR_ETPS_Msk                            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0           (0x0U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00000000 */
#define TIM_SMCR_ETPS_1           (0x1U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00001000 */
#define TIM_SMCR_ETPS_2           (0x2U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00002000 */
#define TIM_SMCR_ETPS_3           (0x3U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00003000 */

#define TIM_SMCR_ECE_Pos          (14U)                                        
#define TIM_SMCR_ECE_Msk          (0x1U << TIM_SMCR_ECE_Pos)                   /*!< 0x00004000 */
#define TIM_SMCR_ECE              TIM_SMCR_ECE_Msk                             /*!<External clock enable */
#define TIM_SMCR_ETP_Pos          (15U)                                        
#define TIM_SMCR_ETP_Msk          (0x1U << TIM_SMCR_ETP_Pos)                   /*!< 0x00008000 */
#define TIM_SMCR_ETP              TIM_SMCR_ETP_Msk                             /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  ******************/
#define TIM_DIER_UIE_Pos          (0U)                                         
#define TIM_DIER_UIE_Msk          (0x1U << TIM_DIER_UIE_Pos)                   /*!< 0x00000001 */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk                             /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos        (1U)                                         
#define TIM_DIER_CC1IE_Msk        (0x1U << TIM_DIER_CC1IE_Pos)                 /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk                           /*!<Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE_Pos        (2U)                                         
#define TIM_DIER_CC2IE_Msk        (0x1U << TIM_DIER_CC2IE_Pos)                 /*!< 0x00000004 */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk                           /*!<Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE_Pos        (3U)                                         
#define TIM_DIER_CC3IE_Msk        (0x1U << TIM_DIER_CC3IE_Pos)                 /*!< 0x00000008 */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk                           /*!<Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE_Pos        (4U)                                         
#define TIM_DIER_CC4IE_Msk        (0x1U << TIM_DIER_CC4IE_Pos)                 /*!< 0x00000010 */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk                           /*!<Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE_Pos        (5U)                                         
#define TIM_DIER_COMIE_Msk        (0x1U << TIM_DIER_COMIE_Pos)                 /*!< 0x00000020 */
#define TIM_DIER_COMIE            TIM_DIER_COMIE_Msk                           /*!<COM interrupt enable */
#define TIM_DIER_TIE_Pos          (6U)                                         
#define TIM_DIER_TIE_Msk          (0x1U << TIM_DIER_TIE_Pos)                   /*!< 0x00000040 */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk                             /*!<Trigger interrupt enable */
#define TIM_DIER_BIE_Pos          (7U)                                         
#define TIM_DIER_BIE_Msk          (0x1U << TIM_DIER_BIE_Pos)                   /*!< 0x00000080 */
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk                             /*!<Break interrupt enable */
#define TIM_DIER_UDE_Pos          (8U)                                         
#define TIM_DIER_UDE_Msk          (0x1U << TIM_DIER_UDE_Pos)                   /*!< 0x00000100 */
#define TIM_DIER_UDE              TIM_DIER_UDE_Msk                             /*!<Update DMA request enable */
#define TIM_DIER_CC1DE_Pos        (9U)                                         
#define TIM_DIER_CC1DE_Msk        (0x1U << TIM_DIER_CC1DE_Pos)                 /*!< 0x00000200 */
#define TIM_DIER_CC1DE            TIM_DIER_CC1DE_Msk                           /*!<Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE_Pos        (10U)                                        
#define TIM_DIER_CC2DE_Msk        (0x1U << TIM_DIER_CC2DE_Pos)                 /*!< 0x00000400 */
#define TIM_DIER_CC2DE            TIM_DIER_CC2DE_Msk                           /*!<Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE_Pos        (11U)                                        
#define TIM_DIER_CC3DE_Msk        (0x1U << TIM_DIER_CC3DE_Pos)                 /*!< 0x00000800 */
#define TIM_DIER_CC3DE            TIM_DIER_CC3DE_Msk                           /*!<Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE_Pos        (12U)                                        
#define TIM_DIER_CC4DE_Msk        (0x1U << TIM_DIER_CC4DE_Pos)                 /*!< 0x00001000 */
#define TIM_DIER_CC4DE            TIM_DIER_CC4DE_Msk                           /*!<Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE_Pos        (13U)                                        
#define TIM_DIER_COMDE_Msk        (0x1U << TIM_DIER_COMDE_Pos)                 /*!< 0x00002000 */
#define TIM_DIER_COMDE            TIM_DIER_COMDE_Msk                           /*!<COM DMA request enable */
#define TIM_DIER_TDE_Pos          (14U)                                        
#define TIM_DIER_TDE_Msk          (0x1U << TIM_DIER_TDE_Pos)                   /*!< 0x00004000 */
#define TIM_DIER_TDE              TIM_DIER_TDE_Msk                             /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  *******************/
#define TIM_SR_UIF_Pos            (0U)                                         
#define TIM_SR_UIF_Msk            (0x1U << TIM_SR_UIF_Pos)                     /*!< 0x00000001 */
#define TIM_SR_UIF                TIM_SR_UIF_Msk                               /*!<Update interrupt Flag */
#define TIM_SR_CC1IF_Pos          (1U)                                         
#define TIM_SR_CC1IF_Msk          (0x1U << TIM_SR_CC1IF_Pos)                   /*!< 0x00000002 */
#define TIM_SR_CC1IF              TIM_SR_CC1IF_Msk                             /*!<Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF_Pos          (2U)                                         
#define TIM_SR_CC2IF_Msk          (0x1U << TIM_SR_CC2IF_Pos)                   /*!< 0x00000004 */
#define TIM_SR_CC2IF              TIM_SR_CC2IF_Msk                             /*!<Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF_Pos          (3U)                                         
#define TIM_SR_CC3IF_Msk          (0x1U << TIM_SR_CC3IF_Pos)                   /*!< 0x00000008 */
#define TIM_SR_CC3IF              TIM_SR_CC3IF_Msk                             /*!<Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF_Pos          (4U)                                         
#define TIM_SR_CC4IF_Msk          (0x1U << TIM_SR_CC4IF_Pos)                   /*!< 0x00000010 */
#define TIM_SR_CC4IF              TIM_SR_CC4IF_Msk                             /*!<Capture/Compare 4 interrupt Flag */
//#define TIM_SR_COMIF_Pos          (5U)                                         
//#define TIM_SR_COMIF_Msk          (0x1U << TIM_SR_COMIF_Pos)                   /*!< 0x00000020 */
//#define TIM_SR_COMIF              TIM_SR_COMIF_Msk                             /*!<COM interrupt Flag */
#define TIM_SR_TIF_Pos            (6U)                                         
#define TIM_SR_TIF_Msk            (0x1U << TIM_SR_TIF_Pos)                     /*!< 0x00000040 */
#define TIM_SR_TIF                TIM_SR_TIF_Msk                               /*!<Trigger interrupt Flag */
//#define TIM_SR_BIF_Pos            (7U)                                         
//#define TIM_SR_BIF_Msk            (0x1U << TIM_SR_BIF_Pos)                     /*!< 0x00000080 */
//#define TIM_SR_BIF                TIM_SR_BIF_Msk                               /*!<Break interrupt Flag */
#define TIM_SR_CC1OF_Pos          (9U)                                         
#define TIM_SR_CC1OF_Msk          (0x1U << TIM_SR_CC1OF_Pos)                   /*!< 0x00000200 */
#define TIM_SR_CC1OF              TIM_SR_CC1OF_Msk                             /*!<Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF_Pos          (10U)                                        
#define TIM_SR_CC2OF_Msk          (0x1U << TIM_SR_CC2OF_Pos)                   /*!< 0x00000400 */
#define TIM_SR_CC2OF              TIM_SR_CC2OF_Msk                             /*!<Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF_Pos          (11U)                                        
#define TIM_SR_CC3OF_Msk          (0x1U << TIM_SR_CC3OF_Pos)                   /*!< 0x00000800 */
#define TIM_SR_CC3OF              TIM_SR_CC3OF_Msk                             /*!<Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF_Pos          (12U)                                        
#define TIM_SR_CC4OF_Msk          (0x1U << TIM_SR_CC4OF_Pos)                   /*!< 0x00001000 */
#define TIM_SR_CC4OF              TIM_SR_CC4OF_Msk                             /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  *******************/
#define TIM_EGR_UG_Pos            (0U)                                         
#define TIM_EGR_UG_Msk            (0x1U << TIM_EGR_UG_Pos)                     /*!< 0x00000001 */
#define TIM_EGR_UG                TIM_EGR_UG_Msk                               /*!<Update Generation */
#define TIM_EGR_CC1G_Pos          (1U)                                         
#define TIM_EGR_CC1G_Msk          (0x1U << TIM_EGR_CC1G_Pos)                   /*!< 0x00000002 */
#define TIM_EGR_CC1G              TIM_EGR_CC1G_Msk                             /*!<Capture/Compare 1 Generation */
#define TIM_EGR_CC2G_Pos          (2U)                                         
#define TIM_EGR_CC2G_Msk          (0x1U << TIM_EGR_CC2G_Pos)                   /*!< 0x00000004 */
#define TIM_EGR_CC2G              TIM_EGR_CC2G_Msk                             /*!<Capture/Compare 2 Generation */
#define TIM_EGR_CC3G_Pos          (3U)                                         
#define TIM_EGR_CC3G_Msk          (0x1U << TIM_EGR_CC3G_Pos)                   /*!< 0x00000008 */
#define TIM_EGR_CC3G              TIM_EGR_CC3G_Msk                             /*!<Capture/Compare 3 Generation */
#define TIM_EGR_CC4G_Pos          (4U)                                         
#define TIM_EGR_CC4G_Msk          (0x1U << TIM_EGR_CC4G_Pos)                   /*!< 0x00000010 */
#define TIM_EGR_CC4G              TIM_EGR_CC4G_Msk                             /*!<Capture/Compare 4 Generation */
#define TIM_EGR_COMG_Pos          (5U)                                         
#define TIM_EGR_COMG_Msk          (0x1U << TIM_EGR_COMG_Pos)                   /*!< 0x00000020 */
#define TIM_EGR_COMG              TIM_EGR_COMG_Msk                             /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos            (6U)                                         
#define TIM_EGR_TG_Msk            (0x1U << TIM_EGR_TG_Pos)                     /*!< 0x00000040 */
#define TIM_EGR_TG                TIM_EGR_TG_Msk                               /*!<Trigger Generation */
//#define TIM_EGR_BG_Pos            (7U)                                         
//#define TIM_EGR_BG_Msk            (0x1U << TIM_EGR_BG_Pos)                     /*!< 0x00000080 */
//#define TIM_EGR_BG                TIM_EGR_BG_Msk                               /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  ******************/
#define TIM_CCMR1_CC1S_Pos        (0U)                                         
#define TIM_CCMR1_CC1S_Msk        (0x3U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR1_CC1S            TIM_CCMR1_CC1S_Msk                           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0          (0x0U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR1_CC1S_1          (0x1U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000001 */
#define TIM_CCMR1_CC1S_2          (0x2U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000002 */
#define TIM_CCMR1_CC1S_3          (0x3U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000003 */

#define TIM_CCMR1_OC1FE_Pos       (2U)                                         
#define TIM_CCMR1_OC1FE_Msk       (0x1U << TIM_CCMR1_OC1FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE           TIM_CCMR1_OC1FE_Msk                          /*!<Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE_Pos       (3U)                                         
#define TIM_CCMR1_OC1PE_Msk       (0x1U << TIM_CCMR1_OC1PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE           TIM_CCMR1_OC1PE_Msk                          /*!<Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M_Pos        (4U)                                         
#define TIM_CCMR1_OC1M_Msk        (0x7U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000070 */
#define TIM_CCMR1_OC1M            TIM_CCMR1_OC1M_Msk                           /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0          (0x0U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR1_OC1M_1          (0x1U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR1_OC1M_2          (0x2U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000020 */  
#define TIM_CCMR1_OC1M_3          (0x3U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000030 */
#define TIM_CCMR1_OC1M_4          (0x4U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR1_OC1M_5          (0x5U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000050 */
#define TIM_CCMR1_OC1M_6          (0x6U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000060 */
#define TIM_CCMR1_OC1M_7          (0x7U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000070 */


#define TIM_CCMR1_OC1CE_Pos       (7U)                                         
#define TIM_CCMR1_OC1CE_Msk       (0x1U << TIM_CCMR1_OC1CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE           TIM_CCMR1_OC1CE_Msk                          /*!<Output Compare 1Clear Enable */

#define TIM_CCMR1_CC2S_Pos        (8U)                                         
#define TIM_CCMR1_CC2S_Msk        (0x3U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR1_CC2S            TIM_CCMR1_CC2S_Msk                           /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0          (0x0U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR1_CC2S_1          (0x1U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000100 */
#define TIM_CCMR1_CC2S_2          (0x2U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000200 */
#define TIM_CCMR1_CC2S_3          (0x3U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000300 */

#define TIM_CCMR1_OC2FE_Pos       (10U)                                        
#define TIM_CCMR1_OC2FE_Msk       (0x1U << TIM_CCMR1_OC2FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE           TIM_CCMR1_OC2FE_Msk                          /*!<Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE_Pos       (11U)                                        
#define TIM_CCMR1_OC2PE_Msk       (0x1U << TIM_CCMR1_OC2PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE           TIM_CCMR1_OC2PE_Msk                          /*!<Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M_Pos        (12U)                                        
#define TIM_CCMR1_OC2M_Msk        (0x7U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00007000 */
#define TIM_CCMR1_OC2M            TIM_CCMR1_OC2M_Msk                           /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0          (0x0U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR1_OC2M_1          (0x1U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR1_OC2M_2          (0x2U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_3          (0x3U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_4          (0x4U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_5          (0x5U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_6          (0x6U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_7          (0x7U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00002000 */

#define TIM_CCMR1_OC2CE_Pos       (15U)                                        
#define TIM_CCMR1_OC2CE_Msk       (0x1U << TIM_CCMR1_OC2CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE           TIM_CCMR1_OC2CE_Msk                          /*!<Output Compare 2 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC_Pos      (2U)                                         
#define TIM_CCMR1_IC1PSC_Msk      (0x3U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC          TIM_CCMR1_IC1PSC_Msk                         /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0        (0x0U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000004 */
#define TIM_CCMR1_IC1PSC_1        (0x1U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000005 */
#define TIM_CCMR1_IC1PSC_2        (0x2U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000006 */
#define TIM_CCMR1_IC1PSC_3        (0x3U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000007 */

#define TIM_CCMR1_IC1F_Pos        (4U)                                         
#define TIM_CCMR1_IC1F_Msk        (0xFU << TIM_CCMR1_IC1F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F            TIM_CCMR1_IC1F_Msk                           /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0          (0x0U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR1_IC1F_1          (0x1U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR1_IC1F_2          (0x2U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR1_IC1F_3          (0x3U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000030 */
#define TIM_CCMR1_IC1F_4          (0x4U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR1_IC1F_5          (0x5U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000050 */
#define TIM_CCMR1_IC1F_6          (0x6U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000060 */
#define TIM_CCMR1_IC1F_7          (0x7U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000070 */


#define TIM_CCMR1_IC2PSC_Pos      (10U)                                        
#define TIM_CCMR1_IC2PSC_Msk      (0x3U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC          TIM_CCMR1_IC2PSC_Msk                         /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0        (0x0U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000000 */
#define TIM_CCMR1_IC2PSC_1        (0x1U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000400 */
#define TIM_CCMR1_IC2PSC_2        (0x2U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000800 */
#define TIM_CCMR1_IC2PSC_3        (0x3U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000C00 */

#define TIM_CCMR1_IC2F_Pos        (12U)                                        
#define TIM_CCMR1_IC2F_Msk        (0xFU << TIM_CCMR1_IC2F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F            TIM_CCMR1_IC2F_Msk                           /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0          (0x0U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR1_IC2F_1          (0x1U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR1_IC2F_2          (0x2U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_IC2F_3          (0x3U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00003000 */

/******************  Bit definition for TIM_CCMR2 register  ******************/
#define TIM_CCMR2_CC3S_Pos        (0U)                                         
#define TIM_CCMR2_CC3S_Msk        (0x3U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR2_CC3S            TIM_CCMR2_CC3S_Msk                           /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0          (0x0U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR2_CC3S_1          (0x1U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000001 */
#define TIM_CCMR2_CC3S_2          (0x2U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000002 */
#define TIM_CCMR2_CC3S_3          (0x3U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */

#define TIM_CCMR2_OC3FE_Pos       (2U)                                         
#define TIM_CCMR2_OC3FE_Msk       (0x1U << TIM_CCMR2_OC3FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE           TIM_CCMR2_OC3FE_Msk                          /*!<Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE_Pos       (3U)                                         
#define TIM_CCMR2_OC3PE_Msk       (0x1U << TIM_CCMR2_OC3PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE           TIM_CCMR2_OC3PE_Msk                          /*!<Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M_Pos        (4U)                                         
#define TIM_CCMR2_OC3M_Msk        (0x7U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000070 */
#define TIM_CCMR2_OC3M            TIM_CCMR2_OC3M_Msk                           /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0          (0x0U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR2_OC3M_1          (0x1U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR2_OC3M_2          (0x2U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR2_OC3M_3          (0x3U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000030 */
#define TIM_CCMR2_OC3M_4          (0x4U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR2_OC3M_5          (0x5U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000050 */
#define TIM_CCMR2_OC3M_6          (0x6U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000060 */
#define TIM_CCMR2_OC3M_7          (0x7U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000070 */

#define TIM_CCMR2_OC3CE_Pos       (7U)                                         
#define TIM_CCMR2_OC3CE_Msk       (0x1U << TIM_CCMR2_OC3CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE           TIM_CCMR2_OC3CE_Msk                          /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos        (8U)                                         
#define TIM_CCMR2_CC4S_Msk        (0x3U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR2_CC4S            TIM_CCMR2_CC4S_Msk                           /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0          (0x0U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR2_CC4S_1          (0x1U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000100 */
#define TIM_CCMR2_CC4S_2          (0x2U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000200 */
#define TIM_CCMR2_CC4S_3          (0x3U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000300 */

#define TIM_CCMR2_OC4FE_Pos       (10U)                                        
#define TIM_CCMR2_OC4FE_Msk       (0x1U << TIM_CCMR2_OC4FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE           TIM_CCMR2_OC4FE_Msk                          /*!<Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE_Pos       (11U)                                        
#define TIM_CCMR2_OC4PE_Msk       (0x1U << TIM_CCMR2_OC4PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE           TIM_CCMR2_OC4PE_Msk                          /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos        (12U)                                        
#define TIM_CCMR2_OC4M_Msk        (0x7U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00007000 */
#define TIM_CCMR2_OC4M            TIM_CCMR2_OC4M_Msk                           /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0          (0x0U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR2_OC4M_1          (0x1U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR2_OC4M_2          (0x2U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR2_OC4M_3          (0x3U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00000030 */
#define TIM_CCMR2_OC4M_4          (0x4U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR2_OC4M_5          (0x5U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00000050 */
#define TIM_CCMR2_OC4M_6          (0x6U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00000060 */
#define TIM_CCMR2_OC4M_7          (0x7U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00000070 */


#define TIM_CCMR2_OC4CE_Pos       (15U)                                        
#define TIM_CCMR2_OC4CE_Msk       (0x1U << TIM_CCMR2_OC4CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE           TIM_CCMR2_OC4CE_Msk                          /*!<Output Compare 4 Clear Enable */

/*---------------------------------------------------------------------------*/
//#define TIM_CCMR2_CC3S_Pos        (0U)                                         
//#define TIM_CCMR2_CC3S_Msk        (0x3U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */
//#define TIM_CCMR2_CC3S            TIM_CCMR1_CC1S_Msk                           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
//#define TIM_CCMR2_CC3S_0          (0x0U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000000 */
//#define TIM_CCMR2_CC3S_1          (0x1U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000001 */
//#define TIM_CCMR2_CC3S_2          (0x2U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000002 */
//#define TIM_CCMR2_CC3S_3          (0x3U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */

#define TIM_CCMR2_IC3PSC_Pos      (2U)                                         
#define TIM_CCMR2_IC3PSC_Msk      (0x3U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC          TIM_CCMR2_IC3PSC_Msk                         /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0        (0x0U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000000 */
#define TIM_CCMR2_IC3PSC_1        (0x1U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000004 */
#define TIM_CCMR2_IC3PSC_2        (0x2U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000008 */
#define TIM_CCMR2_IC3PSC_3        (0x3U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0000000C */

#define TIM_CCMR2_IC3F_Pos        (4U)                                         
#define TIM_CCMR2_IC3F_Msk        (0xFU << TIM_CCMR2_IC3F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F            TIM_CCMR2_IC3F_Msk                           /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0          (0x0U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR2_IC3F_1          (0x1U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR2_IC3F_2          (0x2U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR2_IC3F_3          (0x3U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000030 */
#define TIM_CCMR2_IC3F_4          (0x4U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR2_IC3F_5          (0x5U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000050 */
#define TIM_CCMR2_IC3F_6          (0x6U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000060 */
#define TIM_CCMR2_IC3F_7          (0x7U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000070 */

#define TIM_CCMR2_IC4PSC_Pos      (10U)                                        
#define TIM_CCMR2_IC4PSC_Msk      (0x3U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC          TIM_CCMR2_IC4PSC_Msk                         /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */			//????????????????
#define TIM_CCMR2_IC4PSC_0        (0x0U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000000 */
#define TIM_CCMR2_IC4PSC_1        (0x1U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000400 */
#define TIM_CCMR2_IC4PSC_2        (0x2U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000800 */

#define TIM_CCMR2_IC4F_Pos        (12U)                                        
#define TIM_CCMR2_IC4F_Msk        (0xFU << TIM_CCMR2_IC4F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F            TIM_CCMR2_IC4F_Msk                           /*!<IC4F[3:0] bits (Input Capture 4 Filter) */						//????????????????
#define TIM_CCMR2_IC4F_0          (0x0U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00000000 */
#define TIM_CCMR2_IC4F_1          (0x1U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR2_IC4F_2          (0x2U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR2_IC4F_3          (0x3U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00003000 */


/*******************  Bit definition for TIM_CCER register  ******************/
#define TIM_CCER_CC1E_Pos         (0U)                                         
#define TIM_CCER_CC1E_Msk         (0x1U << TIM_CCER_CC1E_Pos)                  /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                            /*!<Capture/Compare 1 output enable */
#define TIM_CCER_CC1P_Pos         (1U)                                         
#define TIM_CCER_CC1P_Msk         (0x1U << TIM_CCER_CC1P_Pos)                  /*!< 0x00000002 */
#define TIM_CCER_CC1P             TIM_CCER_CC1P_Msk                            /*!<Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE_Pos        (2U)                                         
#define TIM_CCER_CC1NE_Msk        (0x1U << TIM_CCER_CC1NE_Pos)                 /*!< 0x00000004 */
#define TIM_CCER_CC1NE            TIM_CCER_CC1NE_Msk                           /*!<Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP_Pos        (3U)                                         
#define TIM_CCER_CC1NP_Msk        (0x1U << TIM_CCER_CC1NP_Pos)                 /*!< 0x00000008 */
#define TIM_CCER_CC1NP            TIM_CCER_CC1NP_Msk                           /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos         (4U)                                         
#define TIM_CCER_CC2E_Msk         (0x1U << TIM_CCER_CC2E_Pos)                  /*!< 0x00000010 */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk                            /*!<Capture/Compare 2 output enable */
#define TIM_CCER_CC2P_Pos         (5U)                                         
#define TIM_CCER_CC2P_Msk         (0x1U << TIM_CCER_CC2P_Pos)                  /*!< 0x00000020 */
#define TIM_CCER_CC2P             TIM_CCER_CC2P_Msk                            /*!<Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE_Pos        (6U)                                         
#define TIM_CCER_CC2NE_Msk        (0x1U << TIM_CCER_CC2NE_Pos)                 /*!< 0x00000040 */
#define TIM_CCER_CC2NE            TIM_CCER_CC2NE_Msk                           /*!<Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP_Pos        (7U)                                         
#define TIM_CCER_CC2NP_Msk        (0x1U << TIM_CCER_CC2NP_Pos)                 /*!< 0x00000080 */
#define TIM_CCER_CC2NP            TIM_CCER_CC2NP_Msk                           /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos         (8U)                                         
#define TIM_CCER_CC3E_Msk         (0x1U << TIM_CCER_CC3E_Pos)                  /*!< 0x00000100 */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk                            /*!<Capture/Compare 3 output enable */
#define TIM_CCER_CC3P_Pos         (9U)                                         
#define TIM_CCER_CC3P_Msk         (0x1U << TIM_CCER_CC3P_Pos)                  /*!< 0x00000200 */
#define TIM_CCER_CC3P             TIM_CCER_CC3P_Msk                            /*!<Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE_Pos        (10U)                                        
#define TIM_CCER_CC3NE_Msk        (0x1U << TIM_CCER_CC3NE_Pos)                 /*!< 0x00000400 */
#define TIM_CCER_CC3NE            TIM_CCER_CC3NE_Msk                           /*!<Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP_Pos        (11U)                                        
#define TIM_CCER_CC3NP_Msk        (0x1U << TIM_CCER_CC3NP_Pos)                 /*!< 0x00000800 */
#define TIM_CCER_CC3NP            TIM_CCER_CC3NP_Msk                           /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos         (12U)                                        
#define TIM_CCER_CC4E_Msk         (0x1U << TIM_CCER_CC4E_Pos)                  /*!< 0x00001000 */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk                            /*!<Capture/Compare 4 output enable */
#define TIM_CCER_CC4P_Pos         (13U)                                        
#define TIM_CCER_CC4P_Msk         (0x1U << TIM_CCER_CC4P_Pos)                  /*!< 0x00002000 */
#define TIM_CCER_CC4P             TIM_CCER_CC4P_Msk                            /*!<Capture/Compare 4 output Polarity */
#define TIM_CCER_CC4NP_Pos        (15U)                                        
#define TIM_CCER_CC4NP_Msk        (0x1U << TIM_CCER_CC4NP_Pos)                 /*!< 0x00008000 */
#define TIM_CCER_CC4NP            TIM_CCER_CC4NP_Msk                           /*!<Capture/Compare 4 Complementary output Polarity */
#define  TIM_CCER_CC5E            ((uint32_t)0x00010000)            /*!<Capture/Compare 5 output enable */
#define  TIM_CCER_CC5P            ((uint32_t)0x00020000)            /*!<Capture/Compare 5 output Polarity */
#define  TIM_CCER_CC6E            ((uint32_t)0x00100000)            /*!<Capture/Compare 6 output enable */
#define  TIM_CCER_CC6P            ((uint32_t)0x00200000)            /*!<Capture/Compare 6 output Polarity */

/*******************  Bit definition for TIM_CNT register  *******************/
#define TIM_CNT_CNT_Pos           (0U)                                         
#define TIM_CNT_CNT_Msk           (0xFFFFU << TIM_CNT_CNT_Pos)             		 /*!< 0x0000FFFF */
#define TIM_CNT_CNT               TIM_CNT_CNT_Msk                              /*!<Counter Value */

/*******************  Bit definition for TIM_PSC register  *******************/
#define TIM_PSC_PSC_Pos           (0U)                                         
#define TIM_PSC_PSC_Msk           (0xFFFFU << TIM_PSC_PSC_Pos)                 /*!< 0x0000FFFF */
#define TIM_PSC_PSC               TIM_PSC_PSC_Msk                              /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  *******************/
#define TIM_ARR_ARR_Pos           (0U)                                         
#define TIM_ARR_ARR_Msk           (0xFFFFU << TIM_ARR_ARR_Pos)             		 /*!< 0x0000FFFF */
#define TIM_ARR_ARR               TIM_ARR_ARR_Msk                              /*!<actual auto-reload Value */

///*******************  Bit definition for TIM_RCR register  *******************/
//#define TIM_RCR_REP_Pos           (0U)                                         
//#define TIM_RCR_REP_Msk           (0xFFU << TIM_RCR_REP_Pos)                   /*!< 0x000000FF */
//#define TIM_RCR_REP               TIM_RCR_REP_Msk                              /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  ******************/
#define TIM_CCR1_CCR1_Pos         (0U)                                         
#define TIM_CCR1_CCR1_Msk         (0xFFFFU << TIM_CCR1_CCR1_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1             TIM_CCR1_CCR1_Msk                            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  ******************/
#define TIM_CCR2_CCR2_Pos         (0U)                                         
#define TIM_CCR2_CCR2_Msk         (0xFFFFU << TIM_CCR2_CCR2_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2             TIM_CCR2_CCR2_Msk                            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  ******************/
#define TIM_CCR3_CCR3_Pos         (0U)                                         
#define TIM_CCR3_CCR3_Msk         (0xFFFFU << TIM_CCR3_CCR3_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3             TIM_CCR3_CCR3_Msk                            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  ******************/
#define TIM_CCR4_CCR4_Pos         (0U)                                         
#define TIM_CCR4_CCR4_Msk         (0xFFFFU << TIM_CCR4_CCR4_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4             TIM_CCR4_CCR4_Msk                            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  ******************/
#define TIM_BDTR_DTG_Pos          (0U)                                         
#define TIM_BDTR_DTG_Msk          (0xFFU << TIM_BDTR_DTG_Pos)                  /*!< 0x000000FF */
#define TIM_BDTR_DTG              TIM_BDTR_DTG_Msk                             /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0            (0x01U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000001 */
#define TIM_BDTR_DTG_1            (0x02U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000002 */
#define TIM_BDTR_DTG_2            (0x04U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000004 */
#define TIM_BDTR_DTG_3            (0x08U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000008 */
#define TIM_BDTR_DTG_4            (0x10U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000010 */
#define TIM_BDTR_DTG_5            (0x20U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000020 */
#define TIM_BDTR_DTG_6            (0x40U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000040 */
#define TIM_BDTR_DTG_7            (0x80U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000080 */

#define TIM_BDTR_LOCK_Pos         (8U)                                         
#define TIM_BDTR_LOCK_Msk         (0x3U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000300 */
#define TIM_BDTR_LOCK             TIM_BDTR_LOCK_Msk                            /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0           (0x1U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000100 */
#define TIM_BDTR_LOCK_1           (0x2U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000200 */

#define TIM_BDTR_OSSI_Pos         (10U)                                        
#define TIM_BDTR_OSSI_Msk         (0x1U << TIM_BDTR_OSSI_Pos)                  /*!< 0x00000400 */
#define TIM_BDTR_OSSI             TIM_BDTR_OSSI_Msk                            /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos         (11U)                                        
#define TIM_BDTR_OSSR_Msk         (0x1U << TIM_BDTR_OSSR_Pos)                  /*!< 0x00000800 */
#define TIM_BDTR_OSSR             TIM_BDTR_OSSR_Msk                            /*!<Off-State Selection for Run mode */
#define TIM_BDTR_BKE_Pos          (12U)                                        
#define TIM_BDTR_BKE_Msk          (0x1U << TIM_BDTR_BKE_Pos)                   /*!< 0x00001000 */
#define TIM_BDTR_BKE              TIM_BDTR_BKE_Msk                             /*!<Break enable */
#define TIM_BDTR_BKP_Pos          (13U)                                        
#define TIM_BDTR_BKP_Msk          (0x1U << TIM_BDTR_BKP_Pos)                   /*!< 0x00002000 */
#define TIM_BDTR_BKP              TIM_BDTR_BKP_Msk                             /*!<Break Polarity */
#define TIM_BDTR_AOE_Pos          (14U)                                        
#define TIM_BDTR_AOE_Msk          (0x1U << TIM_BDTR_AOE_Pos)                   /*!< 0x00004000 */
#define TIM_BDTR_AOE              TIM_BDTR_AOE_Msk                             /*!<Automatic Output enable */
#define TIM_BDTR_MOE_Pos          (15U)                                        
#define TIM_BDTR_MOE_Msk          (0x1U << TIM_BDTR_MOE_Pos)                   /*!< 0x00008000 */
#define TIM_BDTR_MOE              TIM_BDTR_MOE_Msk                             /*!<Main Output enable */

/******************  Bit definition for TIM_CCMR3 register  *******************/
#define  TIM_CCMR3_OC5FE                     ((uint32_t)0x00000004)            /*!<Output Compare 5 Fast enable */
#define  TIM_CCMR3_OC5PE                     ((uint32_t)0x00000008)            /*!<Output Compare 5 Preload enable */

#define  TIM_CCMR3_OC5M                      ((uint32_t)0x00000070)            /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
#define  TIM_CCMR3_OC5M_0                    ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR3_OC5M_1                    ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR3_OC5M_2                    ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR3_OC5M_3                    ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_CCMR3_OC5CE                     ((uint32_t)0x00000080)            /*!<Output Compare 5 Clear Enable */

#define  TIM_CCMR3_OC6FE                     ((uint32_t)0x00000400)            /*!<Output Compare 4 Fast enable */
#define  TIM_CCMR3_OC6PE                     ((uint32_t)0x00000800)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR3_OC6M                      ((uint32_t)0x00007000)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR3_OC6M_0                    ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR3_OC6M_1                    ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR3_OC6M_2                    ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR3_OC6M_3                    ((uint32_t)0x00100000)            /*!<Bit 3 */

#define  TIM_CCMR3_OC6CE                     ((uint32_t)0x00008000)            /*!<Output Compare 4 Clear Enable */

#define TIM_AF1_BKINE_Pos          (0U)
#define TIM_AF1_BKINE_Msk          (0x1U << TIM_AF1_BKINE_Pos)
#define TIM_AF1_BKINE              TIM_AF1_BKINE_Msk

#define TIM_AF1_BKCMP1E_Pos          (0U)
#define TIM_AF1_BKCMP1E_Msk          (0x1U << TIM_AF1_BKINE_Pos)
#define TIM_AF1_BKINE              TIM_AF1_BKINE_Msk

/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (UART)       */
/*                                                                            */
/******************************************************************************/

/*
* @brief Specific device feature definitions.
*/

/* Support of LIN feature */
#define UART_LIN_SUPPORT

/* Support of Smartcard feature */
#define UART_SMARTCARD_SUPPORT

/* Support of Irda feature */
#define UART_IRDA_SUPPORT

/* Support of Wake Up from Stop Mode feature */
#define UART_WUSM_SUPPORT

/******************  Bit definition for UART_CR1 register  *******************/
#define UART_CR1_UE_Pos              (0U)                                     
#define UART_CR1_UE_Msk              (0x1U << UART_CR1_UE_Pos)               /*!< 0x00000001 */
#define UART_CR1_UE                  UART_CR1_UE_Msk                         /*!< UART Enable */
//#define UART_CR1_UESM_Pos            (1U)                                     
//#define UART_CR1_UESM_Msk            (0x1U << UART_CR1_UESM_Pos)             /*!< 0x00000002 */
//#define UART_CR1_UESM                UART_CR1_UESM_Msk                       /*!< UART Enable in STOP Mode */
#define UART_CR1_RE_Pos              (2U)                                     
#define UART_CR1_RE_Msk              (0x1U << UART_CR1_RE_Pos)               /*!< 0x00000004 */
#define UART_CR1_RE                  UART_CR1_RE_Msk                         /*!< Receiver Enable */
#define UART_CR1_TE_Pos              (3U)                                     
#define UART_CR1_TE_Msk              (0x1U << UART_CR1_TE_Pos)               /*!< 0x00000008 */
#define UART_CR1_TE                  UART_CR1_TE_Msk                         /*!< Transmitter Enable */
#define UART_CR1_IDLEIE_Pos          (4U)                                     
#define UART_CR1_IDLEIE_Msk          (0x1U << UART_CR1_IDLEIE_Pos)           /*!< 0x00000010 */
#define UART_CR1_IDLEIE              UART_CR1_IDLEIE_Msk                     /*!< IDLE Interrupt Enable */
#define UART_CR1_RXNEIE_Pos          (5U)                                     
#define UART_CR1_RXNEIE_Msk          (0x1U << UART_CR1_RXNEIE_Pos)           /*!< 0x00000020 */
#define UART_CR1_RXNEIE              UART_CR1_RXNEIE_Msk                     /*!< RXNE Interrupt Enable */
#define UART_CR1_TCIE_Pos            (6U)                                     
#define UART_CR1_TCIE_Msk            (0x1U << UART_CR1_TCIE_Pos)             /*!< 0x00000040 */
#define UART_CR1_TCIE                UART_CR1_TCIE_Msk                       /*!< Transmission Complete Interrupt Enable */
#define UART_CR1_TXEIE_Pos           (7U)                                     
#define UART_CR1_TXEIE_Msk           (0x1U << UART_CR1_TXEIE_Pos)            /*!< 0x00000080 */
#define UART_CR1_TXEIE               UART_CR1_TXEIE_Msk                      /*!< TXE Interrupt Enable */
#define UART_CR1_PEIE_Pos            (8U)                                     
#define UART_CR1_PEIE_Msk            (0x1U << UART_CR1_PEIE_Pos)             /*!< 0x00000100 */
#define UART_CR1_PEIE                UART_CR1_PEIE_Msk                       /*!< PE Interrupt Enable */
#define UART_CR1_PS_Pos              (9U)                                     
#define UART_CR1_PS_Msk              (0x1U << UART_CR1_PS_Pos)               /*!< 0x00000200 */
#define UART_CR1_PS                  UART_CR1_PS_Msk                         /*!< Parity Selection */
#define UART_CR1_PCE_Pos             (10U)                                    
#define UART_CR1_PCE_Msk             (0x1U << UART_CR1_PCE_Pos)              /*!< 0x00000400 */
#define UART_CR1_PCE                 UART_CR1_PCE_Msk                        /*!< Parity Control Enable */
#define UART_CR1_WAKE_Pos            (11U)                                    
#define UART_CR1_WAKE_Msk            (0x1U << UART_CR1_WAKE_Pos)             /*!< 0x00000800 */
#define UART_CR1_WAKE                UART_CR1_WAKE_Msk                       /*!< Receiver Wakeup method */
#define UART_CR1_M_Pos               (12U)   
#define UART_CR1_M_Msk               (0x10001U << UART_CR1_M_Pos)            /*!< 0x10001000 */
#define UART_CR1_M                   UART_CR1_M_Msk                          /*!< Word length */
#define UART_CR1_M0_Pos              (12U)   
#define UART_CR1_M0_Msk              (0x1U << UART_CR1_M0_Pos)               /*!< 0x00001000 */
#define UART_CR1_M0                  UART_CR1_M0_Msk                         /*!< Word length - Bit 0 */
#define UART_CR1_MME_Pos             (13U)                                    
#define UART_CR1_MME_Msk             (0x1U << UART_CR1_MME_Pos)              /*!< 0x00002000 */
#define UART_CR1_MME                 UART_CR1_MME_Msk                        /*!< Mute Mode Enable */
#define UART_CR1_CMIE_Pos            (14U)                                    
#define UART_CR1_CMIE_Msk            (0x1U << UART_CR1_CMIE_Pos)             /*!< 0x00004000 */
#define UART_CR1_CMIE                UART_CR1_CMIE_Msk                       /*!< Character match interrupt enable */
//#define UART_CR1_OVER8_Pos           (15U)                                    
//#define UART_CR1_OVER8_Msk           (0x1U << UART_CR1_OVER8_Pos)            /*!< 0x00008000 */
//#define UART_CR1_OVER8               UART_CR1_OVER8_Msk                      /*!< Oversampling by 8-bit or 16-bit mode */
#define UART_CR1_DEDT_Pos            (16U)                                    
#define UART_CR1_DEDT_Msk            (0x1FU << UART_CR1_DEDT_Pos)            /*!< 0x001F0000 */
#define UART_CR1_DEDT                UART_CR1_DEDT_Msk                       /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define UART_CR1_DEDT_0              (0x01U << UART_CR1_DEDT_Pos)            /*!< 0x00010000 */
#define UART_CR1_DEDT_1              (0x02U << UART_CR1_DEDT_Pos)            /*!< 0x00020000 */
#define UART_CR1_DEDT_2              (0x04U << UART_CR1_DEDT_Pos)            /*!< 0x00040000 */
#define UART_CR1_DEDT_3              (0x08U << UART_CR1_DEDT_Pos)            /*!< 0x00080000 */
#define UART_CR1_DEDT_4              (0x10U << UART_CR1_DEDT_Pos)            /*!< 0x00100000 */
#define UART_CR1_DEAT_Pos            (21U)                                    
#define UART_CR1_DEAT_Msk            (0x1FU << UART_CR1_DEAT_Pos)            /*!< 0x03E00000 */
#define UART_CR1_DEAT                UART_CR1_DEAT_Msk                       /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define UART_CR1_DEAT_0              (0x01U << UART_CR1_DEAT_Pos)            /*!< 0x00200000 */
#define UART_CR1_DEAT_1              (0x02U << UART_CR1_DEAT_Pos)            /*!< 0x00400000 */
#define UART_CR1_DEAT_2              (0x04U << UART_CR1_DEAT_Pos)            /*!< 0x00800000 */
#define UART_CR1_DEAT_3              (0x08U << UART_CR1_DEAT_Pos)            /*!< 0x01000000 */
#define UART_CR1_DEAT_4              (0x10U << UART_CR1_DEAT_Pos)            /*!< 0x02000000 */
//#define UART_CR1_RTOIE_Pos           (26U)                                    
//#define UART_CR1_RTOIE_Msk           (0x1U << UART_CR1_RTOIE_Pos)            /*!< 0x04000000 */
//#define UART_CR1_RTOIE               UART_CR1_RTOIE_Msk                      /*!< Receive Time Out interrupt enable */
//#define UART_CR1_EOBIE_Pos           (27U)                                    
//#define UART_CR1_EOBIE_Msk           (0x1U << UART_CR1_EOBIE_Pos)            /*!< 0x08000000 */
//#define UART_CR1_EOBIE               UART_CR1_EOBIE_Msk                      /*!< End of Block interrupt enable */
#define UART_CR1_M1_Pos              (28U)   
#define UART_CR1_M1_Msk              (0x1UL << UART_CR1_M1_Pos)               /*!< 0x10000000 */
#define UART_CR1_M1                  UART_CR1_M1_Msk                         /*!< Word length - Bit 1 */

/******************  Bit definition for UART_CR2 register  *******************/
#define UART_CR2_ADDM7_Pos           (4U)                                     
#define UART_CR2_ADDM7_Msk           (0x1U << UART_CR2_ADDM7_Pos)            /*!< 0x00000010 */
#define UART_CR2_ADDM7               UART_CR2_ADDM7_Msk                      /*!< 7-bit or 4-bit Address Detection */
//#define UART_CR2_LBDL_Pos            (5U)                                     
//#define UART_CR2_LBDL_Msk            (0x1U << UART_CR2_LBDL_Pos)             /*!< 0x00000020 */
//#define UART_CR2_LBDL                UART_CR2_LBDL_Msk                       /*!< LIN Break Detection Length */
//#define UART_CR2_LBDIE_Pos           (6U)                                     
//#define UART_CR2_LBDIE_Msk           (0x1U << UART_CR2_LBDIE_Pos)            /*!< 0x00000040 */
//#define UART_CR2_LBDIE               UART_CR2_LBDIE_Msk                      /*!< LIN Break Detection Interrupt Enable */
//#define UART_CR2_LBCL_Pos            (8U)                                     
//#define UART_CR2_LBCL_Msk            (0x1U << UART_CR2_LBCL_Pos)             /*!< 0x00000100 */
//#define UART_CR2_LBCL                UART_CR2_LBCL_Msk                       /*!< Last Bit Clock pulse */
//#define UART_CR2_CPHA_Pos            (9U)                                     
//#define UART_CR2_CPHA_Msk            (0x1U << UART_CR2_CPHA_Pos)             /*!< 0x00000200 */
//#define UART_CR2_CPHA                UART_CR2_CPHA_Msk                       /*!< Clock Phase */
//#define UART_CR2_CPOL_Pos            (10U)                                    
//#define UART_CR2_CPOL_Msk            (0x1U << UART_CR2_CPOL_Pos)             /*!< 0x00000400 */
//#define UART_CR2_CPOL                UART_CR2_CPOL_Msk                       /*!< Clock Polarity */
//#define UART_CR2_CLKEN_Pos           (11U)                                    
//#define UART_CR2_CLKEN_Msk           (0x1U << UART_CR2_CLKEN_Pos)            /*!< 0x00000800 */
//#define UART_CR2_CLKEN               UART_CR2_CLKEN_Msk                      /*!< Clock Enable */
#define UART_CR2_STOP_Pos            (12U)                                    
#define UART_CR2_STOP_Msk            (0x3U << UART_CR2_STOP_Pos)             /*!< 0x00003000 */
#define UART_CR2_STOP                UART_CR2_STOP_Msk                       /*!< STOP[1:0] bits (STOP bits) */
#define UART_CR2_STOP_0              (0x0U << UART_CR2_STOP_Pos)             /*!< 0x00000000 */
#define UART_CR2_STOP_1              (0x1U << UART_CR2_STOP_Pos)             /*!< 0x00001000 */
#define UART_CR2_STOP_2              (0x2U << UART_CR2_STOP_Pos)             /*!< 0x00002000 */
//#define UART_CR2_LINEN_Pos           (14U)                                    
//#define UART_CR2_LINEN_Msk           (0x1U << UART_CR2_LINEN_Pos)            /*!< 0x00004000 */
//#define UART_CR2_LINEN               UART_CR2_LINEN_Msk                      /*!< LIN mode enable */
#define UART_CR2_SWAP_Pos            (15U)                                    
#define UART_CR2_SWAP_Msk            (0x1U << UART_CR2_SWAP_Pos)             /*!< 0x00008000 */
#define UART_CR2_SWAP                UART_CR2_SWAP_Msk                       /*!< SWAP TX/RX pins */
#define UART_CR2_RXINV_Pos           (16U)                                    
#define UART_CR2_RXINV_Msk           (0x1U << UART_CR2_RXINV_Pos)            /*!< 0x00010000 */
#define UART_CR2_RXINV               UART_CR2_RXINV_Msk                      /*!< RX pin active level inversion */
#define UART_CR2_TXINV_Pos           (17U)                                    
#define UART_CR2_TXINV_Msk           (0x1U << UART_CR2_TXINV_Pos)            /*!< 0x00020000 */
#define UART_CR2_TXINV               UART_CR2_TXINV_Msk                      /*!< TX pin active level inversion */
#define UART_CR2_DATAINV_Pos         (18U)                                    
#define UART_CR2_DATAINV_Msk         (0x1U << UART_CR2_DATAINV_Pos)          /*!< 0x00040000 */
#define UART_CR2_DATAINV             UART_CR2_DATAINV_Msk                    /*!< Binary data inversion */
#define UART_CR2_MSBFIRST_Pos        (19U)                                    
#define UART_CR2_MSBFIRST_Msk        (0x1U << UART_CR2_MSBFIRST_Pos)         /*!< 0x00080000 */
#define UART_CR2_MSBFIRST            UART_CR2_MSBFIRST_Msk                   /*!< Most Significant Bit First */
//#define UART_CR2_ABREN_Pos           (20U)                                    
//#define UART_CR2_ABREN_Msk           (0x1U << UART_CR2_ABREN_Pos)            /*!< 0x00100000 */
//#define UART_CR2_ABREN               UART_CR2_ABREN_Msk                      /*!< Auto Baud-Rate Enable*/
//#define UART_CR2_ABRMODE_Pos         (21U)                                    
//#define UART_CR2_ABRMODE_Msk         (0x3U << UART_CR2_ABRMODE_Pos)          /*!< 0x00600000 */
//#define UART_CR2_ABRMODE             UART_CR2_ABRMODE_Msk                    /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
//#define UART_CR2_ABRMODE_0           (0x1U << UART_CR2_ABRMODE_Pos)          /*!< 0x00200000 */
//#define UART_CR2_ABRMODE_1           (0x2U << UART_CR2_ABRMODE_Pos)          /*!< 0x00400000 */
//#define UART_CR2_RTOEN_Pos           (23U)                                    
//#define UART_CR2_RTOEN_Msk           (0x1U << UART_CR2_RTOEN_Pos)            /*!< 0x00800000 */
//#define UART_CR2_RTOEN               UART_CR2_RTOEN_Msk                      /*!< Receiver Time-Out enable */
#define UART_CR2_ADD_Pos             (24U)                                    
#define UART_CR2_ADD_Msk             (0xFFU << UART_CR2_ADD_Pos)             /*!< 0xFF000000 */
#define UART_CR2_ADD                 UART_CR2_ADD_Msk                        /*!< Address of the UART node */

/******************  Bit definition for UART_CR3 register  *******************/
#define UART_CR3_EIE_Pos             (0U)                                     
#define UART_CR3_EIE_Msk             (0x1U << UART_CR3_EIE_Pos)              /*!< 0x00000001 */
#define UART_CR3_EIE                 UART_CR3_EIE_Msk                        /*!< Error Interrupt Enable */
//#define UART_CR3_IREN_Pos            (1U)                                     
//#define UART_CR3_IREN_Msk            (0x1U << UART_CR3_IREN_Pos)             /*!< 0x00000002 */
//#define UART_CR3_IREN                UART_CR3_IREN_Msk                       /*!< IrDA mode Enable */
//#define UART_CR3_IRLP_Pos            (2U)                                     
//#define UART_CR3_IRLP_Msk            (0x1U << UART_CR3_IRLP_Pos)             /*!< 0x00000004 */
//#define UART_CR3_IRLP                UART_CR3_IRLP_Msk                       /*!< IrDA Low-Power */
#define UART_CR3_HDSEL_Pos           (3U)                                     
#define UART_CR3_HDSEL_Msk           (0x1U << UART_CR3_HDSEL_Pos)            /*!< 0x00000008 */
#define UART_CR3_HDSEL               UART_CR3_HDSEL_Msk                      /*!< Half-Duplex Selection */
//#define UART_CR3_NACK_Pos            (4U)                                     
//#define UART_CR3_NACK_Msk            (0x1U << UART_CR3_NACK_Pos)             /*!< 0x00000010 */
//#define UART_CR3_NACK                UART_CR3_NACK_Msk                       /*!< SmartCard NACK enable */
//#define UART_CR3_SCEN_Pos            (5U)                                     
//#define UART_CR3_SCEN_Msk            (0x1U << UART_CR3_SCEN_Pos)             /*!< 0x00000020 */
//#define UART_CR3_SCEN                UART_CR3_SCEN_Msk                       /*!< SmartCard mode enable */
#define UART_CR3_DMAR_Pos            (6U)                                     
#define UART_CR3_DMAR_Msk            (0x1U << UART_CR3_DMAR_Pos)             /*!< 0x00000040 */
#define UART_CR3_DMAR                UART_CR3_DMAR_Msk                       /*!< DMA Enable Receiver */
#define UART_CR3_DMAT_Pos            (7U)                                     
#define UART_CR3_DMAT_Msk            (0x1U << UART_CR3_DMAT_Pos)             /*!< 0x00000080 */
#define UART_CR3_DMAT                UART_CR3_DMAT_Msk                       /*!< DMA Enable Transmitter */
#define UART_CR3_RTSE_Pos            (8U)                                     
#define UART_CR3_RTSE_Msk            (0x1U << UART_CR3_RTSE_Pos)             /*!< 0x00000100 */
#define UART_CR3_RTSE                UART_CR3_RTSE_Msk                       /*!< RTS Enable */
#define UART_CR3_CTSE_Pos            (9U)                                     
#define UART_CR3_CTSE_Msk            (0x1U << UART_CR3_CTSE_Pos)             /*!< 0x00000200 */
#define UART_CR3_CTSE                UART_CR3_CTSE_Msk                       /*!< CTS Enable */
#define UART_CR3_CTSIE_Pos           (10U)                                    
#define UART_CR3_CTSIE_Msk           (0x1U << UART_CR3_CTSIE_Pos)            /*!< 0x00000400 */
#define UART_CR3_CTSIE               UART_CR3_CTSIE_Msk                      /*!< CTS Interrupt Enable */
//#define UART_CR3_ONEBIT_Pos          (11U)                                    
//#define UART_CR3_ONEBIT_Msk          (0x1U << UART_CR3_ONEBIT_Pos)           /*!< 0x00000800 */
//#define UART_CR3_ONEBIT              UART_CR3_ONEBIT_Msk                     /*!< One sample bit method enable */
#define UART_CR3_OVRDIS_Pos          (12U)                                    
#define UART_CR3_OVRDIS_Msk          (0x1U << UART_CR3_OVRDIS_Pos)           /*!< 0x00001000 */
#define UART_CR3_OVRDIS              UART_CR3_OVRDIS_Msk                     /*!< Overrun Disable */
//#define UART_CR3_DDRE_Pos            (13U)                                    
//#define UART_CR3_DDRE_Msk            (0x1U << UART_CR3_DDRE_Pos)             /*!< 0x00002000 */
//#define UART_CR3_DDRE                UART_CR3_DDRE_Msk                       /*!< DMA Disable on Reception Error */
#define UART_CR3_DEM_Pos             (14U)                                    
#define UART_CR3_DEM_Msk             (0x1U << UART_CR3_DEM_Pos)              /*!< 0x00004000 */
#define UART_CR3_DEM                 UART_CR3_DEM_Msk                        /*!< Driver Enable Mode */
#define UART_CR3_DEP_Pos             (15U)                                    
#define UART_CR3_DEP_Msk             (0x1U << UART_CR3_DEP_Pos)              /*!< 0x00008000 */
#define UART_CR3_DEP                 UART_CR3_DEP_Msk                        /*!< Driver Enable Polarity Selection */
//#define UART_CR3_SCARCNT_Pos         (17U)                                    
//#define UART_CR3_SCARCNT_Msk         (0x7U << UART_CR3_SCARCNT_Pos)          /*!< 0x000E0000 */
//#define UART_CR3_SCARCNT             UART_CR3_SCARCNT_Msk                    /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
//#define UART_CR3_SCARCNT_0           (0x1U << UART_CR3_SCARCNT_Pos)          /*!< 0x00020000 */
//#define UART_CR3_SCARCNT_1           (0x2U << UART_CR3_SCARCNT_Pos)          /*!< 0x00040000 */
//#define UART_CR3_SCARCNT_2           (0x4U << UART_CR3_SCARCNT_Pos)          /*!< 0x00080000 */
//#define UART_CR3_WUS_Pos             (20U)                                    
//#define UART_CR3_WUS_Msk             (0x3U << UART_CR3_WUS_Pos)              /*!< 0x00300000 */
//#define UART_CR3_WUS                 UART_CR3_WUS_Msk                        /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
//#define UART_CR3_WUS_0               (0x1U << UART_CR3_WUS_Pos)              /*!< 0x00100000 */
//#define UART_CR3_WUS_1               (0x2U << UART_CR3_WUS_Pos)              /*!< 0x00200000 */
//#define UART_CR3_WUFIE_Pos           (22U)                                    
//#define UART_CR3_WUFIE_Msk           (0x1U << UART_CR3_WUFIE_Pos)            /*!< 0x00400000 */
//#define UART_CR3_WUFIE               UART_CR3_WUFIE_Msk                      /*!< Wake Up Interrupt Enable */

/******************  Bit definition for UART_BRR register  *******************/
//#define UART_BRR_DIV_FRACTION_Pos    (0U)                                     
//#define UART_BRR_DIV_FRACTION_Msk    (0xFU << UART_BRR_DIV_FRACTION_Pos)     /*!< 0x0000000F */
//#define UART_BRR_DIV_FRACTION        UART_BRR_DIV_FRACTION_Msk               /*!< Fraction of UARTDIV */
//#define UART_BRR_DIV_MANTISSA_Pos    (4U)                                     
//#define UART_BRR_DIV_MANTISSA_Msk    (0xFFFU << UART_BRR_DIV_MANTISSA_Pos)   /*!< 0x0000FFF0 */
//#define UART_BRR_DIV_MANTISSA        UART_BRR_DIV_MANTISSA_Msk               /*!< Mantissa of UARTDIV */

#define UART_BRR_BRR_Pos    						(0U)                                     
#define UART_BRR_BRR_Msk    						(0xFFFFFU << UART_BRR_BRR_Pos)     			/*!< 0x000FFFFF */
#define UART_BRR_BRR        						UART_BRR_BRR_Msk               					/*!< Fraction of UARTDIV */


///******************  Bit definition for UART_GTPR register  ******************/
//#define UART_GTPR_PSC_Pos            (0U)                                     
//#define UART_GTPR_PSC_Msk            (0xFFU << UART_GTPR_PSC_Pos)            /*!< 0x000000FF */
//#define UART_GTPR_PSC                UART_GTPR_PSC_Msk                       /*!< PSC[7:0] bits (Prescaler value) */
//#define UART_GTPR_GT_Pos             (8U)                                     
//#define UART_GTPR_GT_Msk             (0xFFU << UART_GTPR_GT_Pos)             /*!< 0x0000FF00 */
//#define UART_GTPR_GT                 UART_GTPR_GT_Msk                        /*!< GT[7:0] bits (Guard time value) */
//
//
///*******************  Bit definition for UART_RTOR register  *****************/
//#define UART_RTOR_RTO_Pos            (0U)                                     
//#define UART_RTOR_RTO_Msk            (0xFFFFFFU << UART_RTOR_RTO_Pos)        /*!< 0x00FFFFFF */
//#define UART_RTOR_RTO                UART_RTOR_RTO_Msk                       /*!< Receiver Time Out Value */
//#define UART_RTOR_BLEN_Pos           (24U)                                    
//#define UART_RTOR_BLEN_Msk           (0xFFU << UART_RTOR_BLEN_Pos)           /*!< 0xFF000000 */
//#define UART_RTOR_BLEN               UART_RTOR_BLEN_Msk                      /*!< Block Length */

/*******************  Bit definition for UART_RQR register  ******************/
//#define UART_RQR_ABRRQ_Pos           (0U)                                     
//#define UART_RQR_ABRRQ_Msk           (0x1U << UART_RQR_ABRRQ_Pos)            /*!< 0x00000001 */
//#define UART_RQR_ABRRQ               UART_RQR_ABRRQ_Msk                      /*!< Auto-Baud Rate Request */
#define UART_RQR_SBKRQ_Pos           (1U)                                     
#define UART_RQR_SBKRQ_Msk           (0x1U << UART_RQR_SBKRQ_Pos)            /*!< 0x00000002 */
#define UART_RQR_SBKRQ               UART_RQR_SBKRQ_Msk                      /*!< Send Break Request */
#define UART_RQR_MMRQ_Pos            (2U)                                     
#define UART_RQR_MMRQ_Msk            (0x1U << UART_RQR_MMRQ_Pos)             /*!< 0x00000004 */
#define UART_RQR_MMRQ                UART_RQR_MMRQ_Msk                       /*!< Mute Mode Request */
#define UART_RQR_RXFRQ_Pos           (3U)                                     
#define UART_RQR_RXFRQ_Msk           (0x1U << UART_RQR_RXFRQ_Pos)            /*!< 0x00000008 */
#define UART_RQR_RXFRQ               UART_RQR_RXFRQ_Msk                      /*!< Receive Data flush Request */
//#define UART_RQR_TXFRQ_Pos           (4U)                                     
//#define UART_RQR_TXFRQ_Msk           (0x1U << UART_RQR_TXFRQ_Pos)            /*!< 0x00000010 */
//#define UART_RQR_TXFRQ               UART_RQR_TXFRQ_Msk                      /*!< Transmit data flush Request */

/*******************  Bit definition for UART_ISR register  ******************/
#define UART_ISR_PE_Pos              (0U)                                     
#define UART_ISR_PE_Msk              (0x1U << UART_ISR_PE_Pos)               /*!< 0x00000001 */
#define UART_ISR_PE                  UART_ISR_PE_Msk                         /*!< Parity Error */
#define UART_ISR_FE_Pos              (1U)                                     
#define UART_ISR_FE_Msk              (0x1U << UART_ISR_FE_Pos)               /*!< 0x00000002 */
#define UART_ISR_FE                  UART_ISR_FE_Msk                         /*!< Framing Error */
#define UART_ISR_NF_Pos              (2U)                                     
#define UART_ISR_NF_Msk              (0x1U << UART_ISR_NF_Pos)               /*!< 0x00000004 */
#define UART_ISR_NF                  UART_ISR_NF_Msk                         /*!< Noise detected Flag */
#define UART_ISR_ORE_Pos             (3U)                                     
#define UART_ISR_ORE_Msk             (0x1U << UART_ISR_ORE_Pos)              /*!< 0x00000008 */
#define UART_ISR_ORE                 UART_ISR_ORE_Msk                        /*!< OverRun Error */
#define UART_ISR_IDLE_Pos            (4U)                                     
#define UART_ISR_IDLE_Msk            (0x1U << UART_ISR_IDLE_Pos)             /*!< 0x00000010 */
#define UART_ISR_IDLE                UART_ISR_IDLE_Msk                       /*!< IDLE line detected */
#define UART_ISR_RXNE_Pos            (5U)                                     
#define UART_ISR_RXNE_Msk            (0x1U << UART_ISR_RXNE_Pos)             /*!< 0x00000020 */
#define UART_ISR_RXNE                UART_ISR_RXNE_Msk                       /*!< Read Data Register Not Empty */
#define UART_ISR_TC_Pos              (6U)                                     
#define UART_ISR_TC_Msk              (0x1U << UART_ISR_TC_Pos)               /*!< 0x00000040 */
#define UART_ISR_TC                  UART_ISR_TC_Msk                         /*!< Transmission Complete */
#define UART_ISR_TXE_Pos             (7U)                                     
#define UART_ISR_TXE_Msk             (0x1U << UART_ISR_TXE_Pos)              /*!< 0x00000080 */
#define UART_ISR_TXE                 UART_ISR_TXE_Msk                        /*!< Transmit Data Register Empty */
//#define UART_ISR_LBDF_Pos            (8U)                                     
//#define UART_ISR_LBDF_Msk            (0x1U << UART_ISR_LBDF_Pos)             /*!< 0x00000100 */
//#define UART_ISR_LBDF                UART_ISR_LBDF_Msk                       /*!< LIN Break Detection Flag */
#define UART_ISR_CTSIF_Pos           (9U)                                     
#define UART_ISR_CTSIF_Msk           (0x1U << UART_ISR_CTSIF_Pos)            /*!< 0x00000200 */
#define UART_ISR_CTSIF               UART_ISR_CTSIF_Msk                      /*!< CTS interrupt flag */
#define UART_ISR_CTS_Pos             (10U)                                    
#define UART_ISR_CTS_Msk             (0x1U << UART_ISR_CTS_Pos)              /*!< 0x00000400 */
#define UART_ISR_CTS                 UART_ISR_CTS_Msk                        /*!< CTS flag */
//#define UART_ISR_RTOF_Pos            (11U)                                    
//#define UART_ISR_RTOF_Msk            (0x1U << UART_ISR_RTOF_Pos)             /*!< 0x00000800 */
//#define UART_ISR_RTOF                UART_ISR_RTOF_Msk                       /*!< Receiver Time Out */
//#define UART_ISR_EOBF_Pos            (12U)                                    
//#define UART_ISR_EOBF_Msk            (0x1U << UART_ISR_EOBF_Pos)             /*!< 0x00001000 */
//#define UART_ISR_EOBF                UART_ISR_EOBF_Msk                       /*!< End Of Block Flag */
//#define UART_ISR_ABRE_Pos            (14U)                                    
//#define UART_ISR_ABRE_Msk            (0x1U << UART_ISR_ABRE_Pos)             /*!< 0x00004000 */
//#define UART_ISR_ABRE                UART_ISR_ABRE_Msk                       /*!< Auto-Baud Rate Error */
//#define UART_ISR_ABRF_Pos            (15U)                                    
//#define UART_ISR_ABRF_Msk            (0x1U << UART_ISR_ABRF_Pos)             /*!< 0x00008000 */
//#define UART_ISR_ABRF                UART_ISR_ABRF_Msk                       /*!< Auto-Baud Rate Flag */
#define UART_ISR_BUSY_Pos            (16U)                                    
#define UART_ISR_BUSY_Msk            (0x1U << UART_ISR_BUSY_Pos)             /*!< 0x00010000 */
#define UART_ISR_BUSY                UART_ISR_BUSY_Msk                       /*!< Busy Flag */
#define UART_ISR_CMF_Pos             (17U)                                    
#define UART_ISR_CMF_Msk             (0x1U << UART_ISR_CMF_Pos)              /*!< 0x00020000 */
#define UART_ISR_CMF                 UART_ISR_CMF_Msk                        /*!< Character Match Flag */
#define UART_ISR_SBKF_Pos            (18U)                                    
#define UART_ISR_SBKF_Msk            (0x1U << UART_ISR_SBKF_Pos)             /*!< 0x00040000 */
#define UART_ISR_SBKF                UART_ISR_SBKF_Msk                       /*!< Send Break Flag */
#define UART_ISR_RWU_Pos             (19U)                                    
#define UART_ISR_RWU_Msk             (0x1U << UART_ISR_RWU_Pos)              /*!< 0x00080000 */
#define UART_ISR_RWU                 UART_ISR_RWU_Msk                        /*!< Receive Wake Up from mute mode Flag */
//#define UART_ISR_WUF_Pos             (20U)                                    
//#define UART_ISR_WUF_Msk             (0x1U << UART_ISR_WUF_Pos)              /*!< 0x00100000 */
//#define UART_ISR_WUF                 UART_ISR_WUF_Msk                        /*!< Wake Up from stop mode Flag */
#define UART_ISR_TEACK_Pos           (21U)                                    
#define UART_ISR_TEACK_Msk           (0x1U << UART_ISR_TEACK_Pos)            /*!< 0x00200000 */
#define UART_ISR_TEACK               UART_ISR_TEACK_Msk                      /*!< Transmit Enable Acknowledge Flag */
#define UART_ISR_REACK_Pos           (22U)                                    
#define UART_ISR_REACK_Msk           (0x1U << UART_ISR_REACK_Pos)            /*!< 0x00400000 */
#define UART_ISR_REACK               UART_ISR_REACK_Msk                      /*!< Receive Enable Acknowledge Flag */

/*******************  Bit definition for UART_ICR register  ******************/
#define UART_ICR_PECF_Pos            (0U)                                     
#define UART_ICR_PECF_Msk            (0x1U << UART_ICR_PECF_Pos)             /*!< 0x00000001 */
#define UART_ICR_PECF                UART_ICR_PECF_Msk                       /*!< Parity Error Clear Flag */
#define UART_ICR_FECF_Pos            (1U)                                     
#define UART_ICR_FECF_Msk            (0x1U << UART_ICR_FECF_Pos)             /*!< 0x00000002 */
#define UART_ICR_FECF                UART_ICR_FECF_Msk                       /*!< Framing Error Clear Flag */
#define UART_ICR_NCF_Pos             (2U)                                     
#define UART_ICR_NCF_Msk             (0x1U << UART_ICR_NCF_Pos)              /*!< 0x00000004 */
#define UART_ICR_NCF                 UART_ICR_NCF_Msk                        /*!< Noise detected Clear Flag */
#define UART_ICR_ORECF_Pos           (3U)                                     
#define UART_ICR_ORECF_Msk           (0x1U << UART_ICR_ORECF_Pos)            /*!< 0x00000008 */
#define UART_ICR_ORECF               UART_ICR_ORECF_Msk                      /*!< OverRun Error Clear Flag */
#define UART_ICR_IDLECF_Pos          (4U)                                     
#define UART_ICR_IDLECF_Msk          (0x1U << UART_ICR_IDLECF_Pos)           /*!< 0x00000010 */
#define UART_ICR_IDLECF              UART_ICR_IDLECF_Msk                     /*!< IDLE line detected Clear Flag */
#define UART_ICR_TCCF_Pos            (6U)                                     
#define UART_ICR_TCCF_Msk            (0x1U << UART_ICR_TCCF_Pos)             /*!< 0x00000040 */
#define UART_ICR_TCCF                UART_ICR_TCCF_Msk                       /*!< Transmission Complete Clear Flag */
//#define UART_ICR_LBDCF_Pos           (8U)                                     
//#define UART_ICR_LBDCF_Msk           (0x1U << UART_ICR_LBDCF_Pos)            /*!< 0x00000100 */
//#define UART_ICR_LBDCF               UART_ICR_LBDCF_Msk                      /*!< LIN Break Detection Clear Flag */
#define UART_ICR_CTSCF_Pos           (9U)                                     
#define UART_ICR_CTSCF_Msk           (0x1U << UART_ICR_CTSCF_Pos)            /*!< 0x00000200 */
#define UART_ICR_CTSCF               UART_ICR_CTSCF_Msk                      /*!< CTS Interrupt Clear Flag */
//#define UART_ICR_RTOCF_Pos           (11U)                                    
//#define UART_ICR_RTOCF_Msk           (0x1U << UART_ICR_RTOCF_Pos)            /*!< 0x00000800 */
//#define UART_ICR_RTOCF               UART_ICR_RTOCF_Msk                      /*!< Receiver Time Out Clear Flag */
//#define UART_ICR_EOBCF_Pos           (12U)                                    
//#define UART_ICR_EOBCF_Msk           (0x1U << UART_ICR_EOBCF_Pos)            /*!< 0x00001000 */
//#define UART_ICR_EOBCF               UART_ICR_EOBCF_Msk                      /*!< End Of Block Clear Flag */
#define UART_ICR_CMCF_Pos            (17U)                                    
#define UART_ICR_CMCF_Msk            (0x1U << UART_ICR_CMCF_Pos)             /*!< 0x00020000 */
#define UART_ICR_CMCF                UART_ICR_CMCF_Msk                       /*!< Character Match Clear Flag */
//#define UART_ICR_WUCF_Pos            (20U)                                    
//#define UART_ICR_WUCF_Msk            (0x1U << UART_ICR_WUCF_Pos)             /*!< 0x00100000 */
//#define UART_ICR_WUCF                UART_ICR_WUCF_Msk                       /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for UART_RDR register  ******************/
#define UART_RDR_RDR                 ((uint16_t)0x01FFU)                      /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for UART_TDR register  ******************/
#define UART_TDR_TDR                 ((uint16_t)0x01FFU)                      /*!< TDR[8:0] bits (Transmit Data value) */




/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (USART)       */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_UE                        ((uint32_t)0x00000001)            /*!< USART Enable */
#define  USART_CR1_UESM                      ((uint32_t)0x00000002)            /*!< USART Enable in STOP Mode */
#define  USART_CR1_RE                        ((uint32_t)0x00000004)            /*!< Receiver Enable */
#define  USART_CR1_TE                        ((uint32_t)0x00000008)            /*!< Transmitter Enable */
#define  USART_CR1_IDLEIE                    ((uint32_t)0x00000010)            /*!< IDLE Interrupt Enable */
#define  USART_CR1_RXNEIE                    ((uint32_t)0x00000020)            /*!< RXNE Interrupt Enable */
#define  USART_CR1_TCIE                      ((uint32_t)0x00000040)            /*!< Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((uint32_t)0x00000080)            /*!< TXE Interrupt Enable */
#define  USART_CR1_PEIE                      ((uint32_t)0x00000100)            /*!< PE Interrupt Enable */
#define  USART_CR1_PS                        ((uint32_t)0x00000200)            /*!< Parity Selection */
#define  USART_CR1_PCE                       ((uint32_t)0x00000400)            /*!< Parity Control Enable */
#define  USART_CR1_WAKE                      ((uint32_t)0x00000800)            /*!< Receiver Wakeup method */
#define  USART_CR1_M0                        ((uint32_t)0x00001000)            /*!< Word length */
#define  USART_CR1_MME                       ((uint32_t)0x00002000)            /*!< Mute Mode Enable */
#define  USART_CR1_CMIE                      ((uint32_t)0x00004000)            /*!< Character match interrupt enable */
#define  USART_CR1_OVER8                     ((uint32_t)0x00008000)            /*!< Oversampling by 8-bit or 16-bit mode */
#define  USART_CR1_DEDT                      ((uint32_t)0x001F0000)            /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define  USART_CR1_DEDT_0                    ((uint32_t)0x00010000)            /*!< Bit 0 */
#define  USART_CR1_DEDT_1                    ((uint32_t)0x00020000)            /*!< Bit 1 */
#define  USART_CR1_DEDT_2                    ((uint32_t)0x00040000)            /*!< Bit 2 */
#define  USART_CR1_DEDT_3                    ((uint32_t)0x00080000)            /*!< Bit 3 */
#define  USART_CR1_DEDT_4                    ((uint32_t)0x00100000)            /*!< Bit 4 */
#define  USART_CR1_DEAT                      ((uint32_t)0x03E00000)            /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define  USART_CR1_DEAT_0                    ((uint32_t)0x00200000)            /*!< Bit 0 */
#define  USART_CR1_DEAT_1                    ((uint32_t)0x00400000)            /*!< Bit 1 */
#define  USART_CR1_DEAT_2                    ((uint32_t)0x00800000)            /*!< Bit 2 */
#define  USART_CR1_DEAT_3                    ((uint32_t)0x01000000)            /*!< Bit 3 */
#define  USART_CR1_DEAT_4                    ((uint32_t)0x02000000)            /*!< Bit 4 */
#define  USART_CR1_RTOIE                     ((uint32_t)0x04000000)            /*!< Receive Time Out interrupt enable */
#define  USART_CR1_EOBIE                     ((uint32_t)0x08000000)            /*!< End of Block interrupt enable */
#define  USART_CR1_M1                        ((uint32_t)0x10000000)            /*!< Word length */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADDM7                     ((uint32_t)0x00000010)            /*!< 7-bit or 4-bit Address Detection */
#define  USART_CR2_LBDL                      ((uint32_t)0x00000020)            /*!< LIN Break Detection Length */
#define  USART_CR2_LBDIE                     ((uint32_t)0x00000040)            /*!< LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((uint32_t)0x00000100)            /*!< Last Bit Clock pulse */
#define  USART_CR2_CPHA                      ((uint32_t)0x00000200)            /*!< Clock Phase */
#define  USART_CR2_CPOL                      ((uint32_t)0x00000400)            /*!< Clock Polarity */
#define  USART_CR2_CLKEN                     ((uint32_t)0x00000800)            /*!< Clock Enable */
#define  USART_CR2_STOP                      ((uint32_t)0x00003000)            /*!< STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((uint32_t)0x00001000)            /*!< Bit 0 */
#define  USART_CR2_STOP_1                    ((uint32_t)0x00002000)            /*!< Bit 1 */
#define  USART_CR2_LINEN                     ((uint32_t)0x00004000)            /*!< LIN mode enable */
#define  USART_CR2_SWAP                      ((uint32_t)0x00008000)            /*!< SWAP TX/RX pins */
#define  USART_CR2_RXINV                     ((uint32_t)0x00010000)            /*!< RX pin active level inversion */
#define  USART_CR2_TXINV                     ((uint32_t)0x00020000)            /*!< TX pin active level inversion */
#define  USART_CR2_DATAINV                   ((uint32_t)0x00040000)            /*!< Binary data inversion */
#define  USART_CR2_MSBFIRST                  ((uint32_t)0x00080000)            /*!< Most Significant Bit First */
#define  USART_CR2_ABREN                     ((uint32_t)0x00100000)            /*!< Auto Baud-Rate Enable*/
#define  USART_CR2_ABRMODE                   ((uint32_t)0x00600000)            /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
#define  USART_CR2_ABRMODE_0                 ((uint32_t)0x00200000)            /*!< Bit 0 */
#define  USART_CR2_ABRMODE_1                 ((uint32_t)0x00400000)            /*!< Bit 1 */
#define  USART_CR2_RTOEN                     ((uint32_t)0x00800000)            /*!< Receiver Time-Out enable */
#define  USART_CR2_ADD                       ((uint32_t)0xFF000000)            /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((uint32_t)0x00000001)            /*!< Error Interrupt Enable */
#define  USART_CR3_IREN                      ((uint32_t)0x00000002)            /*!< IrDA mode Enable */
#define  USART_CR3_IRLP                      ((uint32_t)0x00000004)            /*!< IrDA Low-Power */
#define  USART_CR3_HDSEL                     ((uint32_t)0x00000008)            /*!< Half-Duplex Selection */
#define  USART_CR3_NACK                      ((uint32_t)0x00000010)            /*!< SmartCard NACK enable */
#define  USART_CR3_SCEN                      ((uint32_t)0x00000020)            /*!< SmartCard mode enable */
#define  USART_CR3_DMAR                      ((uint32_t)0x00000040)            /*!< DMA Enable Receiver */
#define  USART_CR3_DMAT                      ((uint32_t)0x00000080)            /*!< DMA Enable Transmitter */
#define  USART_CR3_RTSE                      ((uint32_t)0x00000100)            /*!< RTS Enable */
#define  USART_CR3_CTSE                      ((uint32_t)0x00000200)            /*!< CTS Enable */
#define  USART_CR3_CTSIE                     ((uint32_t)0x00000400)            /*!< CTS Interrupt Enable */
#define  USART_CR3_ONEBIT                    ((uint32_t)0x00000800)            /*!< One sample bit method enable */
#define  USART_CR3_OVRDIS                    ((uint32_t)0x00001000)            /*!< Overrun Disable */
#define  USART_CR3_DDRE                      ((uint32_t)0x00002000)            /*!< DMA Disable on Reception Error */
#define  USART_CR3_DEM                       ((uint32_t)0x00004000)            /*!< Driver Enable Mode */
#define  USART_CR3_DEP                       ((uint32_t)0x00008000)            /*!< Driver Enable Polarity Selection */
#define  USART_CR3_SCARCNT                   ((uint32_t)0x000E0000)            /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
#define  USART_CR3_SCARCNT_0                 ((uint32_t)0x00020000)            /*!< Bit 0 */
#define  USART_CR3_SCARCNT_1                 ((uint32_t)0x00040000)            /*!< Bit 1 */
#define  USART_CR3_SCARCNT_2                 ((uint32_t)0x00080000)            /*!< Bit 2 */
#define  USART_CR3_WUS                       ((uint32_t)0x00300000)            /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
#define  USART_CR3_WUS_0                     ((uint32_t)0x00100000)            /*!< Bit 0 */
#define  USART_CR3_WUS_1                     ((uint32_t)0x00200000)            /*!< Bit 1 */
#define  USART_CR3_WUFIE                     ((uint32_t)0x00400000)            /*!< Wake Up Interrupt Enable */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_FRACTION              ((uint16_t)0x000F)                /*!< Fraction of USARTDIV */
#define  USART_BRR_DIV_MANTISSA              ((uint16_t)0xFFF0)                /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((uint16_t)0x00FF)                /*!< PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_GT                       ((uint16_t)0xFF00)                /*!< GT[7:0] bits (Guard time value) */


/*******************  Bit definition for USART_RTOR register  *****************/
#define  USART_RTOR_RTO                      ((uint32_t)0x00FFFFFF)            /*!< Receiver Time Out Value */
#define  USART_RTOR_BLEN                     ((uint32_t)0xFF000000)            /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
#define  USART_RQR_ABRRQ                    ((uint16_t)0x0001)                /*!< Auto-Baud Rate Request */
#define  USART_RQR_SBKRQ                    ((uint16_t)0x0002)                /*!< Send Break Request */
#define  USART_RQR_MMRQ                     ((uint16_t)0x0004)                /*!< Mute Mode Request */
#define  USART_RQR_RXFRQ                    ((uint16_t)0x0008)                /*!< Receive Data flush Request */
#define  USART_RQR_TXFRQ                    ((uint16_t)0x0010)                /*!< Transmit data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
#define  USART_ISR_PE                        ((uint32_t)0x00000001)            /*!< Parity Error */
#define  USART_ISR_FE                        ((uint32_t)0x00000002)            /*!< Framing Error */
#define  USART_ISR_NE                        ((uint32_t)0x00000004)            /*!< Noise detected Flag */
#define  USART_ISR_ORE                       ((uint32_t)0x00000008)            /*!< OverRun Error */
#define  USART_ISR_IDLE                      ((uint32_t)0x00000010)            /*!< IDLE line detected */
#define  USART_ISR_RXNE                      ((uint32_t)0x00000020)            /*!< Read Data Register Not Empty */
#define  USART_ISR_TC                        ((uint32_t)0x00000040)            /*!< Transmission Complete */
#define  USART_ISR_TXE                       ((uint32_t)0x00000080)            /*!< Transmit Data Register Empty */
#define  USART_ISR_LBD                       ((uint32_t)0x00000100)            /*!< LIN Break Detection Flag */
#define  USART_ISR_CTSIF                     ((uint32_t)0x00000200)            /*!< CTS interrupt flag */
#define  USART_ISR_CTS                       ((uint32_t)0x00000400)            /*!< CTS flag */
#define  USART_ISR_RTOF                      ((uint32_t)0x00000800)            /*!< Receiver Time Out */
#define  USART_ISR_EOBF                      ((uint32_t)0x00001000)            /*!< End Of Block Flag */
#define  USART_ISR_ABRE                      ((uint32_t)0x00004000)            /*!< Auto-Baud Rate Error */
#define  USART_ISR_ABRF                      ((uint32_t)0x00008000)            /*!< Auto-Baud Rate Flag */
#define  USART_ISR_BUSY                      ((uint32_t)0x00010000)            /*!< Busy Flag */
#define  USART_ISR_CMF                       ((uint32_t)0x00020000)            /*!< Character Match Flag */
#define  USART_ISR_SBKF                      ((uint32_t)0x00040000)            /*!< Send Break Flag */
#define  USART_ISR_RWU                       ((uint32_t)0x00080000)            /*!< Receive Wake Up from mute mode Flag */
#define  USART_ISR_WUF                       ((uint32_t)0x00100000)            /*!< Wake Up from stop mode Flag */
#define  USART_ISR_TEACK                     ((uint32_t)0x00200000)            /*!< Transmit Enable Acknowledge Flag */
#define  USART_ISR_REACK                     ((uint32_t)0x00400000)            /*!< Receive Enable Acknowledge Flag */

/*******************  Bit definition for USART_ICR register  ******************/
#define  USART_ICR_PECF                      ((uint32_t)0x00000001)            /*!< Parity Error Clear Flag */
#define  USART_ICR_FECF                      ((uint32_t)0x00000002)            /*!< Framing Error Clear Flag */
#define  USART_ICR_NCF                      ((uint32_t)0x00000004)             /*!< Noise detected Clear Flag */
#define  USART_ICR_ORECF                     ((uint32_t)0x00000008)            /*!< OverRun Error Clear Flag */
#define  USART_ICR_IDLECF                    ((uint32_t)0x00000010)            /*!< IDLE line detected Clear Flag */
#define  USART_ICR_TCCF                      ((uint32_t)0x00000040)            /*!< Transmission Complete Clear Flag */
#define  USART_ICR_LBDCF                     ((uint32_t)0x00000100)            /*!< LIN Break Detection Clear Flag */
#define  USART_ICR_CTSCF                     ((uint32_t)0x00000200)            /*!< CTS Interrupt Clear Flag */
#define  USART_ICR_RTOCF                     ((uint32_t)0x00000800)            /*!< Receiver Time Out Clear Flag */
#define  USART_ICR_EOBCF                     ((uint32_t)0x00001000)            /*!< End Of Block Clear Flag */
#define  USART_ICR_CMCF                      ((uint32_t)0x00020000)            /*!< Character Match Clear Flag */
#define  USART_ICR_WUCF                      ((uint32_t)0x00100000)            /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
#define  USART_RDR_RDR                       ((uint16_t)0x01FF)                /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for USART_TDR register  ******************/
#define  USART_TDR_TDR                       ((uint16_t)0x01FF)                /*!< TDR[8:0] bits (Transmit Data value) */

/******************************************************************************/
/*                                                                            */
/*                         Window WATCHDOG (WWDG)                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for WWDG_CR register  ********************/
#define WWDG_CR_T_Pos           (0U)                                           
#define WWDG_CR_T_Msk           (0x7FU << WWDG_CR_T_Pos)                       /*!< 0x0000007F */
#define WWDG_CR_T               WWDG_CR_T_Msk                                  /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CR_T_0             (0x01U << WWDG_CR_T_Pos)                       /*!< 0x00000001 */
#define WWDG_CR_T_1             (0x02U << WWDG_CR_T_Pos)                       /*!< 0x00000002 */
#define WWDG_CR_T_2             (0x04U << WWDG_CR_T_Pos)                       /*!< 0x00000004 */
#define WWDG_CR_T_3             (0x08U << WWDG_CR_T_Pos)                       /*!< 0x00000008 */
#define WWDG_CR_T_4             (0x10U << WWDG_CR_T_Pos)                       /*!< 0x00000010 */
#define WWDG_CR_T_5             (0x20U << WWDG_CR_T_Pos)                       /*!< 0x00000020 */
#define WWDG_CR_T_6             (0x40U << WWDG_CR_T_Pos)                       /*!< 0x00000040 */

/* Legacy defines */
#define  WWDG_CR_T0 WWDG_CR_T_0
#define  WWDG_CR_T1 WWDG_CR_T_1
#define  WWDG_CR_T2 WWDG_CR_T_2
#define  WWDG_CR_T3 WWDG_CR_T_3
#define  WWDG_CR_T4 WWDG_CR_T_4
#define  WWDG_CR_T5 WWDG_CR_T_5
#define  WWDG_CR_T6 WWDG_CR_T_6

#define WWDG_CR_WDGA_Pos        (7U)                                           
#define WWDG_CR_WDGA_Msk        (0x1U << WWDG_CR_WDGA_Pos)                     /*!< 0x00000080 */
#define WWDG_CR_WDGA            WWDG_CR_WDGA_Msk                               /*!< Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define WWDG_CFR_W_Pos          (0U)                                           
#define WWDG_CFR_W_Msk          (0x7FU << WWDG_CFR_W_Pos)                      /*!< 0x0000007F */
#define WWDG_CFR_W              WWDG_CFR_W_Msk                                 /*!< W[6:0] bits (7-bit window value) */
#define WWDG_CFR_W_0            (0x01U << WWDG_CFR_W_Pos)                      /*!< 0x00000001 */
#define WWDG_CFR_W_1            (0x02U << WWDG_CFR_W_Pos)                      /*!< 0x00000002 */
#define WWDG_CFR_W_2            (0x04U << WWDG_CFR_W_Pos)                      /*!< 0x00000004 */
#define WWDG_CFR_W_3            (0x08U << WWDG_CFR_W_Pos)                      /*!< 0x00000008 */
#define WWDG_CFR_W_4            (0x10U << WWDG_CFR_W_Pos)                      /*!< 0x00000010 */
#define WWDG_CFR_W_5            (0x20U << WWDG_CFR_W_Pos)                      /*!< 0x00000020 */
#define WWDG_CFR_W_6            (0x40U << WWDG_CFR_W_Pos)                      /*!< 0x00000040 */

/* Legacy defines */
#define  WWDG_CFR_W0 WWDG_CFR_W_0
#define  WWDG_CFR_W1 WWDG_CFR_W_1
#define  WWDG_CFR_W2 WWDG_CFR_W_2
#define  WWDG_CFR_W3 WWDG_CFR_W_3
#define  WWDG_CFR_W4 WWDG_CFR_W_4
#define  WWDG_CFR_W5 WWDG_CFR_W_5
#define  WWDG_CFR_W6 WWDG_CFR_W_6

#define WWDG_CFR_WDGTB_Pos      (7U)                                           
#define WWDG_CFR_WDGTB_Msk      (0x3U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000180 */
#define WWDG_CFR_WDGTB          WWDG_CFR_WDGTB_Msk                             /*!< WDGTB[1:0] bits (Timer Base) */
#define WWDG_CFR_WDGTB_0        (0x0U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000000 */
#define WWDG_CFR_WDGTB_1        (0x1U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000080 */
#define WWDG_CFR_WDGTB_2        (0x2U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000100 */
#define WWDG_CFR_WDGTB_3        (0x3U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000180 */


/* Legacy defines */
#define  WWDG_CFR_WDGTB0 WWDG_CFR_WDGTB_0
#define  WWDG_CFR_WDGTB1 WWDG_CFR_WDGTB_1

#define WWDG_CFR_EWI_Pos        (9U)                                           
#define WWDG_CFR_EWI_Msk        (0x1U << WWDG_CFR_EWI_Pos)                     /*!< 0x00000200 */
#define WWDG_CFR_EWI            WWDG_CFR_EWI_Msk                               /*!< Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define WWDG_SR_EWIF_Pos        (0U)                                           
#define WWDG_SR_EWIF_Msk        (0x1U << WWDG_SR_EWIF_Pos)                     /*!< 0x00000001 */
#define WWDG_SR_EWIF            WWDG_SR_EWIF_Msk                               /*!< Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                    PI                                      */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for PI_CR register  ********************/
#define PI_CR_EN_Pos           (0U)
#define PI_CR_EN_Msk           (0x1U << PI_CR_EN_Pos)
#define PI_CR_EN               PI_CR_EN_Msk

#define PI_CR_AUTO_Pos         (1U)
#define PI_CR_AUTO_Msk         (0x1U << PI_CR_AUTO_Pos)
#define PI_CR_AUTO             PI_CR_AUTO_Msk

#define PI_CR_MODU_Pos         (2U)
#define PI_CR_MODU_Msk         (0x3U << PI_CR_MODU_Pos)
#define PI_CR_MODU             PI_CR_MODU_Msk
#define PI_CR_MODU_PI0         (0x0U << PI_CR_MODU_Pos)
#define PI_CR_MODU_PI1         (0x1U << PI_CR_MODU_Pos)
#define PI_CR_MODU_PI2         (0x2U << PI_CR_MODU_Pos)


#define PI_CR_COMP0_Pos           (8U)
#define PI_CR_COMP0_Msk           (0x1U << PI_CR_COMP0_Pos)
#define PI_CR_COMP0               PI_CR_COMP0_Msk

#define PI_CR_COMP1_Pos           (9U)
#define PI_CR_COMP1_Msk           (0x1U << PI_CR_COMP1_Pos)
#define PI_CR_COMP1               PI_CR_COMP1_Msk

#define PI_CR_COMP2_Pos           (10U)
#define PI_CR_COMP2_Msk           (0x1U << PI_CR_COMP2_Pos)
#define PI_CR_COMP2               PI_CR_COMP2_Msk

/*******************  Bit definition for PI_ARG register  ********************/
#define PI_ARG_KI_Pos          (0U)
#define PI_ARG_KI_Msk          (0xFFFFU << PI_ARG_KI_Pos)
#define PI_ARG_KI               PI_ARG_KI_Msk

#define PI_ARG_KP_Pos          (16U)
#define PI_ARG_KP_Msk          (0xFFFFU << PI_ARG_KP_Pos)
#define PI_ARG_KP               PI_ARG_KP_Msk


/******************************************************************************/
/*                                                                            */
/*                                    MATH                                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for MATH_CR register  ********************/
#define MATH_CR_START_Pos           (0U)
#define MATH_CR_START_Msk           (0x1U << MATH_CR_START_Pos)
#define MATH_CR_START               MATH_CR_START_Msk

#define MATH_CR_MODE_Pos           (1U)
#define MATH_CR_MODE_Msk           (0x1U << MATH_CR_MODE_Pos)
#define MATH_CR_MODE               MATH_CR_MODE_Msk

#define MATH_CR_MODE_SQRT          (0x1U << MATH_CR_MODE_Pos)
#define MATH_CR_MODE_DIV           0

#define MATH_CR_ERROR_Pos          (8U)
#define MATH_CR_ERROR_Msk          (0x1U << MATH_CR_ERROR_Pos)
#define MATH_CR_ERROR              MATH_CR_ERROR_Msk

#define MATH_CR_DONE_Pos          (9U)
#define MATH_CR_DONE_Msk          (0x1U << MATH_CR_DONE_Pos)
#define MATH_CR_DONE              MATH_CR_DONE_Msk


/******************************************************************************/
/*                                                                            */
/*                                CORDIC                                      */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CORDIC_CR register  ********************/
#define CORDIC_CSR_FUNC_Pos           (0U)
#define CORDIC_CSR_FUNC_Msk           (0xFU << CORDIC_CSR_FUNC_Pos)
#define CORDIC_CSR_FUNC               CORDIC_CSR_FUNC_Msk

#define CORDIC_CSR_PRECISION_Pos      (4U)
#define CORDIC_CSR_PRECISION_Msk      (0xFU << CORDIC_CSR_PRECISION_Pos)
#define CORDIC_CSR_PRECISION          CORDIC_CSR_PRECISION_Msk

#define CORDIC_CSR_SCALE_Pos          (8U)
#define CORDIC_CSR_SCALE_Msk          (0x7U << CORDIC_CSR_SCALE_Pos)
#define CORDIC_CSR_SCALE              CORDIC_CSR_SCALE_Msk

#define CORDIC_CSR_IEN_Pos            (16U)
#define CORDIC_CSR_IEN_Msk            (0x1U << CORDIC_CSR_IEN_Pos)
#define CORDIC_CSR_IEN                CORDIC_CSR_IEN_Msk

#define CORDIC_CSR_DMAREN_Pos            (17U)
#define CORDIC_CSR_DMAREN_Msk            (0x1U << CORDIC_CSR_DMAREN_Pos)
#define CORDIC_CSR_DMAREN                CORDIC_CSR_DMAREN_Msk

#define CORDIC_CSR_DMAWEN_Pos            (18U)
#define CORDIC_CSR_DMAWEN_Msk            (0x1U << CORDIC_CSR_DMAWEN_Pos)
#define CORDIC_CSR_DMAWEN                CORDIC_CSR_DMAWEN_Msk

#define CORDIC_CSR_NRES_Pos            (19U)
#define CORDIC_CSR_NRES_Msk            (0x1U << CORDIC_CSR_NRES_Pos)
#define CORDIC_CSR_NRES                CORDIC_CSR_NRES_Msk

#define CORDIC_CSR_NARGS_Pos            (20U)
#define CORDIC_CSR_NARGS_Msk            (0x1U << CORDIC_CSR_NARGS_Pos)
#define CORDIC_CSR_NARGS                CORDIC_CSR_NARGS_Msk

#define CORDIC_CSR_RESSIZE_Pos          (21U)
#define CORDIC_CSR_RESSIZE_Msk          (0x1U << CORDIC_CSR_RESSIZE_Pos)
#define CORDIC_CSR_RESSIZE              CORDIC_CSR_RESSIZE_Msk

#define CORDIC_CSR_ARGSIZE_Pos          (22U)
#define CORDIC_CSR_ARGSIZE_Msk          (0x1U << CORDIC_CSR_ARGSIZE_Pos)
#define CORDIC_CSR_ARGSIZE              CORDIC_CSR_ARGSIZE_Msk

#define CORDIC_CSR_RRDY_Pos          (31U)
#define CORDIC_CSR_RRDY_Msk          (0x1U << CORDIC_CSR_RRDY_Pos)
#define CORDIC_CSR_RRDY              CORDIC_CSR_RRDY_Msk


/******************************************************************************/
/*                                                                            */
/*                       CRC calculation unit (CRC)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((uint32_t)0xFFFFFFFF) /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((uint8_t)0xFF)        /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((uint32_t)0x00000001) /*!< RESET the CRC computation unit bit */
#define  CRC_CR_POLSIZE                      ((uint32_t)0x00000018) /*!< Polynomial size bits (only for STM32F072 devices)*/
#define  CRC_CR_POLSIZE_0                    ((uint32_t)0x00000008) /*!< Polynomial size bit 0 (only for STM32F072 devices) */
#define  CRC_CR_POLSIZE_1                    ((uint32_t)0x00000010) /*!< Polynomial size bit 1 (only for STM32F072 devices) */
#define  CRC_CR_REV_IN                       ((uint32_t)0x00000060) /*!< REV_IN Reverse Input Data bits */
#define  CRC_CR_REV_IN_0                     ((uint32_t)0x00000020) /*!< REV_IN Bit 0 */
#define  CRC_CR_REV_IN_1                     ((uint32_t)0x00000040) /*!< REV_IN Bit 1 */
#define  CRC_CR_REV_OUT                      ((uint32_t)0x00000080) /*!< REV_OUT Reverse Output Data bits */

/*******************  Bit definition for CRC_INIT register  *******************/
#define  CRC_INIT_INIT                       ((uint32_t)0xFFFFFFFF) /*!< Initial CRC value bits */

/*******************  Bit definition for CRC_POL register  ********************/
#define  CRC_POL_POL                         ((uint32_t)0xFFFFFFFF) /*!< Coefficients of the polynomial (only for STM32F072 devices) */



/******************************************************************************/
/*                                                                            */
/*                                  ANALOG                                    */
/*                                                                            */
/******************************************************************************/

#define ANALOG_DAC_CFGR_REF_VSEL_Pos          (4U)
#define ANALOG_DAC_CFGR_REF_VSEL_Msk          (0x7U << ANALOG_DAC_CFGR_REF_VSEL_Pos)
#define ANALOG_DAC_CFGR_REF_VSEL              ANALOG_DAC_CFGR_REF_VSEL_Msk

#define ANALOG_DAC_CFGR_BUFF_ISEL_Pos         (2U)
#define ANALOG_DAC_CFGR_BUFF_ISEL_Msk         (0x3U << ANALOG_DAC_CFGR_BUFF_ISEL_Pos)
#define ANALOG_DAC_CFGR_BUFF_ISEL             ANALOG_DAC_CFGR_BUFF_ISEL_Msk

#define ANALOG_DAC_CFGR_BUFF_EN_Pos           (1U)
#define ANALOG_DAC_CFGR_BUFF_EN_Msk           (0x1U << ANALOG_DAC_CFGR_BUFF_EN_Pos)
#define ANALOG_DAC_CFGR_BUFF_EN               ANALOG_DAC_CFGR_BUFF_EN_Msk

#define ANALOG_DAC_CFGR_DACEN_Pos             (0U)
#define ANALOG_DAC_CFGR_DACEN_Msk             (0x1U << ANALOG_DAC_CFGR_DACEN_Pos)
#define ANALOG_DAC_CFGR_DACEN                 ANALOG_DAC_CFGR_DACEN_Msk


#define ANALOG_OPAx_CFGR1_SHORT_OUT_Pos       (2U)
#define ANALOG_OPAx_CFGR1_SHORT_OUT_Msk       (0x1U << ANALOG_OPAx_CFGR1_SHORT_OUT_Pos)
#define ANALOG_OPAx_CFGR1_SHORT_OUT           ANALOG_OPAx_CFGR1_SHORT_OUT_Msk

#define ANALOG_OPAx_CFGR1_SHORT_IN_Pos        (1U)
#define ANALOG_OPAx_CFGR1_SHORT_IN_Msk        (0x1U << ANALOG_OPAx_CFGR1_SHORT_IN_Pos)
#define ANALOG_OPAx_CFGR1_SHORT_IN            ANALOG_OPAx_CFGR1_SHORT_IN_Msk

#define ANALOG_OPAx_CFGR1_OPA_EN_Pos          (0U)
#define ANALOG_OPAx_CFGR1_OPA_EN_Msk          (0x1U << ANALOG_OPAx_CFGR1_OPA_EN_Pos)
#define ANALOG_OPAx_CFGR1_OPA_EN               ANALOG_OPAx_CFGR1_OPA_EN_Msk

#define ANALOG_OPAx_CFGR2_RSVD_Pos            (7U)
#define ANALOG_OPAx_CFGR2_RSVD_Msk            (0x3U << ANALOG_OPAx_CFGR2_RSVD_Pos)
#define ANALOG_OPAx_CFGR2_RSVD                 ANALOG_OPAx_CFGR2_RSVD_Msk

#define ANALOG_OPAx_CFGR2_ISEL_Pos            (5U)
#define ANALOG_OPAx_CFGR2_ISEL_Msk            (0x3U << ANALOG_OPAx_CFGR2_ISEL_Pos)
#define ANALOG_OPAx_CFGR2_ISEL                 ANALOG_OPAx_CFGR2_ISEL_Msk

#define ANALOG_OPAx_CFGR2_CMVSEL_Pos          (3U)
#define ANALOG_OPAx_CFGR2_CMVSEL_Msk          (0x3U << ANALOG_OPAx_CFGR2_CMVSEL_Pos)
#define ANALOG_OPAx_CFGR2_CMVSEL               ANALOG_OPAx_CFGR2_CMVSEL_Msk

#define ANALOG_OPAx_CFGR2_SINGLE_OUT_Pos      (2U)
#define ANALOG_OPAx_CFGR2_SINGLE_OUT_Msk      (0x1U << ANALOG_OPAx_CFGR2_SINGLE_OUT_Pos)
#define ANALOG_OPAx_CFGR2_SINGLE_OUT           ANALOG_OPAx_CFGR2_SINGLE_OUT_Msk

#define ANALOG_OPAx_CFGR2_RESEL_Pos           (0U)
#define ANALOG_OPAx_CFGR2_RESEL_Msk           (0x3U << ANALOG_OPAx_CFGR2_RESEL_Pos)
#define ANALOG_OPAx_CFGR2_RESEL                ANALOG_OPAx_CFGR2_RESEL_Msk



#define ANALOG_OPA_BUF_ISEL_Pos               (3U)
#define ANALOG_OPA_BUF_ISEL_Msk               (0x3U << ANALOG_OPA_BUF_ISEL_Pos)
#define ANALOG_OPA_BUF_ISEL                   ANALOG_OPA_BUF_ISEL_Msk

#define ANALOG_OPA_BUF_SEL_Pos               (1U)
#define ANALOG_OPA_BUF_SEL_Msk               (0x3U << ANALOG_OPA_BUF_SEL_Pos)
#define ANALOG_OPA_BUF_SEL                   ANALOG_OPA_BUF_SEL_Msk

#define ANALOG_OPA_BUF_EN_Pos                (0U)
#define ANALOG_OPA_BUF_EN_Msk                (0x1U << ANALOG_OPA_BUF_EN_Pos)
#define ANALOG_OPA_BUF_EN                    ANALOG_OPA_BUF_EN_Msk



#define ANALOG_CMP_FILT0_Pos                 (0U)
#define ANALOG_CMP_FILT0_Msk                 (0xFFFFU << ANALOG_CMP_FILT0_Pos)
#define ANALOG_CMP_FILT0                     ANALOG_CMP_FILT0_Msk

#define ANALOG_CMP_FILT1_Pos                 (16U)
#define ANALOG_CMP_FILT1_Msk                 (0xFFFFU << ANALOG_CMP_FILT1_Pos)
#define ANALOG_CMP_FILT1                     ANALOG_CMP_FILT1_Msk



#define ANALOG_CMPx_CSR_EN_Pos               (0U)
#define ANALOG_CMPx_CSR_EN_Msk               (0x1U << ANALOG_CMPx_CSR_EN_Pos)
#define ANALOG_CMPx_CSR_EN                   ANALOG_CMPx_CSR_EN_Msk

#define ANALOG_CMPx_CSR_HYST_Pos               (2U)
#define ANALOG_CMPx_CSR_HYST_Msk               (0x3U << ANALOG_CMPx_CSR_HYST_Pos)
#define ANALOG_CMPx_CSR_HYST                   ANALOG_CMPx_CSR_HYST_Msk

#define ANALOG_CMPx_CSR_DLY_Pos               (4U)
#define ANALOG_CMPx_CSR_DLY_Msk               (0x3U << ANALOG_CMPx_CSR_DLY_Pos)
#define ANALOG_CMPx_CSR_DLY                   ANALOG_CMPx_CSR_DLY_Msk

#define ANALOG_CMPx_CSR_NSEL_Pos               (6U)
#define ANALOG_CMPx_CSR_NSEL_Msk               (0x3U << ANALOG_CMPx_CSR_NSEL_Pos)
#define ANALOG_CMPx_CSR_NSEL                   ANALOG_CMPx_CSR_NSEL_Msk

#define ANALOG_CMPx_CSR_PSEL_Pos               (8U)
#define ANALOG_CMPx_CSR_PSEL_Msk               (0x3U << ANALOG_CMPx_CSR_PSEL_Pos)
#define ANALOG_CMPx_CSR_PSEL                   ANALOG_CMPx_CSR_PSEL_Msk

#define ANALOG_CMPx_CSR_POL_Pos                (15U)
#define ANALOG_CMPx_CSR_POL_Msk                (0x1U << ANALOG_CMPx_CSR_POL_Pos)
#define ANALOG_CMPx_CSR_POL                    ANALOG_CMPx_CSR_POL_Msk

#define ANALOG_CMPx_CSR_RIF_Pos                (16U)
#define ANALOG_CMPx_CSR_RIF_Msk                (0x1U << ANALOG_CMPx_CSR_RIF_Pos)
#define ANALOG_CMPx_CSR_RIF                    ANALOG_CMPx_CSR_RIF_Msk

#define ANALOG_CMPx_CSR_FIF_Pos                (17U)
#define ANALOG_CMPx_CSR_FIF_Msk                (0x1U << ANALOG_CMPx_CSR_FIF_Pos)
#define ANALOG_CMPx_CSR_FIF                    ANALOG_CMPx_CSR_FIF_Msk

#define ANALOG_CMPx_CSR_RIE_Pos                (20U)
#define ANALOG_CMPx_CSR_RIE_Msk                (0x1U << ANALOG_CMPx_CSR_RIE_Pos)
#define ANALOG_CMPx_CSR_RIE                    ANALOG_CMPx_CSR_RIE_Msk

#define ANALOG_CMPx_CSR_FIE_Pos                (21U)
#define ANALOG_CMPx_CSR_FIE_Msk                (0x1U << ANALOG_CMPx_CSR_FIE_Pos)
#define ANALOG_CMPx_CSR_FIE                    ANALOG_CMPx_CSR_FIE_Msk

#define ANALOG_CMPx_CSR_BLKSEL_Pos             (24U)
#define ANALOG_CMPx_CSR_BLKSEL_Msk             (0x7U << ANALOG_CMPx_CSR_BLKSEL_Pos)
#define ANALOG_CMPx_CSR_BLKSEL                 ANALOG_CMPx_CSR_BLKSEL_Msk

#define ANALOG_CMPx_CSR_VALUE_Pos              (30U)
#define ANALOG_CMPx_CSR_VALUE_Msk              (0x1U << ANALOG_CMPx_CSR_VALUE_Pos)
#define ANALOG_CMPx_CSR_VALUE                  ANALOG_CMPx_CSR_VALUE_Msk

#define ANALOG_CMPx_CSR_LOCK_Pos              (31U)
#define ANALOG_CMPx_CSR_LOCK_Msk              (0x1U << ANALOG_CMPx_CSR_LOCK_Pos)
#define ANALOG_CMPx_CSR_LOCK                  ANALOG_CMPx_CSR_LOCK_Msk





/**
  * @}
  */

 /**
  * @}
  */


/** @addtogroup Exported_macro
  * @{
  */

/****************************** ADC Instances *********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC)

/****************************** CRC Instances *********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)
                                      
/******************************* DMA Instances ********************************/
#define IS_DMA_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Channel1) || \
                                       ((INSTANCE) == DMA1_Channel2))

/****************************** GPIO Instances ********************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE)  (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB) || \
                                         ((INSTANCE) == GPIOC) || \
                                         ((INSTANCE) == GPIOF))
                                         
/**************************** GPIO Alternate Function Instances ***************/
#define IS_GPIO_AF_INSTANCE(INSTANCE)   (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB))

/****************************** GPIO Lock Instances ***************************/
#define IS_GPIO_LOCK_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB))

/****************************** I2C Instances *********************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) ((INSTANCE) == I2C1)

/****************** I2C Instances : wakeup capability from stop modes *********/
#define IS_I2C_WAKEUP_FROMSTOP_INSTANCE(INSTANCE) ((INSTANCE) == I2C1)

/****************************** I2S Instances *********************************/
#define IS_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SPI1)

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/****************************** SMBUS Instances *********************************/
#define IS_SMBUS_ALL_INSTANCE(INSTANCE) ((INSTANCE) == I2C1)

/****************************** SPI Instances *********************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SPI1)

/****************************** TIM Instances *********************************/
#define IS_TIM_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_CC1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_CC2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_CC3_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_CC4_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))


#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_HALL_INTERFACE_INSTANCE(INSTANCE)\
  0

#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE)\
  0

#define IS_TIM_XOR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_MASTER_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_SLAVE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE)\
  0

#define IS_TIM_DMABURST_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_BREAK_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM0) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    )

#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   0

#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE) 0

#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_DMA_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))
    
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))
    
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM0))

#define IS_TIM_REMAP_INSTANCE(INSTANCE)\
  0

#define IS_TIM_ADVANCED_INSTANCE(INSTANCE)\
  0

/**
  * @}
  */


/******************************************************************************/
/*  For a painless codes migration between the CA32 device product       */
/*  lines, the aliases defined below are put in place to overcome the         */
/*  differences in the interrupt handlers and IRQn definitions.               */
/*  No need to update developed interrupt code when moving across             */
/*  product lines within the same CA32 Family                                */
/******************************************************************************/

/* Aliases for __IRQn */
#define ADC1_COMP_IRQn                   ADC1_IRQn
#define DMA1_Ch1_IRQn                    DMA1_Channel1_IRQn
#define DMA1_Ch2_3_DMA2_Ch1_2_IRQn       DMA1_Channel2_3_IRQn
#define DMA1_Channel4_5_6_7_IRQn         DMA1_Channel4_5_IRQn
#define DMA1_Ch4_7_DMA2_Ch3_5_IRQn       DMA1_Channel4_5_IRQn
#define VDDIO2_IRQn                      PVD_IRQn
#define PVD_VDDIO2_IRQn                  PVD_IRQn
#define RCC_CRS_IRQn                     RCC_IRQn


/* Aliases for __IRQHandler */
#define ADC1_COMP_IRQHandler             ADC1_IRQHandler
#define DMA1_Ch1_IRQHandler              DMA1_Channel1_IRQHandler
#define DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler DMA1_Channel2_3_IRQHandler
#define DMA1_Channel4_5_6_7_IRQHandler   DMA1_Channel4_5_IRQHandler
#define DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler DMA1_Channel4_5_IRQHandler
#define VDDIO2_IRQHandler                PVD_IRQHandler
#define PVD_VDDIO2_IRQHandler            PVD_IRQHandler
#define RCC_CRS_IRQHandler               RCC_IRQHandler


#ifdef __cplusplus

}
#endif /* __cplusplus */

#endif /* __CA32M030_H */


