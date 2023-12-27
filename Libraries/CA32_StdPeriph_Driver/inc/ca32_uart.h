/**
  ******************************************************************************
  * @file    ca32_uart.h
  * @brief   This file contains all the functions prototypes for the UART 
  *          firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CA32_UART_H
#define __CA32_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ca32.h"

/** @addtogroup CA32_StdPeriph_Driver
  * @{
  */

/** @addtogroup UART
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/

   
   
/** 
  * @brief  UART Init Structure definition  
  */ 

typedef struct
{
  uint32_t UART_BaudRate;            /*!< This member configures the UART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (16 * (UART_InitStruct->UART_BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 16) + 0.5 */

  uint32_t UART_WordLength;          /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UART_Word_Length */

  uint32_t UART_StopBits;            /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_Stop_Bits */

  uint32_t UART_Parity;              /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */
 
  uint32_t UART_Mode;                /*!< Specifies wether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Mode */

  uint32_t UART_HardwareFlowControl; /*!< Specifies wether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref UART_Hardware_Flow_Control*/
} UART_InitTypeDef;

/** 
  * @brief  UART Clock Init Structure definition
  */ 

typedef struct
{
  uint32_t UART_Clock;             /*!< Specifies whether the UART clock is enabled or disabled.
                                         This parameter can be a value of @ref UART_Clock */

  uint32_t UART_CPOL;              /*!< Specifies the steady state of the serial clock.
                                         This parameter can be a value of @ref UART_Clock_Polarity */

  uint32_t UART_CPHA;              /*!< Specifies the clock transition on which the bit capture is made.
                                         This parameter can be a value of @ref UART_Clock_Phase */

  uint32_t UART_LastBit;           /*!< Specifies whether the clock pulse corresponding to the last transmitted
                                         data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                                         This parameter can be a value of @ref UART_Last_Bit */
} UART_ClockInitTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup UART_Exported_Constants
  * @{
  */ 

#define IS_UART_ALL_PERIPH(PERIPH) (((PERIPH) == UART1) || \
                                     ((PERIPH) == UART2) || \
                                     ((PERIPH) == UART3) || \
                                     ((PERIPH) == UART4) || \
                                     ((PERIPH) == UART5) || \
                                     ((PERIPH) == UART6) || \
                                     ((PERIPH) == UART7) || \
                                     ((PERIPH) == UART8))

#define IS_UART_123_PERIPH(PERIPH) (((PERIPH) == UART1) || \
                                     ((PERIPH) == UART2) || \
                                     ((PERIPH) == UART3))

/** @defgroup UART_Word_Length 
  * @{
  */ 

#define UART_WordLength_8b                  ((uint32_t)0x00000000)
#define UART_WordLength_9b                  UART_CR1_M0 
#define UART_WordLength_7b                  UART_CR1_M1 
#define IS_UART_WORD_LENGTH(LENGTH) (((LENGTH) == UART_WordLength_8b) || \
                                      ((LENGTH) == UART_WordLength_9b) || \
                                      ((LENGTH) == UART_WordLength_7b))
/**
  * @}
  */ 

/** @defgroup UART_Stop_Bits 
  * @{
  */ 

#define UART_StopBits_1                     ((uint32_t)0x00000000)
#define UART_StopBits_2                     UART_CR2_STOP_2
#define IS_UART_STOPBITS(STOPBITS) (((STOPBITS) == UART_StopBits_1) || \
                                     ((STOPBITS) == UART_StopBits_2))
/**
  * @}
  */ 

/** @defgroup UART_Parity 
  * @{
  */ 

#define UART_Parity_No                      ((uint32_t)0x00000000)
#define UART_Parity_Even                    UART_CR1_PCE
#define UART_Parity_Odd                     (UART_CR1_PCE | UART_CR1_PS) 
#define IS_UART_PARITY(PARITY) (((PARITY) == UART_Parity_No) || \
                                 ((PARITY) == UART_Parity_Even) || \
                                 ((PARITY) == UART_Parity_Odd))
/**
  * @}
  */ 

/** @defgroup UART_Mode 
  * @{
  */ 

#define UART_Mode_Rx                        UART_CR1_RE
#define UART_Mode_Tx                        UART_CR1_TE
#define IS_UART_MODE(MODE) ((((MODE) & (uint32_t)0xFFFFFFF3) == 0x00) && \
                              ((MODE) != (uint32_t)0x00))
/**
  * @}
  */ 

/** @defgroup UART_Hardware_Flow_Control 
  * @{
  */ 

#define UART_HardwareFlowControl_None       ((uint32_t)0x00000000)
#define UART_HardwareFlowControl_RTS        UART_CR3_RTSE
#define UART_HardwareFlowControl_CTS        UART_CR3_CTSE
#define UART_HardwareFlowControl_RTS_CTS    (UART_CR3_RTSE | UART_CR3_CTSE)
#define IS_UART_HARDWARE_FLOW_CONTROL(CONTROL)\
                              (((CONTROL) == UART_HardwareFlowControl_None) || \
                               ((CONTROL) == UART_HardwareFlowControl_RTS) || \
                               ((CONTROL) == UART_HardwareFlowControl_CTS) || \
                               ((CONTROL) == UART_HardwareFlowControl_RTS_CTS))
/**
  * @}
  */ 

/** @defgroup UART_Clock 
  * @{
  */ 
  
#define UART_Clock_Disable                  ((uint32_t)0x00000000)
#define UART_Clock_Enable                   UART_CR2_CLKEN
#define IS_UART_CLOCK(CLOCK) (((CLOCK) == UART_Clock_Disable) || \
                               ((CLOCK) == UART_Clock_Enable))
/**
  * @}
  */ 

/** @defgroup UART_Clock_Polarity 
  * @{
  */
  
#define UART_CPOL_Low                       ((uint32_t)0x00000000)
#define UART_CPOL_High                      UART_CR2_CPOL
#define IS_UART_CPOL(CPOL) (((CPOL) == UART_CPOL_Low) || ((CPOL) == UART_CPOL_High))

/**
  * @}
  */ 

/** @defgroup UART_Clock_Phase
  * @{
  */

#define UART_CPHA_1Edge                     ((uint32_t)0x00000000)
#define UART_CPHA_2Edge                     UART_CR2_CPHA
#define IS_UART_CPHA(CPHA) (((CPHA) == UART_CPHA_1Edge) || ((CPHA) == UART_CPHA_2Edge))

/**
  * @}
  */

/** @defgroup UART_Last_Bit
  * @{
  */

#define UART_LastBit_Disable                ((uint32_t)0x00000000)
#define UART_LastBit_Enable                 UART_CR2_LBCL
#define IS_UART_LASTBIT(LASTBIT) (((LASTBIT) == UART_LastBit_Disable) || \
                                   ((LASTBIT) == UART_LastBit_Enable))
/**
  * @}
  */
  
/** @defgroup UART_DMA_Requests 
  * @{
  */

#define UART_DMAReq_Tx                      UART_CR3_DMAT
#define UART_DMAReq_Rx                      UART_CR3_DMAR
#define IS_UART_DMAREQ(DMAREQ) ((((DMAREQ) & (uint32_t)0xFFFFFF3F) == 0x00) && \
                                  ((DMAREQ) != (uint32_t)0x00))

/**
  * @}
  */ 

/** @defgroup UART_DMA_Recception_Error
  * @{
  */

#define UART_DMAOnError_Enable              ((uint32_t)0x00000000)
#define UART_DMAOnError_Disable             UART_CR3_DDRE
#define IS_UART_DMAONERROR(DMAERROR) (((DMAERROR) == UART_DMAOnError_Disable)|| \
                                       ((DMAERROR) == UART_DMAOnError_Enable))
/**
  * @}
  */ 

/** @defgroup UART_MuteMode_WakeUp_methods
  * @{
  */

#define UART_WakeUp_IdleLine                ((uint32_t)0x00000000)
#define UART_WakeUp_AddressMark             UART_CR1_WAKE
#define IS_UART_MUTEMODE_WAKEUP(WAKEUP) (((WAKEUP) == UART_WakeUp_IdleLine) || \
                                          ((WAKEUP) == UART_WakeUp_AddressMark))
/**
  * @}
  */

/** @defgroup UART_Address_Detection
  * @{
  */ 

#define UART_AddressLength_4b               ((uint32_t)0x00000000)
#define UART_AddressLength_7b               UART_CR2_ADDM7
#define IS_UART_ADDRESS_DETECTION(ADDRESS) (((ADDRESS) == UART_AddressLength_4b) || \
                                             ((ADDRESS) == UART_AddressLength_7b))
/**
  * @}
  */ 

/** @defgroup UART_StopMode_WakeUp_methods
  * @note     These parameters are only available for CA32M030 devices 
  * @{
  */ 

#define UART_WakeUpSource_AddressMatch      ((uint32_t)0x00000000)
#define UART_WakeUpSource_StartBit          UART_CR3_WUS_1
#define UART_WakeUpSource_RXNE              (UART_CR3_WUS_0 | UART_CR3_WUS_1)
#define IS_UART_STOPMODE_WAKEUPSOURCE(SOURCE) (((SOURCE) == UART_WakeUpSource_AddressMatch) || \
                                                ((SOURCE) == UART_WakeUpSource_StartBit) || \
                                                ((SOURCE) == UART_WakeUpSource_RXNE))
/**
  * @}
  */ 

/** @defgroup UART_LIN_Break_Detection_Length 
  * @{
  */
  
#define UART_LINBreakDetectLength_10b       ((uint32_t)0x00000000)
#define UART_LINBreakDetectLength_11b       UART_CR2_LBDL
#define IS_UART_LIN_BREAK_DETECT_LENGTH(LENGTH) \
                               (((LENGTH) == UART_LINBreakDetectLength_10b) || \
                                ((LENGTH) == UART_LINBreakDetectLength_11b))
/**
  * @}
  */

/** @defgroup UART_IrDA_Low_Power 
  * @{
  */

#define UART_IrDAMode_LowPower              UART_CR3_IRLP
#define UART_IrDAMode_Normal                ((uint32_t)0x00000000)
#define IS_UART_IRDA_MODE(MODE) (((MODE) == UART_IrDAMode_LowPower) || \
                                  ((MODE) == UART_IrDAMode_Normal))
/**
  * @}
  */ 

/** @defgroup UART_DE_Polarity 
  * @{
  */

#define UART_DEPolarity_High                ((uint32_t)0x00000000)
#define UART_DEPolarity_Low                 UART_CR3_DEP
#define IS_UART_DE_POLARITY(POLARITY) (((POLARITY) == UART_DEPolarity_Low) || \
                                        ((POLARITY) == UART_DEPolarity_High))
/**
  * @}
  */ 

/** @defgroup UART_Inversion_Pins 
  * @{
  */

#define UART_InvPin_Tx                      UART_CR2_TXINV
#define UART_InvPin_Rx                      UART_CR2_RXINV
#define IS_UART_INVERSTION_PIN(PIN) ((((PIN) & (uint32_t)0xFFFCFFFF) == 0x00) && \
                                       ((PIN) != (uint32_t)0x00))

/**
  * @}
  */ 

/** @defgroup UART_OVR_DETECTION
  * @{
  */

#define UART_OVRDetection_Enable            ((uint32_t)0x00000000)
#define UART_OVRDetection_Disable           UART_CR3_OVRDIS
#define IS_UART_OVRDETECTION(OVR) (((OVR) == UART_OVRDetection_Enable)|| \
                                    ((OVR) == UART_OVRDetection_Disable))
/**
  * @}
  */ 
/** @defgroup UART_Request 
  * @{
  */

#define UART_Request_ABRRQ                  UART_RQR_ABRRQ
#define UART_Request_SBKRQ                  UART_RQR_SBKRQ
#define UART_Request_MMRQ                   UART_RQR_MMRQ
#define UART_Request_RXFRQ                  UART_RQR_RXFRQ
#define UART_Request_TXFRQ                  UART_RQR_TXFRQ

#define IS_UART_REQUEST(REQUEST) (((REQUEST) == UART_Request_TXFRQ) || \
                                   ((REQUEST) == UART_Request_RXFRQ) || \
                                   ((REQUEST) == UART_Request_MMRQ) || \
                                   ((REQUEST) == UART_Request_SBKRQ) || \
                                   ((REQUEST) == UART_Request_ABRRQ))
/**
  * @}
  */ 

/** @defgroup UART_Flags 
  * @{
  */
#define UART_FLAG_REACK                     UART_ISR_REACK
#define UART_FLAG_TEACK                     UART_ISR_TEACK
#define UART_FLAG_WU                        UART_ISR_WUF
#define UART_FLAG_RWU                       UART_ISR_RWU
#define UART_FLAG_SBK                       UART_ISR_SBKF
#define UART_FLAG_CM                        UART_ISR_CMF
#define UART_FLAG_BUSY                      UART_ISR_BUSY
#define UART_FLAG_ABRF                      UART_ISR_ABRF
#define UART_FLAG_ABRE                      UART_ISR_ABRE
#define UART_FLAG_EOB                       UART_ISR_EOBF
#define UART_FLAG_RTO                       UART_ISR_RTOF
#define UART_FLAG_nCTSS                     UART_ISR_CTS 
#define UART_FLAG_CTS                       UART_ISR_CTSIF
#define UART_FLAG_LBD                       UART_ISR_LBD
#define UART_FLAG_TXE                       UART_ISR_TXE
#define UART_FLAG_TC                        UART_ISR_TC
#define UART_FLAG_RXNE                      UART_ISR_RXNE
#define UART_FLAG_IDLE                      UART_ISR_IDLE
#define UART_FLAG_ORE                       UART_ISR_ORE
#define UART_FLAG_NE                        UART_ISR_NE
#define UART_FLAG_FE                        UART_ISR_FE
#define UART_FLAG_PE                        UART_ISR_PE
#define IS_UART_FLAG(FLAG) (((FLAG) == UART_FLAG_PE) || ((FLAG) == UART_FLAG_TXE) || \
                             ((FLAG) == UART_FLAG_TC) || ((FLAG) == UART_FLAG_RXNE) || \
                             ((FLAG) == UART_FLAG_IDLE) || ((FLAG) == UART_FLAG_LBD) || \
                             ((FLAG) == UART_FLAG_CTS) || ((FLAG) == UART_FLAG_ORE) || \
                             ((FLAG) == UART_FLAG_NE) || ((FLAG) == UART_FLAG_FE) || \
                             ((FLAG) == UART_FLAG_nCTSS) || ((FLAG) == UART_FLAG_RTO) || \
                             ((FLAG) == UART_FLAG_EOB) || ((FLAG) == UART_FLAG_ABRE) || \
                             ((FLAG) == UART_FLAG_ABRF) || ((FLAG) == UART_FLAG_BUSY) || \
                             ((FLAG) == UART_FLAG_CM) || ((FLAG) == UART_FLAG_SBK) || \
                             ((FLAG) == UART_FLAG_RWU) || ((FLAG) == UART_FLAG_WU) || \
                             ((FLAG) == UART_FLAG_TEACK)|| ((FLAG) == UART_FLAG_REACK))

#define IS_UART_CLEAR_FLAG(FLAG) (((FLAG) == UART_FLAG_WU) || ((FLAG) == UART_FLAG_TC) || \
                                   ((FLAG) == UART_FLAG_IDLE) || ((FLAG) == UART_FLAG_ORE) || \
                                   ((FLAG) == UART_FLAG_NE) || ((FLAG) == UART_FLAG_FE) || \
                                   ((FLAG) == UART_FLAG_LBD) || ((FLAG) == UART_FLAG_CTS) || \
                                   ((FLAG) == UART_FLAG_RTO) || ((FLAG) == UART_FLAG_EOB) || \
                                   ((FLAG) == UART_FLAG_CM) || ((FLAG) == UART_FLAG_PE))
/**
  * @}
  */ 

/** @defgroup UART_Interrupt_definition 
  * @brief UART Interrupt definition
  * UART_IT possible values
  * Elements values convention: 0xZZZZYYXX
  *   XX: Position of the corresponding Interrupt
  *   YY: Register index
  *   ZZZZ: Flag position
  * @{
  */

#define UART_IT_WU                          ((uint32_t)0x00140316) 
#define UART_IT_CM                          ((uint32_t)0x0011010E)
#define UART_IT_EOB                         ((uint32_t)0x000C011B) 
#define UART_IT_RTO                         ((uint32_t)0x000B011A)
#define UART_IT_PE                          ((uint32_t)0x00000108)
#define UART_IT_TXE                         ((uint32_t)0x00070107)
#define UART_IT_TC                          ((uint32_t)0x00060106)
#define UART_IT_RXNE                        ((uint32_t)0x00050105)
#define UART_IT_IDLE                        ((uint32_t)0x00040104)
#define UART_IT_LBD                         ((uint32_t)0x00080206) 
#define UART_IT_CTS                         ((uint32_t)0x0009030A) 
#define UART_IT_ERR                         ((uint32_t)0x00000300)
#define UART_IT_ORE                         ((uint32_t)0x00030300)
#define UART_IT_NE                          ((uint32_t)0x00020300)
#define UART_IT_FE                          ((uint32_t)0x00010300)

#define IS_UART_CONFIG_IT(IT) (((IT) == UART_IT_PE) || ((IT) == UART_IT_TXE) || \
                                ((IT) == UART_IT_TC) || ((IT) == UART_IT_RXNE) || \
                                ((IT) == UART_IT_IDLE) || ((IT) == UART_IT_LBD) || \
                                ((IT) == UART_IT_CTS) || ((IT) == UART_IT_ERR) || \
                                ((IT) == UART_IT_RTO) || ((IT) == UART_IT_EOB) || \
                                ((IT) == UART_IT_CM) || ((IT) == UART_IT_WU))

#define IS_UART_GET_IT(IT) (((IT) == UART_IT_PE) || ((IT) == UART_IT_TXE) || \
                             ((IT) == UART_IT_TC) || ((IT) == UART_IT_RXNE) || \
                             ((IT) == UART_IT_IDLE) || ((IT) == UART_IT_LBD) || \
                             ((IT) == UART_IT_CTS) || ((IT) == UART_IT_ORE) || \
                             ((IT) == UART_IT_NE) || ((IT) == UART_IT_FE) || \
                             ((IT) == UART_IT_RTO) || ((IT) == UART_IT_EOB) || \
                             ((IT) == UART_IT_CM) || ((IT) == UART_IT_WU))

#define IS_UART_CLEAR_IT(IT) (((IT) == UART_IT_TC) || ((IT) == UART_IT_PE) || \
                               ((IT) == UART_IT_FE) || ((IT) == UART_IT_NE) || \
                               ((IT) == UART_IT_ORE) || ((IT) == UART_IT_IDLE) || \
                               ((IT) == UART_IT_LBD) || ((IT) == UART_IT_CTS) || \
                               ((IT) == UART_IT_RTO) || ((IT) == UART_IT_EOB) || \
                               ((IT) == UART_IT_CM) || ((IT) == UART_IT_WU))
/**
  * @}
  */

/** @defgroup UART_Global_definition 
  * @{
  */

#define IS_UART_BAUDRATE(BAUDRATE) (((BAUDRATE) > 0) && ((BAUDRATE) < 0x005B8D81))
#define IS_UART_DE_ASSERTION_DEASSERTION_TIME(TIME) ((TIME) <= 0x1F)
#define IS_UART_AUTO_RETRY_COUNTER(COUNTER) ((COUNTER) <= 0x7)
#define IS_UART_TIMEOUT(TIMEOUT) ((TIMEOUT) <= 0x00FFFFFF)
#define IS_UART_DATA(DATA) ((DATA) <= 0x1FF)

/**
  * @}
  */ 

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Initialization and Configuration functions *********************************/
void UART_DeInit(UART_TypeDef* UARTx);
void UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct);
void UART_StructInit(UART_InitTypeDef* UART_InitStruct);
void UART_ClockInit(UART_TypeDef* UARTx, UART_ClockInitTypeDef* UART_ClockInitStruct);
void UART_ClockStructInit(UART_ClockInitTypeDef* UART_ClockInitStruct);
void UART_Cmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_DirectionModeCmd(UART_TypeDef* UARTx, uint32_t UART_DirectionMode, FunctionalState NewState);
void UART_SetPrescaler(UART_TypeDef* UARTx, uint8_t UART_Prescaler);
void UART_OverSampling8Cmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_OneBitMethodCmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_MSBFirstCmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_DataInvCmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_InvPinCmd(UART_TypeDef* UARTx, uint32_t UART_InvPin, FunctionalState NewState);
void UART_SWAPPinCmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_ReceiverTimeOutCmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_SetReceiverTimeOut(UART_TypeDef* UARTx, uint32_t UART_ReceiverTimeOut);

/* STOP Mode functions ********************************************************/
void UART_STOPModeCmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_StopModeWakeUpSourceConfig(UART_TypeDef* UARTx, uint32_t UART_WakeUpSource);

/* Data transfers functions ***************************************************/
void UART_SendData(UART_TypeDef* UARTx, uint16_t Data);
uint16_t UART_ReceiveData(UART_TypeDef* UARTx);

/* Multi-Processor Communication functions ************************************/
void UART_SetAddress(UART_TypeDef* UARTx, uint8_t UART_Address);
void UART_MuteModeWakeUpConfig(UART_TypeDef* UARTx, uint32_t UART_WakeUp);
void UART_MuteModeCmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_AddressDetectionConfig(UART_TypeDef* UARTx, uint32_t UART_AddressLength);


/* Half-duplex mode function **************************************************/
void UART_HalfDuplexCmd(UART_TypeDef* UARTx, FunctionalState NewState);


/* RS485 mode functions *******************************************************/
void UART_DECmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_DEPolarityConfig(UART_TypeDef* UARTx, uint32_t UART_DEPolarity);
void UART_SetDEAssertionTime(UART_TypeDef* UARTx, uint32_t UART_DEAssertionTime);
void UART_SetDEDeassertionTime(UART_TypeDef* UARTx, uint32_t UART_DEDeassertionTime);

/* DMA transfers management functions *****************************************/
void UART_DMACmd(UART_TypeDef* UARTx, uint32_t UART_DMAReq, FunctionalState NewState);
void UART_DMAReceptionErrorConfig(UART_TypeDef* UARTx, uint32_t UART_DMAOnError);

/* Interrupts and flags management functions **********************************/
void UART_ITConfig(UART_TypeDef* UARTx, uint32_t UART_IT, FunctionalState NewState);
void UART_RequestCmd(UART_TypeDef* UARTx, uint32_t UART_Request, FunctionalState NewState);
void UART_OverrunDetectionConfig(UART_TypeDef* UARTx, uint32_t UART_OVRDetection);
FlagStatus UART_GetFlagStatus(UART_TypeDef* UARTx, uint32_t UART_FLAG);
void UART_ClearFlag(UART_TypeDef* UARTx, uint32_t UART_FLAG);
ITStatus UART_GetITStatus(UART_TypeDef* UARTx, uint32_t UART_IT);
void UART_ClearITPendingBit(UART_TypeDef* UARTx, uint32_t UART_IT);

#ifdef __cplusplus
}
#endif

#endif /* __CA32_UART_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
