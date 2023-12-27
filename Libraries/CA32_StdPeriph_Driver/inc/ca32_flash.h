/**
  ******************************************************************************
  * @file    ca32_flash.h
  * @brief   This file contains all the functions prototypes for the FLASH
  *          firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CA32_FLASH_H
#define __CA32_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ca32.h"

/** @addtogroup CA32_StdPeriph_Driver
  * @{
  */

/** @addtogroup FLASH
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  FLASH Status
  */ 
typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_COMPLETE,
  FLASH_TIMEOUT,
  FLASH_UNALIGN
}FLASH_Status;

/* Exported constants --------------------------------------------------------*/
  
/** @defgroup FLASH_Exported_Constants
  * @{
  */ 
  
/** @defgroup FLASH_Latency 
  * @{
  */ 
#define FLASH_Latency_0                ((uint32_t)0x00000000)  /*!< FLASH Zero Latency cycle */
#define FLASH_Latency_1                FLASH_ACR_LATENCY       /*!< FLASH One Latency cycle */

#define IS_FLASH_LATENCY(LATENCY) (((LATENCY) == FLASH_Latency_0) || \
                                   ((LATENCY) == FLASH_Latency_1))
/**
  * @}
  */ 

/** @defgroup FLASH_Interrupts 
  * @{
  */
   
#define FLASH_IT_EOP                   FLASH_CR_EOPIE  /*!< End of programming interrupt source */
#define FLASH_IT_ERR                   FLASH_CR_ERRIE  /*!< Error interrupt source */
#define IS_FLASH_IT(IT) ((((IT) & (uint32_t)0xFFFFEBFF) == 0x00000000) && (((IT) != 0x00000000)))
/**
  * @}
  */ 

/** @defgroup FLASH_Address 
  * @{
  */
 #define IS_FLASH_PROGRAM_ADDRESS(ADDRESS) (((ADDRESS) >= 0x08000000) && ((ADDRESS) <= 0x0800FFFF))
/**
  * @}
  */

/** @defgroup FLASH_OB_DATA_ADDRESS 
  * @{
  */  
#define OB_DATA_ADDRESS_DATA0     (0x1FF80004)
#define OB_DATA_ADDRESS_DATA1     (0x1FF80006)	
#define IS_OB_DATA_ADDRESS(ADDRESS) (((ADDRESS) == OB_DATA_ADDRESS_DATA0) || ((ADDRESS) == OB_DATA_ADDRESS_DATA1)) 

/**
  * @}
  */

/** @defgroup FLASH_Option_Bytes_Write_Protection
  * @{
  */
#define OB_WRPR0_PAGE0to1              (0x00000001U) /* Write protection of page 0 to 1 */
#define OB_WRPR0_PAGE2to3              (0x00000002U) /* Write protection of page 2 to 3 */
#define OB_WRPR0_PAGE4to5              (0x00000004U) /* Write protection of page 4 to 5 */
#define OB_WRPR0_PAGE6to7              (0x00000008U) /* Write protection of page 6 to 7 */
#define OB_WRPR0_PAGE8to9              (0x00000010U) /* Write protection of page 8 to 9 */
#define OB_WRPR0_PAGE10to11            (0x00000020U) /* Write protection of page 10 to 11 */
#define OB_WRPR0_PAGE12to13            (0x00000040U) /* Write protection of page 12 to 13 */
#define OB_WRPR0_PAGE14to15            (0x00000080U) /* Write protection of page 14 to 15 */
#define OB_WRPR0_PAGE16to17            (0x00000100U) /* Write protection of page 16 to 17 */
#define OB_WRPR0_PAGE18to19            (0x00000200U) /* Write protection of page 18 to 19 */
#define OB_WRPR0_PAGE20to21            (0x00000400U) /* Write protection of page 20 to 21 */
#define OB_WRPR0_PAGE22to23            (0x00000800U) /* Write protection of page 22 to 23 */
#define OB_WRPR0_PAGE24to25            (0x00001000U) /* Write protection of page 24 to 25 */
#define OB_WRPR0_PAGE26to27            (0x00002000U) /* Write protection of page 26 to 27 */
#define OB_WRPR0_PAGE28to29            (0x00004000U) /* Write protection of page 28 to 29 */
#define OB_WRPR0_PAGE30to31            (0x00008000U) /* Write protection of page 30 to 31 */
#define OB_WRPR0_PAGE32to33            (0x00010000U) /* Write protection of page 32 to 33 */
#define OB_WRPR0_PAGE34to35            (0x00020000U) /* Write protection of page 34 to 35 */
#define OB_WRPR0_PAGE36to37            (0x00040000U) /* Write protection of page 36 to 37 */
#define OB_WRPR0_PAGE38to39            (0x00080000U) /* Write protection of page 38 to 39 */
#define OB_WRPR0_PAGE40to41            (0x00100000U) /* Write protection of page 40 to 41 */
#define OB_WRPR0_PAGE42to43            (0x00200000U) /* Write protection of page 42 to 43 */
#define OB_WRPR0_PAGE44to45            (0x00400000U) /* Write protection of page 44 to 45 */
#define OB_WRPR0_PAGE46to47            (0x00800000U) /* Write protection of page 46 to 47 */
#define OB_WRPR0_PAGE48to49            (0x01000000U) /* Write protection of page 48 to 49 */
#define OB_WRPR0_PAGE50to51            (0x02000000U) /* Write protection of page 50 to 51 */
#define OB_WRPR0_PAGE52to53            (0x04000000U) /* Write protection of page 52 to 53 */
#define OB_WRPR0_PAGE54to55            (0x08000000U) /* Write protection of page 54 to 55 */
#define OB_WRPR0_PAGE56to57            (0x10000000U) /* Write protection of page 56 to 57 */
#define OB_WRPR0_PAGE58to59            (0x20000000U) /* Write protection of page 58 to 59 */
#define OB_WRPR0_PAGE60to61            (0x40000000U) /* Write protection of page 60 to 61 */
#define OB_WRPR0_PAGE62to63            (0x80000000U) /* Write protection of page 62 to 63 */
#define OB_WRPR0_ALLPAGES              (0xFFFFFFFFU) /*!< Write protection of page 0 to 63 */ 

#define OB_WRPR1_PAGE64to65            (0x00000001U) /* Write protection of page 64 to 65 */
#define OB_WRPR1_PAGE66to67            (0x00000002U) /* Write protection of page 66 to 67 */
#define OB_WRPR1_PAGE68to69            (0x00000004U) /* Write protection of page 68 to 69 */
#define OB_WRPR1_PAGE70to71            (0x00000008U) /* Write protection of page 70 to 71 */
#define OB_WRPR1_PAGE72to73            (0x00000010U) /* Write protection of page 72 to 73 */
#define OB_WRPR1_PAGE74to75            (0x00000020U) /* Write protection of page 74 to 75 */
#define OB_WRPR1_PAGE76to77            (0x00000040U) /* Write protection of page 76 to 77 */
#define OB_WRPR1_PAGE78to79            (0x00000080U) /* Write protection of page 78 to 79 */
#define OB_WRPR1_PAGE80to81            (0x00000100U) /* Write protection of page 80 to 81 */
#define OB_WRPR1_PAGE82to83            (0x00000200U) /* Write protection of page 82 to 83 */
#define OB_WRPR1_PAGE84to85            (0x00000400U) /* Write protection of page 84 to 85 */
#define OB_WRPR1_PAGE86to87            (0x00000800U) /* Write protection of page 86 to 87 */
#define OB_WRPR1_PAGE88to89            (0x00001000U) /* Write protection of page 88 to 89 */
#define OB_WRPR1_PAGE90to91            (0x00002000U) /* Write protection of page 90 to 91 */
#define OB_WRPR1_PAGE92to93            (0x00004000U) /* Write protection of page 92 to 93 */
#define OB_WRPR1_PAGE94to95            (0x00008000U) /* Write protection of page 94 to 95 */
#define OB_WRPR1_PAGE96to97            (0x00010000U) /* Write protection of page 96 to 97 */
#define OB_WRPR1_PAGE98to99            (0x00020000U) /* Write protection of page 98 to 99 */
#define OB_WRPR1_PAGE100to101          (0x00040000U) /* Write protection of page 100 to 101 */
#define OB_WRPR1_PAGE102to103          (0x00080000U) /* Write protection of page 102 to 103 */
#define OB_WRPR1_PAGE104to105          (0x00100000U) /* Write protection of page 104 to 105 */
#define OB_WRPR1_PAGE106to107          (0x00200000U) /* Write protection of page 106 to 107 */
#define OB_WRPR1_PAGE108to109          (0x00400000U) /* Write protection of page 108 to 109 */
#define OB_WRPR1_PAGE110to111          (0x00800000U) /* Write protection of page 110 to 111 */
#define OB_WRPR1_PAGE112to113          (0x01000000U) /* Write protection of page 112 to 113 */
#define OB_WRPR1_PAGE114to115          (0x02000000U) /* Write protection of page 114 to 115 */
#define OB_WRPR1_PAGE116to117          (0x04000000U) /* Write protection of page 116 to 117 */
#define OB_WRPR1_PAGE118to119          (0x08000000U) /* Write protection of page 118 to 119 */
#define OB_WRPR1_PAGE120to121          (0x10000000U) /* Write protection of page 120 to 121 */
#define OB_WRPR1_PAGE122to123          (0x20000000U) /* Write protection of page 122 to 123 */
#define OB_WRPR1_PAGE124to125          (0x40000000U) /* Write protection of page 124 to 125 */
#define OB_WRPR1_PAGE126to127          (0x80000000U) /* Write protection of page 126 to 127 */
#define OB_WRPR1_ALLPAGES              (0xFFFFFFFFU) /*!< Write protection of page 64 to 127 */

#define IS_OB_WRP(PAGE) (((PAGE) != 0x0000000))
/**
  * @}
  */

/** @defgroup FLASH_Option_Bytes_Read_Protection 
  * @{
  */ 

/** 
  * @brief  FLASH_Read Protection Level  
  */ 
#define OB_RDP_Level_0   ((uint8_t)0xAA)
#define OB_RDP_Level_1   ((uint8_t)0xFF)
/*#define OB_RDP_Level_2   ((uint8_t)0xCC)*/ /* Warning: When enabling read protection level 2 
                                                it's no more possible to go back to level 1 or 0 */

#define IS_OB_RDP(LEVEL) (((LEVEL) == OB_RDP_Level_0)||\
                          ((LEVEL) == OB_RDP_Level_1))/*||\
                          ((LEVEL) == OB_RDP_Level_2))*/
/**
  * @}
  */ 

/** @defgroup FLASH_Option_Bytes_Watchdog 
  * @{
  */

#define OB_IWDG_SW                     ((uint8_t)0x01)  /*!< Software IWDG selected */
#define OB_IWDG_HW                     ((uint8_t)0x00)  /*!< Hardware IWDG selected */
#define IS_OB_IWDG_SOURCE(SOURCE) (((SOURCE) == OB_IWDG_SW) || ((SOURCE) == OB_IWDG_HW))

/**
  * @}
  */

/** @defgroup FLASH_Option_Bytes_nRST_STOP 
  * @{
  */

#define OB_STOP_NoRST                  ((uint8_t)0x02) /*!< No reset generated when entering in STOP */
#define OB_STOP_RST                    ((uint8_t)0x00) /*!< Reset generated when entering in STOP */
#define IS_OB_STOP_SOURCE(SOURCE) (((SOURCE) == OB_STOP_NoRST) || ((SOURCE) == OB_STOP_RST))

/**
  * @}
  */

/** @defgroup FLASH_Option_Bytes_nRST_STDBY 
  * @{
  */

#define OB_STDBY_NoRST                 ((uint8_t)0x04) /*!< No reset generated when entering in STANDBY */
#define OB_STDBY_RST                   ((uint8_t)0x00) /*!< Reset generated when entering in STANDBY */
#define IS_OB_STDBY_SOURCE(SOURCE) (((SOURCE) == OB_STDBY_NoRST) || ((SOURCE) == OB_STDBY_RST))

/**
  * @}
  */

/** @defgroup FLASH_Option_Bytes_BOOT1
  * @{
  */

#define OB_BOOT1_RESET                 ((uint8_t)0x00) /*!< BOOT1 Reset */
#define OB_BOOT1_SET                   ((uint8_t)0x10) /*!< BOOT1 Set */
#define IS_OB_BOOT1(BOOT1) (((BOOT1) == OB_BOOT1_RESET) || ((BOOT1) == OB_BOOT1_SET))

/**
  * @}
  */

/** @defgroup FLASH_Option_Bytes_BOOT0
  * @{
  */

#define OB_BOOT0_RESET                 ((uint8_t)0x00) /*!< BOOT0 Reset */
#define OB_BOOT0_SET                   ((uint8_t)0x08) /*!< BOOT0 Set */
#define IS_OB_BOOT0(BOOT0) (((BOOT0) == OB_BOOT0_RESET) || ((BOOT0) == OB_BOOT0_SET))

/**
  * @}
  */

/** @defgroup FLASH_Option_Bytes_BOOT0SW
  * @{
  */

#define OB_BOOT0_SW                   ((uint8_t)0x80) /*!< BOOT0 pin disabled */ 
#define OB_BOOT0_HW                   ((uint8_t)0x00) /*!< BOOT0 pin bonded with GPIO */ 
#define IS_OB_BOOT0SW(BOOT0) (((BOOT0) == OB_BOOT0_SW) || ((BOOT0) == OB_BOOT0_HW))

/**
  * @}
  */
  
/** @defgroup FLASH_Option_Bytes_VDDA_Analog_Monitoring
  * @{
  */

//#define OB_VDDA_ANALOG_ON              ((uint8_t)0x20) /*!< Analog monitoring on VDDA Power source ON */
//#define OB_VDDA_ANALOG_OFF             ((uint8_t)0x00) /*!< Analog monitoring on VDDA Power source OFF */

//#define IS_OB_VDDA_ANALOG(ANALOG) (((ANALOG) == OB_VDDA_ANALOG_ON) || ((ANALOG) == OB_VDDA_ANALOG_OFF))

/**
  * @}
  */    

/** @defgroup FLASH_Option_Bytes_SRAM_Parity_Enable 
  * @{
  */

//#define OB_SRAM_PARITY_SET              ((uint8_t)0x00) /*!< SRAM parity enable Set */
//#define OB_SRAM_PARITY_RESET            ((uint8_t)0x40) /*!< SRAM parity enable reset */

//#define IS_OB_SRAM_PARITY(PARITY) (((PARITY) == OB_SRAM_PARITY_SET) || ((PARITY) == OB_SRAM_PARITY_RESET))

/**
  * @}
  */ 
  
/** @defgroup FLASH_Flags 
  * @{
  */ 

#define FLASH_FLAG_BSY                 FLASH_SR_BSY     /*!< FLASH Busy flag */
#define FLASH_FLAG_PGERR               FLASH_SR_PGERR   /*!< FLASH Programming error flag */
#define FLASH_FLAG_WRPERR              FLASH_SR_WRPERR  /*!< FLASH Write protected error flag */
#define FLASH_FLAG_EOP                 FLASH_SR_EOP     /*!< FLASH End of Programming flag */
 
#define IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFFCB) == 0x00000000) && ((FLAG) != 0x00000000))

#define IS_FLASH_GET_FLAG(FLAG)  (((FLAG) == FLASH_FLAG_BSY) || ((FLAG) == FLASH_FLAG_PGERR) || \
                                  ((FLAG) == FLASH_FLAG_WRPERR) || ((FLAG) == FLASH_FLAG_EOP))
/**
  * @}
  */ 

/** @defgroup FLASH_Timeout_definition 
  * @{
  */ 
#define FLASH_ER_PRG_TIMEOUT         ((uint32_t)0x000B0000)


/**
  * @}
  */

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
  
/** 
  * @brief  FLASH memory functions that can be executed from FLASH.  
  */  
/* FLASH Interface configuration functions ************************************/
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_PrefetchBufferCmd(FunctionalState NewState);
FlagStatus FLASH_GetPrefetchBufferStatus(void);

/* FLASH Memory Programming functions *****************************************/
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_Program(uint32_t Address, uint32_t * Data_buffer, uint32_t size);
/* FLASH Option Bytes Programming functions *****************************************/
void FLASH_OB_Unlock(void);
void FLASH_OB_Lock(void);
void FLASH_OB_Launch(void);
FLASH_Status FLASH_OB_Erase(void);
FLASH_Status FLASH_OB_EnableWRP(uint32_t WRPR0, uint32_t WRPR1);
FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP);
FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
FLASH_Status FLASH_OB_BOOTConfig(uint8_t OB_BOOT1);
FLASH_Status FLASH_OB_BOOT0Config(uint8_t OB_BOOT0);
FLASH_Status FLASH_OB_BOOT0SWConfig(uint8_t OB_BOOT0SW);
FLASH_Status FLASH_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG);
FLASH_Status FLASH_OB_SRAMParityConfig(uint8_t OB_SRAM_Parity);
FLASH_Status FLASH_OB_WriteUser(uint8_t OB_USER);
FLASH_Status FLASH_OB_ProgramData(uint32_t Address, uint8_t Data);
uint8_t FLASH_OB_GetUser(void);
void FLASH_OB_GetWRP(uint32_t *WRPR0, uint32_t *WRPR1);
FlagStatus FLASH_OB_GetRDP(void);

/* FLASH Interrupts and flags management functions **********************************/
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);

/** @defgroup FLASH_Legacy 
  * @{
  */
#define FLASH_EraseOptionBytes               FLASH_OB_Erase
#define FLASH_EnableWriteProtection	         FLASH_OB_EnableWRP
#define FLASH_UserOptionByteConfig	         FLASH_OB_UserConfig
#define FLASH_ProgramOptionByteData          FLASH_OB_ProgramData
#define FLASH_GetUserOptionByte	             FLASH_OB_GetUser
#define FLASH_GetWriteProtectionOptionByte   FLASH_OB_GetWRP

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __CA32_FLASH_H */

/**
  * @}
  */

/**
  * @}
  */ 

/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
