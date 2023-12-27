/**
  ******************************************************************************
  * @file    ca32_pwr.h
  * @brief   This file contains all the functions prototypes for the PWR 
  *          firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CA32_PWR_H
#define __CA32_PWR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ca32.h"


/** @defgroup PWR_PVD_detection_level 
  * @{
  */ 

#define PWR_PVD_0                  (0<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_1                  (1<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_2                  (2<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_3                  (3<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_4                  (4<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_5                  (5<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_6                  (6<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_7                  (7<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_8                  (8<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_9                  (9<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_10                 (10<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_11                 (11<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_12                 (12<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_13                 (13<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_14                 (14<<PWR_CFGR_PVD_REG_Pos)
#define PWR_PVD_15                 (15<<PWR_CFGR_PVD_REG_Pos)
																 
																 
#define PWR_BOR_0                  (0<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_1                  (1<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_2                  (2<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_3                  (3<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_4                  (4<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_5                  (5<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_6                  (6<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_7                  (7<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_8                  (8<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_9                  (9<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_10                 (10<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_11                 (11<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_12                 (12<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_13                 (13<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_14                 (14<<PWR_CFGR_BOR_REG_Pos)
#define PWR_BOR_15                 (15<<PWR_CFGR_BOR_REG_Pos)
/**
  * @}
  */

 
/** @defgroup PWR_Regulator_state_is_Sleep_STOP_mode 
  * @{
  */

#define PWR_Regulator_ON                ((uint32_t)0x00000000)
#define PWR_Regulator_LowPower          PWR_CR_LPSDSR
#define IS_PWR_REGULATOR(REGULATOR) (((REGULATOR) == PWR_Regulator_ON) || \
                                     ((REGULATOR) == PWR_Regulator_LowPower))
/**
  * @}
  */

/** @defgroup PWR_SLEEP_mode_entry 
  * @{
  */

#define PWR_SLEEPEntry_WFI              ((uint8_t)0x01)
#define PWR_SLEEPEntry_WFE              ((uint8_t)0x02)
#define IS_PWR_SLEEP_ENTRY(ENTRY) (((ENTRY) == PWR_SLEEPEntry_WFI) || ((ENTRY) == PWR_SLEEPEntry_WFE))
 
/**
  * @}
  */

/** @defgroup LDO Current Mode
  * @{
  */

#define LDOCurrentMode_LowPower              ((uint8_t)0x01)
#define LDOCurrentMode_Normal                ((uint8_t)0x02)
#define LDOCurrentMode_Fast                  ((uint8_t)0x02)
#define IS_PWR_LDO_CURRENT_MODE(ENTRY) (((ENTRY) == LDOCurrentMode_LowPower) || ((ENTRY) == LDOCurrentMode_Normal) || ((ENTRY) == LDOCurrentMode_Fast))
 
#define PWR_STOPEntry_WFI               ((uint8_t)0x01)
#define PWR_STOPEntry_WFE               ((uint8_t)0x02)
#define IS_PWR_STOP_ENTRY(ENTRY) (((ENTRY) == PWR_STOPEntry_WFI) || ((ENTRY) == PWR_STOPEntry_WFE) ||)
 
#define PWR_StandbyEntry_WFI               ((uint8_t)0x01)
#define PWR_StandbyEntry_WFE               ((uint8_t)0x02)
#define IS_PWR_STANDBY_ENTRY(ENTRY) (((ENTRY) == PWR_StandbyEntry_WFI) || ((ENTRY) == PWR_StandbyEntry_WFE))

#define PWR_FLAG_PINRST                PWR_CSR_PADRSTF
#define PWR_FLAG_PORRST                PWR_CSR_PORRSTF
#define PWR_FLAG_STDBYRST              PWR_CSR_PSORSTF
#define PWR_FLAG_BORRST                PWR_CSR_BORRSTF
#define IS_PWR_FLAG(ENTRY) (((ENTRY) == PWR_CSR_PADRSTF) || ((ENTRY) == PWR_CSR_PORRSTF) || ((ENTRY) == PWR_CSR_PSORSTF) || ((ENTRY) == PWR_CSR_BORRSTF))
/**
  * @}
  */

 
/**
  * @}
  */

/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Function used to set the PWR configuration to the default reset state ******/
void PWR_DeInit(void);

/* Backup Domain Access function **********************************************/
void PWR_BackupAccessCmd(FunctionalState NewState);

/* PVD configuration functions ************************************************/
void PWR_PVDCmd(FunctionalState NewState);
void PWR_PVDConfig(uint32_t PVD_Level);
FlagStatus PWR_PVDGetStatus(void);
	
void PWR_BORCmd(FunctionalState NewState);
void PWR_BORResetCmd(FunctionalState NewState);
void PWR_BOROutputCmd(FunctionalState NewState);
void PWR_BORConfig(uint32_t BOR_Level);
FlagStatus PWR_BORGetStatus(void);

/* WakeUp pins configuration functions ****************************************/
void PWR_WakeUpPinCmd(uint32_t PWR_WakeUpPin, FunctionalState NewState);

/* Low Power modes configuration functions ************************************/
void PWR_EnterSleepMode(uint8_t PWR_SLEEPEntry);
void PWR_EnterSTOPMode(uint8_t PWR_STOPEntry);
void PWR_EnterStandbyMode(uint8_t PWR_StandbyEntry);

/* Flags management functions *************************************************/
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(void);

void PWR_LDOCurrentModeConfig(uint8_t LDOCurrentMode);

void PWR_LDOResetCmd(FunctionalState NewState);
void PWR_PADResetCmd(FunctionalState NewState);
void PWR_PORResetCmd(FunctionalState NewState);
void PWR_IWDGResetCmd(FunctionalState NewState);

void PWR_BGPLevelConfig(uint8_t BGPLevel);
void PWR_BGPCmd(FunctionalState NewState);

#ifdef __cplusplus
}
#endif

#endif /* __CA32_PWR_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
