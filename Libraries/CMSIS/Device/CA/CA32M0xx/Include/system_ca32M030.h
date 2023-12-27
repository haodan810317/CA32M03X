/**
  ******************************************************************************
  * @file    system_ca32m030.h
  * @brief   CMSIS Cortex-M0 Device System Source File for CA32 devices.  
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup ca32m030_system
  * @{
  */  
  
/**
  * @brief Define to prevent recursive inclusion
  */
#ifndef __SYSTEM_CA32M030_H
#define __SYSTEM_CA32M030_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup CA32_System_Includes
  * @{
  */

/**
  * @}
  */


/** @addtogroup CA32_System_Exported_types
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      3) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) by calling HAL API function HAL_RCC_ClockConfig()
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */
extern const uint8_t AHBPrescTable[16];   /*!< AHB prescalers table values */
extern const uint8_t PLLVcoTable[4];      /*!< PLLVCO prescalers table values */ 

/**
  * @}
  */

/** @addtogroup CA32_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup CA32_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup CA32_System_Exported_Functions
  * @{
  */
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
extern uint32_t Delay(uint32_t ms);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_CA32M030_H */

/**
  * @}
  */
  
/**
  * @}
  */  
/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
