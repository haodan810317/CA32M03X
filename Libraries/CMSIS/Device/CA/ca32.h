#ifndef __CA32_H
#define __CA32_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
   
/** @addtogroup Library_configuration_section
  * @{
  */
  
/**
  * @brief CA32 Family
  */

	 
#if !defined  (HSI_VALUE) 
#define HSI_VALUE  ((uint32_t)24000000)
#endif


#if defined (CA32F003)
  #include "ca32f003.h"
#elif defined (CA32M030) || defined (CA32M031)
  #include "ca32m030.h"
#else
 #error "Please select first the target CA32 device used in your application"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */ 
typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum 
{
  ERROR = 0, 
  SUCCESS = !ERROR
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))


/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "ca32_hal.h"
#endif /* USE_HAL_DRIVER */

#ifdef USE_STDPERIPH_DRIVER
  #include "ca32_conf.h"
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CA32_H */
/**
  * @}
  */

/**
  * @}
  */
  



/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
