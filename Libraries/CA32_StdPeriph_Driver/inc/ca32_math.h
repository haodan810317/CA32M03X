/**
  ******************************************************************************
  * @file    ca32_math.h
  * @brief   MATH operation
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CA32_MATH_H
#define __CA32_MATH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ca32.h"


/**
  * @}
  */ 

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/

#define HW_DIV(DIVIDEND, DIVISOR, RES) {\
  MATH->DATA1 = DIVIDEND; \
	MATH->DATA2 = DIVISOR; \
	MATH->CR = MATH_CR_START; \
	while((MATH->CR & MATH_CR_DONE) == 0); \
	RES = MATH->RESULT1; \
}

#define HW_DIV_FULL(DIVIDEND, DIVISOR, QUOTIENT, REMAINDER) {\
  MATH->DATA1 = DIVIDEND; \
	MATH->DATA2 = DIVISOR; \
	MATH->CR = MATH_CR_START; \
	while((MATH->CR & MATH_CR_DONE) == 0); \
	QUOTIENT = MATH->RESULT1; \
	REMAINDER = MATH->RESULT2; \
}

#define HW_SQRT(REDICAND, RES) {\
  MATH->DATA1 = REDICAND; \
	MATH->CR = MATH_CR_START | MATH_CR_MODE_SQRT; \
	while((MATH->CR & MATH_CR_DONE) == 0); \
	RES = MATH->RESULT1; \
}


#define HW_SINCOS(THETA, MOD, SINVALUE, COSVALUE) {\
	volatile unsigned int u32TMP;\
	CORDIC->CSR = (0<<CORDIC_CSR_FUNC_Pos) | (1<<CORDIC_CSR_RESSIZE_Pos) | (1<<CORDIC_CSR_ARGSIZE_Pos) | (15<<CORDIC_CSR_PRECISION_Pos);\
	CORDIC->WDATA = ((THETA)&0x0000FFFF) | (MOD<<16);\
	while((CORDIC->CSR & CORDIC_CSR_RRDY) == 0);\
	u32TMP = CORDIC->RDATA;\
	SINVALUE = u32TMP >> 16;\
	COSVALUE = u32TMP & 0xFFFF;\
}

#define HW_PHASE(X, Y, ARCTAN, MOD) {\
	volatile unsigned int u32TMP;\
	CORDIC->CSR = (2<<CORDIC_CSR_FUNC_Pos) | (1<<CORDIC_CSR_RESSIZE_Pos) | (1<<CORDIC_CSR_ARGSIZE_Pos) | (15<<CORDIC_CSR_PRECISION_Pos);\
	CORDIC->WDATA = ((X)&0x0000FFFF) | (Y<<16);\
	while((CORDIC->CSR & CORDIC_CSR_RRDY) == 0);\
	u32TMP = CORDIC->RDATA;\
	MOD = u32TMP >> 16;\
	ARCTAN = u32TMP & 0xFFFF;\
}


/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif /* __CA32_WWDG_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT ThingsChip *****END OF FILE****/
