/**
  ******************************************************************************
  * @file    ca32_flash.c
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the FLASH peripheral:
  *            - FLASH Interface configuration
  *            - FLASH Memory Programming
  *            - Option Bytes Programming
  *            - Interrupts and flags management
  *
  *  @verbatim
 ===============================================================================
                    ##### How to use this driver #####
 ===============================================================================
    [..] This driver provides functions to configure and program the Flash 
         memory of all CA32 devices. These functions are split in 4 groups
         (#) FLASH Interface configuration functions: this group includes the 
             management of following features:
             (++) Set the latency
             (++) Enable/Disable the prefetch buffer

         (#) FLASH Memory Programming functions: this group includes all needed 
             functions to erase and program the main memory:
             (++) Lock and Unlock the Flash interface.
             (++) Erase function: Erase Page, erase all pages.
             (++) Program functions: Half Word and Word write.

         (#) FLASH Option Bytes Programming functions: this group includes all 
             needed functions to:
             (++) Lock and Unlock the Flash Option bytes.
             (++) Launch the Option Bytes loader
             (++) Erase the Option Bytes
             (++)Set/Reset the write protection
             (++) Set the Read protection Level
             (++) Program the user option Bytes
             (++) Set/Reset the BOOT1 bit
             (++) Enable/Disable the VDDA Analog Monitoring
             (++) Get the user option bytes
             (++) Get the Write protection
             (++) Get the read protection status

         (#) FLASH Interrupts and flag management functions: this group includes 
             all needed functions to:
             (++) Enable/Disable the flash interrupt sources
             (++) Get flags status
             (++) Clear flags
             (++) Get Flash operation status
             (++) Wait for last flash operation

 @endverbatim
  
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ca32_flash.h"

/** @addtogroup CA32_StdPeriph_Driver
  * @{
  */

/** @defgroup FLASH 
  * @brief FLASH driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
 
/** @defgroup FLASH_Private_Functions
  * @{
  */ 

/** @defgroup FLASH_Group1 FLASH Interface configuration functions
  *  @brief   FLASH Interface configuration functions 
 *
@verbatim   
 ===============================================================================
               ##### FLASH Interface configuration functions #####
 ===============================================================================

    [..] FLASH_Interface configuration_Functions, includes the following functions:
       (+) void FLASH_SetLatency(uint32_t FLASH_Latency):
    [..] To correctly read data from Flash memory, the number of wait states (LATENCY) 
     must be correctly programmed according to the frequency of the CPU clock (HCLK) 
    [..]
        +--------------------------------------------- +
        |  Wait states  |   HCLK clock frequency (MHz) |
        |---------------|------------------------------|
        |0WS(1CPU cycle)|       0 < HCLK <= 24         |
        |---------------|------------------------------|
        |1WS(2CPU cycle)|       24 < HCLK <= 48        |
        +----------------------------------------------+
    [..]
       (+) void FLASH_PrefetchBufferCmd(FunctionalState NewState);
    [..]
     All these functions don't need the unlock sequence.

@endverbatim
  * @{
  */

/**
  * @brief  Sets the code latency value.
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *          This parameter can be one of the following values:
  *             @arg FLASH_Latency_0: FLASH Zero Latency cycle
  *             @arg FLASH_Latency_1: FLASH One Latency cycle
  * @retval None
  */
void FLASH_SetLatency(uint32_t FLASH_Latency)
{
   uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FLASH_LATENCY(FLASH_Latency));

  /* Read the ACR register */
  tmpreg = FLASH->ACR;  

  /* Sets the Latency value */
  tmpreg &= (uint32_t) (~((uint32_t)FLASH_ACR_LATENCY));
  tmpreg |= FLASH_Latency;

  /* Write the ACR register */
  FLASH->ACR = tmpreg;
}

/**
  * @brief  Enables or disables the Prefetch Buffer.
  * @param  NewState: new state of the FLASH prefetch buffer.
  *          This parameter can be: ENABLE or DISABLE. 
  * @retval None
  */
void FLASH_PrefetchBufferCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

//  if(NewState != DISABLE)
//  {
//    FLASH->ACR |= FLASH_ACR_PRFTBE;
//  }
//  else
//  {
//    FLASH->ACR &= (uint32_t)(~((uint32_t)FLASH_ACR_PRFTBE));
//  }
}

/**
  * @brief  Checks whether the FLASH Prefetch Buffer status is set or not.
  * @param  None
  * @retval FLASH Prefetch Buffer Status (SET or RESET).
  */
FlagStatus FLASH_GetPrefetchBufferStatus(void)
{
  FlagStatus bitstatus = RESET;

//  if ((FLASH->ACR & FLASH_ACR_PRFTBS) != (uint32_t)RESET)
//  {
//    bitstatus = SET;
//  }
//  else
//  {
//    bitstatus = RESET;
//  }
  /* Return the new state of FLASH Prefetch Buffer Status (SET or RESET) */
  return bitstatus; 
}

/**
  * @}
  */

/** @defgroup FLASH_Group2 FLASH Memory Programming functions
 *  @brief   FLASH Memory Programming functions
 *
@verbatim   
 ===============================================================================
                ##### FLASH Memory Programming functions #####
 ===============================================================================

    [..] The FLASH Memory Programming functions, includes the following functions:
       (+) void FLASH_Unlock(void);
       (+) void FLASH_Lock(void);
       (+) FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
       (+) FLASH_Status FLASH_EraseAllPages(void);
       (+) FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
       (+) FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);

    [..] Any operation of erase or program should follow these steps:
       
       (#) Call the FLASH_Unlock() function to enable the flash control register and 
           program memory access
       (#) Call the desired function to erase page or program data
       (#) Call the FLASH_Lock() to disable the flash program memory access 
      (recommended to protect the FLASH memory against possible unwanted operation)

@endverbatim
  * @{
  */

/**
  * @brief  Unlocks the FLASH control register and program memory access.
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
  if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Unlocking the program memory access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
}

/**
  * @brief  Locks the Program memory access.
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH control register and program memory access */
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
  * @brief  Erases a specified page in program memory.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  Page_Address: The page address in program memory to be erased.
  * @note   A Page is erased in the Program memory only if the address to load 
  *         is the start address of a page (multiple of 512 bytes).
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ErasePage(uint32_t Page_Address)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Page_Address));
 
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  { 
    /* If the previous operation is completed, proceed to erase the page */
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = Page_Address;
    FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    /* Disable the PER Bit */
    FLASH->CR &= ~FLASH_CR_PER;
  }
    
  /* Return the Erase Status */
  return status;
}

/**
  * @brief  Erases all FLASH pages.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseAllPages(void)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase all pages */
     FLASH->CR |= FLASH_CR_MER;
     FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

    /* Disable the MER Bit */
    FLASH->CR &= ~FLASH_CR_MER;
  }

  /* Return the Erase Status */
  return status;
}

/**
  * @brief  Programs a word at a specified address.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  Address: specifies the address to be programmed, must 4Bytes ALIGN.
  * @param  Data_buffer: specifies the data to be programmed, must 4Bytes ALIGN.
  * @param  size: data byte numbers, must 4Bytes ALIGN.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_Program(uint32_t Address, uint32_t * Data_buffer, uint32_t size)
{
  FLASH_Status status = FLASH_COMPLETE;
	int i;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));
	
  if( (Address%4) || ((uint32_t)Data_buffer%4) || (size%4) )
  {
    return FLASH_UNALIGN;
  }

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new first 
    half word */
    FLASH->CR |= FLASH_CR_PG;
  
		for(i=0; i<size/4; i++)
		{
			*(__IO uint32_t*)Address = (uint32_t)Data_buffer[i];
			Address += 4;
		}
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
 
    /* Disable the PG Bit */
    FLASH->CR &= ~FLASH_CR_PG;
  }
   
  /* Return the Program Status */
  return status;
}

/**
  * @brief  Programs a word at a specified address.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  Address: specifies the address to be programmed, must 4Bytes ALIGN.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
  __IO uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));
	
  if( Address%4 )
  {
    return FLASH_UNALIGN;
  }

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new first 
    half word */
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint16_t*)Address = (uint16_t)Data;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
 
    if(status == FLASH_COMPLETE)
    {
      /* If the previous operation is completed, proceed to program the new second 
      half word */
      tmp = Address + 2;

      *(__IO uint16_t*) tmp = Data >> 16;
    
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
        
      /* Disable the PG Bit */
      FLASH->CR &= ~FLASH_CR_PG;
    }
    else
    {
      /* Disable the PG Bit */
      FLASH->CR &= ~FLASH_CR_PG;
    }
  }
   
  /* Return the Program Status */
  return status;
}

/**
  * @brief  Programs a half word at a specified address.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  Address: specifies the address to be programmed, must 2Bytes ALIGN.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));
	
  if( Address%2 )
  {
    return FLASH_UNALIGN;
  }

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new data */
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint16_t*)Address = Data;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    /* Disable the PG Bit */
    FLASH->CR &= ~FLASH_CR_PG;
  } 
  
  /* Return the Program Status */
  return status;
}

/**
  * @}
  */
  
/** @defgroup FLASH_Group3 Option Bytes Programming functions
 *  @brief   Option Bytes Programming functions 
 *
@verbatim   
 ===============================================================================
                ##### Option Bytes Programming functions #####
 ===============================================================================

    [..] The FLASH_Option Bytes Programming_functions, includes the following functions:
       (+) void FLASH_OB_Unlock(void);
       (+) void FLASH_OB_Lock(void);
       (+) void FLASH_OB_Launch(void);
       (+) FLASH_Status FLASH_OB_Erase(void);
       (+) FLASH_Status FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
       (+) FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP);
       (+) FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
       (+) FLASH_Status FLASH_OB_BOOTConfig(uint8_t OB_BOOT1);
       (+) FLASH_Status FLASH_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG);
       (+) FLASH_Status FLASH_OB_WriteUser(uint8_t OB_USER);
       (+) FLASH_OB_ProgramData(uint32_t Address, uint8_t Data);
       (+) uint8_t FLASH_OB_GetUser(void);
       (+) uint32_t FLASH_OB_GetWRP(void);
       (+) FlagStatus FLASH_OB_GetRDP(void);

    [..] Any operation of erase or program should follow these steps:

   (#) Call the FLASH_OB_Unlock() function to enable the Option Bytes registers access

   (#) Call one or several functions to program the desired option bytes 
      (++) FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP) => to set the desired read Protection Level
      (++) FLASH_Status FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState) 
           => to Enable/Disable the desired sector write protection
      (++) FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY) 
           => to configure the user option Bytes: IWDG, STOP and the Standby.
      (++) FLASH_Status FLASH_OB_BOOTConfig(uint8_t OB_BOOT1)
           => to set or reset BOOT1 
      (++) FLASH_Status FLASH_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG) 
           => to enable or disable the VDDA Analog Monitoring 			 
      (++) You can write all User Options bytes at once using a single function
           by calling FLASH_Status FLASH_OB_WriteUser(uint8_t OB_USER)
      (++) FLASH_OB_ProgramData(uint32_t Address, uint8_t Data) to program the 
           two half word in the option bytes

   (#) Once all needed option bytes to be programmed are correctly written, call the
      FLASH_OB_Launch(void) function to launch the Option Bytes programming process.

   (#) Call the FLASH_OB_Lock() to disable the Option Bytes registers access (recommended
      to protect the option Bytes against possible unwanted operations)

@endverbatim
  * @{
  */

/**
  * @brief  Unlocks the option bytes block access.
  * @param  None
  * @retval None
  */
void FLASH_OB_Unlock(void)
{
  if((FLASH->CR & FLASH_CR_OPTWRE) == RESET)
  { 
    /* Unlocking the option bytes block access */
    FLASH->OPTKEYR = FLASH_OPTKEY1;
    FLASH->OPTKEYR = FLASH_OPTKEY2;
  }
}

/**
  * @brief  Locks the option bytes block access.
  * @param  None
  * @retval None
  */
void FLASH_OB_Lock(void)
{
  /* Set the OPTWREN Bit to lock the option bytes block access */
  FLASH->CR &= ~FLASH_CR_OPTWRE;
}

/**
  * @brief  Launch the option byte loading.
  * @param  None
  * @retval None
  */
void FLASH_OB_Launch(void)
{
  /* Set the OBL_Launch bit to launch the option byte loading */
  FLASH->CR |= FLASH_CR_OBL_LAUNCH;
}

/**
  * @brief  Erases the FLASH option bytes.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @note   This functions erases all option bytes except the Read protection (RDP).
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_Erase(void)
{
  uint16_t rdptmp = OB_RDP_Level_0;

  FLASH_Status status = FLASH_COMPLETE;

  /* Get the actual read protection Option Byte value */ 
	rdptmp = FLASH->OBR & FLASH_OBR_RDP;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  if(status == FLASH_COMPLETE)
  {   
    /* If the previous operation is completed, proceed to erase the option bytes */
    FLASH->CR |= FLASH_CR_OPTER;
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    if(status == FLASH_COMPLETE)
    {
      /* If the erase operation is completed, disable the OPTER Bit */
      FLASH->CR &= ~FLASH_CR_OPTER;
       
      /* Enable the Option Bytes Programming operation */
      FLASH->CR |= FLASH_CR_OPTPG;

      /* Restore the last read protection Option Byte value */
			if((rdptmp & 0xFF) == OB_RDP_Level_0/* || (rdptmp & 0xFF) == OB_RDP_Level_2 */)
			{
				OB->RDP = (uint16_t)((((~rdptmp)<<8)&0xFF00) | (rdptmp&0x00FF));
			}
			else
			{
				OB->RDP = 0xFFFF;
			}

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
 
      if(status != FLASH_TIMEOUT)
      {
        /* if the program operation is completed, disable the OPTPG Bit */
        FLASH->CR &= ~FLASH_CR_OPTPG;
      }
    }
    else
    {
      if (status != FLASH_TIMEOUT)
      {
        /* Disable the OPTER Bit */
        FLASH->CR &= ~FLASH_CR_OPTER;
      }
    }  
  }
  /* Return the erase status */
  return status;
}


/**
  * @brief  Read the FLASH option bytes.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  OB_read: OB_TypeDef pointer.
  * @retval None
  */
static void FLASH_OB_Read(OB_TypeDef *OB_read)
{	
	OB_read->RDP = OB->RDP;
	OB_read->USER = OB->USER;
	OB_read->DATA0 = OB->DATA0;
	OB_read->DATA1 = OB->DATA1;
	OB_read->WRP0 = OB->WRP0;
	OB_read->WRP1 = OB->WRP1;
	OB_read->WRP2 = OB->WRP2;
	OB_read->WRP3 = OB->WRP3;
	OB_read->WRP4 = OB->WRP4;
	OB_read->WRP5 = OB->WRP5;
	OB_read->WRP6 = OB->WRP6;
	OB_read->WRP7 = OB->WRP7;

	return;
}

/**
  * @brief  Write the FLASH option bytes.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  OB_write: OB_TypeDef pointer.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
static FLASH_Status FLASH_OB_Write(OB_TypeDef *OB_write)
{
  FLASH_Status status = FLASH_COMPLETE;
	
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);  
  if(status == FLASH_COMPLETE)
	{		
		FLASH->CR |= FLASH_CR_OPTER;
    FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    if(status == FLASH_COMPLETE)
    {
      /* If the erase operation is completed, disable the OPTER Bit */
      FLASH->CR &= ~FLASH_CR_OPTER;
      
      /* Enable the Option Bytes Programming operation */
      FLASH->CR |= FLASH_CR_OPTPG;
			
			OB->RDP = OB_write->RDP;
			OB->USER = OB_write->USER;
			OB->DATA0 = OB_write->DATA0;
			OB->DATA1 = OB_write->DATA1;
			OB->WRP0 = OB_write->WRP0;
			OB->WRP1 = OB_write->WRP1;
			OB->WRP2 = OB_write->WRP2;
			OB->WRP3 = OB_write->WRP3;
			OB->WRP4 = OB_write->WRP4;
			OB->WRP5 = OB_write->WRP5;
			OB->WRP6 = OB_write->WRP6;
			OB->WRP7 = OB_write->WRP7;

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT); 
    
      if(status != FLASH_TIMEOUT)
      {
        /* if the program operation is completed, disable the OPTPG Bit */
        FLASH->CR &= ~FLASH_CR_OPTPG;
      }
    }
    else 
    {
      if(status != FLASH_TIMEOUT)
      {
        /* Disable the OPTER Bit */
        FLASH->CR &= ~FLASH_CR_OPTER;
      }
    }
	}
	
  /* Return the operation Status */
  return status;
}


/**
  * @brief  Write protects the desired pages
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  WRPR0_en: specifies the address of the pages to be write protected.
  *          This parameter can be:
  *             @arg OB_WRPR0_PAGE0to1..OB_WRPR0_PAGE62to63
  *             @arg OB_WRPR0_ALLPAGES
  * @param  WRPR1_en: specifies the address of the pages to be write protected.
  *          This parameter can be:
  *             @arg OB_WRPR1_PAGE64to65..OB_WRPR1_PAGE126to127
  *             @arg OB_WRPR1_ALLPAGES
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_EnableWRP(uint32_t WRPR0, uint32_t WRPR1)
{
  FLASH_Status status = FLASH_COMPLETE;
	OB_TypeDef OB_tmp;
	
	/* Check the parameters */
  assert_param(IS_OB_WRP(WRPR0));
	assert_param(IS_OB_WRP(WRPR1));
	
	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);  
  if(status == FLASH_COMPLETE)
	{		
		FLASH_OB_Read(&OB_tmp);
		
		WRPR0 = (uint32_t)(~WRPR0);
		WRPR1 = (uint32_t)(~WRPR1);
		
		OB_tmp.WRP0 = (uint16_t)((((~WRPR0)<<8)&0xFF00) | (WRPR0&0x00FF));
		OB_tmp.WRP1 = (uint16_t)((((~(WRPR0>>8))<<8)&0xFF00) | ((WRPR0>>8)&0x00FF));
		OB_tmp.WRP2 = (uint16_t)((((~(WRPR0>>16))<<8)&0xFF00) | ((WRPR0>>16)&0x00FF));
		OB_tmp.WRP3 = (uint16_t)((((~(WRPR0>>24))<<8)&0xFF00) | ((WRPR0>>24)&0x00FF));
		
		OB_tmp.WRP4 = (uint16_t)((((~WRPR1)<<8)&0xFF00) | (WRPR1&0x00FF));
		OB_tmp.WRP5 = (uint16_t)((((~(WRPR1>>8))<<8)&0xFF00) | ((WRPR1>>8)&0x00FF));
		OB_tmp.WRP6 = (uint16_t)((((~(WRPR1>>16))<<8)&0xFF00) | ((WRPR1>>16)&0x00FF));
		OB_tmp.WRP7 = (uint16_t)((((~(WRPR1>>24))<<8)&0xFF00) | ((WRPR1>>24)&0x00FF));
						
		status = FLASH_OB_Write(&OB_tmp);
	}
	
  /* Return the write protection operation Status */
  return status;
}

/**
  * @brief  Enables or disables the read out protection.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  FLASH_ReadProtection_Level: specifies the read protection level. 
  *          This parameter can be:
  *             @arg OB_RDP_Level_0: No protection
  *             @arg OB_RDP_Level_1: Read protection of the memory
  *             @arg OB_RDP_Level_2: Chip protection
  * @note   When enabling OB_RDP level 2 it's no more possible to go back to level 1 or 0
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP)
{
  FLASH_Status status = FLASH_COMPLETE;
	OB_TypeDef OB_tmp;

  /* Check the parameters */
  assert_param(IS_OB_RDP(OB_RDP));
	
	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);  
  if(status == FLASH_COMPLETE)
	{
		FLASH_OB_Read(&OB_tmp);
		
		if(OB_RDP == OB_RDP_Level_0/* || OB_RDP == OB_RDP_Level_2 */)
		{
			OB_tmp.RDP = (uint16_t)((((~OB_RDP)<<8)&0xFF00) | (OB_RDP&0x00FF));
		}
		else
		{
			OB_tmp.RDP = (uint16_t)0xFFFF;
		}
		
		status = FLASH_OB_Write(&OB_tmp);
	}
	
  /* Return the protection operation Status */
  return status;
}

/**
  * @brief  Programs the FLASH User Option Byte: IWDG_SW / RST_STOP / RST_STDBY.
  * @brief  Sets or resets the IWDG_SW option bit.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  OB_IWDG: Selects the IWDG mode
  *          This parameter can be one of the following values:
  *             @arg OB_IWDG_SW: Software IWDG selected
  *             @arg OB_IWDG_HW: Hardware IWDG selected
  * @param  OB_STOP: Reset event when entering STOP mode.
  *          This parameter can be one of the following values:
  *             @arg OB_STOP_NoRST: No reset generated when entering in STOP
  *             @arg OB_STOP_RST: Reset generated when entering in STOP
  * @param  OB_STDBY: Reset event when entering Standby mode.
  *          This parameter can be one of the following values:
  *             @arg OB_STDBY_NoRST: No reset generated when entering in STANDBY
  *             @arg OB_STDBY_RST: Reset generated when entering in STANDBY
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY)
{
  FLASH_Status status = FLASH_COMPLETE;
	OB_TypeDef OB_tmp;
	uint16_t user_tmp; 

  /* Check the parameters */
  assert_param(IS_OB_IWDG_SOURCE(OB_IWDG));
  assert_param(IS_OB_STOP_SOURCE(OB_STOP));
  assert_param(IS_OB_STDBY_SOURCE(OB_STDBY));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
		FLASH_OB_Read(&OB_tmp);
		
		user_tmp = OB_tmp.USER & 0xFF;
		user_tmp &= ~OB_IWDG_SW;
		user_tmp |= OB_IWDG;
		user_tmp &= ~OB_STOP_NoRST;
		user_tmp |= OB_STOP;
		user_tmp &= ~OB_STDBY_NoRST;
		user_tmp |= OB_STDBY;
		
		OB_tmp.USER = (uint16_t)((((~user_tmp)<<8)&0xFF00) | (user_tmp&0x00FF));
		
		status = FLASH_OB_Write(&OB_tmp);
  }
	
  /* Return the Option Byte program Status */
  return status;
}


/**
  * @brief  Sets or resets the BOOT1 option bit.
  * @param  OB_BOOT1: Set or Reset the BOOT1 option bit.
  *          This parameter can be one of the following values:
  *             @arg OB_BOOT1_RESET: BOOT1 option bit reset
  *             @arg OB_BOOT1_SET: BOOT1 option bit set
  * @retval None
  */
FLASH_Status FLASH_OB_BOOTConfig(uint8_t OB_BOOT1)
{
  FLASH_Status status = FLASH_COMPLETE;
	OB_TypeDef OB_tmp;
	uint16_t user_tmp;

  /* Check the parameters */
  assert_param(IS_OB_BOOT1(OB_BOOT1));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {  
		FLASH_OB_Read(&OB_tmp);
		
		user_tmp = OB_tmp.USER & 0xFF;
		user_tmp &= ~OB_BOOT1_SET;
		user_tmp |= OB_BOOT1;
		
		OB_tmp.USER = (uint16_t)((((~user_tmp)<<8)&0xFF00) | (user_tmp&0x00FF));
		
		status = FLASH_OB_Write(&OB_tmp);
  }
	
  /* Return the Option Byte program Status */
  return status;
}

/**
  * @brief  Sets or resets the BOOT0 option bit.
  * @param  OB_BOOT0: Set or Reset the BOOT0 option bit.
  *          This parameter can be one of the following values:
  *             @arg OB_BOOT0_RESET: BOOT0 option bit reset
  *             @arg OB_BOOT0_SET: BOOT0 option bit set
  * @retval None
  */
FLASH_Status FLASH_OB_BOOT0Config(uint8_t OB_BOOT0)
{
  FLASH_Status status = FLASH_COMPLETE;
	OB_TypeDef OB_tmp;
	uint16_t user_tmp;

  /* Check the parameters */
  assert_param(IS_OB_BOOT0(OB_BOOT0));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {  
		FLASH_OB_Read(&OB_tmp);
		
		user_tmp = OB_tmp.USER & 0xFF;
		user_tmp &= ~OB_BOOT0_SET;
		user_tmp |= OB_BOOT0;
		
		OB_tmp.USER = (uint16_t)((((~user_tmp)<<8)&0xFF00) | (user_tmp&0x00FF));
		
		status = FLASH_OB_Write(&OB_tmp);
  }
	
  /* Return the Option Byte program Status */
  return status;
}

/**
  * @brief  Sets or resets the BOOT0SW option bit.  
  * @param  OB_BOOT0SW: Set or Reset the BOOT0_SW option bit.
  *          This parameter can be one of the following values:
  *             @arg OB_BOOT0_HW: BOOT0_SW option bit reset
  *             @arg OB_BOOT0_SW: BOOT0_SW option bit set
  * @retval None
  */
FLASH_Status FLASH_OB_BOOT0SWConfig(uint8_t OB_BOOT0SW)
{
  FLASH_Status status = FLASH_COMPLETE; 
	OB_TypeDef OB_tmp;
	uint16_t user_tmp;
	
  /* Check the parameters */
  assert_param(IS_OB_BOOT0SW(OB_BOOT0SW));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
 if(status == FLASH_COMPLETE)
  {
		FLASH_OB_Read(&OB_tmp);
		
		user_tmp = OB_tmp.USER & 0xFF;
		user_tmp &= ~OB_BOOT0_SW;
		user_tmp |= OB_BOOT0SW;
		
		OB_tmp.USER = (uint16_t)((((~user_tmp)<<8)&0xFF00) | (user_tmp&0x00FF));
		
		status = FLASH_OB_Write(&OB_tmp);
  }
	
  /* Return the Option Byte program Status */
  return status;
}

/**
  * @brief  Sets or resets the analogue monitoring on VDDA Power source.
  * @param  OB_VDDA_ANALOG: Selects the analog monitoring on VDDA Power source.
  *          This parameter can be one of the following values:
  *             @arg OB_VDDA_ANALOG_ON: Analog monitoring on VDDA Power source ON
  *             @arg OB_VDDA_ANALOG_OFF: Analog monitoring on VDDA Power source OFF
  * @retval None
  */
FLASH_Status FLASH_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG)
{
  FLASH_Status status = FLASH_COMPLETE; 

//  /* Check the parameters */
//  assert_param(IS_OB_VDDA_ANALOG(OB_VDDA_ANALOG));

//  /* Wait for last operation to be completed */
//  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
//  
//  if(status == FLASH_COMPLETE)
//  {  
//  }
	
  /* Return the Option Byte program Status */
  return status;
}

/**
  * @brief  Sets or resets the SRAM parity.
  * @param  OB_SRAM_Parity: Set or Reset the SRAM parity enable bit.
  *          This parameter can be one of the following values:
  *             @arg OB_SRAM_PARITY_SET: Set SRAM parity.
  *             @arg OB_SRAM_PARITY_RESET: Reset SRAM parity.
  * @retval None
  */
FLASH_Status FLASH_OB_SRAMParityConfig(uint8_t OB_SRAM_Parity)
{
  FLASH_Status status = FLASH_COMPLETE; 

//  /* Check the parameters */
//  assert_param(IS_OB_SRAM_PARITY(OB_SRAM_Parity));

//  /* Wait for last operation to be completed */
//  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
//  
//  if(status == FLASH_COMPLETE)
//  {  
//  }
	
  /* Return the Option Byte program Status */
  return status;
}

/**
  * @brief  Programs the FLASH User Option Byte: IWDG_SW, BOOT0, BOOT1 and BOOT0SEL.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  OB_USER: Selects all user option bytes
  *          This parameter is a combination of the following values:
  *             @arg OB_IWDG_SW / OB_IWDG_HW: Software / Hardware WDG selected
  *             @arg OB_STOP_NoRST / OB_STOP_RST: No reset / Reset generated when entering in STOP
  *             @arg OB_STDBY_NoRST / OB_STDBY_RST: No reset / Reset generated when entering in STANDBY
  *             @arg OB_BOOT1_RESET / OB_BOOT1_SET: BOOT1 Reset / Set
  *             @arg OB_BOOT0_RESET / OB_BOOT0_SET: BOOT0 Reset / Set
  *             @arg OB_BOOT0_SW / OB_BOOT0_HW: BOOT0 pin disabled / BOOT0 pin bonded with GPIO      
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_WriteUser(uint8_t OB_USER)
{
  FLASH_Status status = FLASH_COMPLETE; 
	OB_TypeDef OB_tmp;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
		FLASH_OB_Read(&OB_tmp);
		
		OB_tmp.USER = (uint16_t)((((~OB_USER)<<8)&0xFF00) | (OB_USER&0x00FF));
		
		status = FLASH_OB_Write(&OB_tmp);
  }    
	
  /* Return the Option Byte program Status */
  return status;

}

/**
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  Address: specifies the address to be programmed.
  *          This parameter can be 0x1FF80004 or 0x1FF80006. 
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_ProgramData(uint32_t Address, uint8_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
	OB_TypeDef OB_tmp;
	
  /* Check the parameters */
  assert_param(IS_OB_DATA_ADDRESS(Address));
	
	/* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  if(status == FLASH_COMPLETE)
  {
		FLASH_OB_Read(&OB_tmp);

		if(Address == OB_DATA_ADDRESS_DATA0)
		{
			OB_tmp.DATA0 = (uint16_t)((((~Data)<<8)&0xFF00) | (Data&0x00FF));
		}
		else if(Address == OB_DATA_ADDRESS_DATA1)
		{
			OB_tmp.DATA1 = (uint16_t)((((~Data)<<8)&0xFF00) | (Data&0x00FF));
		}
		
		status = FLASH_OB_Write(&OB_tmp);
  }
	
  /* Return the Option Byte Data Program Status */
  return status;
}

/**
  * @brief  Returns the FLASH User Option Bytes values.
  * @param  None
  * @retval The FLASH User Option Bytes .
  */
uint8_t FLASH_OB_GetUser(void)
{
  /* Return the User Option Byte */
  return (uint8_t)(FLASH->OBR >> FLASH_OBR_USER_Pos);
}

/**
  * @brief  Returns the FLASH Write Protection Option Bytes value.
  * @param  The FLASH Write Protection Option Bytes value Pointer, value bit: 0 protected, 1 unprotected
  * @retval None
  */
void FLASH_OB_GetWRP(uint32_t *WRPR0, uint32_t *WRPR1)
{
  /* Return the FLASH write protection Register value */
	*WRPR0 = (uint32_t)(FLASH->WRPR0);
	*WRPR1 = (uint32_t)(FLASH->WRPR1);
}

/**
  * @brief  Checks whether the FLASH Read out Protection Status is set or not.
  * @param  None
  * @retval FLASH ReadOut Protection Status(SET or RESET)
  */
FlagStatus FLASH_OB_GetRDP(void)
{
  FlagStatus readstatus = RESET;
  
  if ((uint8_t)(FLASH->OBR & FLASH_OBR_RDP) != OB_RDP_Level_0)
  {
    readstatus = SET;
  }
  else
  {
    readstatus = RESET;
  }
	
  return readstatus;
}

/**
  * @}
  */

/** @defgroup FLASH_Group4 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim   
 ===============================================================================
             ##### Interrupts and flags management functions #####
 ===============================================================================  

@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the specified FLASH interrupts.
  * @param  FLASH_IT: specifies the FLASH interrupt sources to be enabled or 
  *         disabled.
  *          This parameter can be any combination of the following values:
  *             @arg FLASH_IT_EOP: FLASH end of programming Interrupt
  *             @arg FLASH_IT_ERR: FLASH Error Interrupt
  * @retval None 
  */
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FLASH_IT(FLASH_IT)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if(NewState != DISABLE)
  {
    /* Enable the interrupt sources */
    FLASH->CR |= FLASH_IT;
  }
  else
  {
    /* Disable the interrupt sources */
    FLASH->CR &= ~(uint32_t)FLASH_IT;
  }
}

/**
  * @brief  Checks whether the specified FLASH flag is set or not.
  * @param  FLASH_FLAG: specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *             @arg FLASH_FLAG_BSY: FLASH write/erase operations in progress flag 
  *             @arg FLASH_FLAG_PGERR: FLASH Programming error flag flag
  *             @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *             @arg FLASH_FLAG_EOP: FLASH End of Programming flag
  * @retval The new state of FLASH_FLAG (SET or RESET).
  */
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_FLASH_GET_FLAG(FLASH_FLAG));

  if((FLASH->SR & FLASH_FLAG) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the new state of FLASH_FLAG (SET or RESET) */
  return bitstatus; 
}

/**
  * @brief  Clears the FLASH's pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *             @arg FLASH_FLAG_PGERR: FLASH Programming error flag flag
  *             @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *             @arg FLASH_FLAG_EOP: FLASH End of Programming flag
  * @retval None
  */
void FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
  /* Check the parameters */
  assert_param(IS_FLASH_CLEAR_FLAG(FLASH_FLAG));
  
  /* Clear the flags */
  FLASH->SR = FLASH_FLAG;
}

/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_BUSY, FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP or FLASH_COMPLETE.
  */
FLASH_Status FLASH_GetStatus(void)
{
  FLASH_Status FLASHstatus = FLASH_COMPLETE;
  
  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) 
  {
    FLASHstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
    { 
      FLASHstatus = FLASH_ERROR_WRP;
    }
    else 
    {
      if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
      {
        FLASHstatus = FLASH_ERROR_PROGRAM; 
      }
      else
      {
        FLASHstatus = FLASH_COMPLETE;
      }
    }
  }
  /* Return the FLASH Status */
  return FLASHstatus;
}


/**
  * @brief  Waits for a FLASH operation to complete or a TIMEOUT to occur.
  * @param  Timeout: FLASH programming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout)
{ 
  FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the FLASH Status */
  status = FLASH_GetStatus();
  
  /* Wait for a FLASH operation to complete or a TIMEOUT to occur */
  while((status == FLASH_BUSY) && (Timeout != 0x00))
  {
    status = FLASH_GetStatus();
    Timeout--;
  }
  
  if(Timeout == 0x00 )
  {
    status = FLASH_TIMEOUT;
  }
  /* Return the operation status */
  return status;
}

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
