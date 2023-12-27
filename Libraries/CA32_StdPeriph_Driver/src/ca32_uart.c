/**
  ******************************************************************************
  * @file    ca32_uart.c
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Universal asynchronous receiver
  *          transmitter (UART):
  *           + Initialization and Configuration
  *           + Data transfers
  *           + Multi-Processor Communication
  *           + Half-duplex mode
  *           + RS485 mode  
  *           + DMA transfers management
  *           + Interrupts and flags management
  *           
  *  @verbatim
 ===============================================================================
                       ##### How to use this driver #####
 ===============================================================================
    [..]
        (#) Enable peripheral clock using RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE)
            function for UART1 or using RCC_APB1PeriphClockCmd(RCC_APB1Periph_UARTx, ENABLE)
        (#) According to the UART mode, enable the GPIO clocks using 
            RCC_AHBPeriphClockCmd() function. (The I/O can be TX, RX, CTS, 
            or and SCLK). 
        (#) Peripheral's alternate function: 
            (++) Connect the pin to the desired peripherals' Alternate 
                 Function (AF) using GPIO_PinAFConfig() function.
            (++) Configure the desired pin in alternate function by:
                 GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF.
            (++) Select the type, pull-up/pull-down and output speed via 
                 GPIO_PuPd, GPIO_OType and GPIO_Speed members.
            (++) Call GPIO_Init() function.        
        (#) Program the Baud Rate, Word Length , Stop Bit, Parity, Hardware 
            flow control and Mode(Receiver/Transmitter) using the SPI_Init()
            function.  
        (#) Enable the NVIC and the corresponding interrupt using the function 
            UART_ITConfig() if you need to use interrupt mode.   
        (#) Enable the UART using the UART_Cmd() function.   
    [..]
            Refer to Multi-Processor, half-duplex, sub-sections
            for more details.
            
@endverbatim
       
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ca32_uart.h"
#include "ca32_rcc.h"

/** @addtogroup CA32_StdPeriph_Driver
  * @{
  */

/** @defgroup UART 
  * @brief UART driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*!< UART CR1 register clear Mask ((~(uint32_t)0xFFFFE6F3)) */
#define CR1_CLEAR_MASK            ((uint32_t)(UART_CR1_M | UART_CR1_PCE | \
                                              UART_CR1_PS | UART_CR1_TE | \
                                              UART_CR1_RE))

/*!< UART CR2 register clock bits clear Mask ((~(uint32_t)0xFFFFF0FF)) */
#define CR2_CLOCK_CLEAR_MASK      ((uint32_t)(UART_CR2_CLKEN | UART_CR2_CPOL | \
                                              UART_CR2_CPHA | UART_CR2_LBCL))

/*!< UART CR3 register clear Mask ((~(uint32_t)0xFFFFFCFF)) */
#define CR3_CLEAR_MASK            ((uint32_t)(UART_CR3_RTSE | UART_CR3_CTSE))

/*!< UART Interrupts mask */
#define IT_MASK                   ((uint32_t)0x000000FF)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup UART_Private_Functions
  * @{
  */

/** @defgroup UART_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions 
 *
@verbatim   
 ===============================================================================
          ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]
        This subsection provides a set of functions allowing to initialize the UART 
        in asynchronous modes.
        (+) For the asynchronous mode only these parameters can be configured: 
          (++) Baud Rate.
          (++) Word Length.
          (++) Stop Bit.
          (++) Parity: If the parity is enabled, then the MSB bit of the data written
               in the data register is transmitted but is changed by the parity bit.
               Depending on the frame length defined by the M bit (8-bits or 9-bits),
               the possible UART frame formats are as listed in the following table:

   +-------------------------------------------------------------+     
   |   M bit |  PCE bit  |            UART frame                |
   |---------------------|---------------------------------------|             
   |    0    |    0      |    | SB | 8 bit data | STB |          |
   |---------|-----------|---------------------------------------|  
   |    0    |    1      |    | SB | 7 bit data | PB | STB |     |
   |---------|-----------|---------------------------------------|  
   |    1    |    0      |    | SB | 9 bit data | STB |          |
   |---------|-----------|---------------------------------------|  
   |    1    |    1      |    | SB | 8 bit data | PB | STB |     |
   +-------------------------------------------------------------+            

          (++) Hardware flow control.
          (++) Receiver/transmitter modes.
    [..] The UART_Init() function follows the UART  asynchronous configuration 
         procedure(details for the procedure are available in reference manual.
    [..] These parameters can be configured using the UART_ClockInit() function.

@endverbatim
  * @{
  */
  
/**
  * @brief  Deinitializes the UARTx peripheral registers to their default reset values.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @retval None
  */
void UART_DeInit(UART_TypeDef* UARTx)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));

  if (UARTx == UART2)
  {
    RCC_APB2PeriphResetCmd(RCC_APB1Periph_UART2, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB1Periph_UART2, DISABLE);
  }
	
  return;
}

/**
  * @brief  Initializes the UARTx peripheral according to the specified
  *         parameters in the UART_InitStruct .
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @retval None
  */
void UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct)
{
  uint32_t divider = 0, apbclock = 0, tmpreg = 0;
  RCC_ClocksTypeDef RCC_ClocksStatus;
  
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_BAUDRATE(UART_InitStruct->UART_BaudRate));  
  assert_param(IS_UART_WORD_LENGTH(UART_InitStruct->UART_WordLength));
  assert_param(IS_UART_STOPBITS(UART_InitStruct->UART_StopBits));
  assert_param(IS_UART_PARITY(UART_InitStruct->UART_Parity));
  assert_param(IS_UART_MODE(UART_InitStruct->UART_Mode));
  assert_param(IS_UART_HARDWARE_FLOW_CONTROL(UART_InitStruct->UART_HardwareFlowControl));
  
  /* Disable UART */
  UARTx->CR1 &= (uint32_t)~((uint32_t)UART_CR1_UE);
  
  /*---------------------------- UART CR2 Configuration -----------------------*/
  tmpreg = UARTx->CR2;
  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)UART_CR2_STOP);
  
  /* Configure the UART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to UART_StopBits value */
  tmpreg |= (uint32_t)UART_InitStruct->UART_StopBits;
  
  /* Write to UART CR2 */
  UARTx->CR2 = tmpreg;
  
  /*---------------------------- UART CR1 Configuration -----------------------*/
  tmpreg = UARTx->CR1;
  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);
  
  /* Configure the UART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to UART_WordLength value */
  /* Set PCE and PS bits according to UART_Parity value */
  /* Set TE and RE bits according to UART_Mode value */
  tmpreg |= (uint32_t)UART_InitStruct->UART_WordLength | UART_InitStruct->UART_Parity |
    UART_InitStruct->UART_Mode;
  
  /* Write to UART CR1 */
  UARTx->CR1 = tmpreg;
  
  /*---------------------------- UART CR3 Configuration -----------------------*/  
  tmpreg = UARTx->CR3;
  /* Clear CTSE and RTSE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR3_CLEAR_MASK);
  
  /* Configure the UART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to UART_HardwareFlowControl value */
  tmpreg |= UART_InitStruct->UART_HardwareFlowControl;
  
  /* Write to UART CR3 */
  UARTx->CR3 = tmpreg;
  
  /*---------------------------- UART BRR Configuration -----------------------*/
  /* Configure the UART Baud Rate -------------------------------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  
  if(UARTx == UART2)
	{
		apbclock = RCC_ClocksStatus.UART2CLK_Frequency;
	}
  else
  {
    apbclock = RCC_ClocksStatus.PCLK_Frequency;
  }
  
  /* Determine the integer part */
  {
    /* (divider * 10) computing in case Oversampling mode is 8 Samples */
    divider = (uint32_t)((256 * (apbclock/100)) / (UART_InitStruct->UART_BaudRate/100));
    tmpreg  = (uint32_t)((256 * (apbclock/100)) % (UART_InitStruct->UART_BaudRate/100));
  }
  
  /* round the divider : if fractional part i greater than 0.5 increment divider */
  if (tmpreg >=  (UART_InitStruct->UART_BaudRate/100) / 2)
  {
    divider++;
  } 
  
  /* Write to UART BRR */
  UARTx->BRR = (uint32_t)divider;
}

/**
  * @brief  Fills each UART_InitStruct member with its default value.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void UART_StructInit(UART_InitTypeDef* UART_InitStruct)
{
  /* UART_InitStruct members default value */
  UART_InitStruct->UART_BaudRate = 9600;
  UART_InitStruct->UART_WordLength = UART_WordLength_8b;
  UART_InitStruct->UART_StopBits = UART_StopBits_1;
  UART_InitStruct->UART_Parity = UART_Parity_No ;
  UART_InitStruct->UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
  UART_InitStruct->UART_HardwareFlowControl = UART_HardwareFlowControl_None;  
}


/**
  * @brief  Enables or disables the specified UART peripheral.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  NewState: new state of the UARTx peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART_Cmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected UART by setting the UE bit in the CR1 register */
    UARTx->CR1 |= UART_CR1_UE;
  }
  else
  {
    /* Disable the selected UART by clearing the UE bit in the CR1 register */
    UARTx->CR1 &= (uint32_t)~((uint32_t)UART_CR1_UE);
  }
}

/**
  * @brief  Enables or disables the UART's transmitter or receiver.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_Direction: specifies the UART direction.
  *          This parameter can be any combination of the following values:
  *            @arg UART_Mode_Tx: UART Transmitter
  *            @arg UART_Mode_Rx: UART Receiver
  * @param  NewState: new state of the UART transfer direction.
  *          This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
void UART_DirectionModeCmd(UART_TypeDef* UARTx, uint32_t UART_DirectionMode, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_MODE(UART_DirectionMode));
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 

  if (NewState != DISABLE)
  {
    /* Enable the UART's transfer interface by setting the TE and/or RE bits 
       in the UART CR1 register */
    UARTx->CR1 |= UART_DirectionMode;
  }
  else
  {
    /* Disable the UART's transfer interface by clearing the TE and/or RE bits
       in the UART CR3 register */
    UARTx->CR1 &= (uint32_t)~UART_DirectionMode;
  }
}


/**
  * @brief  Enables or disables the UART's most significant bit first 
  *         transmitted/received following the start bit.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  NewState: new state of the UART most significant bit first
  *         transmitted/received following the start bit.
  *          This parameter can be: ENABLE or DISABLE.
  * @note   This function has to be called before calling UART_Cmd() function.  
  * @retval None
  */
void UART_MSBFirstCmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the most significant bit first transmitted/received following the 
       start bit by setting the MSBFIRST bit in the CR2 register */
    UARTx->CR2 |= UART_CR2_MSBFIRST;
  }
  else
  {
    /* Disable the most significant bit first transmitted/received following the 
       start bit by clearing the MSBFIRST bit in the CR2 register */
    UARTx->CR2 &= (uint32_t)~((uint32_t)UART_CR2_MSBFIRST);
  }
}

/**
  * @brief  Enables or disables the binary data inversion.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  NewState: new defined levels for the UART data.
  *          This parameter can be:
  *            @arg ENABLE: Logical data from the data register are send/received in negative
  *                          logic (1=L, 0=H). The parity bit is also inverted.
  *            @arg DISABLE: Logical data from the data register are send/received in positive
  *                          logic (1=H, 0=L) 
  * @note   This function has to be called before calling UART_Cmd() function.  
  * @retval None
  */
void UART_DataInvCmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the binary data inversion feature by setting the DATAINV bit in 
       the CR2 register */
    UARTx->CR2 |= UART_CR2_DATAINV;
  }
  else
  {
    /* Disable the binary data inversion feature by clearing the DATAINV bit in 
       the CR2 register */
    UARTx->CR2 &= (uint32_t)~((uint32_t)UART_CR2_DATAINV);
  }
}

/**
  * @brief  Enables or disables the Pin(s) active level inversion.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_InvPin: specifies the UART pin(s) to invert.
  *          This parameter can be any combination of the following values:
  *            @arg UART_InvPin_Tx: UART Tx pin active level inversion.
  *            @arg UART_InvPin_Rx: UART Rx pin active level inversion.
  * @param  NewState: new active level status for the UART pin(s).
  *          This parameter can be:
  *            @arg ENABLE: pin(s) signal values are inverted (Vdd =0, Gnd =1).
  *            @arg DISABLE: pin(s) signal works using the standard logic levels (Vdd =1, Gnd =0).
  * @note   This function has to be called before calling UART_Cmd() function.  
  * @retval None
  */
void UART_InvPinCmd(UART_TypeDef* UARTx, uint32_t UART_InvPin, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_INVERSTION_PIN(UART_InvPin));  
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 

  if (NewState != DISABLE)
  {
    /* Enable the active level inversion for selected pins by setting the TXINV 
       and/or RXINV bits in the UART CR2 register */
    UARTx->CR2 |= UART_InvPin;
  }
  else
  {
    /* Disable the active level inversion for selected requests by clearing the 
       TXINV and/or RXINV bits in the UART CR2 register */
    UARTx->CR2 &= (uint32_t)~UART_InvPin;
  }
}

/**
  * @brief  Enables or disables the swap Tx/Rx pins.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  NewState: new state of the UARTx TX/RX pins pinout.
  *          This parameter can be:
  *            @arg ENABLE: The TX and RX pins functions are swapped.
  *            @arg DISABLE: TX/RX pins are used as defined in standard pinout
  * @note   This function has to be called before calling UART_Cmd() function.  
  * @retval None
  */
void UART_SWAPPinCmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the SWAP feature by setting the SWAP bit in the CR2 register */
    UARTx->CR2 |= UART_CR2_SWAP;
  }
  else
  {
    /* Disable the SWAP feature by clearing the SWAP bit in the CR2 register */
    UARTx->CR2 &= (uint32_t)~((uint32_t)UART_CR2_SWAP);
  }
}

/**
  * @}
  */


/** @defgroup UART_Group4 Data transfers functions
 *  @brief   Data transfers functions 
 *
@verbatim   
 ===============================================================================
                    ##### Data transfers functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage 
         the UART data transfers.
    [..] During an UART reception, data shifts in least significant bit first 
         through the RX pin. When a transmission is taking place, a write instruction to 
         the UART_TDR register stores the data in the shift register.
    [..] The read access of the UART_RDR register can be done using 
         the UART_ReceiveData() function and returns the RDR value.
         Whereas a write access to the UART_TDR can be done using UART_SendData()
         function and stores the written data into TDR.

@endverbatim
  * @{
  */

/**
  * @brief  Transmits single data through the UARTx peripheral.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  Data: the data to transmit.
  * @retval None
  */
void UART_SendData(UART_TypeDef* UARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_DATA(Data)); 
    
  /* Transmit Data */
  UARTx->TDR = (Data & (uint16_t)0x01FF);
}

/**
  * @brief  Returns the most recent received data by the UARTx peripheral.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @retval The received data.
  */
uint16_t UART_ReceiveData(UART_TypeDef* UARTx)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  
  /* Receive Data */
  return (uint16_t)(UARTx->RDR & (uint16_t)0x01FF);
}

/**
  * @}
  */

/** @defgroup UART_Group5 MultiProcessor Communication functions
 *  @brief   Multi-Processor Communication functions 
 *
@verbatim   
 ===============================================================================
             ##### Multi-Processor Communication functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the UART
         multiprocessor communication.
    [..] For instance one of the UARTs can be the master, its TX output is
         connected to the RX input of the other UART. The others are slaves,
         their respective TX outputs are logically ANDed together and connected 
         to the RX input of the master. UART multiprocessor communication is 
         possible through the following procedure:
         (#) Program the Baud rate, Word length = 9 bits, Stop bits, Parity, 
             Mode transmitter or Mode receiver and hardware flow control values 
             using the UART_Init() function.
         (#) Configures the UART address using the UART_SetAddress() function.
         (#) Configures the wake up methode (UART_WakeUp_IdleLine or 
             UART_WakeUp_AddressMark) using UART_WakeUpConfig() function only 
             for the slaves.
         (#) Enable the UART using the UART_Cmd() function.
         (#) Enter the UART slaves in mute mode using UART_ReceiverWakeUpCmd() 
             function.
    [..] The UART Slave exit from mute mode when receive the wake up condition.

@endverbatim
  * @{
  */

/**
  * @brief  Sets the address of the UART node.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_Address: Indicates the address of the UART node.
  * @retval None
  */
void UART_SetAddress(UART_TypeDef* UARTx, uint8_t UART_Address)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  
  /* Clear the UART address */
  UARTx->CR2 &= (uint32_t)~((uint32_t)UART_CR2_ADD);
  /* Set the UART address node */
  UARTx->CR2 |=((uint32_t)UART_Address << (uint32_t)0x18);
}

/**
  * @brief  Enables or disables the UART's mute mode.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  NewState: new state of the UART mute mode.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART_MuteModeCmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 
  
  if (NewState != DISABLE)
  {
    /* Enable the UART mute mode by setting the MME bit in the CR1 register */
    UARTx->CR1 |= UART_CR1_MME;
  }
  else
  {
    /* Disable the UART mute mode by clearing the MME bit in the CR1 register */
    UARTx->CR1 &= (uint32_t)~((uint32_t)UART_CR1_MME);
  }
}

/**
  * @brief  Selects the UART WakeUp method from mute mode.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_WakeUp: specifies the UART wakeup method.
  *          This parameter can be one of the following values:
  *            @arg UART_WakeUp_IdleLine: WakeUp by an idle line detection
  *            @arg UART_WakeUp_AddressMark: WakeUp by an address mark
  * @retval None
  */
void UART_MuteModeWakeUpConfig(UART_TypeDef* UARTx, uint32_t UART_WakeUp)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_MUTEMODE_WAKEUP(UART_WakeUp));

  UARTx->CR1 &= (uint32_t)~((uint32_t)UART_CR1_WAKE);
  UARTx->CR1 |= UART_WakeUp;
}

/**
  * @brief  Configure the the UART Address detection length.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_AddressLength: specifies the UART address length detection.
  *          This parameter can be one of the following values:
  *            @arg UART_AddressLength_4b: 4-bit address length detection 
  *            @arg UART_AddressLength_7b: 7-bit address length detection 
  * @retval None
  */
void UART_AddressDetectionConfig(UART_TypeDef* UARTx, uint32_t UART_AddressLength)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_ADDRESS_DETECTION(UART_AddressLength));

  UARTx->CR2 &= (uint32_t)~((uint32_t)UART_CR2_ADDM7);
  UARTx->CR2 |= UART_AddressLength;
}

/**
  * @}
  */


/** @defgroup UART_Group7 Halfduplex mode function
 *  @brief   Half-duplex mode function 
 *
@verbatim   
 ===============================================================================
                   ##### Half-duplex mode function #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the UART
         Half-duplex communication.
    [..] The UART can be configured to follow a single-wire half-duplex protocol 
         where the TX and RX lines are internally connected.
    [..] UART Half duplex communication is possible through the following procedure:
         (#) Program the Baud rate, Word length, Stop bits, Parity, Mode transmitter 
             or Mode receiver and hardware flow control values using the UART_Init()
            function.
         (#) Configures the UART address using the UART_SetAddress() function.
         (#) Enable the half duplex mode using UART_HalfDuplexCmd() function.
         (#) Enable the UART using the UART_Cmd() function.
         -@- The RX pin is no longer used.
         -@- In Half-duplex mode the following bits must be kept cleared:
             (+@) CLKEN bits in the UART_CR2 register.

@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the UART's Half Duplex communication.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  NewState: new state of the UART Communication.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART_HalfDuplexCmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the Half-Duplex mode by setting the HDSEL bit in the CR3 register */
    UARTx->CR3 |= UART_CR3_HDSEL;
  }
  else
  {
    /* Disable the Half-Duplex mode by clearing the HDSEL bit in the CR3 register */
    UARTx->CR3 &= (uint32_t)~((uint32_t)UART_CR3_HDSEL);
  }
}

/**
  * @}
  */



/** @defgroup UART_Group10 RS485 mode function
 *  @brief  RS485 mode function 
 *
@verbatim  
 ===============================================================================
                        ##### RS485 mode functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the UART
         RS485 flow control.
    [..] RS485 flow control (Driver enable feature) handling is possible through
         the following procedure:
         (#) Program the Baud rate, Word length = 8 bits, Stop bits, Parity, 
             Transmitter/Receiver modes and hardware flow control values using 
             the UART_Init() function.
         (#) Enable the Driver Enable using the UART_DECmd() function.
         (#) Configures the Driver Enable polarity using the UART_DEPolarityConfig()
             function.
         (#) Configures the Driver Enable assertion time using UART_SetDEAssertionTime() 
             function and deassertion time using the UART_SetDEDeassertionTime()
             function.    
         (#) Enable the UART using the UART_Cmd() function.
      -@-  
       (+@) The assertion and dessertion times are expressed in sample time units (1/8 or 
            1/16 bit time, depending on the oversampling rate).
       
@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the UART's DE functionality.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  NewState: new state of the driver enable mode.
  *          This parameter can be: ENABLE or DISABLE.      
  * @retval None
  */
void UART_DECmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the DE functionality by setting the DEM bit in the CR3 register */
    UARTx->CR3 |= UART_CR3_DEM;
  }
  else
  {
    /* Disable the DE functionality by clearing the DEM bit in the CR3 register */
    UARTx->CR3 &= (uint32_t)~((uint32_t)UART_CR3_DEM);
  }
}

/**
  * @brief  Configures the UART's DE polarity
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_DEPolarity: specifies the DE polarity.
  *          This parameter can be one of the following values:
  *            @arg UART_DEPolarity_Low
  *            @arg UART_DEPolarity_High
  * @retval None
  */
void UART_DEPolarityConfig(UART_TypeDef* UARTx, uint32_t UART_DEPolarity)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_DE_POLARITY(UART_DEPolarity));

  UARTx->CR3 &= (uint32_t)~((uint32_t)UART_CR3_DEP);
  UARTx->CR3 |= UART_DEPolarity;
}

/**
  * @brief  Sets the specified RS485 DE assertion time
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_DEAssertionTime: specifies the time between the activation of
  *         the DE signal and the beginning of the start bit
  * @retval None
  */
void UART_SetDEAssertionTime(UART_TypeDef* UARTx, uint32_t UART_DEAssertionTime)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_DE_ASSERTION_DEASSERTION_TIME(UART_DEAssertionTime)); 

  /* Clear the DE assertion time */
  UARTx->CR1 &= (uint32_t)~((uint32_t)UART_CR1_DEAT);
  /* Set the new value for the DE assertion time */
  UARTx->CR1 |=((uint32_t)UART_DEAssertionTime << (uint32_t)0x15);
}

/**
  * @brief  Sets the specified RS485 DE deassertion time
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_DeassertionTime: specifies the time between the middle of the last 
  *         stop bit in a transmitted message and the de-activation of the DE signal
  * @retval None
  */
void UART_SetDEDeassertionTime(UART_TypeDef* UARTx, uint32_t UART_DEDeassertionTime)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_DE_ASSERTION_DEASSERTION_TIME(UART_DEDeassertionTime)); 

  /* Clear the DE deassertion time */
  UARTx->CR1 &= (uint32_t)~((uint32_t)UART_CR1_DEDT);
  /* Set the new value for the DE deassertion time */
  UARTx->CR1 |=((uint32_t)UART_DEDeassertionTime << (uint32_t)0x10);
}

/**
  * @}
  */

/** @defgroup UART_Group11 DMA transfers management functions
 *  @brief   DMA transfers management functions
 *
@verbatim   
 ===============================================================================
               ##### DMA transfers management functions #####
 ===============================================================================
    [..] This section provides two functions that can be used only in DMA mode.
    [..] In DMA Mode, the UART communication can be managed by 2 DMA Channel 
         requests:
         (#) UART_DMAReq_Tx: specifies the Tx buffer DMA transfer request.
         (#) UART_DMAReq_Rx: specifies the Rx buffer DMA transfer request.
    [..] In this Mode it is advised to use the following function:
         (+) void UART_DMACmd(UART_TypeDef* UARTx, uint16_t UART_DMAReq, 
             FunctionalState NewState).
@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the UART's DMA interface.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  
  * @param  UART_DMAReq: specifies the DMA request.
  *          This parameter can be any combination of the following values:
  *            @arg UART_DMAReq_Tx: UART DMA transmit request
  *            @arg UART_DMAReq_Rx: UART DMA receive request
  * @param  NewState: new state of the DMA Request sources.
  *          This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
void UART_DMACmd(UART_TypeDef* UARTx, uint32_t UART_DMAReq, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_DMAREQ(UART_DMAReq));  
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 

  if (NewState != DISABLE)
  {
    /* Enable the DMA transfer for selected requests by setting the DMAT and/or
       DMAR bits in the UART CR3 register */
    UARTx->CR3 |= UART_DMAReq;
  }
  else
  {
    /* Disable the DMA transfer for selected requests by clearing the DMAT and/or
       DMAR bits in the UART CR3 register */
    UARTx->CR3 &= (uint32_t)~UART_DMAReq;
  }
}


/**
  * @}
  */
/** @defgroup UART_Group12 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions 
 *
@verbatim   
 ===============================================================================
            ##### Interrupts and flags management functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to configure the 
         UART Interrupts sources, Requests and check or clear the flags or pending bits status. 
         The user should identify which mode will be used in his application to 
         manage the communication: Polling mode, Interrupt mode.

 *** Polling Mode ***
 ====================
    [..] In Polling Mode, the SPI communication can be managed by these flags:
         (#) UART_FLAG_REACK: to indicate the status of the Receive Enable 
             acknowledge flag
         (#) UART_FLAG_TEACK: to indicate the status of the Transmit Enable 
             acknowledge flag.
         (#) UART_FLAG_CM: to indicate the status of the Character match flag.
         (#) UART_FLAG_BUSY: to indicate the status of the Busy flag.
         (#) UART_FLAG_RTO: to indicate the status of the Receive time out flag.
         (#) UART_FLAG_nCTSS: to indicate the status of the Inverted nCTS input 
             bit status.
         (#) UART_FLAG_TXE: to indicate the status of the transmit buffer register.
         (#) UART_FLAG_RXNE: to indicate the status of the receive buffer register.
         (#) UART_FLAG_TC: to indicate the status of the transmit operation.
         (#) UART_FLAG_IDLE: to indicate the status of the Idle Line.
         (#) UART_FLAG_CTS: to indicate the status of the nCTS input.
         (#) UART_FLAG_NE: to indicate if a noise error occur.
         (#) UART_FLAG_FE: to indicate if a frame error occur.
         (#) UART_FLAG_PE: to indicate if a parity error occur.
         (#) UART_FLAG_ORE: to indicate if an Overrun error occur.
    [..] In this Mode it is advised to use the following functions:
         (+) FlagStatus UART_GetFlagStatus(UART_TypeDef* UARTx, uint16_t UART_FLAG).
         (+) void UART_ClearFlag(UART_TypeDef* UARTx, uint16_t UART_FLAG).

 *** Interrupt Mode ***
 ======================
    [..] In Interrupt Mode, the UART communication can be managed by 8 interrupt 
         sources and 10 pending bits:
         (+) Pending Bits:
             (##) UART_IT_CM: to indicate the status of Character match interrupt.
             (##) UART_IT_RTO: to indicate the status of Receive time out interrupt.
             (##) UART_IT_CTS: to indicate the status of CTS change interrupt.
             (##) UART_IT_TC: to indicate the status of Transmission complete interrupt.
             (##) UART_IT_IDLE: to indicate the status of IDLE line detected interrupt.
             (##) UART_IT_ORE: to indicate the status of OverRun Error interrupt.
             (##) UART_IT_NE: to indicate the status of Noise Error interrupt.
             (##) UART_IT_FE: to indicate the status of Framing Error interrupt.
             (##) UART_IT_PE: to indicate the status of Parity Error interrupt.  

         (+) Interrupt Source:
             (##) UART_IT_CM: specifies the interrupt source for Character match 
                  interrupt.
             (##) UART_IT_EOB: specifies the interrupt source for End of block
                  interrupt.
             (##) UART_IT_RTO: specifies the interrupt source for Receive time-out
                  interrupt.
             (##) UART_IT_CTS: specifies the interrupt source for CTS change interrupt.
                  detection interrupt.
             (##) UART_IT_TXE: specifies the interrupt source for Tansmit Data 
                  Register empty interrupt.
             (##) UART_IT_TC: specifies the interrupt source for Transmission 
                  complete interrupt.
             (##) UART_IT_RXNE: specifies the interrupt source for Receive Data 
                  register not empty interrupt.
             (##) UART_IT_IDLE: specifies the interrupt source for Idle line 
                  detection interrupt.
             (##) UART_IT_PE: specifies the interrupt source for Parity Error interrupt.
             (##) UART_IT_ERR: specifies the interrupt source for Error interrupt
                  (Frame error, noise error, overrun error)
             -@@- Some parameters are coded in order to use them as interrupt 
                 source or as pending bits.
    [..] In this Mode it is advised to use the following functions:
         (+) void UART_ITConfig(UART_TypeDef* UARTx, uint16_t UART_IT, FunctionalState NewState).
         (+) ITStatus UART_GetITStatus(UART_TypeDef* UARTx, uint16_t UART_IT).
         (+) void UART_ClearITPendingBit(UART_TypeDef* UARTx, uint16_t UART_IT).

@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the specified UART interrupts.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_IT: specifies the UART interrupt sources to be enabled or disabled.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CM:  Character match interrupt.
  *            @arg UART_IT_CTS:  CTS change interrupt.
  *            @arg UART_IT_TXE:  Tansmit Data Register empty interrupt.
  *            @arg UART_IT_TC:  Transmission complete interrupt.
  *            @arg UART_IT_RXNE:  Receive Data register not empty interrupt.
  *            @arg UART_IT_IDLE:  Idle line detection interrupt.
  *            @arg UART_IT_PE:  Parity Error interrupt.
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @param  NewState: new state of the specified UARTx interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART_ITConfig(UART_TypeDef* UARTx, uint32_t UART_IT, FunctionalState NewState)
{
  uint32_t uartreg = 0, itpos = 0, itmask = 0;
  uint32_t uartxbase = 0;
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_CONFIG_IT(UART_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  uartxbase = (uint32_t)UARTx;
  
  /* Get the UART register index */
  uartreg = (((uint16_t)UART_IT) >> 0x08);
  
  /* Get the interrupt position */
  itpos = UART_IT & IT_MASK;
  itmask = (((uint32_t)0x01) << itpos);
  
  if (uartreg == 0x02) /* The IT is in CR2 register */
  {
    uartxbase += 0x04;
  }
  else if (uartreg == 0x03) /* The IT is in CR3 register */
  {
    uartxbase += 0x08;
  }
  else /* The IT is in CR1 register */
  {
  }
  if (NewState != DISABLE)
  {
    *(__IO uint32_t*)uartxbase  |= itmask;
  }
  else
  {
    *(__IO uint32_t*)uartxbase &= ~itmask;
  }
}

/**
  * @brief  Enables the specified UART's Request.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_Request: specifies the UART request.
  *          This parameter can be any combination of the following values:
  *            @arg UART_Request_TXFRQ: Transmit data flush ReQuest
  *            @arg UART_Request_RXFRQ: Receive data flush ReQuest
  *            @arg UART_Request_MMRQ: Mute Mode ReQuest
  * @param  NewState: new state of the DMA interface when reception error occurs.
  *          This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
void UART_RequestCmd(UART_TypeDef* UARTx, uint32_t UART_Request, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_REQUEST(UART_Request));
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 

  if (NewState != DISABLE)
  {
    /* Enable the UART ReQuest by setting the dedicated request bit in the RQR
       register.*/
      UARTx->RQR |= UART_Request;
  }
  else
  {
    /* Disable the UART ReQuest by clearing the dedicated request bit in the RQR
       register.*/
    UARTx->RQR &= (uint32_t)~UART_Request;
  }
}

/**
  * @brief  Enables or disables the UART's Overrun detection.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_OVRDetection: specifies the OVR detection status in case of OVR error.
  *          This parameter can be any combination of the following values:
  *            @arg UART_OVRDetection_Enable: OVR error detection enabled when
  *                                            the UART OVR error is asserted.
  *            @arg UART_OVRDetection_Disable: OVR error detection disabled when
  *                                             the UART OVR error is asserted.
  * @retval None
  */
void UART_OverrunDetectionConfig(UART_TypeDef* UARTx, uint32_t UART_OVRDetection)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_OVRDETECTION(UART_OVRDetection));
  
  /* Clear the OVR detection bit */
  UARTx->CR3 &= (uint32_t)~((uint32_t)UART_CR3_OVRDIS);
  /* Set the new value for the OVR detection bit */
  UARTx->CR3 |= UART_OVRDetection;
}

/**
  * @brief  Checks whether the specified UART flag is set or not.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg UART_FLAG_REACK:  Receive Enable acknowledge flag.
  *            @arg UART_FLAG_TEACK:  Transmit Enable acknowledge flag.
  *            @arg UART_FLAG_CM:  Character match flag.
  *            @arg UART_FLAG_BUSY:  Busy flag.
  *            @arg UART_FLAG_RTO:  Receive time out flag.
  *            @arg UART_FLAG_nCTSS:  Inverted nCTS input bit status.
  *            @arg UART_FLAG_CTS:  CTS Change flag.
  *            @arg UART_FLAG_TXE:  Transmit data register empty flag.
  *            @arg UART_FLAG_TC:  Transmission Complete flag.
  *            @arg UART_FLAG_RXNE:  Receive data register not empty flag.
  *            @arg UART_FLAG_IDLE:  Idle Line detection flag.
  *            @arg UART_FLAG_ORE:  OverRun Error flag.
  *            @arg UART_FLAG_NE:  Noise Error flag.
  *            @arg UART_FLAG_FE:  Framing Error flag.
  *            @arg UART_FLAG_PE:  Parity Error flag.
  * @retval The new state of UART_FLAG (SET or RESET).
  */
FlagStatus UART_GetFlagStatus(UART_TypeDef* UARTx, uint32_t UART_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_FLAG(UART_FLAG));
  
  if ((UARTx->ISR & UART_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the UARTx's pending flags.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_FLAG: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg UART_FLAG_CM:  Character match flag.
  *            @arg UART_FLAG_RTO:  Receive time out flag.
  *            @arg UART_FLAG_CTS:  CTS Change flag.
  *            @arg UART_FLAG_TC:  Transmission Complete flag.
  *            @arg UART_FLAG_IDLE:  IDLE line detected flag.
  *            @arg UART_FLAG_ORE:  OverRun Error flag.
  *            @arg UART_FLAG_NE: Noise Error flag.
  *            @arg UART_FLAG_FE: Framing Error flag.
  *            @arg UART_FLAG_PE:   Parity Errorflag.
  *   
  * @note     RXNE pending bit is cleared by a read to the UART_RDR register 
  *           (UART_ReceiveData()) or by writing 1 to the RXFRQ in the register
  *           UART_RQR (UART_RequestCmd()).
  * @note     TC flag can be also cleared by software sequence: a read operation
  *           to UART_SR register (UART_GetFlagStatus()) followed by a write 
  *           operation to UART_TDR register (UART_SendData()).
  * @note     TXE flag is cleared by a write to the UART_TDR register (UART_SendData())
  *           or by writing 1 to the TXFRQ in the register UART_RQR (UART_RequestCmd()).
  * @note     SBKF flag is cleared by 1 to the SBKRQ in the register UART_RQR
  *           (UART_RequestCmd()).
  * @retval None
  */
void UART_ClearFlag(UART_TypeDef* UARTx, uint32_t UART_FLAG)
{
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_CLEAR_FLAG(UART_FLAG));
     
  UARTx->ICR = UART_FLAG;
}

/**
  * @brief  Checks whether the specified UART interrupt has occurred or not.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_IT: specifies the UART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_RTO:  Receive time out interrupt.
  *            @arg UART_IT_CTS:  CTS change interrupt.
  *            @arg UART_IT_TXE:  Tansmit Data Register empty interrupt.
  *            @arg UART_IT_TC:  Transmission complete interrupt.
  *            @arg UART_IT_RXNE:  Receive Data register not empty interrupt.
  *            @arg UART_IT_IDLE:  Idle line detection interrupt.
  *            @arg UART_IT_ORE:  OverRun Error interrupt.
  *            @arg UART_IT_NE:  Noise Error interrupt.
  *            @arg UART_IT_FE:  Framing Error interrupt.
  *            @arg UART_IT_PE:  Parity Error interrupt.
  * @retval The new state of UART_IT (SET or RESET).
  */
ITStatus UART_GetITStatus(UART_TypeDef* UARTx, uint32_t UART_IT)
{
  uint32_t bitpos = 0, itmask = 0, uartreg = 0;
  ITStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_GET_IT(UART_IT)); 
  
  /* Get the UART register index */
  uartreg = (((uint16_t)UART_IT) >> 0x08);
  /* Get the interrupt position */
  itmask = UART_IT & IT_MASK;
  itmask = (uint32_t)0x01 << itmask;
  
  if (uartreg == 0x01) /* The IT  is in CR1 register */
  {
    itmask &= UARTx->CR1;
  }
  else if (uartreg == 0x02) /* The IT  is in CR2 register */
  {
    itmask &= UARTx->CR2;
  }
  else /* The IT  is in CR3 register */
  {
    itmask &= UARTx->CR3;
  }
  
  bitpos = UART_IT >> 0x10;
  bitpos = (uint32_t)0x01 << bitpos;
  bitpos &= UARTx->ISR;
  if ((itmask != (uint16_t)RESET)&&(bitpos != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  
  return bitstatus;  
}

/**
  * @brief  Clears the UARTx's interrupt pending bits.
  * @param  UARTx: where x can be 1 to select the UART peripheral.
  * @param  UART_IT: specifies the interrupt pending bit to clear.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CM:  Character match interrupt.
  *            @arg UART_IT_RTO:  Receive time out interrupt.
  *            @arg UART_IT_CTS:  CTS change interrupt.
  *            @arg UART_IT_TC:  Transmission complete interrupt.
  *            @arg UART_IT_IDLE:  IDLE line detected interrupt.
  *            @arg UART_IT_ORE:  OverRun Error interrupt.
  *            @arg UART_IT_NE:  Noise Error interrupt.
  *            @arg UART_IT_FE:  Framing Error interrupt.
  *            @arg UART_IT_PE:  Parity Error interrupt.
  *
  * @note     RXNE pending bit is cleared by a read to the UART_RDR register 
  *           (UART_ReceiveData()) or by writing 1 to the RXFRQ in the register 
  *           UART_RQR (UART_RequestCmd()).
  * @note     TC pending bit can be also cleared by software sequence: a read 
  *           operation to UART_SR register (UART_GetITStatus()) followed by  
  *           a write operation to UART_TDR register (UART_SendData()).
  * @note     TXE pending bit is cleared by a write to the UART_TDR register 
  *           (UART_SendData()) or by writing 1 to the TXFRQ in the register 
  *           UART_RQR (UART_RequestCmd()).
  * @retval None
  */
void UART_ClearITPendingBit(UART_TypeDef* UARTx, uint32_t UART_IT)
{
  uint32_t bitpos = 0, itmask = 0;
  /* Check the parameters */
  assert_param(IS_UART_ALL_PERIPH(UARTx));
  assert_param(IS_UART_CLEAR_IT(UART_IT)); 
  
  bitpos = UART_IT >> 0x10;
  itmask = ((uint32_t)0x01 << (uint32_t)bitpos);
  UARTx->ICR = (uint32_t)itmask;
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
