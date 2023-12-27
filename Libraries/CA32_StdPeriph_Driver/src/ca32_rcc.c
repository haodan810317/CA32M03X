/**
  ******************************************************************************
  * @file    ca32_rcc.c
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Reset and clock control (RCC) peripheral:
  *           + System, AHB and APB busses clocks configuration
  *           + Peripheral clocks configuration
  *           + flags management
  *
	* @version V1.0.0
  * @date    9-Feb-2023
  *  @verbatim
 ===============================================================================
                        ##### RCC specific features #####
 ===============================================================================
    [..] After reset the device is running from HSI (16 MHz) with Flash 0 WS, 
         all peripherals are off except internal SRAM, Flash and SWD.
         (#) There is 1/2 prescaler on High speed (AHB) and no prescaler on Low speed (APB) busses;
             all peripherals mapped on these busses are running at HSI/2 speed.
         (#) The clock for all peripherals is switched off, except the SRAM and FLASH.
         (#) All GPIOs are in input floating state, except the SWD pins which
             are assigned to be used for debug purpose.
    [..] Once the device started from reset, the user application has to:
         (#) Configure the System clock frequency and Flash settings
         (#) Configure the AHB and APB busses prescalers
         (#) Enable the clock for the peripheral(s) to be used
         (#) Configure the clock source(s) for peripherals which clocks are not
             derived from the System clock (ADC)
  @endverbatim
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "ca32_rcc.h"

/** @addtogroup CA32_StdPeriph_Driver
  * @{
  */

/** @defgroup RCC 
  * @brief RCC driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* ---------------------- RCC registers mask -------------------------------- */
/* RCC Flag Mask */

#define PERIPH_BB_BASE        PERIPH_BASE

/* ------------ RCC registers bit address in the alias region ----------- */
#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)

/* --- CR Register ---*/

/* Alias word address of HSION bit */
#define CR_OFFSET                 (RCC_OFFSET + 0x00)
#define HSION_BitNumber           0x00
#define CR_HSION_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (HSION_BitNumber * 4))


/* --- CSR Register ---*/

/* Alias word address of LSION bit */
#define CSR_OFFSET                (RCC_OFFSET + 0x24)
#define LSION_BitNumber           0x00
#define CSR_LSION_BB              (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (LSION_BitNumber * 4))


/* ---------------------- RCC registers mask -------------------------------- */
/* RCC Flag Mask */
#define FLAG_MASK                 ((uint8_t)0x1F)

/* CR register byte 3 (Bits[23:16]) base address */
#define CR_BYTE3_ADDRESS          ((uint32_t)0x40023802)

/* ICSCR register byte 4 (Bits[31:24]) base address */
#define ICSCR_BYTE4_ADDRESS       ((uint32_t)0x40023807)

/* CFGR register byte 3 (Bits[23:16]) base address */
#define CFGR_BYTE3_ADDRESS        ((uint32_t)0x4002380A)

/* CFGR register byte 4 (Bits[31:24]) base address */
#define CFGR_BYTE4_ADDRESS        ((uint32_t)0x4002380B)

/* CIR register byte 2 (Bits[15:8]) base address */
#define CIR_BYTE2_ADDRESS         ((uint32_t)0x4002380D)

/* CIR register byte 3 (Bits[23:16]) base address */
#define CIR_BYTE3_ADDRESS         ((uint32_t)0x4002380E)

/* CSR register byte 2 (Bits[15:8]) base address */
#define CSR_BYTE2_ADDRESS         ((uint32_t)0x40023835)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t ADCPrescTable[4] = {1, 2, 3, 0}; 

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup RCC_Private_Functions
  * @{
  */

/** @defgroup Initialization and Calibration functions
 *  @brief   Initialization and Calibration functions
 *
@verbatim

 ===============================================================================
 ##### Internal clocks configuration functions #####
 ===============================================================================
    [..] This section provides functions allowing to configure the internal clocks.
         (#) HSI (high-speed internal), 56 MHz factory-trimmed RC used directly.
             The HSI clock can be used also to clock the UART, I2C peripherals.
@endverbatim
  * @{
  */

/**
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  * @note      HSI ON and used as system clock source 
  * @note      AHB prescaler set to 2, APB prescaler set to 1.
  * @param  None
  * @retval None
  */
void RCC_DeInit(void)
{

  RCC->CR = 0x00001201;
	RCC->CFGR = 0x01009880;
}

/**
  * @brief  Selects the clock source to output on MCO pin.
  * @note   GPIO should be configured in alternate function mode.   
  * @param  RCC_MCODiv: specifies the MCO prescaler.
  *   This parameter can be one of the following values: 
  *     @arg RCC_CFGR_MCOPRE_NONE: no division applied to MCO clock 
  *     @arg RCC_CFGR_MCOPRE_LSI: LSI applied to MCO clock
  *     @arg RCC_CFGR_MCOPRE_DIV4: division by 4 applied to MCO clock
  *     @arg RCC_CFGR_MCOPRE_DIV8: division by 8 applied to MCO clock
  *     @arg RCC_CFGR_MCOPRE_DIV16: division by 16 applied to MCO clock
  *     @arg RCC_CFGR_MCOPRE_DIV32: division by 32 applied to MCO clock             
  *     @arg RCC_CFGR_MCOPRE_DIV64: division by 64 applied to MCO clock
  *     @arg RCC_CFGR_MCOPRE_DIV128: division by 128 applied to MCO clock
  * @retval None
  */
void RCC_MCOConfig(uint32_t RCC_MCODiv)
{
  uint32_t tmpreg = 0;
  
  tmpreg = RCC->CFGR;
  
  /* Clear HPRE[21 : 23] bits */
  tmpreg &= ~RCC_CFGR_MCOPRE;
  
  /* Set MCODIV bits according to RCC_SYSCLK value */
  tmpreg |= RCC_MCODiv;
  
  /* Store the new value */
  RCC->CFGR = tmpreg;
}


/**
  * @brief  Adjusts the Internal High Speed oscillator (HSI) calibration value.
  * @note   The calibration is used to compensate for the variations in voltage
  *         and temperature that influence the frequency of the internal HSI RC.
  * @param  HSICalibrationValue: specifies the HSI calibration trimming value.
  *          This parameter must be a number between 0 and 0x1F.
  * @retval None
  */
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_RCC_HSI_CALIBRATION_VALUE(HSICalibrationValue));
  
  tmpreg = RCC->CR;
  
  /* Clear HSITRIM[4:0] bits */
  tmpreg &= ~RCC_CR_HSICAL;
  
  /* Set the HSITRIM[4:0] bits according to HSICalibrationValue value */
  tmpreg |= (uint32_t)HSICalibrationValue << RCC_CR_HSICAL_Pos;

  /* Store the new value */
  RCC->CR = tmpreg;
}

/**
  * @}
  */ 

/** @defgroup RCC_Group2 System AHB and APB busses clocks configuration functions
 *  @brief   System, AHB and APB busses clocks configuration functions
 *
@verbatim
 ===============================================================================
     ##### System, AHB and APB busses clocks configuration functions #####
 ===============================================================================

    [..] This section provide functions allowing to configure the System, AHB and 
         APB busses clocks.
         (#) Several clock sources can be used to drive the System clock (SYSCLK): HSI
             The AHB clock (HCLK) is derived from System clock through configurable prescaler
             and used to clock the CPU, memory and peripherals mapped on AHB bus (DMA and GPIO).
             and APB (PCLK) clocks are derived from AHB clock through 
             configurable prescalers and used to clock the peripherals mapped on these busses.
             You can use "RCC_GetClocksFreq()" function to retrieve the frequencies of these clocks.

         -@- All the peripheral clocks are derived from the System clock (SYSCLK) except:
             (+@) The ADC clock which is derived from HSI.
             (+@) The I2C clock which is derived from HSI or system clock (SYSCLK).
       
         (#) The maximum frequency of the SYSCLK, HCLK and PCLK is 56 MHz.
             Depending on the maximum frequency, the FLASH wait states (WS) should be 
             adapted accordingly:
        +--------------------------------------------- +
        |  Wait states  |   HCLK clock frequency (MHz) |
        |---------------|------------------------------|
        |0WS(1CPU cycle)|       0 < HCLK <= 28         |
        |---------------|------------------------------|
        |1WS(2CPU cycle)|       28 < HCLK <= 56        |
        +----------------------------------------------+

         (#) After reset, the System clock source is the HSI/2 (28 MHz).
  
    [..] It is recommended to use the following software sequences to tune the number
         of wait states needed to access the Flash memory with the CPU frequency (HCLK).
         (+) Increasing the CPU frequency
         (++) Program Flash WS to 1, using "FLASH_SetLatency(FLASH_Latency_1)" function
         (++) Check that the new number of WS is taken into account by reading FLASH_ACR
         (++) Modify the CPU clock source, using "RCC_SYSCLKConfig()" function
         (++) If needed, modify the CPU clock prescaler by using "RCC_HCLKConfig()" function
         (++) Check that the new CPU clock source is taken into account by reading 
              the clock source status, using "RCC_GetSYSCLKSource()" function 
         (+) Decreasing the CPU frequency
         (++) Modify the CPU clock source, using "RCC_SYSCLKConfig()" function
         (++) If needed, modify the CPU clock prescaler by using "RCC_HCLKConfig()" function
         (++) Check that the new CPU clock source is taken into account by reading 
              the clock source status, using "RCC_GetSYSCLKSource()" function
         (++) Program the new number of WS, using "FLASH_SetLatency()" function
         (++) Check that the new number of WS is taken into account by reading FLASH_ACR

@endverbatim
  * @{
  */


/**
  * @brief  Configures the AHB clock (HCLK).
  * @param  RCC_SYSCLK: defines the AHB clock divider. This clock is derived from 
  *         the system clock (SYSCLK).
  *          This parameter can be one of the following values:
  *            @arg RCC_SYSCLK_Div1:   AHB clock = SYSCLK
  *            @arg RCC_SYSCLK_Div2:   AHB clock = SYSCLK/2
  *            @arg RCC_SYSCLK_Div4:   AHB clock = SYSCLK/4
  *            @arg RCC_SYSCLK_Div8:   AHB clock = SYSCLK/8
  *            @arg RCC_SYSCLK_Div16:  AHB clock = SYSCLK/16
  *            @arg RCC_SYSCLK_Div64:  AHB clock = SYSCLK/64
  *            @arg RCC_SYSCLK_Div128: AHB clock = SYSCLK/128
  *            @arg RCC_SYSCLK_Div256: AHB clock = SYSCLK/256
  *            @arg RCC_SYSCLK_Div512: AHB clock = SYSCLK/512
  * @retval None
  */
void RCC_HCLKConfig(uint32_t RCC_SYSCLK)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_RCC_HCLK(RCC_SYSCLK));
  
  tmpreg = RCC->CFGR;
  
  /* Clear HPRE[3:0] bits */
  tmpreg &= ~RCC_CFGR_HPRE;
  
  /* Set HPRE[3:0] bits according to RCC_SYSCLK value */
  tmpreg |= RCC_SYSCLK;
  
  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/**
  * @brief  Configures the APB clock (PCLK).
  * @param  RCC_HCLK: defines the APB clock divider. This clock is derived from 
  *         the AHB clock (HCLK).
  *          This parameter can be one of the following values:
  *            @arg RCC_HCLK_Div1: APB clock = HCLK
  *            @arg RCC_HCLK_Div2: APB clock = HCLK/2
  *            @arg RCC_HCLK_Div4: APB clock = HCLK/4
  *            @arg RCC_HCLK_Div8: APB clock = HCLK/8
  *            @arg RCC_HCLK_Div16: APB clock = HCLK/16
  * @retval None
  */
void RCC_PCLKConfig(uint32_t RCC_HCLK)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_RCC_PCLK(RCC_HCLK));
  
  tmpreg = RCC->CFGR;
  
  /* Clear PPRE[2:0] bits */
  tmpreg &= ~RCC_CFGR_PPRE;
  
  /* Set PPRE[2:0] bits according to RCC_HCLK value */
  tmpreg |= RCC_HCLK;
  
  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/**
  * @brief  Configures the ADC clock (ADCCLK) or disable it.
  * @note   This function is obsolete.
  *         For proper ADC clock selection, refer to ADC_ClockModeConfig() in the ADC driver
  * @param  RCC_ADCCLK: defines the ADC clock source. This clock is derived 
  *         from the HSI clock
  *          This parameter can be one of the following values:
  *             @arg RCC_ADCCLK_Disable: ADC clock disable
  *             @arg RCC_ADCCLK_HSI_Div2: ADC clock = HCLK/2 = 28 MHz
  *             @arg RCC_ADCCLK_HSI_Div3: ADC clock = HCLK/3 = 18.7 MHz
  *             @arg RCC_ADCCLK_HSI_Div4: ADC clock = HCLK/4 = 14 MHz
  *             @arg RCC_ADCCLK_HSI_Div5: ADC clock = HCLK/5 = 11.2 MHz
  *             @arg RCC_ADCCLK_HSI_Div6: ADC clock = HCLK/6 = 9.3 MHz
  *             @arg RCC_ADCCLK_HSI_Div7: ADC clock = HCLK/7 = 8 MHz
  *             @arg RCC_ADCCLK_HSI_Div8: ADC clock = HCLK/8 = 7 MHz
  *             @arg RCC_ADCCLK_HSI_Div9: ADC clock = HCLK/9 = 6.2 MHz
  *             @arg RCC_ADCCLK_HSI_Div10: ADC clock = HCLK/10 = 5.6 MHz
  *             @arg RCC_ADCCLK_HSI_Div11: ADC clock = HCLK/11 = 5.09 MHz
  *             @arg RCC_ADCCLK_HSI_Div12: ADC clock = HCLK/12 = 4.6 MHz
  *             @arg RCC_ADCCLK_HSI_Div13: ADC clock = HCLK/13 = 4.3 MHz
  *             @arg RCC_ADCCLK_HSI_Div14: ADC clock = HCLK/14 = 4 MHz
  * @retval None
  */
//void RCC_ADCCLKConfig(uint32_t RCC_ADCCLK)
//{ 
//  /* Check the parameters */
//  assert_param(IS_RCC_ADCCLK(RCC_ADCCLK));

//  /* Clear ADCPRE bit */
//  RCC->CFGR &= ~RCC_CFGR_ADCPRE;

//  /* Set ADCPRE bits according to RCC_PCLK value */
//  RCC->CFGR |= RCC_ADCCLK;

//	return;
//}


/**
  * @brief  Enables or Disables Kclock of ADC.
  * @param  NewState: new state of the specified clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
//void RCC_ADCKClockCmd(FunctionalState NewState)
//{
//	/* Check the parameters */
//  assert_param(IS_FUNCTIONAL_STATE(NewState));
//	
//	if (NewState != DISABLE)
//  {
//		RCC->CFGR |= RCC_CFGR_ADCKEN;
//	}
//	else
//	{
//		RCC->CFGR &= ~RCC_CFGR_ADCKEN;
//	}
//	
//	return;
//}

/**
  * @brief  configures the ADC Kclock.
  * @param  RCC_ADCKCLK: defines the ADC clock source. This clock is derived 
  *         from the HSI clock
            RCC_CFGR_ADCKPRE_DIV2 adc kclk = HSI / 2
            RCC_CFGR_ADCKPRE_DIV4 adc kclk = HSI / 4
            RCC_CFGR_ADCKPRE_DIV8 adc kclk = HSI / 8
            RCC_CFGR_ADCKPRE_NONE adc kclk = HSI
  * @retval None
  */
void RCC_ADCKClockConfig(uint32_t RCC_ADCKCLK)
{
  /* Check the parameters */
  assert_param(IS_RCC_ADCKCLK(RCC_ADCKCLK));

  /* Clear ADCPRE bit */
  RCC->CFGR &= ~RCC_CFGR_ADCKPRE;
	
  /* Set ADCPRE bits according to RCC_PCLK value */
  RCC->CFGR |= RCC_ADCKCLK;
	
	return;
}

/**
  * @brief  Returns the frequencies of the System, AHB and APB busses clocks.
  * @note    The frequency returned by this function is not the real frequency
  *           in the chip. It is calculated based on the predefined constant and
  *           the source selected by RCC_SYSCLKConfig():
  *                                              
  * @note     If SYSCLK source is HSI, function returns constant HSI_VALUE(*)
  *               
  * @note     (*) HSI_VALUE is a constant defined in ca32.h file (default value
  *               56 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature, refer to RCC_AdjustHSICalibrationValue().   
  *    
  * @param  RCC_Clocks: pointer to a RCC_ClocksTypeDef structure which will hold 
  *         the clocks frequencies. 
  *     
  * @note   This function can be used by the user application to compute the 
  *         baudrate for the communication peripherals or configure other parameters.
  * @note   Each time SYSCLK, HCLK and/or PCLK clock changes, this function
  *         must be called to update the structure's field. Otherwise, any
  *         configuration based on this function will be incorrect.
  *    
  * @retval None
  */
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
  uint32_t tmp = 0, pllmul = 0, plldiv = 0, pllvco = 0, pllclk = 0, presc = 0;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;
  
  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
      break;
    case 0x01:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmul = (RCC->PLLR & RCC_PLLR_M) >> RCC_PLLR_M_Pos;
      plldiv = (RCC->PLLR & RCC_PLLR_N) >> RCC_PLLR_N_Pos;
      pllvco = (RCC->PLLR & RCC_PLLR_OD) >> RCC_PLLR_OD_Pos;
      /* HSI oscillator clock selected as PLL clock entry */
      pllclk = (((HSI_VALUE) * pllmul) / plldiv);
      pllclk >>= PLLVcoTable[pllvco];
    
      RCC_Clocks->SYSCLK_Frequency = pllclk;
      break;
    default: /* HSI used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
      break;
  }

  /* Compute HCLK, PCLK clocks frequencies -----------------------------------*/
  /* Get HCLK prescaler */
  tmp = RCC->CFGR & RCC_CFGR_HPRE;
  tmp = tmp >> 4;
  presc = APBAHBPrescTable[tmp]; 
  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;

  /* Get PCLK prescaler */
  tmp = RCC->CFGR & RCC_CFGR_PPRE;
  tmp = tmp >> 8;
  presc = APBAHBPrescTable[tmp];
  /* PCLK clock frequency */
  RCC_Clocks->PCLK_Frequency = RCC_Clocks->HCLK_Frequency >> presc;

  /* Get ADCCLK prescaler */  
  tmp = RCC->CFGR & RCC_CFGR_ADCKPRE;
  tmp = tmp >> RCC_CFGR_ADCKPRE_Pos;
  presc = ADCPrescTable[tmp];
  /* ADCCLK clock frequency */
  if(RCC->CFGR & RCC_CFGR_ADCSWS)  
    RCC_Clocks->ADCCLK_Frequency = pllclk >> presc;
  else
    RCC_Clocks->ADCCLK_Frequency = HSI_VALUE >> presc;  

  /* I2C Clock is PCLK */
  RCC_Clocks->I2C1CLK_Frequency = RCC_Clocks->PCLK_Frequency;
	
	/* UART1 Clock is PCLK */
	RCC_Clocks->USART1CLK_Frequency = RCC_Clocks->PCLK_Frequency;

	/* UART2 Clock is PCLK */
	RCC_Clocks->UART2CLK_Frequency = RCC_Clocks->PCLK_Frequency;
	
	return;
}

/**
  * @}
  */ 

/** @defgroup RCC_Group3 Peripheral clocks configuration functions
 *  @brief   Peripheral clocks configuration functions 
 *
@verbatim
 ===============================================================================
             #####Peripheral clocks configuration functions #####
 ===============================================================================  

    [..] This section provide functions allowing to configure the Peripheral clocks. 
         (#) To reset the peripherals configuration (to the default state after device reset)
             you can use RCC_AHBPeriphResetCmd(), RCC_APB2PeriphResetCmd() and 
             RCC_APB1PeriphResetCmd() functions.
@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the AHB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.    
  * @param  RCC_AHBPeriph: specifies the AHB peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *             @arg RCC_AHBPeriph_GPIOA: GPIOA clock
  *             @arg RCC_AHBPeriph_GPIOB: GPIOB clock
  *             @arg RCC_AHBPeriph_GPIOB: GPIOB clock  
  *             @arg RCC_AHBPeriph_SVPWM: SVPEM clock
  *             @arg RCC_AHBPeriph_MATH: MATH clock  
  *             @arg RCC_AHBPeriph_CORDIC: CORDIC clock  
  *             @arg RCC_AHBPeriph_CRC: CRC clock  
  *             @arg RCC_AHBPeriph_FLITF: (has effect only when the Flash memory is in power down mode)  
  *             @arg RCC_AHBPeriph_DMA1:  DMA1 clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_AHB_PERIPH(RCC_AHBPeriph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    RCC->AHBENR |= RCC_AHBPeriph;
  }
  else
  {
    RCC->AHBENR &= ~RCC_AHBPeriph;
  }
}


/**
  * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *             @arg RCC_APB2Periph_SYSCFG: SYSCFG clock
  *             @arg RCC_APB2Periph_ADC1:   ADC1 clock
  *             @arg RCC_APB2Periph_USART1:   UART1 clock
  *             @arg RCC_APB2Periph_TIM20:  TIM20 clock
  *             @arg RCC_APB2Periph_SPI:  SPI clock
  *             @arg RCC_APB2Periph_ANALOG:  ANALOG clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB2ENR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2ENR &= ~RCC_APB2Periph;
  }
}


/**
  * @brief  Enables or disables the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  RCC_APB1Periph: specifies the APB1 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *           @arg RCC_APB1Periph_TIM2:   TIM2 clock
  *           @arg RCC_APB1Periph_TIM3:   TIM3 clock
  *           @arg RCC_APB1Periph_TIM6:   TIM6 clock
  *           @arg RCC_APB1Periph_TIM7:   TIM7 clock
  *           @arg RCC_APB1Periph_WWDG:   WWDG clock
  *           @arg RCC_APB1Periph_I2C1:   I2C1 clock
  *           @arg RCC_APB1Periph_I2C2:   I2C2 clock
  *           @arg RCC_APB1Periph_PWR:    PWR clock
  *           @arg RCC_APB1Periph_UART2:  UART2 clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB1ENR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1ENR &= ~RCC_APB1Periph;
  }
  return;
}

/**
  * @brief  Forces or releases AHB peripheral reset.
  * @param  RCC_AHBPeriph: specifies the AHB peripheral to reset.
  *          This parameter can be any combination of the following values:
  *             @arg RCC_AHBPeriph_GPIOA: GPIOA clock
  *             @arg RCC_AHBPeriph_GPIOB: GPIOB clock
  *             @arg RCC_AHBPeriph_GPIOB: GPIOC clock
  *             @arg RCC_AHBPeriph_SVPWM: SVPEM clock
  *             @arg RCC_AHBPeriph_MATH: MATH clock  
  *             @arg RCC_AHBPeriph_CORDIC: CORDIC clock  
  *             @arg RCC_AHBPeriph_CRC: CRC clock  
  * @param  NewState: new state of the specified peripheral reset.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_AHB_RST_PERIPH(RCC_AHBPeriph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->AHBRSTR |= RCC_AHBPeriph;
  }
  else
  {
    RCC->AHBRSTR &= ~RCC_AHBPeriph;
  }
}

/**
  * @brief  Forces or releases High Speed APB (APB2) peripheral reset.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to reset.
  *          This parameter can be any combination of the following values:
  *             @arg RCC_APB2Periph_SYSCFG: SYSCFG clock
  *             @arg RCC_APB2Periph_ADC1:   ADC1 clock
  *             @arg RCC_APB2Periph_USART1: UART1 clock
  *             @arg RCC_APB2Periph_TIM20:  TIM20 clock
  *             @arg RCC_APB2Periph_SPI:    SPI clock
  *             @arg RCC_APB2Periph_ANALOG: ANALOG clock
  * @param  NewState: new state of the specified peripheral reset.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB2RSTR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2RSTR &= ~RCC_APB2Periph;
  }
}

/**
  * @brief  Forces or releases Low Speed APB (APB1) peripheral reset.
  * @param  RCC_APB1Periph: specifies the APB1 peripheral to reset.
  *          This parameter can be any combination of the following values:
  *           @arg RCC_APB1Periph_TIM2:   TIM2 clock
  *           @arg RCC_APB1Periph_TIM3:   TIM3 clock
  *           @arg RCC_APB1Periph_TIM6:   TIM6 clock
  *           @arg RCC_APB1Periph_TIM7:   TIM7 clock
  *           @arg RCC_APB1Periph_WWDG:   WWDG clock
  *           @arg RCC_APB1Periph_I2C1:   I2C1 clock
  *           @arg RCC_APB1Periph_I2C2:   I2C2 clock
  *           @arg RCC_APB1Periph_PWR:    PWR clock
  *           @arg RCC_APB1Periph_UART2:  UART2 clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB1RSTR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1RSTR &= ~RCC_APB1Periph;
  }
}

/**
  * @}
  */

/** @defgroup RCC_Group4 flags management functions
 *  @brief   flags management functions 
 *
@verbatim
 ===============================================================================
             ##### flags management functions #####
 ===============================================================================
@endverbatim
  * @{
  */


/**
  * @brief  Checks whether the specified RCC flag is set or not.
  * @param  RCC_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *             @arg RCC_FLAG_OBLRST: Option Byte Loader (OBL) reset 
  *             @arg RCC_FLAG_PINRST: Pin reset
  *             @arg RCC_FLAG_PORRST: POR/PDR reset
  *             @arg RCC_FLAG_SFTRST: Software reset
  *             @arg RCC_FLAG_WWDGRST: Window Watchdog reset
	*             @arg RCC_FLAG_IWDGRST: Independed Watchdog reset
  * @retval The new state of RCC_FLAG (SET or RESET).
  */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
  uint32_t tmp = 0;
  uint32_t statusreg = 0;
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_RCC_FLAG(RCC_FLAG));

  /* Get the RCC register index */
  tmp = RCC_FLAG >> 5;

  if (tmp == 0)               /* The flag to check is in CR register */
  {
    statusreg = RCC->CR;
  }
	else if (tmp == 2)          /* The flag to check is in CSR register */
  {
    statusreg = RCC->CSR;
  }   

  /* Get the flag position */
  tmp = RCC_FLAG & FLAG_MASK;

  if ((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the flag status */
  return bitstatus;
	
}

/**
  * @brief  Clears the RCC reset flags.
  *         The reset flags are: RCC_FLAG_OBLRST, RCC_FLAG_PINRST,
  *         RCC_FLAG_PORRST, RCC_FLAG_SFTRST, RCC_FLAG_WWDGRST.
  * @param  None
  * @retval None
  */
void RCC_ClearFlag(void)
{
  /* Set RMVF bit to clear the reset flags */
  RCC->CSR |= RCC_CSR_RMVF;
}

/**
  * @brief  Enables or disables the Internal Low Speed oscillator (LSI).  
  * @note     After enabling the LSI, the application software should wait on 
  *           LSIRDY flag to be set indicating that LSI clock is stable and can
  *           be used to clock the IWDG and/or the RTC.
  * @note     LSI can not be disabled if the IWDG is running.  
  * @param  NewState: new state of the LSI.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
  *         clock cycles. 
  * @retval None
  */
void RCC_LSICmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  //*(__IO uint32_t *) CSR_LSION_BB = (uint32_t)NewState;
	if(NewState == ENABLE)
	{
		RCC->CSR |= RCC_CSR_LSION;
	}
	else
	{
		RCC->CSR &= ~RCC_CSR_LSION;
	}
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
