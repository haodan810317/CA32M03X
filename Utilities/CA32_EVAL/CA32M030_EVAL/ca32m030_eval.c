/**
  ******************************************************************************
  * @file    ca32m030_eval.c
  * @brief   This file provides firmware functions to manage Leds, push-buttons, 
  *          COM ports, available on CA32M030-EVAL evaluation board from ThingsChip.
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "ca32m030_eval.h"

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup CA32_EVAL
  * @{
  */ 

/** @addtogroup CA32M030_EVAL
  * @{
  */   
    
/** @defgroup CA32M030_EVAL_LOW_LEVEL 
  * @brief This file provides firmware functions to manage Leds, push-buttons, 
  *        COM ports available on CA32M030-EVAL evaluation board from ThingsChip.
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {KEY1_BUTTON_GPIO_PORT, KEY2_BUTTON_GPIO_PORT}; 

const uint16_t BUTTON_PIN[BUTTONn] = {KEY1_BUTTON_PIN, KEY2_BUTTON_PIN}; 

const uint32_t BUTTON_CLK[BUTTONn] = {KEY1_BUTTON_GPIO_CLK, KEY2_BUTTON_GPIO_CLK};

const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {KEY1_BUTTON_EXTI_LINE, KEY2_BUTTON_EXTI_LINE};

const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {KEY1_BUTTON_EXTI_PORT_SOURCE, KEY2_BUTTON_EXTI_PORT_SOURCE};
								 
const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {KEY1_BUTTON_EXTI_PIN_SOURCE, KEY2_BUTTON_EXTI_PIN_SOURCE}; 

const uint16_t BUTTON_IRQn[BUTTONn] = {KEY1_BUTTON_EXTI_IRQn, KEY2_BUTTON_EXTI_IRQn};

//USART_TypeDef* COM_UART[COMn] = {EVAL_COM1, EVAL_COM2}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};
 
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};

GPIO_TypeDef* COM_RTS_PORT[COMn] = {EVAL_COM1_RTS_GPIO_PORT, EVAL_COM2_RTS_GPIO_PORT};

GPIO_TypeDef* COM_CTS_PORT[COMn] = {EVAL_COM1_CTS_GPIO_PORT, EVAL_COM2_CTS_GPIO_PORT};

const uint32_t COM_UART_CLK[COMn] = {EVAL_COM1_CLK, EVAL_COM2_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK, EVAL_COM2_TX_GPIO_CLK};
 
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK, EVAL_COM2_RX_GPIO_CLK};

const uint32_t COM_RTS_PORT_CLK[COMn] = {EVAL_COM1_RTS_GPIO_CLK, EVAL_COM2_RTS_GPIO_CLK};
 
const uint32_t COM_CTS_PORT_CLK[COMn] = {EVAL_COM1_CTS_GPIO_CLK, EVAL_COM2_CTS_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN};

const uint16_t COM_RTS_PIN[COMn] = {EVAL_COM1_RTS_PIN, EVAL_COM2_RTS_PIN};

const uint16_t COM_CTS_PIN[COMn] = {EVAL_COM1_CTS_PIN, EVAL_COM2_CTS_PIN};
 
const uint16_t COM_TX_PIN_SOURCE[COMn] = {EVAL_COM1_TX_SOURCE, EVAL_COM2_TX_SOURCE};

const uint16_t COM_RX_PIN_SOURCE[COMn] = {EVAL_COM1_RX_SOURCE, EVAL_COM2_RX_SOURCE};

const uint16_t COM_RTS_PIN_SOURCE[COMn] = {EVAL_COM1_RTS_SOURCE, EVAL_COM2_RTS_SOURCE};

const uint16_t COM_CTS_PIN_SOURCE[COMn] = {EVAL_COM1_CTS_SOURCE, EVAL_COM2_CTS_SOURCE};
 
const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF, EVAL_COM2_TX_AF};
 
const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF, EVAL_COM2_RX_AF};

const uint16_t COM_RTS_AF[COMn] = {EVAL_COM1_RTS_AF, EVAL_COM2_RTS_AF};
 
const uint16_t COM_CTS_AF[COMn] = {EVAL_COM1_CTS_AF, EVAL_COM2_CTS_AF};


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/ 

/** @defgroup CA32M030_EVAL_LOW_LEVEL_Private_Functions
  * @{
  */ 

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *          This parameter can be one of following parameters:
  *            @arg LED1
  *            @arg LED2
  *            @arg LED3
  *            @arg LED4
  * @retval None
  */
void TC_EVAL_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
  //GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
	GPIO_PORT[Led]->BRR = GPIO_PIN[Led];   // change by haodan
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *          This parameter can be one of following parameters:
  *            @arg LED1
  *            @arg LED2
  *            @arg LED3
  *            @arg LED4  
  * @retval None
  */
void TC_EVAL_LEDOn(Led_TypeDef Led)
{
  //GPIO_PORT[Led]->BRR = GPIO_PIN[Led];
	GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];       // change by haodan
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *          This parameter can be one of following parameters:
  *            @arg LED1
  *            @arg LED2
  *            @arg LED3
  *            @arg LED4 
  * @retval None
  */
void TC_EVAL_LEDOff(Led_TypeDef Led)
{
  //GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
	GPIO_PORT[Led]->BRR = GPIO_PIN[Led];  // change by haodan
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *          This parameter can be one of following parameters:
  *            @arg LED1
  *            @arg LED2
  *            @arg LED3
  *            @arg LED4  
  * @retval None
  */
void TC_EVAL_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *          This parameter can be one of following parameters:
  *            @arg BUTTON_KEY2: KEY2 Push Button
  *            @arg BUTTON_KEY1: KEY1 Push Button 
  * @param  Button_Mode: Specifies Button mode.
  *          This parameter can be one of following parameters:   
  *            @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *            @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability
  * @retval None
  */
void TC_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_AHBPeriphClockCmd(BUTTON_CLK[Button], ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);

  if (Button_Mode == BUTTON_MODE_EXTI)
  {
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *          This parameter can be one of following parameters:
  *            @arg BUTTON_KEY2: KEY2 Push Button
  *            @arg BUTTON_KEY1: KEY1 Push Button     
  * @retval The Button GPIO pin value.
  */
uint32_t TC_EVAL_PBGetState(Button_TypeDef Button)
{
  /* There is no Wakeup button on CA32M030-EVAL. */
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

#ifdef EVAL_UART
/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *          This parameter can be one of following parameters:    
  *            @arg COM1
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that
  *         contains the configuration information for the specified UART peripheral.
  * @retval None
  */
void TC_EVAL_COMInit(COM_TypeDef COM, UART_InitTypeDef* UART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);
	if(UART_CR3_RTSE & UART_InitStruct->UART_HardwareFlowControl)
	{
		RCC_AHBPeriphClockCmd(COM_RTS_PORT_CLK[COM], ENABLE);
	}
	if(UART_CR3_CTSE & UART_InitStruct->UART_HardwareFlowControl)
	{
		RCC_AHBPeriphClockCmd(COM_CTS_PORT_CLK[COM], ENABLE);
	}

	/* Enable UART clock */
	if(COM == COM1)
		RCC_APB2PeriphClockCmd(COM_UART_CLK[COM], ENABLE); 
	else 
		RCC_APB1PeriphClockCmd(COM_UART_CLK[COM], ENABLE); 

	/* Connect PXx to UARTx_Tx */
	GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);

	/* Connect PXx to UARTx_Rx */
	GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);
	
	/* Connect to UARTx_RTS */
	if(UART_CR3_RTSE & UART_InitStruct->UART_HardwareFlowControl)
	{
		GPIO_PinAFConfig(COM_RTS_PORT[COM], COM_RTS_PIN_SOURCE[COM], COM_RTS_AF[COM]);
	}
	/* Connect to UARTx_CTS */
	if(UART_CR3_CTSE & UART_InitStruct->UART_HardwareFlowControl)
	{
		GPIO_PinAFConfig(COM_CTS_PORT[COM], COM_CTS_PIN_SOURCE[COM], COM_CTS_AF[COM]);
	}
	
	/* Configure UART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);
		
	/* Configure UART Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
	GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);
	
	/* Configure UART RTS as alternate function push-pull */
	if(UART_CR3_RTSE & UART_InitStruct->UART_HardwareFlowControl)
	{
		GPIO_InitStructure.GPIO_Pin = COM_RTS_PIN[COM];
		GPIO_Init(COM_RTS_PORT[COM], &GPIO_InitStructure);
	}
	
	/* Configure UART CTS as alternate function push-pull */
	if(UART_CR3_CTSE & UART_InitStruct->UART_HardwareFlowControl)
	{
		GPIO_InitStructure.GPIO_Pin = COM_CTS_PIN[COM];
		GPIO_Init(COM_CTS_PORT[COM], &GPIO_InitStructure);
	}
	

  /* UART configuration */
	if(COM == COM1)
		USART_Init(EVAL_COM1, (USART_InitTypeDef *)UART_InitStruct);
	else if(COM == COM2)
		UART_Init(EVAL_COM2, UART_InitStruct);
	else 
		return;
	
  /* Enable UART */
	if(COM == COM1)
		USART_Cmd(EVAL_COM1, ENABLE);
	else if(COM == COM2)
		UART_Cmd(EVAL_COM2, ENABLE);
	
	return;
}
#endif


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

