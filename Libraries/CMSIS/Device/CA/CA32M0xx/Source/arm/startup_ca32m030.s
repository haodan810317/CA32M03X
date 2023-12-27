;******************** (C) COPYRIGHT 2020 ThingsChip ********************
;* File Name          : startup_ca32f003.s
;* Description        : 
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == Reset_Handler
;*                      - Set the vector table entries with the exceptions ISR address
;*                      - Branches to __main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the CortexM0 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;* <<< Use Configuration Wizard in Context Menu >>>
;*******************************************************************************
;
;* Redistribution and use in source and binary forms, with or without modification,
;* are permitted provided that the following conditions are met:
;*   1. Redistributions of source code must retain the above copyright notice,
;*      this list of conditions and the following disclaimer.
;*   2. Redistributions in binary form must reproduce the above copyright notice,
;*      this list of conditions and the following disclaimer in the documentation
;*      and/or other materials provided with the distribution.
;*   3. Neither the name of ThingsChip nor the names of its contributors
;*      may be used to endorse or promote products derived from this software
;*      without specific prior written permission.
;*
;*******************************************************************************

; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size		EQU     0x400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size      EQU     0x400

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp                   ; Top of Stack
                DCD     Reset_Handler                  ; Reset Handler
                DCD     NMI_Handler                    ; NMI Handler
                DCD     HardFault_Handler              ; Hard Fault Handler
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     SVC_Handler                    ; SVCall Handler
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     PendSV_Handler                 ; PendSV Handler
                DCD     SysTick_Handler                ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler                ; Window Watchdog
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     FLASH_IRQHandler               ; FLASH
                DCD     0                              ; Reserved
                DCD     EXTI0_7_IRQHandler             ; EXTI Line 0 to 7
                DCD     EXTI8_15_IRQHandler            ; EXTI Line 8 to 15
                DCD     0                              ; Reserved
								DCD     0                              ; Reserved
                DCD     DMA1_Channel1_IRQHandler       ; DMA1 Channel 1
                DCD     DMA1_Channel2_3_IRQHandler     ; DMA1 Channel 2 and 3
                DCD     DMA1_Channel4_5_IRQHandler     ; DMA1 Channel 4 and 5
                DCD     ADC1_IRQHandler                ; ADC1 
                DCD     TIM20_IRQHandler	        		 ; TIM20
								DCD     0                              ; Reserved
                DCD     CORDIC_IRQHandler	        		 ; CORDIC
                DCD     TIM3_IRQHandler                ; TIM3
                DCD     I2C1_OA1_IRQHandler            ; I2C1 Slave no strech mode Address 1 match interrupt
                DCD     I2C1_OA2_IRQHandler            ; I2C1 Slave no strech mode Address 2 match interrupt
                DCD     TIM2_IRQHandler                ; TIM2
                DCD     COMP0_IRQHandler               ; CMP0
                DCD     TIM6_IRQHandler                ; TIM6
                DCD     TIM7_IRQHandler                ; TIM7
                DCD     I2C1_IRQHandler                ; I2C1
                DCD     COMP1_IRQHandler               ; CMP1
                DCD     SPI1_IRQHandler                ; SPI1
                DCD     0                              ; Reserved
                DCD     USART1_IRQHandler              ; USART1
                DCD     UART2_IRQHandler               ; UART2
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved


__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler routine
Reset_Handler    PROC
                 EXPORT  Reset_Handler                 [WEAK]
        IMPORT  __main
        IMPORT  SystemInit  
                 LDR     R0, =SystemInit
                 BLX     R0
                 LDR     R0, =__main
                 BX      R0
                 ENDP
					 
;I2C1_IRQHandler  PROC
                 ;LDR     R2, =0x40005418
                 ;LDR     R3, =0x00FF0008
				 ;LDR     R4, =0x00A10008
				 ;LDR     R5, =0x08
				 ;LDR     R6, =0x20000008
				 ;LDR     R6, [R6, #0]
				 ;LDR     R7, =0x20000024
				 ;LDR     R0, [R2, #0]
				 ;ANDS    R0, R0, R3
				 ;BNE     NOT_ADR
				 ;LDR     R7, [R7,R6]
				 ;STR     R7, [R2,#16]
				 
				 ;STR     R5, [R2,#4]
;NOT_ADR
                 ;ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                    [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler              [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                    [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler                 [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler                [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDG_IRQHandler                [WEAK]
                EXPORT  FLASH_IRQHandler               [WEAK]
                EXPORT  EXTI0_7_IRQHandler             [WEAK]
                EXPORT  EXTI8_15_IRQHandler            [WEAK]
                EXPORT  DMA1_Channel1_IRQHandler       [WEAK]
                EXPORT  DMA1_Channel2_3_IRQHandler     [WEAK]
                EXPORT  DMA1_Channel4_5_IRQHandler     [WEAK]					
                EXPORT  ADC1_IRQHandler                [WEAK]
                EXPORT  TIM20_IRQHandler               [WEAK]
                EXPORT	CORDIC_IRQHandler              [WEAK]
                EXPORT  TIM3_IRQHandler                [WEAK]
                EXPORT  I2C1_OA1_IRQHandler            [WEAK]
                EXPORT  I2C1_OA2_IRQHandler            [WEAK]
                EXPORT  TIM2_IRQHandler                [WEAK]
                EXPORT	COMP0_IRQHandler               [WEAK]
                EXPORT  TIM6_IRQHandler                [WEAK]
                EXPORT  TIM7_IRQHandler                [WEAK]
                EXPORT  I2C1_IRQHandler                [WEAK]
                EXPORT	COMP1_IRQHandler               [WEAK]                  
                EXPORT  SPI1_IRQHandler                [WEAK]
                EXPORT  USART1_IRQHandler              [WEAK]
                EXPORT  UART2_IRQHandler               [WEAK]


WWDG_IRQHandler
FLASH_IRQHandler
EXTI0_7_IRQHandler
EXTI8_15_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_3_IRQHandler
DMA1_Channel4_5_IRQHandler
ADC1_IRQHandler
TIM20_IRQHandler
CORDIC_IRQHandler
TIM3_IRQHandler
I2C1_OA1_IRQHandler
I2C1_OA2_IRQHandler
TIM2_IRQHandler
COMP0_IRQHandler
TIM6_IRQHandler
TIM7_IRQHandler
I2C1_IRQHandler
COMP1_IRQHandler
SPI1_IRQHandler
USART1_IRQHandler
UART2_IRQHandler

                B       .

                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB

                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit

                 ELSE

                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap

__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END

;************************ (C) COPYRIGHT ThingsChip *****END OF FILE*****
