/**
  ******************************************************************************
  * @file    UI_UART_Interface.h
  * @author  IMS Systems Lab and Technical Marketing - Motion Control group
  * @version V1.0.0
  * @date    26-November-2015
  * @brief   Header for UI_UART_Interface.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) YYYY STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UI_UART_INTERFACE_H
#define __UI_UART_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
//#include <stdlib.h>
#include <ctype.h>   
#include <string.h>
#include "stm32f3xx_hal.h"
   
/* Exported types ------------------------------------------------------------*/
typedef enum {
  FAILED = 0,
  PASSED = !FAILED
} UART_TestStatus_t;

/* Exported constants --------------------------------------------------------*/
#define UI_COMMUNICATION_FW_VERSION_INTEGER_PART     1
#define UI_COMMUNICATION_FW_VERSION_DECIMAL_PART     00  // FW V1.00
#define FW_VERSION_INTEGER_PART     1
#define FW_VERSION_DECIMAL_PART     50  // FW V1.50

#define USART_ENTER_CHAR                0x0D // \r character
#define USART_BACKSPACE_CHAR            0x08 // \b character
#define USART_SPACE_CHAR                0x20 // ' ' character
    
/* DSMPS Board and peripheral defines - PMBus connector */

/* USART3 Config (V2)*************************************************************/
#define UI_USART                           USART3
#define UI_UART_CLK_ENABLE()              __USART3_CLK_ENABLE();
#define UI_UART_DMA_CLK_ENABLE()          __DMA1_CLK_ENABLE()
#define UI_UART_RX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define UI_UART_TX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()

#define UI_UART_FORCE_RESET()             __USART3_FORCE_RESET()
#define UI_UART_RELEASE_RESET()           __USART3_RELEASE_RESET()

/* Definition for USARTx Pins */
#define UI_UART_TX_PIN                    GPIO_PIN_9
#define UI_UART_TX_GPIO_PORT              GPIOB
#define UI_UART_TX_AF                     GPIO_AF7_USART3
#define UI_UART_RX_PIN                    GPIO_PIN_8
#define UI_UART_RX_GPIO_PORT              GPIOB
#define UI_UART_RX_AF                     GPIO_AF7_USART3

/* Definition for USARTx's DMA */
#define UI_UART_TX_DMA_STREAM              DMA1_Channel2
#define UI_UART_RX_DMA_STREAM              DMA1_Channel3

/* Definition for USARTx's NVIC */
#define UI_UART_DMA_TX_IRQn                DMA1_Channel2_IRQn
#define UI_UART_DMA_RX_IRQn                DMA1_Channel3_IRQn
#define UI_UART_DMA_TX_IRQHandler          DMA1_Channel2_IRQHandler
#define UI_UART_DMA_RX_IRQHandler          DMA1_Channel3_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler

/* USART1 Config (V1)**************************************************************/
//#define UI_USART                        USART1
//#define UI_UART_CLK_ENABLE()           __USART1_CLK_ENABLE();
//#define UI_UART_DMA_CLK_ENABLE()       __DMA1_CLK_ENABLE()
//#define UI_UART_RX_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
//#define UI_UART_TX_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
//
//#define UI_UART_FORCE_RESET()          __USART1_FORCE_RESET()
//#define UI_UART_RELEASE_RESET()        __USART1_RELEASE_RESET()
//
///* Definition for USARTx Pins */
//#define UI_UART_TX_PIN                 GPIO_PIN_6
//#define UI_UART_TX_GPIO_PORT           GPIOB
//#define UI_UART_TX_AF                  GPIO_AF7_USART1
//#define UI_UART_RX_PIN                 GPIO_PIN_7
//#define UI_UART_RX_GPIO_PORT           GPIOB
//#define UI_UART_RX_AF                  GPIO_AF7_USART1
//
///* Definition for USARTx's DMA */
//#define UI_UART_TX_DMA_STREAM          DMA1_Channel4
//#define UI_UART_RX_DMA_STREAM          DMA1_Channel5
//
///* Definition for USARTx's NVIC */
//#define UI_UART_DMA_TX_IRQn            DMA1_Channel4_IRQn
//#define UI_UART_DMA_RX_IRQn            DMA1_Channel5_IRQn
//#define UI_UART_DMA_TX_IRQHandler      DMA1_Channel4_IRQHandler
//#define UI_UART_DMA_RX_IRQHandler      DMA1_Channel5_IRQHandler

/******************************************************************************/

/* Size of Reception buffer */
#define UI_UART_RXBUFFERSIZE               100 
   
/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef UI_UartHandle;
extern uint8_t aUI_UartRxBuffer[];
/* Last Char received */
extern uint8_t cUI_UartRxLastChar;

/* Special messages */
static uint8_t aUartDeleteCharMsg[] = " \b";

/* Exported macros -----------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
   
/* Exported functions ------------------------------------------------------- */
void UI_UART_MspInit(UART_HandleTypeDef *huart);
void UI_UART_MspDeInit(UART_HandleTypeDef *huart);
void UI_UART_Config(void);
void UI_UART_SendBeginMsg(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void UI_UART_DMA_RX_IRQHandler(void);
void UI_UART_DMA_TX_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /*__UI_UART_INTERFACE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
