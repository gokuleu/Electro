/**
  ******************************************************************************
  * @file    DPS_Communication.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 2.0.0
  * @date    19-Apr-2017 
  * @brief   This file contains prototypes of Communication functions and related
  *          parameters
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DPS_COMMUNICATION_H
#define __DPS_COMMUNICATION_H
    
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
    
/* Exported types ------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Exported constants --------------------------------------------------------*/
//#define DISCOVERY_STM32F3348    /**< if defined the Rx an Tx Pin are mapped on PB3(Tx) and PB4(Rx), otherwise on PB6(Tx) and PB7(Rx). NOTE: on discovery board PB6 and PB7 are usaed for red and blue LEDs - defined in Debug Discovery STM32F334 Configuration */
#define OPTOCOUPLER             /**< if defined Tx polarity is iverted */

/* Baud Rate Configuration */
//#define USART_BAUD_RATE                 230400
//#define USART_BAUD_RATE                 115200
#define USART_BAUD_RATE                 57600
//#define USART_BAUD_RATE                 38400
//#define USART_BAUD_RATE                 19200
//#define USART_BAUD_RATE                 9600

#define ACK_MESSAGE_OK          0xCD          /**< response msg if communication was correcly received */
#define ACK_MESSAGE_FAILED      0xFA          /**< response msg if communication was not correcly received */
#define PRIMARY_HEADER          0x7E          /**< Header byte before status sending by Primary MCU - " ~ " (underfreq + overfreq + main_ov + main_uv + bus_ov + bus_uv) */

#define USART_TX_TIMER          1000          /**< Communication each  USART_TX_TIMER ms */

#define USARTx_TX_BUFFER_SIZE   0x02          /**< size of communication Tx messages for PFC (Rx for DC/DC): ID and PFC status */
#define USARTx_RX_BUFFER_SIZE   0x02          /**< size of communication Rx messages for PFC (Tx for DC/DC): ACK message */


/* Definition for USARTx clock and GPIO resources - see board_confing.h file for conflicts */

#ifdef DISCOVERY_STM32F3348
  /* On Discovery F334 board: USART2 on PB3 (Tx) and PB4 (Rx) */
//  #define USARTx                           USART2
//  #define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE()
//  #define DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
//  #define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
//  #define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE() 
//
//  #define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
//  #define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()
//
//  /* Definition for USARTx Pins */
//  #define USARTx_TX_PIN                    GPIO_PIN_3 
//  #define USARTx_TX_GPIO_PORT              GPIOB 
//  #define USARTx_TX_AF                     GPIO_AF7_USART2
//  #define USARTx_RX_PIN                    GPIO_PIN_4 
//  #define USARTx_RX_GPIO_PORT              GPIOB 
//  #define USARTx_RX_AF                     GPIO_AF7_USART2
//
//  /* Definition for USARTx's DMA */
//  #define USARTx_TX_DMA_CHANNEL            DMA1_Channel7        /* see RM pag 182 */
//  #define USARTx_RX_DMA_CHANNEL            DMA1_Channel6        /* see RM pag 182 */
//
//  /* Definition for USARTx's NVIC */
//  #define USARTx_DMA_TX_IRQn                   DMA1_Channel7_IRQn
//  #define USARTx_DMA_TX_IRQHandler             DMA1_Channel7_IRQHandler
//  #define USARTx_DMA_RX_IRQn                   DMA1_Channel6_IRQn
//  #define USARTx_DMA_RX_IRQHandler             DMA1_Channel6_IRQHandler
//
//  #define USARTx_IRQn                          USART2_IRQn
//  #define USARTx_IRQHandler                    USART2_IRQHandler

#else
/* final application on DSMPS Control Board: USART3 on PB9 (Tx) and PB8 (Rx) */
  #define USARTx                           USART3
  #define USARTx_CLK_ENABLE()              __USART3_CLK_ENABLE()
  #define DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
  #define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
  #define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE() 

  #define USARTx_FORCE_RESET()             __USART3_FORCE_RESET()
  #define USARTx_RELEASE_RESET()           __USART3_RELEASE_RESET()  

  /* Definition for USARTx Pins */
  #define USARTx_TX_PIN                    GPIO_PIN_9
  #define USARTx_TX_GPIO_PORT              GPIOB  
  #define USARTx_TX_AF                     GPIO_AF7_USART3      /* see DS pag 41 */
  #define USARTx_RX_PIN                    GPIO_PIN_8
  #define USARTx_RX_GPIO_PORT              GPIOB 
  #define USARTx_RX_AF                     GPIO_AF7_USART3      /* see DS pag 41 */

  /* Definition for USARTx's DMA */
  #define USARTx_TX_DMA_CHANNEL            DMA1_Channel2        /* see RM pag 180 */
  #define USARTx_RX_DMA_CHANNEL            DMA1_Channel3        /* see RM pag 180 */

  /* Definition for USARTx's NVIC */
  #define USARTx_DMA_TX_IRQn              DMA1_Channel2_IRQn
  #define USARTx_DMA_TX_IRQHandler        DMA1_Channel2_IRQHandler
  #define USARTx_DMA_RX_IRQn              DMA1_Channel3_IRQn
  #define USARTx_DMA_RX_IRQHandler        DMA1_Channel3_IRQHandler

  #define USARTx_IRQn                     USART3_IRQn
  #define USARTx_IRQHandler               USART3_IRQHandler

/* USART1 definitions *********************************************************/

//  #define USARTx                           USART1
//  #define USARTx_CLK_ENABLE()              __USART1_CLK_ENABLE()
//  #define DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
//  #define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
//  #define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE() 
//
//  #define USARTx_FORCE_RESET()             __USART1_FORCE_RESET()
//  #define USARTx_RELEASE_RESET()           __USART1_RELEASE_RESET()  
//
//  /* Definition for USARTx Pins */
//  #define USARTx_TX_PIN                    GPIO_PIN_6
//  #define USARTx_TX_GPIO_PORT              GPIOB  
//  #define USARTx_TX_AF                     GPIO_AF7_USART1      /* see DS pag 41 */
//  #define USARTx_RX_PIN                    GPIO_PIN_7
//  #define USARTx_RX_GPIO_PORT              GPIOB 
//  #define USARTx_RX_AF                     GPIO_AF7_USART1      /* see DS pag 41 */
//
//  /* Definition for USARTx's DMA */
//  #define USARTx_TX_DMA_CHANNEL            DMA1_Channel4        /* see RM pag 180 */
//  #define USARTx_RX_DMA_CHANNEL            DMA1_Channel5        /* see RM pag 180 */
//
//  /* Definition for USARTx's NVIC */
//  #define USARTx_DMA_TX_IRQn              DMA1_Channel4_IRQn
//  #define USARTx_DMA_TX_IRQHandler        DMA1_Channel4_IRQHandler
//  #define USARTx_DMA_RX_IRQn              DMA1_Channel5_IRQn
//  #define USARTx_DMA_RX_IRQHandler        DMA1_Channel5_IRQHandler
//
//  #define USARTx_IRQn                     USART1_IRQn
//  #define USARTx_IRQHandler               USART1_IRQHandler
/******************************************************************************/

  /* User Interface USART communication defines - TO UPDATE - to implement the protocol */
//  #define UI_USART                            USART1
//  #define UI_USART_CLK_ENABLE()               __USART1_CLK_ENABLE()
//  #define UI_USART_DMA_CLK_ENABLE()           __DMA1_CLK_ENABLE()
//  #define UI_USART_RX_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE()
//  #define UI_USART_TX_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE() 
//
//  #define UI_USART_FORCE_RESET()              __USART1_FORCE_RESET()
//  #define UI_USART_RELEASE_RESET()            __USART1_RELEASE_RESET()  
//
//  /* Definition for USARTx Pins */
//  #define UI_USART_TX_PIN                     GPIO_PIN_6
//  #define UI_USART_TX_GPIO_PORT               GPIOB  
//  #define UI_USART_TX_AF                      GPIO_AF7_USART1      /* see DS pag 41 */
//  #define UI_USART_RX_PIN                     GPIO_PIN_7
//  #define UI_USART_RX_GPIO_PORT               GPIOB 
//  #define UI_USART_RX_AF                      GPIO_AF7_USART1      /* see DS pag 41 */
//
//  /* Definition for USARTx's DMA */
//  #define UI_USART_TX_DMA_CHANNEL             DMA1_Channel4        /* see RM pag 180 */
//  #define UI_USART_RX_DMA_CHANNEL             DMA1_Channel5        /* see RM pag 180 */
//
//  /* Definition for USARTx's NVIC */
//  #define UI_USART_DMA_TX_IRQn                DMA1_Channel4_IRQn
//  #define UI_USART_DMA_TX_IRQHandler          DMA1_Channel4_IRQHandler
//  #define UI_USART_DMA_RX_IRQn                DMA1_Channel5_IRQn
//  #define UI_USART_DMA_RX_IRQHandler          DMA1_Channel5_IRQHandler
//
//  #define UI_USART_IRQn                       USART1_IRQn
//  #define UI_USART_IRQHandler                 USART1_IRQHandler
#endif

/* I2C (PM-BUS) */
// TO CONFIGURE CLOCK AND GPIO RESOURCES FOR PM-BUS


/* Exported variables --------------------------------------------------------*/
extern uint8_t UsartTxBuffer[];    /**< Transmit message buffer */
extern uint8_t UsartRxBuffer[];    /**< Receive message buffer */

/* Function prototypes -------------------------------------------------------*/
uint16_t ReceivePrimaryStatus(void);
void SendPrimaryStatus(uint8_t PrimaryStatus2Send);
void SendAck(uint8_t AckResponse);
void USART_Task(uint8_t bPrimaryStatus2Send);

#endif /* __DPS_COMMUNICATION_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/


