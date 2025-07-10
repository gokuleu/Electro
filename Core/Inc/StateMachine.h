/**
  ******************************************************************************
  * @file    StateMachine.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 1.0.0
  * @date    31-May-2016
  * @brief   This file contains prototypes of state machine related functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
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
#ifndef __STATEMACHINE_H
#define __STATEMACHINE_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
/* State machine functions */
void STM_StateMachineTask(void);
void STM_SetStateMachineStatus(DSMPS_State_t DSMPS_NewState);
DSMPS_State_t STM_GetStateMachineStatus(void);
void STM_SetConverterEnabledFag(bool bState);
bool STM_GetConverterEnabledFag(void);

/* LED functions */
void LED_On(LED_Struct_t* LED);
void LED_Off(LED_Struct_t* LED);
void LED_Toggle(LED_Struct_t* LED);
void LED_SetParams(LED_Struct_t* LED, LED_Modality_t newLEDModality, uint8_t nBlinkCounter, uint16_t hBlinkPeriodms, uint16_t hhBlinkRepetitionPeriodms);
void LED_DecreaseBlinkCounter(LED_Struct_t* LED);
void LED_Task(LED_Struct_t* LED);
void LED_Init(LED_Struct_t* LED, GPIO_TypeDef* LED_GPIOPort, uint16_t LED_GPIOPin);

#endif /* __STATEMACHINE_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/


