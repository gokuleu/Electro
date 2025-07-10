/**
  ******************************************************************************
  * @file    Fault_Processing.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 1.1.0
  * @date    07-Nov-2016 
  * @brief   This file contains prototypes of Fault detection functions
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
#ifndef __FAULT_PROCESSING_H
#define __FAULT_PROCESSING_H

/* Includes ------------------------------------------------------------------*/
#include "DSMPS_type.h"
    
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void HRTIM1_FLT_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void FLT_FaultCheck(void);
void FLT_StartUpFailedProcedure(void);
//void FLT_SetSystemFault(uint16_t hNewError);
//void FLT_ClearSystemFault(uint16_t hErrorCleared);
void FLT_ClearAllSystemFaults(void);
uint16_t FLT_GetSystemFault(void);
uint16_t FLT_GetLastShownSystemFault(void);
void FLT_StoreLastSystemFault(void);

#endif /* __FAULT_PROCESSING_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/


