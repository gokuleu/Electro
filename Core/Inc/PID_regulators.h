/**
  ******************************************************************************
  * @file    PID_regulators.h
  * @author  IMS Systems Lab and Technical Marketing
  * @version 1.3.0
  * @date    24-May-2017
  * @brief   This file contains the prototypes of PI(D) related functions.
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

#ifndef __PI_REGULATORS__H
#define __PI_REGULATORS__H

/* Includes ------------------------------------------------------------------*/
#include "DSMPS_type.h"

/* Defines  ------------------------------------------------------------------*/
#define DIFFERENTIAL_TERM_ENABLED
#define DIFFERENTIAL_TERM_ENABLED_EX

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* General PID functions -----------*/
void PID_Set_ProportionalGain(PID_Struct_t *PID_Struct, int16_t hProportionalGain);
void PID_Set_IntegralGain(PID_Struct_t *PID_Struct, int16_t hIntegralGain);
void PID_Reset_IntegralTerm(PID_Struct_t *PID_Struct);
void PID_Set_IntegralTerm(PID_Struct_t *PID_Struct, int32_t wIntegralTerm);
void PID_Set_OutputUpperLimit(PID_Struct_t *PID_Struct, int16_t hOutputUpperLimit);
void PID_Set_OutputLowerLimit(PID_Struct_t *PID_Struct, int16_t hOutputLowerLimit);
int16_t PID_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_t *PID_Struct);      // MC traditional PID
int32_t PID_Controller(int16_t hReference, int16_t hPresentFeedback, PID_Struct_t *PID_Struct);     // modified PID

/* General extended PID functions -----------*/
void PID_Set_ProportionalGain_ex(PID_Struct_ex_t *PID_Struct, int16_t hProportionalGain);
void PID_Set_IntegralGain_ex(PID_Struct_ex_t *PID_Struct, int16_t hIntegralGain);
void PID_Reset_IntegralTerm_ex(PID_Struct_ex_t *PID_Struct);
void PID_Set_IntegralTerm_ex(PID_Struct_ex_t *PID_Struct, int32_t wIntegralTerm);
void PID_Set_OutputUpperLimit_ex(PID_Struct_ex_t *PID_Struct, int32_t wOutputUpperLimit);
void PID_Set_OutputLowerLimit_ex(PID_Struct_ex_t *PID_Struct, int32_t wOutputLowerLimit);
int32_t PID_Regulator_ex(int16_t hReference, int16_t hPresentFeedback, PID_Struct_ex_t *PID_Struct);      // MC traditional PID for extended PID

/* PFC specific PID functions -----------*/
void DCDC_PID_Init(PID_Struct_t *PID_Vout);
void DCDC_PID_Init_ex(PID_Struct_ex_t *PID_Vout);

/* Exported variables ------------------------------------------------------- */

#endif

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
