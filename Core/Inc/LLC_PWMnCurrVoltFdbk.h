/**
  ******************************************************************************
  * @file    LLC_PWMnCurrVoltFdbk.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 2.0.0
  * @date    23-May-2017 
  * @brief   This file contains prototypes of PWM drive and acquisition functions
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
#ifndef __LLC_PWMNCURRVOLTFDBK_H
#define __LLC_PWMNCURRVOLTFDBK_H

/* Includes ------------------------------------------------------------------*/
#include "LLC_type.h"
    
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void PWM_PeriodActuation(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMPeriodHRTimTick);
void PWM_SynchRectActuation(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMPeriodHRTimTick, int16_t hDelaySRRising1, int16_t hDelaySRFalling1, int16_t hDelaySRRising2, int16_t hDelaySRFalling2);
void PWM_SynchRectActuationDT(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMPeriodHRTimTick, int16_t hDelaySRRising1, int16_t hDelaySRFalling1, int16_t hDelaySRRising2, int16_t hDelaySRFalling2, int16_t hDeadTimeRising, int16_t hDeadTimeFalling);
void PWM_PeriodSynchRectActuationDT(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMPeriodHRTimTick, int16_t hDelaySRRising1, int16_t hDelaySRFalling1, int16_t hDelaySRRising2, int16_t hDelaySRFalling2, int16_t hDeadTime);
void PWM_DeadTimeActuation(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMDeadTimeDTTick);
void PWM_UpdateBurstModeParams(HRTIM_HandleTypeDef * hhrtim, uint16_t hHRTIMBurstModePeriod, uint16_t hHRTIMBurstModeIdle);
void PWM_FanActuation(TIM_HandleTypeDef * htim, uint32_t Channel, uint16_t hPulseDurationTimTick);
uint16_t PWM_GetVdsSRMeasure(DCDC_SRLeg_t SRLeg);
uint16_t PWM_ConvertCurrentValue(float fCurrentA);

#endif /* __LLC_PWMNCURRVOLTFDBK_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/


