/**
  ******************************************************************************
  * @file    Control_Layer.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 1.1.0
  * @date    03-May-2017 
  * @brief   This file contains prototypes of Control Layer functions
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
#ifndef __CONTROL_LAYER_H
#define __CONTROL_LAYER_H

/* Includes ------------------------------------------------------------------*/
#include "DSMPS_type.h"
#include "LLC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void SetDelayTime(uint16_t volatile *phCounter_to_set, uint16_t hNew_counter_value);
bool DelayTimeIsElapsed(uint16_t volatile *phCounter_to_check);

void CTR_InitEnvironment(void);
void CTR_InitControlParameters(void);
void CTR_UpdateRegulatorOutput(void);
uint16_t CTR_SetPWMFrequency(uint32_t wPWMFreqHz);
void CTR_SetPWMDeadTime(uint16_t hPWMDeadTime_ns);
void CTR_SetSRFixedDelays(DCDC_ConfigParamStruct_t* pConfigParamStruct);
void CTR_UpdateConfigParam(DCDC_ConfigParamStruct_t* pConfigParamStruct, HRTIM_HandleTypeDef * hhrtim);
uint16_t CTR_ExecuteVoltageLoop(uint16_t hVoltageReference, uint16_t hVoltageMeasure);
void CTR_BurstModeControl(uint16_t hOuputCurrent, HRTIM_HandleTypeDef * hhrtim);
void CTR_AdaptiveSynchRectEdgeCalculation(void);
void CTR_FanEnable(void);
void CTR_FanDisable(void);
void CTR_FanSpeedRegulationTempLoad(float fTemperature, uint16_t hOutputCurrent, TIM_HandleTypeDef * hFantim);
void CTR_FanSpeedRegulationTempLoadDis(DCDC_ConfigParamStruct_t* pConfigParamStruct, float fTemperature, uint16_t hOutputCurrent, TIM_HandleTypeDef * hFantim);
void CTR_FanSpeedRegulationLoad(uint16_t hOutputCurrent, TIM_HandleTypeDef * hFantim);
void CTR_SRAutomaticTurnOnOffLoad(uint16_t hOutputCurrent, HRTIM_HandleTypeDef * hhrtim);
void CTR_GateDriverEnable(void);
void CTR_GateDriverDisable(void);
void CTR_SRGateDriverEnable(void);
void CTR_SRGateDriverDisable(void);
void CTR_GateDriverBootstrapCapPrecharge(HRTIM_HandleTypeDef * hhrtim);
void CTR_PWMOutputEnable(HRTIM_HandleTypeDef * hhrtim);
void CTR_PWMFastOutputEnable(HRTIM_HandleTypeDef * hhrtim);
void CTR_PWMOutputDisable(HRTIM_HandleTypeDef * hhrtim);
void CTR_PWMSynchRectOutputEnable(HRTIM_HandleTypeDef * hhrtim);
void CTR_PWMSynchRectOutputDisable(HRTIM_HandleTypeDef * hhrtim);
void CTR_PWMAllOutputsDisable(HRTIM_HandleTypeDef * hhrtim);
void CTR_AdaptiveSynchRectEnable(void);
void CTR_AdaptiveSynchRectDisable(void);
void CTR_BurstModeEnable(HRTIM_HandleTypeDef * hhrtim);
void CTR_BurstModeDisable(HRTIM_HandleTypeDef * hhrtim);

#endif /* __CONTROL_LAYER_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/


