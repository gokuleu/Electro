/**
  ******************************************************************************
  * @file    LLC_type.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 1.1.0
  * @date    23-May-2017 
  * @brief   This file contains type definitions for DCDC control
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
#ifndef __LLC_TYPE_H
#define __LLC_TYPE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "DSMPS_type.h"

/* Exported constants --------------------------------------------------------*/

/** @addtogroup DSMPS_project
  * @{
  */ 
       
/** @defgroup LLC_exported_types LLC exported types
  * @{
  */ 

 /** 
  * @brief  Acquisition struct type definition 
  */
typedef struct 
 {  
   uint16_t hVout;              // ADCx 1st regular acquisition
   uint16_t hVin;               // ADCx 2nd regular acquisition
   uint16_t hIout;              // ADCx 3rd regular acquisition   
   uint16_t hIres;              // Not used
   uint16_t hIresAvg;           // Not used
   uint16_t hTemperature;       // ADCy 1st regular acquisition
   uint16_t hVdsSR1;            // ADCx 1st injected acquisition for SR1 signal
   uint16_t hVdsSR2;            // ADCy 1st injected acquisition for SR1 signal
 } DCDC_MeasureStruct_t;
 
/**
  * @}
  */
 
/** 
  * @brief  SR leg definition 
  */
typedef enum 
 {  
   SR_LEG1,
   SR_LEG2
 } DCDC_SRLeg_t;
 
/**
  * @}
  */
 
/** 
  * @brief  Burst Mode range definition 
  */
typedef enum 
 {  
   BM_RANGE1,
   BM_RANGE2,
   BM_RANGE3,
   BM_NOBURST
 } DCDC_BMRange_t;
 
/**
  * @}
  */ 
 
/** 
  * @brief  Configuration Param Struct
  */
typedef struct 
 {
   bool bConverterEnabled;
   bool bOpenLoopEnabled;
   bool bSREnabled;
   bool bAdaptiveSREnabled;
   bool bBurstModeEnabled;
   bool bFanPWMDrivingEnabled;
   uint32_t wOpenLoopFreq_Hz;
   uint16_t hDeadTimeFullBridge_ns;
   int16_t hFixedSRDelayRising1_ns;
   int16_t hFixedSRDelayFalling1_ns;
   int16_t hFixedSRDelayRising2_ns;
   int16_t hFixedSRDelayFalling2_ns;
   int16_t hRegulatorKpGain;
   int16_t hRegulatorKiGain;   
   bool bConfigurationChanged;
 } DCDC_ConfigParamStruct_t;

/**
  * @}
  */ 

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __LLC_TYPE_H */
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
