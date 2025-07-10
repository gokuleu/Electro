/**
  ******************************************************************************
  * @file    LLC_Globals.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 2.0.0
  * @date    23-May-2017 
  * @brief   This file contains the declarations of the exported 
  *                      variables of module "LLC_Globals.c".
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
#ifndef __LLC_GLOBALS_H
#define __LLC_GLOBALS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "DSMPS_type.h"
#include "LLC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern DCDC_MeasureStruct_t DCDC_MeasureStruct;                 /**< Acquisition struct */
extern DCDC_ConfigParamStruct_t DCDC_ConfigParamStruct;         /**< Configuration param struct */

/* filtered measures */
extern uint16_t hTemperatureFiltered;                   /**< Temperature filtered measure */
extern uint16_t hVoutVoltageFiltered;                   /**< Vout voltage filtered measure */
extern uint16_t hVinVoltageFiltered;                    /**< Vin voltage filtered measure */

extern volatile float fTemperatureDeg;                  /**< LM19CIZ Temperature in deg */

/* control variables */
extern __IO uint16_t hPWMPeriod;                 	/**< value of LLC PWM period in HRTIM ticks */
extern __IO uint16_t hPWMDeadTime;                      /**< value of LLC PWM dead time in HRTIM DT ticks - for update if it is requested (same value for rising and falling)*/
extern __IO uint16_t hPWMDeadTimeHRticks;               /**< value of LLC PWM dead time in HRTIM TIM ticks - for update if it is requested (same value for rising and falling)*/
extern __IO uint16_t hBurstModeIdleDuration;            /**< number of (HRTIM periods -1) in idle during burst mode */
extern __IO uint16_t hBurstModePeriod;                  /**< number of (HRTIM periods -1) that compose burst period (run + idle) in burst mode */
extern volatile uint16_t hVout_Reference;               /**< Voltage out reference */
extern volatile uint16_t hVout_Reference_init;          /**< Voltage out reference init value */

/* threshold values - for current calibration (only for SR and BM) */
extern uint16_t hHallIcOffsetCalib;                     /**< Calibration value for current sensor */
extern uint16_t hSrIoutTurnOnThreshold;                 /**< SR turn-on current threshold variable with calibratated offset */
extern uint16_t hSrIoutTurnOffThreshold;                /**< SR turn-off current threshold variable with calibratated offset */
extern uint16_t hBmIoutRange12ThresholdHigh;            /**< BM range12 high current threshold variable with calibratated offset */
extern uint16_t hBmIoutRange12ThresholdLow;             /**< BM range12 low current threshold variable with calibratated offset */
extern uint16_t hBmIoutRange23ThresholdHigh;            /**< BM range23 high current threshold variable with calibratated offset */
extern uint16_t hBmIoutRange23ThresholdLow;             /**< BM range23 low current threshold variable with calibratated offset */

/* synchronous rectification's edge delays */
extern int16_t hSR_DelayRising1;                       /**< rising edge's delay of SR1 signal in HRTIM ticks */
extern int16_t hSR_DelayFalling1;                      /**< falling edge's delay of SR1 signal in HRTIM ticks */
extern int16_t hSR_DelayRising2;                       /**< rising edge's delay of SR2 signal in HRTIM ticks */
extern int16_t hSR_DelayFalling2;                      /**< falling edge's delay of SR2 signal in HRTIM ticks */

/* Configuration handles */
extern ADC_HandleTypeDef        AdcHandle1;		/**< handle for ADC1 (ADCx) */
extern ADC_HandleTypeDef        AdcHandle2;		/**< handle for ADC2 (ADCy) */
extern HRTIM_HandleTypeDef      hhrtim;			/**< handle for HRTIM */
extern COMP_HandleTypeDef       CompOCHandle;           /**< handle for COMP_OC */
extern COMP_HandleTypeDef       CompSRHandle1;          /**< handle for COMP_VDS_SR1 */
extern COMP_HandleTypeDef       CompSRHandle2;          /**< handle for COMP_VDS_SR2 */
extern UART_HandleTypeDef       UartHandle;             /**< handle for UARTx */
extern DAC_HandleTypeDef        DacOCHandle;            /**< handle for DAC_OC */
extern DAC_HandleTypeDef        DacSRHandle;            /**< handle for DAC_SR */
extern TIM_HandleTypeDef        TimVoutControlHandle;   /**< handle of TIM that schedules Vout control loop */
extern TIM_HandleTypeDef        FanPwmHandle;           /**< handle for FAN PWM TIMx */
extern TIM_HandleTypeDef        TimTempFanCtrlHandle;   /**< handle for TIM that schedules temperature acquisition and fan control */

/* Start-up counters */
extern volatile uint16_t hDSMPS_StartUp_TimeLeft;                /**< counter after init start-up DC/DC */
extern volatile uint16_t hDSMPS_Wait_TimeLeft;                   /**< counter after enter in WAIT state */
extern volatile uint16_t hDSMPS_UndervoltageValidation_TimeLeft; /**< counter to validate undervoltage protection */
extern volatile uint16_t hCommunicationTimeLeft;                /**< counter between two USART communications */
extern volatile uint16_t hStartUpFreqUpdateCounter;             /**< counter used to update the PWM frequency during start-up */

/* DCDC's PID struct */
#ifdef USE_EXTENDED_PID
  extern PID_Struct_ex_t PID_Vout_InitStructure;          /**< Out voltage extended PID */
#else
  extern PID_Struct_t PID_Vout_InitStructure;             /**< Out voltage PID */
#endif

/* state variable of converter */
extern volatile uint16_t PFC_Received_Status;           /**< last PFC received status */
extern LED_Struct_t StatusLED;                          /**< Status LED */
extern LED_Struct_t FaultLED;                           /**< Fault LED */
extern volatile uint16_t FAN_PWMDutyCycle;              /**< PWM duty cycle for fan control */

/* boolean variables */
extern volatile bool bDCDC_OutputEnabled;               /**< TRUE when main PWMs of DCDC are enabled, FALSE otherwise */
extern volatile bool bDCDC_SynchRectOutputEnabled;      /**< TRUE when PWMs of Synch. Rect. are enabled, FALSE otherwise */
extern volatile bool bDCDC_AdaptiveSynchRectEnabled;    /**< TRUE when Adaptive Synch. Rect. is enabled, FALSE otherwise */
extern volatile bool bDCDC_DeadTimeChanged;             /**< TRUE when the dead time was changed and must be updated, FALSE otherwise */
extern volatile bool bCheckUndervoltageEnabled;         /**< TRUE when low out voltage short circuit fault is enabled, FALSE otherwise */
extern volatile bool bCheckOvertemperatureEnabled;      /**< TRUE when overtemperature fault is enabled, FALSE otherwise */
extern volatile bool bCheckOutOvercurrentEnabled;       /**< TRUE when out overcurrent fault is enabled, FALSE otherwise */

/* precharge driver struct variables */
extern volatile Driver_PrechargeStruct_t FullBridgeDriverPrechargeStruct;       /**< precharge struct variable of full bridge drivers */

/* for out voltage burst mode */
extern volatile bool bVoutBurstModeEnabled;             /**< TRUE when out voltage burst mode is active (Vbus> bm_threshold with hysteresis), FALSE otherwise */
extern volatile bool bLightLoadBurstModeEnabled;        /**< TRUE when light load burst mode is active (PWM are in burst if Iout is < threshold), FALSE otherwise */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __LLC_GLOBALS_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
