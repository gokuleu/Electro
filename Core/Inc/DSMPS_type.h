/**
  ******************************************************************************
  * @file    DSMPS_type.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 2.0.0
  * @date    24-May-2017 
  * @brief   This file contains type definitions for DSMPS control
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
#ifndef __DSMPS_TYPE_H
#define __DSMPS_TYPE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f334x8.h"

/* Exported constants --------------------------------------------------------*/

/** @addtogroup DSMPS_project
  * @{
  */

/** @defgroup Boundary_and_error_codes
* @{
*/
    
/** @defgroup Standard_types_boundary_values
* @{
*/       
#define U8_MAX     ((uint8_t)255)
#define S8_MAX     ((int8_t)127)
#define S8_MIN     ((int8_t)-127)
#define U16_MAX    ((uint16_t)65535u)
#define S16_MAX    ((int16_t)32767)
#define S16_MIN    ((int16_t)-32767)
#define U32_MAX    ((uint32_t)4294967295uL)
#define S32_MAX    ((int32_t)2147483647)
#define S32_MIN    ((int32_t)-2147483647)
/**
  * @}
  */

/** @defgroup PFC_Fault_generation_error_codes
* @{
*/
#define  PFC_NO_ERROR                   (uint8_t)(0x00u)
#define  PFC_BUS_OVER_VOLT              (uint8_t)(0x01u)
#define  PFC_BUS_UNDER_VOLT             (uint8_t)(0x02u)
#define  PFC_MAIN_OVER_VOLT             (uint8_t)(0x04u)
#define  PFC_MAIN_UNDER_VOLT            (uint8_t)(0x08u)
#define  PFC_MAIN_OVER_FREQ             (uint8_t)(0x10u)
#define  PFC_MAIN_UNDER_FREQ            (uint8_t)(0x20u)
#define  PFC_OVER_TEMP                  (uint8_t)(0x40u)

#define  PFC_COMMUNICATION_ERROR        (uint16_t)(0xF000u)
/**
  * @}
  */

/** @defgroup DC-DC_Fault_generation_error_codes
* @{
*/        
#define  DCDC_NO_ERROR			(uint16_t)(0x0000u)

#define  DCDC_OUT_OVER_VOLT_ERROR       (uint16_t)(0x0001u)
#define  DCDC_OUT_UNDER_VOLT_ERROR      (uint16_t)(0x0002u)
#define  DCDC_IN_OVER_VOLT_ERROR        (uint16_t)(0x0004u)
#define  DCDC_IN_UNDER_VOLT_ERROR       (uint16_t)(0x0008u)
#define  DCDC_OVER_CURRENT_ERROR        (uint16_t)(0x0010u)
#define  DCDC_OUT_OVER_CURRENT_ERROR    (uint16_t)(0x0020u)
#define  DCDC_OVER_TEMP_ERROR           (uint16_t)(0x0040u)
#define  DCDC_STARTUP_FAILED_ERROR      (uint16_t)(0x0080u)
#define  DCDC_PRIMARY_SIDE_ERROR        (uint16_t)(0x0100u)

#define  DCDC_COMMUNICATION_ERROR       (uint16_t)(0xE000u)
/**
  * @}
  */

/** @defgroup LED_Blinking_defines
* @{
*/
/** LED status Blinking Timing **/
#define LED_BLINK_REPETITION_PERIOD_MS  3000            /**< repetition period in ms between a blinking series and the next one */
#define LED_BLINK_PERIOD_LONG_MS        500             /**< LED on/off long period in ms */
#define LED_BLINK_PERIOD_SHORT_MS       250             /**< LED on/off short period in ms */

/** LED Fault blinking codes **/
/* Common Fault codes for DC/DC converter */
#define LED_DCDC_OUT_OVER_VOLTAGE_BLINK_NUM     3 //modified: before it was 2
#define LED_DCDC_OUT_UNDER_VOLTAGE_BLINK_NUM    2 //modified: before it was 3
#define LED_DCDC_IN_OVER_VOLTAGE_BLINK_NUM      4
#define LED_DCDC_IN_UNDER_VOLTAGE_BLINK_NUM     5
#define LED_DCDC_IN_START_UP_FAILED_BLINK_NUM   6
#define LED_DCDC_OVER_CURRENT_BLINK_NUM         2
#define LED_DCDC_OUT_OVER_CURRENT_BLINK_NUM     3
#define LED_DCDC_OVER_TEMPERATURE_BLINK_NUM     4
#define LED_DCDC_PRIMARY_SIDE_ERROR_BLINK_NUM   5
#define LED_DCDC_COMMUNICATION_ERROR_BLINK_NUM  6

/* Common Fault codes for PFC converter */
#define LED_PFC_BUS_OVER_VOLTAGE_BLINK_NUM      2
#define LED_PFC_BUS_UNDER_VOLTAGE_BLINK_NUM     3
#define LED_PFC_MAINS_OVER_VOLTAGE_BLINK_NUM    4
#define LED_PFC_MAINS_UNDER_VOLTAGE_BLINK_NUM   5
#define LED_PFC_MAINS_OVER_FREQ_BLINK_NUM       6
#define LED_PFC_MAINS_UNDER_FREQ_BLINK_NUM      7
#define LED_PFC_OVERCURRENT_BLINK_NUM           2
#define LED_PFC_OVER_TEMPERATURE_BLINK_NUM      3
#define LED_PFC_SECONDARY_SIDE_ERROR_BLINK_NUM  4       // Not used

/* Blinking periods */
#define LED_BLINK_PERIOD_VOLTAGE_ERROR          LED_BLINK_PERIOD_LONG_MS        /* LED blinking period for voltage related faults */
#define LED_BLINK_PERIOD_NOVOLTAGE_ERROR        LED_BLINK_PERIOD_SHORT_MS       /* LED blinking period for no voltage related faults */
    
/**
  * @}
  */
       
/**
  * @}
  */
    
/** @defgroup DCDC_exported_types DSMPS exported types
* @{
*/


#ifndef __cplusplus
typedef enum {FALSE,TRUE} bool;
#endif

/** 
  * @brief  PI parameters type definition 
  */
 typedef struct 
 {  
   int16_t hKp_Gain;                    /*!< proportional gain */
   uint16_t hKp_Divisor;                /*!< proportional gain divider */
   int16_t hKi_Gain;                    /*!< integral gain */
   uint16_t hKi_Divisor;                /*!< integral gain divider */
   int16_t hLower_Limit_Output;         /*!< lower limit for PID output */
   int16_t hUpper_Limit_Output;         /*!< upper limit for PID output */
   int32_t wLower_Limit_Integral;       /*!< lower limit for integral term */        
   int32_t wUpper_Limit_Integral;       /*!< upper limit for integral term */        
   int32_t wIntegral;                   /*!< integral term */        
   // Actually used only if DIFFERENTIAL_TERM_ENABLED is enabled in PID_regulators.h file
   int16_t hKd_Gain;                    /*!< derivative gain */
   uint16_t hKd_Divisor;                /*!< derivative gain divider */
   int32_t wPreviousError;              /*!< previous error */
 } PID_Struct_t;

 /** 
 * @brief  PI parameters extended type definition - for HRTIM control: the maximum number of timer ticks (U16) che be grater than a S16
  */
 typedef struct 
 {  
   int16_t hKp_Gain;
   uint16_t hKp_Divisor;
   int16_t hKi_Gain;
   uint16_t hKi_Divisor;  
   int32_t wLower_Limit_Output;     //Lower Limit for Output limitation
   int32_t wUpper_Limit_Output;     //Lower Limit for Output limitation
   int32_t wLower_Limit_Integral;   //Lower Limit for Integral term limitation
   int32_t wUpper_Limit_Integral;   //Lower Limit for Integral term limitation
   int32_t wIntegral;
   // Actually used only if DIFFERENTIAL_TERM_ENABLED is enabled 
   int16_t hKd_Gain;
   uint16_t hKd_Divisor;
   int32_t wPreviousError;
 } PID_Struct_ex_t;
/** 
  * @brief enum type definition, it lists all the possible state machine states  
  */

typedef enum
{
DSMPS_IDLE,           /*!< Persistent state, following state can be INIT if all required 
                      input conditions are satisfied or STOP if there is a fault 
                      condition */
DSMPS_INIT,          /*!<"Pass-through" state, the code to be executed only once 
                     between IDLE and START states to initialize control variables.
                     Following state is normally START but it can
                     also be STOP if there is a fault condition */
DSMPS_START,          /*!< Persistent state where the converter's start-up is intended to be
                     executed. The following state is normally 
                     RUN as soon the bus reference voltage is reached. Another 
                     possible following state is STOP if there is a fault condition */
DSMPS_RUN,            /*!< Persistent state with converter running. The following state 
                     is STOP if there is a fault condition */
DSMPS_STOP,           /*!< "Pass-through" state where PWM are disabled. The state machine 
                      can be moved from any condition directly to this state by 
                      SMPS_FaultCheck function. following state is FAULT */
DSMPS_FAULT,          /*!< Persistent state after a fault. Following state is normally
                     WAIT as soon as conditions for moving state machine are detected */
DSMPS_WAIT,       	/*!< Persistent state where the application is intended to stay
                     after the fault conditions disappeared for a defined wait time.
                     Following state is normally IDLE */
} DSMPS_State_t;


/** 
  * @brief enum type definition, it lists all possible LED's modality
  */

typedef enum 
{  
  LED_OFF = 0,          /*!< LED off until new command */
  LED_ON,               /*!< LED on until new command */
  LED_BLINK_N,          /*!< LED blinks n times  */
  LED_BLINK_INF         /*!< LED blinks until new command */
} LED_Modality_t;

/** 
  * @brief struct type definition, it contains all parameters regarding LED's blinking and state
  */
typedef struct 
{  
  LED_Modality_t LED_Modality;          /*!< LED modality, it can be a value of LED_Modality_t type */
  uint8_t nBlinkCount;                  /*!< total number of LED blinks */
  uint8_t nBlinkLeftCount;              /*!< remaining number of LED blinks */
  bool bActualOn;                       /*!< TRUE if LED is currently on, FALSE otherwise */
  uint16_t hBlinkPeriodms;              /*!< blink period in ms */
  uint16_t hBlinkPeriodCounterLeft;     /*!< remaining period counter in ms */
  uint16_t hBlinkRepetitionPeriodms;    /*!< period between a burst of blinks and the next */
  GPIO_TypeDef* LED_GPIOPort;           /*!< GPIO port of LED */       
  uint16_t LED_GPIOPin;                 /*!< GPIO pin of LED */ 
} LED_Struct_t;
  
/** 
  * @brief enum type definition, it lists the gate driver precharge states
  */

typedef enum 
{  
  PRECHARGE_OFF = 0,            /*!< precharge phase is not started yet */
  PRECHARGE_ONGOING,            /*!< precharge state is started */
  PRECHARGE_HOLD                /*!< precharge state is completed, hold state before enabling PWM */
} Driver_PrechargeState_t;

/** 
  * @brief struct type definition, it contains all parameters regarding driver pre-charge
  */

typedef struct 
{  
  Driver_PrechargeState_t prechargeState;
  bool bPrechargeEnabled;
  bool bPrechargeCompleted;    
} Driver_PrechargeStruct_t;

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __DSMPS_TYPE_H */
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
