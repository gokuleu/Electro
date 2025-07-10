/**
  ******************************************************************************
  * @file    Fault_Processing.c
  * @author  IMS Systems Lab
  * @version V2.2.0
  * @date    31-May-2018
  * @brief   DSMPS fault management
  *          This file provides fuctions to check converter's faults and related thresholds
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "LLC_control_param.h"
#include "LLC_board_config_param.h"
#include "LLC_Globals.h"
#include "StateMachine.h"
#include "Fault_Processing.h"
#include "Control_Layer.h"

/** @addtogroup DSMPS_project
  * @{
  */

/** @addtogroup Fault_Processing
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PRIMARY_STATUS_ERROR_MSK        ((uint8_t)(0xFE))/**< bit mask for satus sent by primary MCU */
#define OVER_TEMP_FAULT_NOT_RECOVERABLE                 /**< if defined the over-temperature fault is not recoverable (the board must be powered off) */
#define OVER_VOLTAGE_OUT_FAULT_NOT_RECOVERABLE          /**< if defined the output over-voltage fault is not recoverable (the board must be powered off) */
#define UNDER_VOLTAGE_OUT_FAULT_NOT_RECOVERABLE         /**< if defined the output under-voltage fault is not recoverable (the board must be powered off) */
#define UNDER_VOLTAGE_OUT_VALIDATION_TIME_mS    30      /**< validation time in ms for undervoltage fault */

/* ----- Fault and burst mode thresholds - for nominal values see LLC_Control_Parameters.h file ----- */
/* Out Voltage thresholds */
#define OUT_VOLTAGE_MAX_H               OUT_VOLT_ADC_VALUE(69.6)                                  /**< max voltage for output overvoltage fault detection [V] - if VOUT_ANALOG_WATCHDOG_ENABLED is not defined, otherwise the WDG threshold is WDG_OUT_VOLTAGE_MAX in LLC_Init_Perif.c file */
#define OUT_VOLTAGE_HYSTERESIS          OUT_VOLT_ADC_VALUE(5.8)                                   /**< hysteresis for output under/over voltage fault detection [V] */
#define OUT_VOLTAGE_MAX_L               (OUT_VOLTAGE_MAX_H - OUT_VOLTAGE_HYSTERESIS)            /**< min voltage for output overvoltage fault cancellation [V] */
#define OUT_VOLTAGE_MIN_L               OUT_VOLT_ADC_VALUE(46.4)                                  /**< min voltage for output undervoltage fault detection [V] */
#define OUT_VOLTAGE_MIN_H               (OUT_VOLTAGE_MIN_L + OUT_VOLTAGE_HYSTERESIS)            /**< max voltage for output undervoltage fault cancellation [V] */

/* Input Voltage thresholds */
#define IN_VOLTAGE_MAX_H               IN_VOLT_ADC_VALUE(425)                                   /**< max voltage for input overvoltage fault detection [V] */
#define IN_VOLTAGE_HYSTERESIS          IN_VOLT_ADC_VALUE(12)                                    /**< hysteresis for input under/over voltage fault detection [V] */
#define IN_VOLTAGE_MAX_L               (IN_VOLTAGE_MAX_H - IN_VOLTAGE_HYSTERESIS)               /**< min voltage for input overvoltage fault cancellation [V] */
#define IN_VOLTAGE_MIN_L               IN_VOLT_ADC_VALUE(380)                                   /**< min voltage for input undervoltage fault detection [V] */
#define IN_VOLTAGE_MIN_H               (IN_VOLTAGE_MIN_L + IN_VOLTAGE_HYSTERESIS)               /**< max voltage for input undervoltage fault cancellation [V] */

/* Burst mode on bus voltage thresholds */
#define OUT_VOLTAGE_BURST_MODE_TH_H     OUT_VOLT_ADC_VALUE(63.8)   /**< max voltage for burst mode activation [V] */
#define OUT_VOLTAGE_BURST_MODE_TH_L     OUT_VOLT_ADC_VALUE(58)   /**< min voltage for burst mode de-activation [V] */

/* Temperature thresholds */
#define TEMPERATURE_TH_H                ((uint16_t)55)                                          /**< max temperature for over-temperature fault detection (higher threshold) [°C] */
#define TEMPERATURE_HYSTERESIS          ((uint16_t)5)                                           /**< hysteresis for over-temperature fault detection [°C] */
#define TEMPERATURE_TH_L                (TEMPERATURE_TH_H - TEMPERATURE_HYSTERESIS)             /**< min temperature for over-temperature fault cancellation [°C] */

/* Out current threshold */
#define OUT_CURRENT_MAX                 HALL_IC_AMP2ADC_VALUE(51)                               /**< max output current for out over-current fault detection [A] */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t PFC_Last_System_Fault = (uint16_t)PFC_BUS_UNDER_VOLT;	        /**< last PFC fault variable communicated via UART (if defined) */
static volatile uint16_t DCDC_System_Fault = (uint16_t)DCDC_NO_ERROR;			/**< fault variable */
static volatile uint16_t DCDC_Last_System_Fault = (uint16_t)DCDC_NO_ERROR;		/**< last fault variable value before stop application */
static volatile uint16_t LED_LastFaultShown = (uint16_t)DCDC_NO_ERROR;                  /**< last DCDC fault shown with LED blinking */
static volatile bool bFirstFaultDetected = FALSE;                                       /**< TRUE when the first fault to show with LED blinking was found, FALSE otherwise */

/* Private function prototypes -----------------------------------------------*/
static void FLT_OutVoltageCheck(void);
static void FLT_InputVoltageCheck(void);
static void FLT_TemperatureCheck(void);
static void FLT_OutCurrentCheck(void);
//static void FLT_PrimaryStatusCheck(uint8_t bPrimaryStatus);
static inline bool CheckSystemFaultAsserted(uint16_t hErrorChecked);
static inline void FLT_SetSystemFault(uint16_t hNewError);
static inline void FLT_ClearSystemFault(uint16_t hErrorCleared);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  This function handles HRTIM global fault interrupt request.
* @param  None
* @retval None
*/
void HRTIM1_FLT_IRQHandler(void)
{
  /* Disable outputs (already in idle state) */
//  if(bDCDC_OutputEnabled == TRUE){
    /* Disable PWMs */
//    CTR_PWMOutputDisable(&hhrtim);

    /* Disable all PWMs outputs */
    CTR_PWMAllOutputsDisable(&hhrtim);
    /* reset config enable flag */
    DCDC_ConfigParamStruct.bConverterEnabled = FALSE;
//  }
#ifdef SYNCH_RECTIFICATION
//  if(bDCDC_SynchRectOutputEnabled == TRUE){
    /* Disable output SR PWMs */
//    CTR_PWMSynchRectOutputDisable(&hhrtim);
    /* Disable gate drivers for SR PWMs */
    CTR_SRGateDriverDisable();
//  }
#endif

  /* Set Overcurrent error */
  FLT_SetSystemFault(DCDC_OVER_CURRENT_ERROR);

  /* Update LED blinking if last error is changed */
  if((LED_LastFaultShown != DCDC_OVER_CURRENT_ERROR) && (bFirstFaultDetected == FALSE)){
    /* store last fault shown on LED */
    LED_LastFaultShown = DCDC_OVER_CURRENT_ERROR;
    /* set fault LED Blinking */
    LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_OVER_CURRENT_BLINK_NUM, LED_BLINK_PERIOD_NOVOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
  }
  bFirstFaultDetected = TRUE;
  if(STM_GetStateMachineStatus() != DSMPS_FAULT){
    STM_SetStateMachineStatus(DSMPS_STOP);
  }
  /* Disable Fault interrupt: overcurrent error is not recoverable */
  HAL_HRTIM_FaultModeCtl(&hhrtim, HRTIM_FAULT_OC, HRTIM_FAULTMODECTL_DISABLED);

  /* Clear directly the fault flag to reduce execution time */
  __HAL_HRTIM_CLEAR_IT(&hhrtim, HRTIM_IT_FLT_OC);
}

#ifdef VOUT_ANALOG_WATCHDOG_ENABLED
/**
  * @brief  This function handles ADC interrupt request (AWDG1 Irq).
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void)
{
    /* Disable all PWMs outputs */
    CTR_PWMAllOutputsDisable(&hhrtim);
    /* reset config enable flag */
    DCDC_ConfigParamStruct.bConverterEnabled = FALSE;
//  }
#ifdef SYNCH_RECTIFICATION
    /* Disable gate drivers for SR PWMs */
    CTR_SRGateDriverDisable();
#endif

#ifdef LIGHT_LOAD_BURST_MODE
      CTR_BurstModeDisable(&hhrtim);
#endif

  /* Set Overvoltage error */
  FLT_SetSystemFault(DCDC_OUT_OVER_VOLT_ERROR);

  /* Update LED blinking if last error is changed */
  if((LED_LastFaultShown != DCDC_OUT_OVER_VOLT_ERROR) && (bFirstFaultDetected == FALSE)){
    /* store last fault shown on LED */
    LED_LastFaultShown = DCDC_OUT_OVER_VOLT_ERROR;
    /* set fault LED Blinking */
    LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_OUT_OVER_VOLTAGE_BLINK_NUM, LED_BLINK_PERIOD_VOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
  }
  bFirstFaultDetected = TRUE;
  if(STM_GetStateMachineStatus() != DSMPS_FAULT){
    STM_SetStateMachineStatus(DSMPS_STOP);
  }

#ifdef OVERCURRENT_PROTECTION
    /* Disable Overcurrent protection fault at next start-up only if STOP state is set by communication or debugger */
//    HAL_HRTIM_FaultModeCtl(&hhrtim, HRTIM_FAULT_OC, HRTIM_FAULTMODECTL_DISABLED);
#endif

  /* Disable AWD1 Irq: overvoltage error is not recoverable*/
  __HAL_ADC_DISABLE_IT(&AdcHandle1, ADC_IT_AWD1);

  /* Clear directly AWD1 conversion flag to speed-up the code */
  __HAL_ADC_CLEAR_FLAG(&AdcHandle1, ADC_FLAG_AWD1);

}
#endif

/**
  * @brief  This function checks if a fault has occured
  * @param  None
  * @retval None
  *
  *  this function calls \a FLT_TemperatureCheck(), \a FLT_InputVoltageCheck(), \a FLT_OutVoltageCheck(), and \a FLT_OutCurrentCheck()
  */
void FLT_FaultCheck(void){

  /* reset first fault detected flag */
  bFirstFaultDetected = FALSE;

  /* check output voltage */
  FLT_OutVoltageCheck();

#ifdef INPUT_UNDER_OVER_VOLTAGE_PROTECTION
  /* check input voltage */
  FLT_InputVoltageCheck();
#endif

#ifdef OUT_OVERCURRENT_PROTECTION
  /* check output current */
  FLT_OutCurrentCheck();
#endif

#ifdef OVERTEMPERATURE_PROTECTION
  if(bCheckOvertemperatureEnabled == TRUE){
    /* Check temperature */
    FLT_TemperatureCheck();
  }
#endif

#ifdef DSMPS_COMMUNICATION
  /* Check Primary status */
//  FLT_PrimaryStatusCheck(PFC_Received_Status);       // COMMENT TO DISABLE PFC STATUS CHECK or define START_WITHOUT_PRIMARY_COMMUNICATION in DCDC_control_prm.h file
#endif

}

/**
  * @brief  This function checks if an overtemperature has occurred
  * @param  None
  * @retval None
  *
  *  When a overtemperature condition is present, the \a System_Fault is updated and \a FAULT status is set
  */
void FLT_TemperatureCheck(void){

  /* overtemperature protection */
  if ((fTemperatureDeg > TEMPERATURE_TH_H) || ((fTemperatureDeg > TEMPERATURE_TH_L) && (CheckSystemFaultAsserted(DCDC_OVER_TEMP_ERROR) == TRUE))){
    FLT_SetSystemFault(DCDC_OVER_TEMP_ERROR);

    /* Update LED blinking if last error is changed */
    if((LED_LastFaultShown != DCDC_OVER_TEMP_ERROR) && (bFirstFaultDetected == FALSE)){
      /* store last fault shown on LED */
      LED_LastFaultShown = DCDC_OVER_TEMP_ERROR;
      /* set fault LED Blinking */
      LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_OVER_TEMPERATURE_BLINK_NUM, LED_BLINK_PERIOD_NOVOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
    }
    bFirstFaultDetected = TRUE;
    if(STM_GetStateMachineStatus() != DSMPS_FAULT){
      STM_SetStateMachineStatus(DSMPS_STOP);
    }
  }
#ifndef OVER_TEMP_FAULT_NOT_RECOVERABLE
  else{
    /* clear over temperature flag */
    FLT_ClearSystemFault(DCDC_OVER_TEMP);
  }
#endif
}

/**
  * @brief  This function checks if an output overcurrent has occurred
  * @param  None
  * @retval None
  *
  *  When an output overcurrent condition is present, the \a System_Fault is updated and \a FAULT status is set, not recoverable fault
  */
void FLT_OutCurrentCheck(void){

  if(bCheckOutOvercurrentEnabled == TRUE){
    /* out overcurrent protection */
    if (DCDC_MeasureStruct.hIout > OUT_CURRENT_MAX){
      FLT_SetSystemFault(DCDC_OUT_OVER_CURRENT_ERROR);

      /* Update LED blinking if last error is changed */
      if((LED_LastFaultShown != DCDC_OUT_OVER_CURRENT_ERROR) && (bFirstFaultDetected == FALSE)){
        /* store last fault shown on LED */
        LED_LastFaultShown = DCDC_OUT_OVER_CURRENT_ERROR;
        /* set fault LED Blinking */
        LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_OUT_OVER_CURRENT_BLINK_NUM, LED_BLINK_PERIOD_NOVOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
      }
      bFirstFaultDetected = TRUE;
      if(STM_GetStateMachineStatus() != DSMPS_FAULT){
        STM_SetStateMachineStatus(DSMPS_STOP);
      }
    }
  }
}

/**
  * @brief  This function checks if an over/under voltage on DC output voltage has occurred
  * @param  None
  * @retval None
  *
  *  When a an over/under voltage on DC output voltage condition is present, the \a System_Fault is updated and burst mode or \a FAULT status is set
  */
void FLT_OutVoltageCheck(void){

  /* Out_Voltage filtered before Voltage Control loop */
#ifndef VOUT_ANALOG_WATCHDOG_ENABLED
  /* overvoltage OUT_H protection */
  if ((hVoutVoltageFiltered > OUT_VOLTAGE_MAX_H)||((hVoutVoltageFiltered > OUT_VOLTAGE_MAX_L)&&(CheckSystemFaultAsserted(DCDC_OUT_OVER_VOLT_ERROR) == TRUE)))
    {
      /* set system fault */
      FLT_SetSystemFault(DCDC_OUT_OVER_VOLT_ERROR);
      /* Update LED blinking if last error is changed */
      if((LED_LastFaultShown != DCDC_OUT_OVER_VOLT_ERROR) && (bFirstFaultDetected == FALSE)){
        /* store last fault shown on LED */
        LED_LastFaultShown = DCDC_OUT_OVER_VOLT_ERROR;
        /* set fault LED Blinking */
        LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_OUT_OVER_VOLTAGE_BLINK_NUM, LED_BLINK_PERIOD_VOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
      }
      bFirstFaultDetected = TRUE;
      if(STM_GetStateMachineStatus() != DSMPS_FAULT){
        STM_SetStateMachineStatus(DSMPS_STOP);
      }
    }
  #ifndef OVER_VOLTAGE_OUT_FAULT_NOT_RECOVERABLE
    else{
      FLT_ClearSystemFault(DCDC_OUT_OVER_VOLT);
      }
  #endif
#endif

#ifdef OUT_VOLTAGE_BURST_MODE
  /* Disabling PWM outputs before over-voltage protection */
  if ((hVoutVoltageFiltered > OUT_VOLTAGE_BURST_MODE_TH_H)||((hVoutVoltageFiltered > OUT_VOLTAGE_BURST_MODE_TH_L))&&(bVoutBurstModeEnabled == TRUE)){
    if(bVoutBurstModeEnabled == FALSE){// disable outputs once
//      /* PWM Output disable */
//      CTR_PWMOutputDisable(&hhrtim);
//
//      #ifdef SYNCH_RECTIFICATION
//        /* PWM SR Output disable */
//        CTR_PWMSynchRectOutputDisable(&hhrtim);
//      #endif

      /* Disable all PWMs outputs */
      CTR_PWMAllOutputsDisable(&hhrtim);

      /* disable also light load burst mode to avoid overcurrent at PWM restart */
      CTR_BurstModeDisable(&hhrtim);

      /* call burst mode control fnc with a current greather then the threshold needed to set BMRange at BM_NOBURST value */
      CTR_BurstModeControl(BURST_MODE_IOUT_RANGE23_TH_H + 1, &hhrtim); // 2 BURST THRESHOLDS

      /* set burst mode variable */
      bVoutBurstModeEnabled = TRUE;
    }
  }
  else{     // within the correct boundaries
    if(bVoutBurstModeEnabled == TRUE){ // enable outputs once
      /* PWM Output enable */
      CTR_PWMFastOutputEnable(&hhrtim);
//      CTR_PWMOutputEnable(&hhrtim);
//      #ifdef SYNCH_RECTIFICATION
//        /* PWM SR Output enable */
//        CTR_PWMSynchRectOutputEnable(&hhrtim);
//      #endif
      /* reset burst mode variable */
      bVoutBurstModeEnabled = FALSE;
    }
  }
#endif

#ifdef VOUT_UNDERVOLTAGE_PROTECTION
  /* undervoltage OUT protection */
  if(bCheckUndervoltageEnabled == TRUE){

    static bool bFirstUndervoltageCheck = TRUE;

    /* Vbus undervoltage */
    if ((hVoutVoltageFiltered < OUT_VOLTAGE_MIN_L) || ((hVoutVoltageFiltered < OUT_VOLTAGE_MIN_H)&&(CheckSystemFaultAsserted(DCDC_OUT_UNDER_VOLT_ERROR) == TRUE)))
    {
      if(bFirstUndervoltageCheck == TRUE){
        /* set delay time to validate undervoltage protection */
        SetDelayTime(&hDSMPS_UndervoltageValidation_TimeLeft, UNDER_VOLTAGE_OUT_VALIDATION_TIME_mS);
        bFirstUndervoltageCheck = FALSE;
      }
      else{
        if(DelayTimeIsElapsed(&hDSMPS_UndervoltageValidation_TimeLeft)){
          FLT_SetSystemFault(DCDC_OUT_UNDER_VOLT_ERROR);
          /* Update LED blinking if last error is changed */
          if((LED_LastFaultShown != DCDC_OUT_UNDER_VOLT_ERROR) && (bFirstFaultDetected == FALSE)){
            /* store last fault shown on LED */
            LED_LastFaultShown = DCDC_OUT_UNDER_VOLT_ERROR;
            /* set fault LED Blinking */
            LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_OUT_UNDER_VOLTAGE_BLINK_NUM, LED_BLINK_PERIOD_VOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
          }
          bFirstFaultDetected = TRUE;
          if(STM_GetStateMachineStatus() != DSMPS_FAULT){
            STM_SetStateMachineStatus(DSMPS_STOP);
          }
        }
      }
    }
    else{
      bFirstUndervoltageCheck = TRUE;
#ifndef UNDER_VOLTAGE_OUT_FAULT_NOT_RECOVERABLE
      FLT_ClearSystemFault(DCDC_OUT_UNDER_VOLT_ERROR);
#endif
    }

  }
#endif

}

/**
  * @brief  This function checks if an over/under voltage on DC input voltage has occurred
  * @param  None
  * @retval None
  *
  *  When a an over/under voltage on DC input voltage condition is present, the \a System_Fault is updated and \a FAULT status is set
  */
void FLT_InputVoltageCheck(void){

  /* Input overvoltage check */
  if ((hVinVoltageFiltered > IN_VOLTAGE_MAX_H)||((hVinVoltageFiltered > IN_VOLTAGE_MAX_L)&&(CheckSystemFaultAsserted(DCDC_IN_OVER_VOLT_ERROR) == TRUE)))
  {
    /* set system fault */
    FLT_SetSystemFault(DCDC_IN_OVER_VOLT_ERROR);
    /* Update LED blinking if last error is changed */
    if((LED_LastFaultShown != DCDC_IN_OVER_VOLT_ERROR) && (bFirstFaultDetected == FALSE)){
      /* store last fault shown on LED */
      LED_LastFaultShown = DCDC_IN_OVER_VOLT_ERROR;
      /* set fault LED Blinking */
      LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_IN_OVER_VOLTAGE_BLINK_NUM, LED_BLINK_PERIOD_VOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
    }
    bFirstFaultDetected = TRUE;
    if(STM_GetStateMachineStatus() != DSMPS_FAULT){
      STM_SetStateMachineStatus(DSMPS_STOP);
    }
  }
  else{
    FLT_ClearSystemFault(DCDC_IN_OVER_VOLT_ERROR);
    /* clear last fault shown if it is DCDC_IN_OVER_VOLT_ERROR */
    if(LED_LastFaultShown == DCDC_IN_OVER_VOLT_ERROR)
    {
      LED_LastFaultShown = 0;
    }
  }

  /* Input undervoltage check */
  // if ((hVinVoltageFiltered < IN_VOLTAGE_MIN_L) || ((hVinVoltageFiltered < IN_VOLTAGE_MIN_H)&&(CheckSystemFaultAsserted(DCDC_IN_UNDER_VOLT_ERROR) == TRUE)))
  //   {
  //     FLT_SetSystemFault(DCDC_IN_UNDER_VOLT_ERROR);
  //     /* Update LED blinking if last error is changed */
  //     if((LED_LastFaultShown != DCDC_IN_UNDER_VOLT_ERROR) && (bFirstFaultDetected == FALSE)){
  //       /* store last fault shown on LED */
  //       LED_LastFaultShown = DCDC_IN_UNDER_VOLT_ERROR;
  //       /* set fault LED Blinking */
  //       LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_IN_UNDER_VOLTAGE_BLINK_NUM, LED_BLINK_PERIOD_VOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
  //     }
  //     bFirstFaultDetected = TRUE;
  //     if(STM_GetStateMachineStatus() != DSMPS_FAULT){
  //       STM_SetStateMachineStatus(DSMPS_STOP);
  //     }
  //   }
  //   else{
  //     /* clear system fault */
  //     FLT_ClearSystemFault(DCDC_IN_UNDER_VOLT_ERROR);
  //     /* clear last fault shown if it is DCDC_IN_UNDER_VOLT_ERROR */
  //     if(LED_LastFaultShown == DCDC_IN_UNDER_VOLT_ERROR)
  //     {
  //       LED_LastFaultShown = 0;
  //     }
  //   }
}

/**
  * @brief  This function checks if an error is communicated by PFC
  * @param  bPrimaryStatus: status received with serial communication
  * @retval None
  *
  */
//void FLT_PrimaryStatusCheck(uint8_t bPrimaryStatus) // TO REVIEW!!!
//{
//  if(bPrimaryStatus != ((uint16_t)PFC_START_UP_OK)) //also a not complete start-up of PFC is an error, so syestem goes in WAIT state after PFC ramp-up
//  {
//    /* store PFC status variable before fault */
//    PFC_Last_System_Fault = bPrimaryStatus;
////    DCDC_System_Fault |= DCDC_RECEIVED_ERROR;
////    DSMPS_State = DSMPS_STOP;
//    /* set system fault */
//    FLT_SetSystemFault(DCDC_PRIMARY_SIDE_ERROR);
//    /* Update LED blinking if last error is changed */
//    if((LED_LastFaultShown != DCDC_PRIMARY_SIDE_ERROR) && (bFirstFaultDetected == FALSE)){
//      /* store last fault shown on LED */
//      LED_LastFaultShown = DCDC_PRIMARY_SIDE_ERROR;
//      /* set fault LED Blinking */
//      LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_PRIMARY_SIDE_ERROR_BLINK_NUM, LED_BLINK_PERIOD_NOVOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
//    }
//    bFirstFaultDetected = TRUE;
//    /* set state machine in stop state to disable PWM if it is not already in DSMPS_FAULT state */
//    if(STM_GetStateMachineStatus() != DSMPS_FAULT){
//      /* set stop state */
//      STM_SetStateMachineStatus(DSMPS_STOP);
//    }
//  }
//  else{
//    /* clear fault */
//    FLT_ClearSystemFault(DCDC_PRIMARY_SIDE_ERROR);
//  }
//}

/**
  * @brief  Set start-up failed procedure
  * @param  None
  * @retval None
  *
  */
void FLT_StartUpFailedProcedure(void)
{
  /* Set Overcurrent error */
  FLT_SetSystemFault(DCDC_STARTUP_FAILED_ERROR);

  /* Update LED blinking if last error is changed */
  if((LED_LastFaultShown != DCDC_STARTUP_FAILED_ERROR) && (bFirstFaultDetected == FALSE)){
    /* store last fault shown on LED */
    LED_LastFaultShown = DCDC_STARTUP_FAILED_ERROR;
    /* set fault LED Blinking */
    LED_SetParams(&FaultLED, LED_BLINK_N, LED_DCDC_IN_START_UP_FAILED_BLINK_NUM, LED_BLINK_PERIOD_VOLTAGE_ERROR, LED_BLINK_REPETITION_PERIOD_MS);
  }
  bFirstFaultDetected = TRUE;
  if(STM_GetStateMachineStatus() != DSMPS_FAULT){
    STM_SetStateMachineStatus(DSMPS_STOP);
  }
}

/**
  * @brief  Set system fault error
  * @param  hNewError: or-ed errors to set
  * @retval None
  */
void FLT_SetSystemFault(uint16_t hNewError)
{
  DCDC_System_Fault |= hNewError;
}

/**
  * @brief  Get system fault error
  * @param  None
  * @retval system_err: or-ed errors to set
  */
inline uint16_t FLT_GetSystemFault(void)
{
  return DCDC_System_Fault;
}

/**
  * @brief  Get last shown system fault error
  * @param  None
  * @retval system_err: last shown error
  */
uint16_t FLT_GetLastShownSystemFault(void)
{
  return LED_LastFaultShown;
}

/**
  * @brief  Clear a fault error
  * @param  system_err: or-ed errors to clear
  * @retval None
  */
void FLT_ClearSystemFault(uint16_t hErrorCleared)
{
  DCDC_System_Fault &= ~hErrorCleared;
}

/**
  * @brief  Clear all fault errors
  * @param  None
  * @retval None
  */
void FLT_ClearAllSystemFaults(void)
{
  /* reset current system fault */
  DCDC_System_Fault = DCDC_NO_ERROR;

  /* reset last shown system fault */
  LED_LastFaultShown = DCDC_NO_ERROR;

  /* reset last system fault */
  DCDC_Last_System_Fault = DCDC_NO_ERROR;

  /* set fault LED off */
  LED_SetParams(&FaultLED, LED_OFF, 0, LED_BLINK_PERIOD_LONG_MS, LED_BLINK_REPETITION_PERIOD_MS);
}

/**
  * @brief  Store last system fault
  * @param  hErrorStored: error to be stored
  * @retval None
  *
  * called in DSMPS_STOP state after PWMs disabling
  */
void FLT_StoreLastSystemFault(void)
{
  DCDC_Last_System_Fault = DCDC_System_Fault;
}

/**
  * @brief  check if a system fault error is asserted
  * @param  hErrorChecked: error to check
  * @retval TRUE if the error is asserted, FALSE otherwise
  */
inline bool CheckSystemFaultAsserted(uint16_t hErrorChecked)
{
  if((DCDC_System_Fault & hErrorChecked) == hErrorChecked)
  {
    return TRUE;
  }
  else{
    return FALSE;
  }
}

/**
  * @}
  */

/**
  * @}
  */

/******************** (C) COPYRIGHT STMicroelectronics *******************/
