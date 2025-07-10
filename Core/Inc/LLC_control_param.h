/**
  ******************************************************************************
  * @file    LLC_control_param.h
  * @author  IMS Systems Lab and Technical Marketing
  * @version 2.1.0
  * @date    24-May-2017
  * @brief   This file contains control parameters, frequencies and user defines for
  *          each modality
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
#ifndef __LLC_CONTROL_PARAM_H
#define __LLC_CONTROL_PARAM_H

/* Includes ------------------------------------------------------------------*/
#include "HRTIM_pwm_config_param.h"
#include "LLC_board_config_param.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*-- User's Modalities ------------------------------------------------------ */
#define USE_EXTENDED_PID                       /**< if defined extended PIDs are used with int32_t variables (it is needed when f_min < 140626 with max resolution of HRTIM */

/* Start-up ------------------*/
#define START_WITHOUT_COMMAND                   /**< if defined the power conversion starts without any command if there is no error, otherwise a command from UI or debugger is needed */
#define FREQUENCY_RAMP_UP                       /**< if defined a ramp-up of frequency is performed in START state */
#define GATE_DRIVER_BOOTSTRAP_PRECHARGE         /**< if defined the bootstrap capacitors are precharged closing lowside switches for a control loop time duration */

/* Communication -------------*/
//#define UI_COMMUNICATION                /**< if defined User Interface communication is enabled (to implement UI protocol) */

/* Protections and burst modes ------------*/
#define OUT_VOLTAGE_BURST_MODE                  /**< if defined PWM signals are stopped if output voltage goes above defined thresholds and re-enabled with hysteresys */
#define LIGHT_LOAD_BURST_MODE                   /**< if defined PWM signals are in burst for light loads */
#define OVERCURRENT_PROTECTION                  /**< if defined when the transformer's current is above threshold, the DC/DC is disabled */
#define OUT_OVERCURRENT_PROTECTION              /**< if defined when the output current is above threshold, the DC/DC is disabled */
#define OVERTEMPERATURE_PROTECTION              /**< if defined temperature measure is filtered, converted in Celsius degrees and used to shut-down the PFC */
#define INPUT_UNDER_OVER_VOLTAGE_PROTECTION     /**< if defined it is checked if input voltage is inside the correct range */
//#define VOUT_ANALOG_WATCHDOG_ENABLED            /**< if defined the overvoltage protection is made with the analog watchdog */
#define VOUT_UNDERVOLTAGE_PROTECTION            /**< if defined when the output voltage is below the threshold during DSMPS_RUN state, the DC/DC is disabled */
#define START_UP_FAILED_ENABLED                 /**< if defined DCDC_STARTUP_FAILED_ERROR is set in START state if start-up time is elapsed without reaching reference voltage */

/* Debug --------------------*/
#define OPEN_LOOP_MODE                /**< if defined SR delays and PWM frequency are initially set via debugger or communication (operation mode can be changed runtime)- OPEN LOOP operation, WARNING: output voltage MUST be clamped @48V */
#define DEBUG_COMP_OC_OUT             /**< if defined COMP_OC output is enabled */
//#define DEBUG_COMP_SR_OUT             /**< if defined COMP_SR outputs are enabled */

/* Additional driving -------*/
#define SYNCH_RECTIFICATION             /**< if defined synchronous rectification is enabled to reduce conduction losses - PWMs depend on hSR_DelayRising1-2 and hSR_DelayFalling1-2 variables */
//#define ADAPTIVE_SYNCH_RECTIFICATION    /**< if defined synchronous rectification is enabled to reduce conduction losses - PWMs depend on COMP outputs and turn-off is adapted respect to VDS sensing - it must be define together with SYNCH_RECTIFICATION */
//#define AUTOMATIC_SR_TURN_ON_OFF        /**< if defined SR is automatically turned on and off depending on SR_IOUT_TURN_ON_THRESHOLD and SR_IOUT_TURN_OFF_THRESHOLD */
//#define FAN_PWM_DRIVING                 /**< if defined the FAN is driven by PWM, otherwise the FAN is driven as a GPIO */

/* initial state of additional drivings and burst if defined */
#define SYNCH_RECTIFICATION_INIT_ENABLED                /**< if defined synchronous rectification is initially enabled, the set can be changend runtime */
//#define ADAPTIVE_SYNCH_RECTIFICATION_INIT_ENABLED       /**< if defined adaptive synchronous rectification (with adaptive turn-off) is initially enabled, the set can be changend runtime */
#define FAN_PWM_DRIVING_INIT_ENABLED                    /**< if defined fan PWM driving is initially enabled, the set can be changend runtime */
#define LIGHT_LOAD_BURST_MODE_INIT_ENABLED              /**< if defined, together with LIGHT_LOAD_BURST_MODE, burst mode is set for low load condition */

/* Additional features */
//#define OUT_CURRENT_SENSOR_CALIBRATION                  /**< if defined output current sensor offset is calibrated and both SR and BM threshold are computed considering the updated offset value */


//#define AUTODELAYED_ADCy_INJECTED_TRIGGER
                                                /**< if defined Vds of SR2 is acquired after ADCy_INJECTED_TRIGGER_POINT_NS of PWM_SR2, but if (0 < hSR_DelayFalling2 < ADCy_INJECTED_TRIGGER_POINT_NS)
                                                * the autodelayed trigger will be above the PWM period and it will be disabled: auto-delayed Compare is only valid from the capture up to the period event (RM pag 633).
                                                * if it is not defined, the CMP register is written with an additional writing that considers also the PWM period overflow, to avoid the missing of the trigger in this condition.
                                                */

/*-----------------------------------------------------------------------------*/

/* -- modalities error handling --------------------------------------------- */
#if ((defined(ADAPTIVE_SYNCH_RECTIFICATION) && !defined(SYNCH_RECTIFICATION)))
	#error "ADAPTIVE_SYNCH_RECTIFICATION must be defined together with SYNCH_RECTIFICATION"
#endif

/* -- SPECIFIC APPLICATION PARAMETERS: LLC CONVERTER --------------------------*/

/* -- Frequency Max, Min and Init values -- */
#define HRTIM_MAX_PWM_FREQ_START_UP_HZ          346000     /**< maximum frequency in Hz of PWM signals at the beginning of start-up procedure */
#define HRTIM_MIN_PWM_FREQ_START_UP_HZ          136000     /**< minimum frequency in Hz of PWM signals at the end of start-up procedure for closed loop operation if the control loop is not closed before */
#define HRTIM_MAX_PWM_FREQ_HZ                   247000     /**< maximum frequency in Hz of PWM signals during closed loop regulation : at zero load with 240kHz -> Vout = 48.3 */
#define HRTIM_MIN_PWM_FREQ_HZ                   136000     /**< minimum frequency in Hz of PWM signals during closed loop regulation - WARNING: if f_min < 140626 Hz, extended PID must be used because period_max > S16_MAX */
#define HRTIM_REP_RATE                          0          /**< Repetition rate of HRTIM */

#ifdef FREQUENCY_RAMP_UP
  #define HRTIM_INIT_PWM_FREQ_HZ          HRTIM_MAX_PWM_FREQ_START_UP_HZ        /**< initial or default frequency in Hz of PWM signals */
#else
  #define HRTIM_INIT_PWM_FREQ_HZ          HRTIM_MAX_PWM_FREQ_HZ                 /**< initial or default frequency in Hz of PWM signals */
#endif

/* -- Synchronous Rectification min values -- */
#define SYNCH_RECT_DELAY_RISING1_MIN_NS     ((int16_t)0)         /**< min delay in ns between FB PWM and SR1 PWM rising edges in ns - only for adaptive SR and on-fly delay setting */
#define SYNCH_RECT_DELAY_RISING2_MIN_NS     ((int16_t)0)         /**< min delay in ns between FB PWM and SR2 PWM rising edges in ns - only for adaptive SR and on-fly delay setting */
#define SYNCH_RECT_DELAY_FALLING1_MIN_NS    ((int16_t)50)        /**< min delay in ns between FB PWM and SR1 PWM falling edges in ns - only for an adaptive SR */
#define SYNCH_RECT_DELAY_FALLING2_MIN_NS    ((int16_t)50)        /**< min delay in ns between FB PWM and SR2 PWM falling edges in ns - only for an adaptive SR - WARNING: Consider that auto-delayed Compare is only valid from the capture up to the period event (RM pag 633)*/

/*-- MAX values --------------------------------------------------------------*/
/* -- Synchronous Rectification max values -- */
#define SYNCH_RECT_DELAY_RISING1_MAX_NS     ((int16_t)400)      /**< max delay in ns between FB PWM and SR1 PWM rising edges in ns - only for an adaptive SR */
#define SYNCH_RECT_DELAY_FALLING1_MAX_NS    ((int16_t)400)      /**< max delay in ns between FB PWM and SR1 PWM rising edges in ns - only for an adaptive SR */
#define SYNCH_RECT_DELAY_RISING2_MAX_NS     ((int16_t)400)      /**< max delay in ns between FB PWM and SR2 PWM rising edges in ns - only for an adaptive SR */
#define SYNCH_RECT_DELAY_FALLING2_MAX_NS    ((int16_t)400)      /**< max delay in ns between FB PWM and SR2 PWM rising edges in ns - only for an adaptive SR */

/* -- Synchronous Rectification init values -- */
#define SYNCH_RECT_DELAY_RISING1_INIT_NS     ((int16_t)(250))   /**< init rising delay for SR1 in ns */
#define SYNCH_RECT_DELAY_FALLING1_INIT_NS    ((int16_t)(250))   /**< init falling delay for SR1 in ns */
#define SYNCH_RECT_DELAY_RISING2_INIT_NS     ((int16_t)(250))   /**< init rising delay for SR2 in ns */
#define SYNCH_RECT_DELAY_FALLING2_INIT_NS    ((int16_t)(250))   /**< init falling delay for SR2 in ns */

/* -- Synchronous Rectification init values at max values -- */
//#define SYNCH_RECT_DELAY_RISING1_INIT_NS     SYNCH_RECT_DELAY_RISING1_MAX_NS    /**< init rising delay for SR1 in ns */
//#define SYNCH_RECT_DELAY_FALLING1_INIT_NS    SYNCH_RECT_DELAY_FALLING1_MAX_NS   /**< init falling delay for SR1 in ns */
//#define SYNCH_RECT_DELAY_RISING2_INIT_NS     SYNCH_RECT_DELAY_RISING2_MAX_NS    /**< init rising delay for SR2 in ns */
//#define SYNCH_RECT_DELAY_FALLING2_INIT_NS    SYNCH_RECT_DELAY_FALLING2_MAX_NS   /**< init falling delay for SR2 in ns */

/* -- Adaptive SR incremental/decremental values -- */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY1_NS       ((int16_t)2)            /**< incremental falling delay1 for adaptive SR in ns */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY2_NS       ((int16_t)5)           /**< incremental falling delay2 for adaptive SR in ns */
#define ADAPTIVE_SYNCH_RECT_DECREMENTAL_DELAY_NS        ((int16_t)2)            /**< decremental falling delay for adaptive SR in ns */

/* -- Adaptive SR thresholds values */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS1_TH1_mV     ((uint16_t)1000)        /**< ADC Vds1 threshold1 in mV to increase/decrease falling delay in adaptive SR */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS1_TH2_mV     ((uint16_t)1200)        /**< ADC Vds1 threshold2 in mV to increase falling delay in adaptive SR */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS2_TH1_mV     ((uint16_t)1000)         /**< ADC Vds2 threshold1 in mV to increase/decrease falling delay in adaptive SR */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS2_TH2_mV     ((uint16_t)1200)         /**< ADC Vds2 threshold2 in mV to increase falling delay in adaptive SR */

/*-- Blanking window duration for COMP turn-off in adaptive SR ---------------*/
#define SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_NS  ((int16_t)580)                  /**< minimum turn-on time for SR1/SR2 signals for adaptive SR in ns */

/*-- Turn-off COMP thresholds for adaptive SR --------------------------------*/
#define DAC_SR1_TURN_OFF_COMP_THRESHOLD_mV      ((uint16_t) 900)                /**< COMP Vds1 threshold in mV on DAC to shut-down PWM in adaptive SR */
#define DAC_SR2_TURN_OFF_COMP_THRESHOLD_mV      ((uint16_t) 900)                /**< COMP Vds2 threshold in mV on DAC to shut-down PWM in adaptive SR */

/*-- Turn-on/off COMP thresholds for adaptive SR -----------------------------*/
#define SR_IOUT_TURN_ON_THRESHOLD_A     ((float)(5))                                          /**< Automatic SR Iout turn-on threshold in Ampere if AUTOMATIC_SR_TURN_ON_OFF is defined */
#define SR_IOUT_TURN_OFF_THRESHOLD_A    ((float)(4))                                          /**< Automatic SR Iout turn-off threshold in Ampere if AUTOMATIC_SR_TURN_ON_OFF is defined */
#define SR_IOUT_TURN_ON_THRESHOLD       HALL_IC_AMP2ADC_VALUE(SR_IOUT_TURN_ON_THRESHOLD_A)      /**< Automatic SR Iout turn-on threshold if AUTOMATIC_SR_TURN_ON_OFF is defined */
#define SR_IOUT_TURN_OFF_THRESHOLD      HALL_IC_AMP2ADC_VALUE(SR_IOUT_TURN_OFF_THRESHOLD_A)     /**< Automatic SR Iout turn-off threshold if AUTOMATIC_SR_TURN_ON_OFF is defined */

/*-- Turn-off COMP thresholds for Over Current Protection --------------------*/
#define DAC_OVERCURRENT_COMP_THRESHOLD          ((uint16_t) 3212)  /**< COMP_OC threshold in range [0;4095] on DAC to shut-down PWMs */

/*-- ADC trigger point -------------------------------------------------------*/
#define ADCx_REGULAR_TRIGGER_POINT_NS   ((uint16_t)(550 + DEAD_TIME_RISING_NS)) /**< ADCx regular acquisition trigger point in ns respect to the beginning of HRTIM period (rollover event) */
#define ADCy_REGULAR_TRIGGER_POINT_NS   ((uint16_t)(550 + DEAD_TIME_FALLING_NS)) /**< ADCy regular acquisition trigger point in ns respect to half HRTIM period */
#define ADCx_INJECTED_TRIGGER_POINT_NS  ((uint16_t)(250))                        /**< ADCx injected acquisition trigger point in ns after PWM SR1 turn-off */
#define ADCy_INJECTED_TRIGGER_POINT_NS  ((uint16_t)(250))                        /**< ADCy injected acquisition trigger point in ns after PWM SR2 turn-off */

/*-- Burst Mode control ------------------------------*/
#define BURST_MODE_IDLE_DURATION_RANGE1   36    /**< if defined LIGHT_LOAD_BURST_MODE, BURST_MODE_IDLE_DURATION_RANGE1 + 1 is the number of HRTIM periods that compose the burst idle duration when the burst mode is active and output current is in range1 */
#define BURST_MODE_PERIOD_RANGE1          39    /**< if defined LIGHT_LOAD_BURST_MODE, BURST_MODE_PERIOD_RANGE1 +1 is the number of HRTIM periods that compose the burst period (idle + run) when the burst mode is active and output current is in range1 */
//#define BURST_MODE_IDLE_DURATION_RANGE2   28    /**< if defined LIGHT_LOAD_BURST_MODE, BURST_MODE_IDLE_DURATION_RANGE2 + 1 is the number of HRTIM periods that compose the burst idle duration when the burst mode is active and output current is in range2 */
//#define BURST_MODE_PERIOD_RANGE2          39    /**< if defined LIGHT_LOAD_BURST_MODE, BURST_MODE_PERIOD_RANGE2 +1 is the number of HRTIM periods that compose the burst period (idle + run) when the burst mode is active and output current is in range2 */
//#define BURST_MODE_IDLE_DURATION_RANGE3   19     /**< if defined LIGHT_LOAD_BURST_MODE, BURST_MODE_IDLE_DURATION_RANGE3 + 1 is the number of HRTIM periods that compose the burst idle duration when the burst mode is active and output current is in range3 */
//#define BURST_MODE_PERIOD_RANGE3          39    /**< if defined LIGHT_LOAD_BURST_MODE, BURST_MODE_PERIOD_RANGE3 +1 is the number of HRTIM periods that compose the burst period (idle + run) when the burst mode is active and output current is in range3 */

#define BURST_MODE_IOUT_RANGE12_TH_L_A    ((float)(0.8))                                                /**< output current low threshold in Ampere for range 1/2 boundary */
#define BURST_MODE_IOUT_RANGE12_TH_H_A    ((float)(1.3))                                                /**< output current high threshold in Ampere for range 1/2 boundary */
#define BURST_MODE_IOUT_RANGE23_TH_L_A    ((float)(3.0))                                                /**< output current low threshold in Ampere for range 2/3 boundary */
#define BURST_MODE_IOUT_RANGE23_TH_H_A    ((float)(4.0))                                                /**< output current high threshold in Ampere for range 2/3 boundary */
#define BURST_MODE_IOUT_RANGE12_TH_L      HALL_IC_AMP2ADC_VALUE(BURST_MODE_IOUT_RANGE12_TH_L_A)         /**< output current low threshold for range 1/2 boundary */
#define BURST_MODE_IOUT_RANGE12_TH_H      HALL_IC_AMP2ADC_VALUE(BURST_MODE_IOUT_RANGE12_TH_H_A)         /**< output current high threshold for range 1/2 boundary */
#define BURST_MODE_IOUT_RANGE23_TH_L      HALL_IC_AMP2ADC_VALUE(BURST_MODE_IOUT_RANGE23_TH_L_A)         /**< output current low threshold for range 2/3 boundary */
#define BURST_MODE_IOUT_RANGE23_TH_H      HALL_IC_AMP2ADC_VALUE(BURST_MODE_IOUT_RANGE23_TH_H_A)         /**< output current high threshold for range 2/3 boundary */

/*-- PWM parameter for FAN control ------------------------------*/
#define FAN_PWM_FREQUENCY               10000                                           /* PWM Frequency in Hz - min 367 Hz with prescaler = 2 - see FAN_TIM_Config() */
#define FAN_PWM_INIT_DUTY               FAN_PWM_DUTY_LOW_SPEED                          /**< init duty cycle for fan control */
#define FAN_PWM_DUTY_LOW_SPEED          ((uint16_t)((30*FAN_PWM_TIM_PERIOD)/100))       /**< low speed duty cycle for fan control */
#define FAN_PWM_DUTY_HIGH_SPEED         ((uint16_t)((97*FAN_PWM_TIM_PERIOD)/100))       /**< high speed duty cycle for fan control */
#define FAN_TEMPERATURE_THRESHOLD_H     43                                              /**< temperature high threshold (for hysteresis): if temperature is above the threshold, the fan is driven at high speed, otherwise the speed depends on phase shift value (if FAN_PWM_DRIVING is defined) */
#define FAN_TEMPERATURE_THRESHOLD_L     38                                              /**< temperature low threshold (for hysteresis): if temperature is above the threshold, the fan is driven at high speed, otherwise the speed depends on phase shift value (if FAN_PWM_DRIVING is defined) */
#define FAN_HIGH_LOAD_THRESHOLD         HALL_IC_AMP2ADC_VALUE(33.2)                       /**< output current threshold to activate max duty cycle for fan */
#define FAN_LOW_LOAD_THRESHOLD          HALL_IC_AMP2ADC_VALUE(5.1)                        /**< output current threshold to activate min duty cycle for fan */




#define FAN_DISABLE_LOAD_THRESHOLD      HALL_IC_AMP2ADC_VALUE(3.5)                      /**< output current threshold to disable fan - not used */
#define FAN_ENABLE_LOAD_THRESHOLD       HALL_IC_AMP2ADC_VALUE(4.5)                      /**< output current threshold to enable fan - not used */

/*---------------------------- TASKS EXECUTION FREQUENCIES ----------------------------*/

/****	ADC IRQ-HANDLER frequency, related to PWM  ****/

/****	Voltage Control loop frequency  ****/
#define VOLTAGE_CONTROL_LOOP_FREQ_HZ    ((uint32_t)50000)               /**< execution frequency of Vout control loop */
#define VOLTAGE_CONTROL_TIM_PRSC        ((uint16_t)0)                   /**< prescaler of TIM that schedules Vout control loop in Hz */

/****	Temperature acquisition and fan control frequency  ****/
#define LOW_FREQUENCY_TASK_FREQ_HZ          ((uint16_t)100)             /**< execution of low frequency task in Hz: fan speed, SR turn-on/off, Vin and Temp Filtering, update config params */
#define LOW_FREQUENCY_TASK_TIM_PRSC         ((uint16_t)9999)            /**< prescaler of TIM that schedules execution of low frequency task in Hz: fan, SR and Burst mode control, Vin, Temp Filtering and update config params */

/*--------------------------- PI CONTROL PARAMETERS --------------------------*/

/************** VOUT PID-CONTROLLER INIT VALUES **************/

#define PID_VOUT_KP_DEFAULT                 ((int16_t)3000)    /**< default Kp value of PI regulator */
#define PID_VOUT_KI_DEFAULT                 ((int16_t)1000)    /**< default Ki value of PI regulator */
#define PID_VOUT_KD_DEFAULT                 ((int16_t)1000)    /**< default Kd value of PI regulator - if used */
#define PID_VOUT_INTEGRAL_UPPER_LIMIT       (PID_VOUT_UPPER_LIMIT * VOUT_KIDIV2)    /**< upper limit of integral term of PI regulator */
#define PID_VOUT_INTEGRAL_LOWER_LIMIT       (PID_VOUT_LOWER_LIMIT * VOUT_KIDIV2)    /**< lower limit of integral term of PI regulator */
#define PID_VOUT_UPPER_LIMIT                (HRTIM_MAX_PWM_PERIOD)                  /**< output upper limit of PI regulator - DEBUG - TO REMOVE 2 FACTOR */
#define PID_VOUT_LOWER_LIMIT                (HRTIM_MIN_PWM_PERIOD)                  /**< output lower limit of PI regulator - DEBUG - TO REMOVE 2 FACTOR */

// Vout PID  parameter dividers
#define VOUT_KPDIV2                         ((uint16_t) (256))    /**< default Kp divider value of PI regulator */
#define VOUT_KIDIV2                         ((uint16_t) (4096))   /**< default Ki divider value of PI regulator */
#define VOUT_KDDIV2                         ((uint16_t) (2048))   /**< default Kd divider value of PI regulator - if used */


/********************** START-UP AND STEADY STATE PARAMETERS **********************/

/** Timings parameters **/
#define DSMPS_STARTUP_TIME_DURATION_mS          ((uint16_t)500)                 /**< duration in ms of DSMPS_START state */
#define DSMPS_WAIT_TIME_DURATION_mS             ((uint16_t)2000)                /**< duration in ms of status DSMPS_WAIT before entering in DSMPS_IDLE state */

/** Voltage regulation value **/
#define STARTUP_FINAL_OUT_VOLTAGE               NOMINAL_OUT_VOLTAGE             /**< reference value in Volts for output voltage at end of start-up */
#define NOMINAL_OUT_VOLTAGE                     OUT_VOLT_ADC_VALUE(58)          /**< reference value in Volts for output voltage */

/*----------  Converted values for HRTIM: do not modify ----------------------*/

/* -- HRTIM period calculation ---------------------------------------------- */
#define HRTIM_MAX_PWM_PERIOD                    (FREQ_HZ_TO_PWM_TICKS(HRTIM_MIN_PWM_FREQ_HZ))
#define HRTIM_MIN_PWM_PERIOD                    (FREQ_HZ_TO_PWM_TICKS(HRTIM_MAX_PWM_FREQ_HZ))
#define HRTIM_MIN_PWM_PERIOD_START_UP           (FREQ_HZ_TO_PWM_TICKS(HRTIM_MAX_PWM_FREQ_START_UP_HZ))
#define HRTIM_MAX_PWM_PERIOD_START_UP           (FREQ_HZ_TO_PWM_TICKS(HRTIM_MIN_PWM_FREQ_START_UP_HZ))
#define HRTIM_INIT_PWM_PERIOD                   (FREQ_HZ_TO_PWM_TICKS(HRTIM_INIT_PWM_FREQ_HZ))

/* Constant delays for synch rect in HRTIM ticks count */
#define SYNCH_RECT_DELAY_RISING1_INIT_HR_TICKS   DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_RISING1_INIT_NS)
#define SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS  DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_FALLING1_INIT_NS)
#define SYNCH_RECT_DELAY_RISING2_INIT_HR_TICKS   DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_RISING2_INIT_NS)
#define SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS  DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_FALLING2_INIT_NS)

#define SYNCH_RECT_DELAY_RISING1_MAX_HR_TICKS    DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_RISING1_MAX_NS)
#define SYNCH_RECT_DELAY_FALLING1_MAX_HR_TICKS   DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_FALLING1_MAX_NS)
#define SYNCH_RECT_DELAY_RISING2_MAX_HR_TICKS    DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_RISING2_MAX_NS)
#define SYNCH_RECT_DELAY_FALLING2_MAX_HR_TICKS   DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_FALLING2_MAX_NS)

#define SYNCH_RECT_DELAY_RISING1_MIN_HR_TICKS    DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_RISING1_MIN_NS)
#define SYNCH_RECT_DELAY_FALLING1_MIN_HR_TICKS   DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_FALLING1_MIN_NS)
#define SYNCH_RECT_DELAY_RISING2_MIN_HR_TICKS    DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_RISING2_MIN_NS)
#define SYNCH_RECT_DELAY_FALLING2_MIN_HR_TICKS   DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_DELAY_FALLING2_MIN_NS)

#define SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS    DELAY_NS_2_HRTIM_TICKS_SIGNED(SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_NS)

#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY1_HR_TICKS  DELAY_NS_2_HRTIM_TICKS_SIGNED(ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY1_NS)
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY2_HR_TICKS  DELAY_NS_2_HRTIM_TICKS_SIGNED(ADAPTIVE_SYNCH_RECT_INCREMENTAL_DELAY2_NS)
#define ADAPTIVE_SYNCH_RECT_DECREMENTAL_DELAY_HR_TICKS  DELAY_NS_2_HRTIM_TICKS_SIGNED(ADAPTIVE_SYNCH_RECT_DECREMENTAL_DELAY_NS)

#define ADCx_REGULAR_TRIGGER_POINT_HR_TICKS     DELAY_NS_2_HRTIM_TICKS_SIGNED(ADCx_REGULAR_TRIGGER_POINT_NS)
#define ADCy_REGULAR_TRIGGER_POINT_HR_TICKS     DELAY_NS_2_HRTIM_TICKS_SIGNED(ADCy_REGULAR_TRIGGER_POINT_NS)
#define ADCx_INJECTED_TRIGGER_POINT_HR_TICKS    DELAY_NS_2_HRTIM_TICKS_SIGNED(ADCx_INJECTED_TRIGGER_POINT_NS)
#define ADCy_INJECTED_TRIGGER_POINT_HR_TICKS    DELAY_NS_2_HRTIM_TICKS_SIGNED(ADCy_INJECTED_TRIGGER_POINT_NS)

/* Constant defines for frequency start-up */
#define START_UP_COUNTER_PRESCALER_RATIO        ((uint16_t)100)    /* ratio between the control frequency and the frequency update during start-up phase */
#define START_UP_NUMBER_OF_INCREASING_STEPS     ((uint16_t)((uint32_t)DSMPS_STARTUP_TIME_DURATION_mS*VOLTAGE_CONTROL_LOOP_FREQ_HZ/START_UP_COUNTER_PRESCALER_RATIO/1000))
#define HRTIM_START_UP_DELTA_PERIOD             ((uint16_t)((HRTIM_MAX_PWM_PERIOD_START_UP - HRTIM_MIN_PWM_PERIOD_START_UP)/START_UP_NUMBER_OF_INCREASING_STEPS))

/* Conversion of DAC and ADC thresholds for adaptive SR */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS1_TH1        mV_2_ADC_DAC_VALUE(ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS1_TH1_mV)    /**< ADC Vds1 threshold1 to increase/decrease falling delay in adaptive SR */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS2_TH1        mV_2_ADC_DAC_VALUE(ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS2_TH1_mV)    /**< ADC Vds2 threshold1 to increase/decrease falling delay in adaptive SR */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS1_TH2        mV_2_ADC_DAC_VALUE(ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS1_TH2_mV)    /**< ADC Vds1 threshold2 to increase falling delay in adaptive SR */
#define ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS2_TH2        mV_2_ADC_DAC_VALUE(ADAPTIVE_SYNCH_RECT_INCREMENTAL_VDS2_TH2_mV)    /**< ADC Vds2 threshold2 to increase falling delay in adaptive SR */

#define DAC_SR1_TURN_OFF_COMP_THRESHOLD         mV_2_ADC_DAC_VALUE(DAC_SR1_TURN_OFF_COMP_THRESHOLD_mV)  /**< COMP Vds1 threshold on DAC to shut-down PWM in adaptive SR */
#define DAC_SR2_TURN_OFF_COMP_THRESHOLD         mV_2_ADC_DAC_VALUE(DAC_SR2_TURN_OFF_COMP_THRESHOLD_mV)  /**< COMP Vds2 threshold on DAC to shut-down PWM in adaptive SR */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __LLC_CONTROL_PARAM_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/


