/**
  ******************************************************************************
  * @file    LLC_Globals.c
  * @author  IMS Systems Lab 
  * @version V2.0.0
  * @date    23-May-2017
  * @brief   This file contains all global variables for the control of battery charger
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
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
//#include "DSMPS_type.h"
#include "HRTIM_pwm_config_param.h"
#include "LLC_control_param.h"
#include "LLC_Globals.h"


/** @addtogroup DSMPS_project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @addtogroup DCDC_global_variables
  * @{
  */
DCDC_MeasureStruct_t DCDC_MeasureStruct;                /**< Acquisition struct */
DCDC_ConfigParamStruct_t DCDC_ConfigParamStruct;        /**< Configuration param struct */

/* filtered measures */
uint16_t hTemperatureFiltered;                  /**< Temperature filtered measure */
uint16_t hVoutVoltageFiltered;                  /**< Vout voltage filtered measure */
uint16_t hVinVoltageFiltered;                   /**< Vin voltage filtered measure */

volatile float fTemperatureDeg;                 /**< LM19CIZ Temperature in deg */

/* control variables */
__IO uint16_t hPWMPeriod = HRTIM_INIT_PWM_PERIOD;                       /**< value of LLC PWM period in HRTIM ticks */
__IO uint16_t hPWMDeadTime = DEAD_TIME_RISING_DT_TICKS;                 /**< value of LLC PWM dead time in HRTIM DT ticks - for update if it is requested (same value for rising and falling)*/
__IO uint16_t hPWMDeadTimeHRticks = DEAD_TIME_RISING_HR_TICKS;          /**< value of LLC PWM dead time in HRTIM TIM ticks - for update if it is requested (same value for rising and falling)*/
__IO uint16_t hBurstModeIdleDuration = BURST_MODE_IDLE_DURATION_INIT;   /**< number of (HRTIM periods -1) in idle during burst mode */
__IO uint16_t hBurstModePeriod = BURST_MODE_PERIOD_INIT;                /**< number of (HRTIM periods -1) that compose burst period (run + idle) in burst mode */

volatile uint16_t hVout_Reference;                      /**< Voltage out reference */
volatile uint16_t hVout_Reference_init;                 /**< Voltage out reference init value */

/* threshold values - for current calibration (only for SR and BM) */
uint16_t hHallIcOffsetCalib = HALL_IC_OFFSET_ADC_VAL;                   /**< Calibration value for current sensor */
uint16_t hSrIoutTurnOnThreshold;                                        /**< SR turn-on current threshold variable with calibratated offset */
uint16_t hSrIoutTurnOffThreshold;                                       /**< SR turn-off current threshold variable with calibratated offset */
uint16_t hBmIoutRange12ThresholdHigh;                                   /**< BM range12 high current threshold variable with calibratated offset */
uint16_t hBmIoutRange12ThresholdLow;                                    /**< BM range12 low current threshold variable with calibratated offset */
uint16_t hBmIoutRange23ThresholdHigh;                                   /**< BM range23 high current threshold variable with calibratated offset */
uint16_t hBmIoutRange23ThresholdLow;                                    /**< BM range23 low current threshold variable with calibratated offset */

/* synchronous rectification's edge delays */
int16_t hSR_DelayRising1 = SYNCH_RECT_DELAY_RISING1_INIT_HR_TICKS;           /**< rising edge's delay of SR1 signal in HRTIM ticks */
int16_t hSR_DelayFalling1 = SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS;         /**< falling edge's delay of SR1 signal in HRTIM ticks */
int16_t hSR_DelayRising2 = SYNCH_RECT_DELAY_RISING2_INIT_HR_TICKS;           /**< rising edge's delay of SR2 signal in HRTIM ticks */
int16_t hSR_DelayFalling2 = SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS;         /**< falling edge's delay of SR2 signal in HRTIM ticks */

/* Configuration handles */
ADC_HandleTypeDef       AdcHandle1;		/**< handle for ADC1 (ADCx) */
ADC_HandleTypeDef       AdcHandle2;		/**< handle for ADC2 (ADCy) */
HRTIM_HandleTypeDef     hhrtim;		        /**< handle for HRTIM */
COMP_HandleTypeDef      CompOCHandle;           /**< handle for COMP_OC */
COMP_HandleTypeDef      CompSRHandle1;          /**< handle for COMP_VDS_SR1 */
COMP_HandleTypeDef      CompSRHandle2;          /**< handle for COMP_VDS_SR2 */
UART_HandleTypeDef      UartHandle;             /**< handle for UARTx */
DAC_HandleTypeDef       DacOCHandle;            /**< handle for DAC_OC */
DAC_HandleTypeDef       DacSRHandle;            /**< handle for DAC_SR */
TIM_HandleTypeDef       TimVoutControlHandle;   /**< handle of TIM that schedules Vout control loop */
TIM_HandleTypeDef       FanPwmHandle;           /**< handle for FAN PWM TIMx */
TIM_HandleTypeDef       TimTempFanCtrlHandle;   /**< handle for TIM that schedules temperature acquisition and fan control */

/* Start-up counters */
volatile uint16_t hDSMPS_StartUp_TimeLeft = 0;		                        /**< counter for duration of DSMPS_START state */
volatile uint16_t hDSMPS_Wait_TimeLeft = 0;                                     /**< counter for duration of DSMPS_WAIT state */
volatile uint16_t hDSMPS_UndervoltageValidation_TimeLeft = 0;                   /**< counter to validate undervoltage protection */
volatile uint16_t hCommunicationTimeLeft = 0;                                   /**< counter between two USART communications */
volatile uint16_t hStartUpFreqUpdateCounter = 0;                                /**< counter used to update the PWM frequency during start-up */

/* DCDC's PID struct */
#ifdef USE_EXTENDED_PID
  PID_Struct_ex_t PID_Vout_InitStructure;                                       /**< Out voltage extended PID */
#else
  PID_Struct_t PID_Vout_InitStructure;                                          /**< Out voltage PID */
#endif

/* state variables of converter */
LED_Struct_t StatusLED;                                                         /**< Status LED */
LED_Struct_t FaultLED;                                                          /**< Fault LED */
volatile uint16_t PFC_Received_Status = (uint16_t)PFC_BUS_UNDER_VOLT;           /**< last PFC received status */
volatile uint16_t FAN_PWMDutyCycle;                                             /**< PWM duty cycle for fan control */

/* boolean variables */
volatile bool bDCDC_OutputEnabled = FALSE;                              /**< TRUE when main PWMs of DCDC are enabled, FALSE otherwise */
volatile bool bDCDC_SynchRectOutputEnabled = FALSE;                     /**< TRUE when PWMs of Synch. Rect. are enabled, FALSE otherwise */
volatile bool bDCDC_AdaptiveSynchRectEnabled = FALSE;                   /**< TRUE when Adaptive Synch. Rect. is enabled, FALSE otherwise */
volatile bool bDCDC_DeadTimeChanged = FALSE;                            /**< TRUE when the dead time was changed and must be updated, FALSE otherwise */
volatile bool bCheckUndervoltageEnabled = FALSE;                        /**< TRUE when low out voltage short circuit fault is enabled, FALSE otherwise */
volatile bool bCheckOvertemperatureEnabled = FALSE;                     /**< TRUE when overtemperature fault is enabled, FALSE otherwise */
volatile bool bCheckOutOvercurrentEnabled = FALSE;                      /**< TRUE when out overcurrent fault is enabled, FALSE otherwise */

/* precharge driver struct variables */
volatile Driver_PrechargeStruct_t FullBridgeDriverPrechargeStruct;      /**< precharge struct variable of full bridge drivers */

/* for out voltage burst mode */
volatile bool bVoutBurstModeEnabled = FALSE;                            /**< TRUE when out voltage burst mode is active (Vout> bm_threshold with hysteresis), FALSE otherwise */
volatile bool bLightLoadBurstModeEnabled = FALSE;                       /**< TRUE when light load burst mode is active (PWM are in burst if Iout is < threshold), FALSE otherwise */
/**
  * @}
  */

/**
  * @}
  */ 

 

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
