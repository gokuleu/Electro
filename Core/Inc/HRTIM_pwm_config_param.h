/**
  ******************************************************************************
  * @file    HRTIM_pwm_config_param.h
  * @author  IMS Systems Lab and Technical Marketing
  * @version 1.1.0
  * @date    12-Jul-2017
  * @brief   This file contains definitions used for HRTIM configuration
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
#ifndef __HRTIM_PWM_CONFIG_PARAM_H
#define __HRTIM_PWM_CONFIG_PARAM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/*---------------------- MCU PARAMETERS --------------------------------------*/
/* if prescaler = 0 -> f_HRCK = 144x32MHz = 4.608GHz, res = 217ps, min_PWM_freq = 70.3kHz
  if prescaler = 1 -> f_HRCK = 144x16MHz = 2.304GHz, res = 434ps, min_PWM_freq = 35.1kHz
  if prescaler = 2 -> f_HRCK = 144x8MHz = 1.152GHz, res = 868ps, min_PWM_freq = 17.6kHz
  It is needed to change HRTIM_CLK_HZ and HRTIM_PRESC_RATIO accordingly with HRTIM_PWM_FREQ_HZ */

#define HRTIM_CLK_HZ            4608000000                      /**< frequency in Hz of HRTIM clock */
#define HRTIM_PRESC_RATIO       HRTIM_PRESCALERRATIO_MUL32      /**< prescaler ratio for HRTIM clock */
#define DLL_CALIBRATIONTIMEOUT ((uint32_t)   100)               /**<  DLL Timeout Calibration in ms */

#define TIMx_CLK_HZ             72000000                        /**< frequency in Hz of generic TIMx clock */

/* -- Min and Max values for Compare and period registers - they depend on prescaler: see RM table 82 -- */
#define HRTIM_MIN_CMP_PER_VALUE ((uint16_t) 0x0060) /* min compare and period register value for prescaler = 0 */
#define HRTIM_MAX_CMP_PER_VALUE ((uint16_t) 0xFFDF) /* min compare and period register value for prescaler = 0 */
//#define HRTIM_MIN_CMP_PER_VALUE ((uint16_t) 0x0030) /* min compare and period register value for prescaler = 1 */
//#define HRTIM_MAX_CMP_PER_VALUE ((uint16_t) 0xFFEF) /* min compare and period register value for prescaler = 1 */
//#define HRTIM_MIN_CMP_PER_VALUE ((uint16_t) 0x000C) /* min compare and period register value for prescaler = 2 */
//#define HRTIM_MAX_CMP_PER_VALUE ((uint16_t) 0xFFFB) /* min compare and period register value for prescaler = 2 */

/* Table 82:
presc   min     max
0       0x0060 0xFFDF
1       0x0030 0xFFEF
2       0x0018 0xFFF7
3       0x000C 0xFFFB
*/

/* -- Dead time calculation ------------------------------------------------- */
#define DEAD_TIME_RISING_NS     ((uint16_t)950) /**< initial or default dead time of rising edge in ns - see below for allowable range */
#define DEAD_TIME_FALLING_NS    ((uint16_t)950) /**< initial or default dead time of falling edge in ns - see below for allowable range */
#define DEAD_TIME_MAX_NS        ((uint16_t)2132) /**< max dead time for both rising and falling edges in ns - see below for allowable range */
#define DEAD_TIME_MIN_NS        ((uint16_t)853) /**< min dead time for both rising and falling edges in ns - see below for allowable range */

// note: 550ns last dead time used for STW56N60DM2 for both rising and falling

/* -- PWM out polarity */
#define HRTIM_PWM_FB_HIGHSIDE_OUTPOLARITY       HRTIM_OUTPUTPOLARITY_HIGH
#define HRTIM_PWM_FB_LOWSIDE_OUTPOLARITY        HRTIM_OUTPUTPOLARITY_LOW
#define HRTIM_PWM_SR_HIGHSIDE_OUTPOLARITY       HRTIM_OUTPUTPOLARITY_HIGH
#define HRTIM_PWM_SR_LOWSIDE_OUTPOLARITY        HRTIM_OUTPUTPOLARITY_LOW

/*---------------------- POWER DEVICES PARAMETERS ----------------------------*/

/*-- Burst Mode --------------------------------------------------------------*/
#define BURST_MODE_IDLE_DURATION_INIT   8       /**< if defined LIGHT_LOAD_BURST_MODE, BURST_MODE_IDLE_DURATION_INIT + 1 is the default number of HRTIM periods that compose the burst idle duration when the burst mode is active */
#define BURST_MODE_PERIOD_INIT          11      /**< if defined LIGHT_LOAD_BURST_MODE, BURST_MODE_PERIOD_INIT +1 is the default number of HRTIM periods that compose the burst period (idle + run) when the burst mode is active */

/* -- HRTIM ticks calculation ----------------------------------------------- */
#define HRTIM_CLK_MHZ           ((uint16_t)(HRTIM_CLK_HZ/1000000))  /**< frequency in MHz of HRTIM clock */
#define FREQ_HZ_TO_PWM_TICKS(f) ((uint16_t)(HRTIM_CLK_HZ / f))

/* Convert time interval in ns to TIMx ticks  */
#define TIMx_CLK_MHZ               ((uint16_t)(((uint32_t)(TIMx_CLK_HZ))/1000000))
#define DELAY_NS_2_TIMx_TIKCS(x)   ((uint16_t)((x * TIMx_CLK_MHZ)/1000))

/* -- Dead time calculation: uncomment the correct lines to change DT prescaler -- */

/* if prescaler = 0 -> t_DTG = 868ps, DTmax = 511*t_DTG = 443.54ns */
//#define DEAD_TIME_NS_2_DT_TICKS(x)      ((uint32_t)((x * 1152)/1000))
//#define DEAD_TIME_PRESCALER             HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8              /*!< fDTG = fHRTIM * 8 */

/* if prescaler = 1 -> t_DTG = 1.736ns, DTmax = 511*t_DTG = 887.09ns */
//#define DEAD_TIME_NS_2_DT_TICKS(x)      ((uint32_t)((x * 576)/1000))
//#define DEAD_TIME_PRESCALER             HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL4              /*!< fDTG = fHRTIM * 4 */

/* if prescaler = 2 -> t_DTG = 3.472ns, , DTmax = 511*t_DTG = 1.7742us */
//#define DEAD_TIME_NS_2_DT_TICKS(x)      ((uint32_t)((x * 288)/1000))
//#define DEAD_TIME_PRESCALER             HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL2              /*!< fDTG = fHRTIM * 2 */

/* if prescaler = 3 -> t_DTG = 6.944ns, , DTmax = 511*t_DTG = 3.5484us */
#define DEAD_TIME_NS_2_DT_TICKS(x)      ((uint32_t)((x * 144)/1000))
#define DEAD_TIME_PRESCALER             HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1              /*!< fDTG = fHRTIM */

/* Convert a delay in ns to HRTIM ticks */
#define DELAY_NS_2_HRTIM_TICKS(x)               ((uint16_t)((x * HRTIM_CLK_MHZ)/1000))
#define DELAY_NS_2_HRTIM_TICKS_SIGNED(x)        ((int16_t)((x * HRTIM_CLK_MHZ)/1000))

/* Convert Dead time delay in DT ticks */
#define DEAD_TIME_RISING_DT_TICKS     DEAD_TIME_NS_2_DT_TICKS(DEAD_TIME_RISING_NS)
#define DEAD_TIME_FALLING_DT_TICKS    DEAD_TIME_NS_2_DT_TICKS(DEAD_TIME_FALLING_NS)

/* Convert Dead time delay in HRTIM ticks (dead time registers have another conversion) */
#define DEAD_TIME_RISING_HR_TICKS     DELAY_NS_2_HRTIM_TICKS(DEAD_TIME_RISING_NS)
#define DEAD_TIME_FALLING_HR_TICKS    DELAY_NS_2_HRTIM_TICKS(DEAD_TIME_FALLING_NS)

#define DEAD_TIME_RISING_HR_TICKS_SIGNED     DELAY_NS_2_HRTIM_TICKS_SIGNED(DEAD_TIME_RISING_NS)
#define DEAD_TIME_FALLING_HR_TICKS_SIGNED    DELAY_NS_2_HRTIM_TICKS_SIGNED(DEAD_TIME_FALLING_NS)

#endif  /*__HRTIM_PWM_CONFIG_PARAM_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
