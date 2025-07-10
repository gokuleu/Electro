/**
  ******************************************************************************
  * @file    LLC_board_config_param.h
  * @author  IMS Systems Lab and Technical Marketing
  * @version 1.1.0
  * @date    23-May-2017
  * @brief   This file contains definitions of input/output ports and conversion
  *          factors
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
#ifndef __LLC_BOARD_CONFIG_PARAM_H
#define __LLC_BOARD_CONFIG_PARAM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "HRTIM_pwm_config_param.h"

/* Quick changes of SR PWM waveform */
#define PWM_SR_DEFAULT_PINOUT_SETTING           /* if defined PWM_SR_HS2/PWM_SR_LS1 and PWM_SR_HS1/PWM_SR_LS2 are mapped respectively on TIMC and TIMD outputs, viceversa otherwise */

/* ADC Converting Macros: Volts, Ampere to ADC measure -----------------------*/

/* ADC/DAC to mV */
#define mV_2_ADC_DAC_VALUE(v)           ((uint16_t)((v * 4095)/3300))
#define ADC_DAC_VALUE_2_mV(m)           ((uint16_t)((m * 3300)/4095))

/* out Voltage */
//#define OUT_VOLT_CONV_RATIO             0.05882353 /* DC output voltage partitioning ratio : 10k/(10k+160k) */
#define OUT_VOLT_CONV_RATIO             0.05729 /* DC output voltage partitioning ratio */
#define OUT_VOLT_ADC_VALUE(v)           ((uint16_t)((v * OUT_VOLT_CONV_RATIO * 4095)/3.3))
#define OUT_VOLT_ADC_2_VOLT_VALUE(m)    ((float)((m * 3.3 /(OUT_VOLT_CONV_RATIO * 4095))))

/* input Voltage */
//#define IN_VOLT_CONV_RATIO              0.005808242     /* DC input voltage partitioning ratio : 8*4/2.2*(160/(160 + 7.87k + 390k)) - empiric value */
#define IN_VOLT_CONV_RATIO              0.006471     /* DC input voltage partitioning ratio */
#define IN_VOLT_ADC_VALUE(v)            ((uint16_t)((v * IN_VOLT_CONV_RATIO * 4095)/3.3))
#define IN_VOLT_ADC_2_VOLT_VALUE(m)     ((float)((m * 3.3/(IN_VOLT_CONV_RATIO * 4095))))

/* resonant Current CT transformer */
//#define RESONANT_CURR_CONV_RATIO        0.215       /* (i/200)*43 */
//#define RESONANT_CURR_ADC_VALUE(i)      ((uint16_t)(((i * RESONANT_CURR_CONV_RATIO)*4095)/3.3))

/* output current Hall sensor: Allegro ACS758LCB-100U-PFF-T (unidirectional current sensor) */
#define HALL_IC_OFFSET_ADC_VAL                  ((uint16_t)496)      /* Current sensor offset, typical value (0.1*Vcc) in ADC measure */
#define HALL_IC_SENSITIVITY_mV_A                26                /* Typical sensitivity in [mV/A] */
#define HALL_IC_AMP2ADC_VALUE(i)                ((uint16_t)((((i * HALL_IC_SENSITIVITY_mV_A)/1000)*4095)/3.3f) + HALL_IC_OFFSET_ADC_VAL)        /* converts a current in Ampere to ADC measure - considering offset */
#define HALL_IC_ADC_VALUE2AMP(m)                ((float)((((float)(m - HALL_IC_OFFSET_ADC_VAL)  * 3300) / 4095) / HALL_IC_SENSITIVITY_mV_A))    /* converts an ADC measure to a current in Ampere - considering offset */

/* Temperature */
#define TEMPERATURE_ADC2DEG(t)          (float)((((3.3 * ((float)t))/4096.0) - 1.8663)/(-0.01169));  /* LM19 transfer function */

/* optocouplers and switching delays */
#define OPTOCOUPLERS_PROP_DELAY_NS              100    // from datasheet maximum propagation delay is 150/120ns, typ 95ns (from oscilloscope: 100ns)
#define PRIMARY_MOSFET_TURN_ON_DELAY_NS         90     // from datasheet maximum propagation delay is 120ns, typ 85ns (from oscilloscope: 90ns)
#define PRIMARY_MOSFET_TURN_OFF_DELAY_NS        90     // from datasheet maximum propagation delay is 120ns, typ 85ns (from oscilloscope: 90ns)

#define OPTOCOUPLERS_PROP_DELAY_HR_TICKS        DELAY_NS_2_HRTIM_TICKS(OPTOCOUPLERS_PROP_DELAY_NS)
#define PRIMARY_MOSFET_TURN_ON_DELAY_HR_TICKS   DELAY_NS_2_HRTIM_TICKS(PRIMARY_MOSFET_TURN_ON_DELAY_NS)
#define PRIMARY_MOSFET_TURN_OFF_DELAY_HR_TICKS  DELAY_NS_2_HRTIM_TICKS(PRIMARY_MOSFET_TURN_OFF_DELAY_NS)

/* -- GPIO ports, pins & channels ------------------------------------------- */

/**** PWM and HRTIM ***********************************************************/

/* Configure HRTIM TIMER INDICES */
#define PWM_FB_HS1_LS1_HRTIM_TIMERINDEX         HRTIM_TIMERINDEX_TIMER_A
#define PWM_FB_HS2_LS2_HRTIM_TIMERINDEX         HRTIM_TIMERINDEX_TIMER_B
#define PWM_SR_HS2_LS1_HRTIM_TIMERINDEX         HRTIM_TIMERINDEX_TIMER_C
#define PWM_SR_HS1_LS2_HRTIM_TIMERINDEX         HRTIM_TIMERINDEX_TIMER_D

#define PWM_FB_HS1_LS1_HRTIM_TIMERID            HRTIM_TIMERID_TIMER_A
#define PWM_FB_HS2_LS2_HRTIM_TIMERID            HRTIM_TIMERID_TIMER_B
#define PWM_SR_HS2_LS1_HRTIM_TIMERID            HRTIM_TIMERID_TIMER_C
#define PWM_SR_HS1_LS2_HRTIM_TIMERID            HRTIM_TIMERID_TIMER_D

#define PWM_FB_HS1_LS1_HRTIM_TIMERUPDATE        HRTIM_TIMERUPDATE_A
#define PWM_FB_HS2_LS2_HRTIM_TIMERUPDATE        HRTIM_TIMERUPDATE_B
#define PWM_SR_HS2_LS1_HRTIM_TIMERUPDATE        HRTIM_TIMERUPDATE_C
#define PWM_SR_HS1_LS2_HRTIM_TIMERUPDATE        HRTIM_TIMERUPDATE_D

/* Injected ADC trigger events */
#define HRTIM_INJECT_ADCTRIGGEREVENT_SR1        HRTIM_ADCTRIGGEREVENT24_TIMERC_CMP4
#define HRTIM_INJECT_ADCTRIGGEREVENT_SR2        HRTIM_ADCTRIGGEREVENT24_TIMERD_CMP4

/* Configure HRTIM output: TA1 (PA8) - PWM_FB_HS1 */
#define PWM_FB_HS1_GPIO_PORT       	GPIOA
#define PWM_FB_HS1_GPIO_PIN        	GPIO_PIN_8
#define PWM_FB_HS1_HRTIM_OUTPUT         HRTIM_OUTPUT_TA1

/* Configure HRTIM output: TA2 (PA9) - PWM_FB_LS1 */
#define PWM_FB_LS1_GPIO_PORT       	GPIOA
#define PWM_FB_LS1_GPIO_PIN        	GPIO_PIN_9
#define PWM_FB_LS1_HRTIM_OUTPUT         HRTIM_OUTPUT_TA2

/* Configure HRTIM output: TB1 (PA10) - PWM_FB_HS2 */
#define PWM_FB_HS2_GPIO_PORT       	GPIOA
#define PWM_FB_HS2_GPIO_PIN        	GPIO_PIN_10
#define PWM_FB_HS2_HRTIM_OUTPUT         HRTIM_OUTPUT_TB1

/* Configure HRTIM output: TB2 (PA11) - PWM_FB_LS2 */
#define PWM_FB_LS2_GPIO_PORT       	GPIOA
#define PWM_FB_LS2_GPIO_PIN        	GPIO_PIN_11
#define PWM_FB_LS2_HRTIM_OUTPUT         HRTIM_OUTPUT_TB2

#ifdef PWM_SR_DEFAULT_PINOUT_SETTING
  /* Configure HRTIM output: TC1 (PB12) - PWM_SR_HS2 */
  #define PWM_SR_HS2_GPIO_PORT       	GPIOB
  #define PWM_SR_HS2_GPIO_PIN        	GPIO_PIN_12
  #define PWM_SR_HS2_HRTIM_OUTPUT       HRTIM_OUTPUT_TC1

  /* Configure HRTIM output: TC2 (PB13) - PWM_SR_LS1 */
  #define PWM_SR_LS1_GPIO_PORT       	GPIOB
  #define PWM_SR_LS1_GPIO_PIN        	GPIO_PIN_13
  #define PWM_SR_LS1_HRTIM_OUTPUT       HRTIM_OUTPUT_TC2

  /* Configure HRTIM output: TD1 (PB14) - PWM_SR_HS1 */
  #define PWM_SR_HS1_GPIO_PORT       	GPIOB
  #define PWM_SR_HS1_GPIO_PIN        	GPIO_PIN_14
  #define PWM_SR_HS1_HRTIM_OUTPUT       HRTIM_OUTPUT_TD1

  /* Configure HRTIM output: TD2 (PB15) - PWM_SR_LS2 */
  #define PWM_SR_LS2_GPIO_PORT       	GPIOB
  #define PWM_SR_LS2_GPIO_PIN        	GPIO_PIN_15
  #define PWM_SR_LS2_HRTIM_OUTPUT       HRTIM_OUTPUT_TD2
#else
  /* Configure HRTIM output: TD1 (PB14) - PWM_SR_HS2 */
  #define PWM_SR_HS2_GPIO_PORT       	GPIOB
  #define PWM_SR_HS2_GPIO_PIN        	GPIO_PIN_14
  #define PWM_SR_HS2_HRTIM_OUTPUT       HRTIM_OUTPUT_TD1

  /* Configure HRTIM output: TD2 (PB15) - PWM_SR_LS1 */
  #define PWM_SR_LS1_GPIO_PORT       	GPIOB
  #define PWM_SR_LS1_GPIO_PIN        	GPIO_PIN_15
  #define PWM_SR_LS1_HRTIM_OUTPUT       HRTIM_OUTPUT_TD2

  /* Configure HRTIM output: TC1 (PB12) - PWM_SR_HS1 */
  #define PWM_SR_HS1_GPIO_PORT       	GPIOB
  #define PWM_SR_HS1_GPIO_PIN        	GPIO_PIN_12
  #define PWM_SR_HS1_HRTIM_OUTPUT       HRTIM_OUTPUT_TC1

  /* Configure HRTIM output: TC2 (PB13) - PWM_SR_LS2 */
  #define PWM_SR_LS2_GPIO_PORT       	GPIOB
  #define PWM_SR_LS2_GPIO_PIN        	GPIO_PIN_13
  #define PWM_SR_LS2_HRTIM_OUTPUT       HRTIM_OUTPUT_TC2
#endif


/**** ADC *********************************************************************/

/* Definition for ADCx clock resources */
#define ADCx                            ADC1            /* regular conversion of Vout, injected conversions of VDS_SR1 and VDS_SR2 */
#define ADCy                            ADC2            /* regular conversion of Temperature, injected conversions of VDS_SR1 and VDS_SR2 */
#define ADCx_CLK_ENABLE()               __ADC12_CLK_ENABLE()
#define ADCy_CLK_ENABLE()               __ADC12_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __DMA1_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define ADCy_CHANNEL_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __ADC12_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __ADC12_RELEASE_RESET()

/* Definition for ADCx Vout Channel */
#define ADCx_VOUT_CHANNEL_PIN                   GPIO_PIN_0
#define ADCx_VOUT_CHANNEL_GPIO_PORT             GPIOA
#define ADCx_VOUT_CHANNEL                       ADC_CHANNEL_1
#define ADCx_VOUT_CHANNEL_SAMPLING_TIME         ADC_SAMPLETIME_7CYCLES_5 //ADC_SAMPLETIME_4CYCLES_5 // (7.5 + 12.5)/72M = 278ns = Total conversion time

/* Definition for ADCx Iout Channel */
#define ADCx_IOUT_CHANNEL_PIN                   GPIO_PIN_1
#define ADCx_IOUT_CHANNEL_GPIO_PORT             GPIOA
#define ADCx_IOUT_CHANNEL                       ADC_CHANNEL_2
#define ADCx_IOUT_CHANNEL_SAMPLING_TIME         ADC_SAMPLETIME_7CYCLES_5;

/* Definition for ADCx Vbus Channel */
#define ADCx_VBUS_CHANNEL_PIN                   GPIO_PIN_2
#define ADCx_VBUS_CHANNEL_GPIO_PORT             GPIOA
#define ADCx_VBUS_CHANNEL                       ADC_CHANNEL_3
#define ADCx_VBUS_CHANNEL_SAMPLING_TIME         ADC_SAMPLETIME_7CYCLES_5; //ADC_SAMPLETIME_4CYCLES_5

/* Definition for ADCy Ires Channel */
#define ADCy_IRES_CHANNEL_PIN                   GPIO_PIN_4
#define ADCy_IRES_CHANNEL_GPIO_PORT             GPIOC
#define ADCy_IRES_CHANNEL                       ADC_CHANNEL_5
#define ADCy_IRES_CHANNEL_SAMPLING_TIME         ADC_SAMPLETIME_7CYCLES_5

/* Definition for ADCy Ires AVG Channel */
#define ADCy_IRES_AVG_CHANNEL_PIN               GPIO_PIN_5
#define ADCy_IRES_AVG_CHANNEL_GPIO_PORT         GPIOC
#define ADCy_IRES_AVG_CHANNEL                   ADC_CHANNEL_11
#define ADCy_IRES_AVG_CHANNEL_SAMPLING_TIME     ADC_SAMPLETIME_7CYCLES_5

/* Definition for ADCy Vout Channel - for double acquisition if used (instead of Ires) */
#define ADCy_VOUT_CHANNEL_PIN                   GPIO_PIN_4
#define ADCy_VOUT_CHANNEL_GPIO_PORT             GPIOC
#define ADCy_VOUT_CHANNEL                       ADC_CHANNEL_5
#define ADCy_VOUT_CHANNEL_SAMPLING_TIME         ADC_SAMPLETIME_7CYCLES_5

/* Definition for ADCy Temperature Channel */
#define ADCy_TEMP_CHANNEL_PIN                   GPIO_PIN_0
#define ADCy_TEMP_CHANNEL_GPIO_PORT             GPIOC
#define ADCy_TEMP_CHANNEL                       ADC_CHANNEL_6
#define ADCy_TEMP_CHANNEL_SAMPLING_TIME         ADC_SAMPLETIME_7CYCLES_5

/* Definition for ADCx's Vds1 GPIO pin, port and channel */
#define ADCx_VDS1_CHANNEL_PIN                   GPIO_PIN_3
#define ADCx_VDS1_CHANNEL_GPIO_PORT             GPIOA
#define ADCx_VDS1_CHANNEL                       ADC_CHANNEL_4
#define ADCx_VDS1_CHANNEL_SAMPLING_TIME         ADC_SAMPLETIME_4CYCLES_5

/* Definition for ADCy's Vds2 GPIO pin, port and channel */
#define ADCy_VDS2_CHANNEL_PIN                   GPIO_PIN_2
#define ADCy_VDS2_CHANNEL_GPIO_PORT             GPIOB
#define ADCy_VDS2_CHANNEL                       ADC_CHANNEL_12
#define ADCy_VDS2_CHANNEL_SAMPLING_TIME         ADC_SAMPLETIME_4CYCLES_5

/* Definition for ADCx's DMA - if regular aquisitions are used */
#define ADCx_DMA_CHANNEL                        DMA1_Channel1
#define ADCy_DMA_CHANNEL                        DMA1_Channel4
#define ADCy_DMA_REMAP_CH                       HAL_REMAPDMA_ADC2_DMA1_CH4

/* Definition for DMA NVIC */
#define ADCx_DMA_IRQn                           DMA1_Channel1_IRQn
#define ADCx_DMA_IRQHandler                     DMA1_Channel1_IRQHandler
#define ADCy_DMA_IRQn                           DMA1_Channel4_IRQn
#define ADCy_DMA_IRQHandler                     DMA1_Channel4_IRQHandler

/**** Analog WDG **************************************************************/
#define HRTIM_AWDG1_EVENT                       HRTIM_EVENT_1   /* see RM pag 646 */
#define HRTIM_AWDG1_EVENTSRC                    HRTIM_EVENTSRC_4
#define HRTIM_OUTPUTRESET_AWDG1_EEV             HRTIM_OUTPUTRESET_EEV_1
#define HRTIM_OUTPUTSET_AWDG1_EEV               HRTIM_OUTPUTSET_EEV_1

/**** COMP ********************************************************************/
/* Definition for COMP_OC (Overcurrent Protection) clock resources and pins */
#define COMP_OC                         COMP6
#define COMP_OC_CLK_ENABLE()            __DAC2_CLK_ENABLE()
#define COMP_OC_GPIO_PORT               GPIOB
#define COMP_OC_PIN                     GPIO_PIN_11
#define COMP_OC_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE()
#define COMP_OC_GPIO_CLK_DISABLE()      __GPIOB_CLK_DISABLE()
#define COMP_INVERTINGINPUT_DAC_OC      COMP_INVERTINGINPUT_DAC2_CH1
#define HRTIM_IT_FLT_OC                 HRTIM_IT_FLT3   /* FLT1 for COMP2, FLT2 for COMP4, FLT3 for COMP6 - Table 86 RM */
#define HRTIM_FAULT_OC                  HRTIM_FAULT_3   /* FLT1 for COMP2, FLT2 for COMP4, FLT3 for COMP6 - Table 86 RM */
#define HRTIM_TIMFAULTENABLE_FAULT_OC   HRTIM_TIMFAULTENABLE_FAULT3
#define HRTIM_FLAG_FAULT_OC             HRTIM_FLAG_FLT3

/* Configure COMP_OC debug output */
#define COMP_OC_OUT_GPIO_PORT           GPIOC
#define COMP_OC_OUT_GPIO_PIN            GPIO_PIN_6
#define COMP_OC_OUT_AF                  GPIO_AF7_GPCOMP6

/* Definition for COMP_OC NVIC */
#define COMP_OC_IRQn                    COMP4_6_IRQn
#define COMP_OC_IRQHandler              COMP4_6_IRQHandler

/* Definition for COMP_VDS_SR1 (Adaptive Synch Rect1) clock resources and pins */
#define COMP_VDS_SR1                            COMP2
#define COMP_VDS_SR1_CLK_ENABLE()               __DAC1_CLK_ENABLE()
#define COMP_VDS_SR1_GPIO_PORT                  GPIOA
#define COMP_VDS_SR1_PIN                        GPIO_PIN_7
#define COMP_VDS_SR1_GPIO_CLK_ENABLE()          __GPIOA_CLK_ENABLE()
#define COMP_VDS_SR1_GPIO_CLK_DISABLE()         __GPIOA_CLK_DISABLE()
#define COMP_INVERTINGINPUT_DAC_VDS_SR1         COMP_INVERTINGINPUT_DAC1_CH1
#define HRTIM_SR1_COMP_EVENT                    HRTIM_EVENT_1
#define HRTIM_OUTPUTRESET_SR1_COMP_EEV          HRTIM_OUTPUTRESET_EEV_1         //HRTIM_OUTPUTRESET_EEV_6 - with filter capability see MCU RM pag 646, Table 86
#define HRTIM_CAPTURETRIGGER_SR1_COMP_EEV       HRTIM_CAPTURETRIGGER_EEV_1
#define HRTIM_SR1_COMP_EVENTSRC                 HRTIM_EVENTSRC_2

/* Configure COMP_VDS_SR1 debug output */
#define COMP_VDS_SR1_OUT_GPIO_PORT              GPIOA
#define COMP_VDS_SR1_OUT_GPIO_PIN               GPIO_PIN_12
#define COMP_VDS_SR1_OUT_AF                     GPIO_AF8_GPCOMP2

/* Definition for COMP_VDS_SR1 NVIC */
#define COMP_VDS_SR1_IRQn                       COMP2_IRQn
#define COMP_VDS_SR1_IRQHandler                 COMP2_IRQHandler

/* Definition for COMP_VDS_SR2 (Adaptive Synch Rect2) clock resources and pins */
#define COMP_VDS_SR2                            COMP4
#define COMP_VDS_SR2_CLK_ENABLE()               __DAC1_CLK_ENABLE()
#define COMP_VDS_SR2_GPIO_PORT                  GPIOB
#define COMP_VDS_SR2_PIN                        GPIO_PIN_0
#define COMP_VDS_SR2_GPIO_CLK_ENABLE()          __GPIOB_CLK_ENABLE()
#define COMP_VDS_SR2_GPIO_CLK_DISABLE()         __GPIOB_CLK_DISABLE()
#define COMP_INVERTINGINPUT_DAC_VDS_SR2         COMP_INVERTINGINPUT_DAC1_CH2
#define HRTIM_SR2_COMP_EVENT                    HRTIM_EVENT_2
#define HRTIM_OUTPUTRESET_SR2_COMP_EEV          HRTIM_OUTPUTRESET_EEV_2         //HRTIM_OUTPUTRESET_EEV_7 - with filter capability see MCU RM pag 646, Table 86
#define HRTIM_CAPTURETRIGGER_SR2_COMP_EEV       HRTIM_CAPTURETRIGGER_EEV_2
#define HRTIM_SR2_COMP_EVENTSRC                 HRTIM_EVENTSRC_2

/* Configure COMP_VDS_SR2 debug output */
#define COMP_VDS_SR2_OUT_GPIO_PORT              GPIOB
#define COMP_VDS_SR2_OUT_GPIO_PIN               GPIO_PIN_1
#define COMP_VDS_SR2_OUT_AF                     GPIO_AF8_GPCOMP4

/* Definition for COMP_VDS_SR2 NVIC */
#define COMP_VDS_SR2_IRQn                       COMP4_6_IRQn
#define COMP_VDS_SR2_IRQHandler                 COMP4_6_IRQHandler


/**** Blanking Window related pins ****/

/* Configure HRTIM synch output: PB1 */
#define HRTIM_SCOUT_GPIO_PORT       	        GPIOB
#define HRTIM_SCOUT_GPIO_PIN        	        GPIO_PIN_3

///* Configure TIMx input pin synch. for blanking window */
//#define BLANKING_WINDOW_TIM_ITRG_GPIO_PORT      GPIOA
//#define BLANKING_WINDOW_TIM_ITRG_GPIO_PIN       GPIO_PIN_15
//#define BLANKING_WINDOW_TIM_ITRG_AF             GPIO_AF1_TIM2
//
///* Configure TIMx input pin synch. for blanking window */
//#define BLANKING_WINDOW_TIM_OUT_GPIO_PORT       GPIOB
//#define BLANKING_WINDOW_TIM_OUT_GPIO_PIN        GPIO_PIN_10
//#define BLANKING_WINDOW_TIM_OUT_AF              GPIO_AF1_TIM2

/**** DAC ********************************************************************/

/* Definition for DAC_SR clock resources */
#define DAC_SR                                  DAC1
#define DAC_SR_CLK_ENABLE()                     __DAC1_CLK_ENABLE()
#define DAC_SR_CHANNEL_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
#define DAC_SR_FORCE_RESET()                    __DAC1_FORCE_RESET()
#define DAC_SR_RELEASE_RESET()                  __DAC1_RELEASE_RESET()

/* Definition for DAC_SR Channel1 Pin */
#define DAC_SR_CHANNEL1_PIN                     GPIO_PIN_4
#define DAC_SR_CHANNEL1_GPIO_PORT               GPIOA
#define DAC_SR_CHANNEL1                         DAC1_CHANNEL_1

/* Definition for DAC_SR Channel2 Pin */
#define DAC_SR_CHANNEL2_PIN                     GPIO_PIN_5
#define DAC_SR_CHANNEL2_GPIO_PORT               GPIOA
#define DAC_SR_CHANNEL2                         DAC1_CHANNEL_2

/* Definition for DAC_OC clock resources */
#define DAC_OC                                  DAC2
#define DAC_OC_CLK_ENABLE()                     __DAC2_CLK_ENABLE()
#define DAC_OC_CHANNEL_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
#define DAC_OC_FORCE_RESET()                    __DAC2_FORCE_RESET()
#define DAC_OC_RELEASE_RESET()                  __DAC2_RELEASE_RESET()

/* Definition for DAC_OC Channel1 Pin */
#define DAC_OC_CHANNEL_PIN                      GPIO_PIN_6
#define DAC_OC_CHANNEL_GPIO_PORT                GPIOA
#define DAC_OC_CHANNEL                          DAC2_CHANNEL_1


/**** GPIO ********************************************************************/

/* Definition for digital output clock resources and pins */

#ifdef DISCOVERY_STM32F3348
  /* LED6 (blue) for discovery board */
  #define STATUS_LED_GPIO_PIN                   GPIO_PIN_7
  #define STATUS_LED_GPIO_PORT                  GPIOB
  #define STATUS_LED_GPIO_CLK_ENABLE()          __GPIOB_CLK_ENABLE()
  #define STATUS_LED_GPIO_CLK_DISABLE()         __GPIOB_CLK_DISABLE()

  /* LED6 (red) for discovery board */
  #define FAULT_LED_GPIO_PIN                    GPIO_PIN_6
  #define FAULT_LED_GPIO_PORT                   GPIOB
  #define FAULT_LED_GPIO_CLK_ENABLE()           __GPIOB_CLK_ENABLE()
  #define FAULT_LED_GPIO_CLK_DISABLE()          __GPIOB_CLK_DISABLE()
#else
  #define STATUS_LED_GPIO_PORT       	        GPIOC
  #define STATUS_LED_GPIO_PIN        	        GPIO_PIN_10
  #define STATUS_LED_GPIO_CLK_ENABLE()          __GPIOC_CLK_ENABLE()
  #define STATUS_LED_GPIO_CLK_DISABLE()         __GPIOC_CLK_DISABLE()

  #define FAULT_LED_GPIO_PORT                   GPIOC
  #define FAULT_LED_GPIO_PIN        	        GPIO_PIN_3
  #define FAULT_LED_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
  #define FAULT_LED_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()
#endif

/* Definition for SD_OD SR1 Pin */
#define SD_OD_SR1_GPIO_PORT                     GPIOC
#define SD_OD_SR1_GPIO_PIN                      GPIO_PIN_11
#define SD_OD_SR1_GPIO_CLK_ENABLE()             __GPIOC_CLK_ENABLE()
#define SD_OD_SR1_GPIO_CLK_DISABLE()            __GPIOC_CLK_DISABLE()

/* Definition for SD_OD SR2 Pin */
#define SD_OD_SR2_GPIO_PORT                     GPIOC
#define SD_OD_SR2_GPIO_PIN                      GPIO_PIN_11
#define SD_OD_SR2_GPIO_CLK_ENABLE()             __GPIOC_CLK_ENABLE()
#define SD_OD_SR2_GPIO_CLK_DISABLE()            __GPIOC_CLK_DISABLE()

/* Definition for Out Discharge Pin */
#define OUT_DISCHARGE_GPIO_PORT                 GPIOB
#define OUT_DISCHARGE_GPIO_PIN                  GPIO_PIN_4
#define OUT_DISCHARGE_GPIO_CLK_ENABLE()         __GPIOB_CLK_ENABLE()
#define OUT_DISCHARGE_GPIO_CLK_DISABLE()        __GPIOB_CLK_DISABLE()

/* Definition for Fan PWM Pin */
#define FAN_PWM_GPIO_PORT                       GPIOB
#define FAN_PWM_GPIO_PIN                        GPIO_PIN_1              // It can't be used together with COMP_VDS_SR2 debug output
#define FAN_PWM_GPIO_CLK_ENABLE()               __GPIOB_CLK_ENABLE()
#define FAN_PWM_GPIO_CLK_DISABLE()              __GPIOB_CLK_DISABLE()

/* Definition for FAN TIM clock resources and pins */
#define FAN_PWM_TIM                     TIM3
#define FAN_PWM_CLK_ENABLE()            __TIM3_CLK_ENABLE()
#define FAN_PWM_CLK_DISABLE()           __TIM3_CLK_DISABLE()
#define FAN_PWM_TIM_CHANNEL             TIM_CHANNEL_4
#define FAN_PWM_TIM_AF                  GPIO_AF2_TIM3
#define FAN_PWM_TIM_PRESCALER           2
#define FAN_PWM_TIM_PERIOD              (HAL_RCC_GetSysClockFreq()/((1+FAN_PWM_TIM_PRESCALER)*FAN_PWM_FREQUENCY))

#define DEBUG_DIGITAL_OUT1_GPIO_PORT            GPIOB
#define DEBUG_DIGITAL_OUT1_GPIO_PIN             GPIO_PIN_10
#define DEBUG_DIGITAL_OUT1_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()
#define DEBUG_DIGITAL_OUT1_GPIO_CLK_DISABLE()   __GPIOB_CLK_DISABLE()

///**** COMMUNICATION ****/
// See DPS_Communication.h file for communication pinout assignment
//

#endif  /*__LLC_BOARD_CONFIG_PARAM_H*/
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
