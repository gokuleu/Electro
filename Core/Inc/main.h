/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN.h"
#include "LLC_Control_CV.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

int32_t moving_AC_voltage_measured_fun( int32_t current_val , int32_t MOV_AVG_SAMPLE);
//float moving_AC_voltage_measured_fun( float current_val , float MOV_AVG_SAMPLE)   ;// 0.1 amp Batt_current_measured
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pfc_status_Pin GPIO_PIN_14
#define pfc_status_GPIO_Port GPIOC
#define PFC_CTRL_out_Pin GPIO_PIN_15
#define PFC_CTRL_out_GPIO_Port GPIOC
#define EXT_TEMP_adc_input_Pin GPIO_PIN_0
#define EXT_TEMP_adc_input_GPIO_Port GPIOC
#define DC_Vout_Trmnl_adc_input_Pin GPIO_PIN_1
#define DC_Vout_Trmnl_adc_input_GPIO_Port GPIOC
#define AC_VSNS_adc_input_Pin GPIO_PIN_2
#define AC_VSNS_adc_input_GPIO_Port GPIOC
#define RLY_CTRL_output_Pin GPIO_PIN_3
#define RLY_CTRL_output_GPIO_Port GPIOC
#define DC_Vout_Monitor_adc_input_Pin GPIO_PIN_0
#define DC_Vout_Monitor_adc_input_GPIO_Port GPIOA
#define DC_Output_Current_adc_input_Pin GPIO_PIN_1
#define DC_Output_Current_adc_input_GPIO_Port GPIOA
#define DC_Bus_Monitor_adc_input_Pin GPIO_PIN_2
#define DC_Bus_Monitor_adc_input_GPIO_Port GPIOA
#define V_DS_SR1_adc_input_Pin GPIO_PIN_3
#define V_DS_SR1_adc_input_GPIO_Port GPIOA
#define VDS_SR2_sensing_adc_input_Pin GPIO_PIN_4
#define VDS_SR2_sensing_adc_input_GPIO_Port GPIOA
#define FAN_PWM_Pin GPIO_PIN_1
#define FAN_PWM_GPIO_Port GPIOB
#define PWM_SR_HS2_Pin GPIO_PIN_12
#define PWM_SR_HS2_GPIO_Port GPIOB
#define PWM_SR_LS1_Pin GPIO_PIN_13
#define PWM_SR_LS1_GPIO_Port GPIOB
#define PWM_SR_HS1_Pin GPIO_PIN_14
#define PWM_SR_HS1_GPIO_Port GPIOB
#define PWM_SR_LS2_Pin GPIO_PIN_15
#define PWM_SR_LS2_GPIO_Port GPIOB
#define PWM_FB_HS1_Pin GPIO_PIN_8
#define PWM_FB_HS1_GPIO_Port GPIOA
#define PWM_FB_LS1_Pin GPIO_PIN_9
#define PWM_FB_LS1_GPIO_Port GPIOA
#define PWM_FB_HS2_Pin GPIO_PIN_10
#define PWM_FB_HS2_GPIO_Port GPIOA
#define PWM_FB_LS2_Pin GPIO_PIN_11
#define PWM_FB_LS2_GPIO_Port GPIOA
#define SD_OD_output_Pin GPIO_PIN_11
#define SD_OD_output_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define ADC2_ENABLE 0
#define PFC_DISABLE 0
#define LLC_ENABLE 1
#define CONTROL_ENABLE 1

typedef struct Sensing_raw_t{
  uint16_t vout;
  uint16_t iout;
  uint16_t vbulk;
  uint16_t ch4;
  uint16_t ch5;
  uint16_t ch6;
  uint16_t ch7;
} Sensing_raw_t;

typedef struct Sensing_raw_filtered_t{
  uint16_t vout_f;
  uint16_t iout_f;
  uint16_t vbulk_f;
} Sensing_raw_filtered_t;

typedef enum{
 NO_ERROR,
 OVERCURRENT_IOUT,
 OVERVOLTAGE_VOUT
} ErrorType_t;

uint16_t ma_update(uint16_t x);
void ma_init(uint16_t first_sample);
float ADC_to_iout();
float ADC_to_vbulk();
float ADC_to_vout();
extern Sensing_raw_t Sensing_raw;
// extern ExtU_LLC_Control_CV_T control_in;

extern ExtU_LLC_Control_CV_T LLC_Control_CV_U;
extern ExtY_LLC_Control_CV_T LLC_Control_CV_Y;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
