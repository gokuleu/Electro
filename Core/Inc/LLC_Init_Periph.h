/**
  ******************************************************************************
  * @file    LLC_Init_Periph.h
  * @author  Systems Lab
  * @version 2.0.0
  * @date    23-May-2017 
  * @brief   This file contains prototypes of init peripherals functions
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
#ifndef __LLC_INIT_PERIPH_H
#define __LLC_INIT_PERIPH_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
void GPIO_Config(void);
void ADC_Config(void);
void HRTIM_Config(void);
void DAC_OverCurrProtection_Config(void);
void DAC_AdaptiveSR_Config(void);
void COMP_OverCurrProtection_Config(void);
void COMP_AdaptiveSR_Config(void);
void USART_Config(void);
void TIM6_VoutControl_Config(void);
void FAN_TIM_Config(void);
void TIM16_LowFrequencyTask_Config(void);

#endif /* __LLC_INIT_PERIPH_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/


