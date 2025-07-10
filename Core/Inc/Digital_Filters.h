/**
  ******************************************************************************
  * @file    Digital_filters.h
  * @author  IMS Systems Lab and Technical Marketing 
  * @version 1.1.0
  * @date    26-May-2016 
  * @brief   This file contains the prototypes of Digital filters related functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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
 
#ifndef __DIGITAL_FILTERS__H
#define __DIGITAL_FILTERS__H

/* Includes ------------------------------------------------------------------*/
/* Defines  ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint16_t uSimpleDigitalLowPassFilter(uint16_t hNoFilteredValue, uint16_t *phLastFilteredValue, uint8_t bDigitShift);
int16_t sSimpleDigitalLowPassFilter(int16_t hNoFilteredValue, int16_t *phLastFilteredValue, uint8_t bDigitShift);
uint16_t IIR_DigitalFilter(uint16_t hNoFilteredValue);

/* Exported variables ------------------------------------------------------- */

#endif 

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
