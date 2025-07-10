/**
  ******************************************************************************
  * @file    LLC_PWMnCurrVoltFdbk.c
  * @author  IMS Systems Lab 
  * @version V2.0.0
  * @date    23-May-2017
  * @brief   PWM output and measure feedback.
  *          This file provides fuctions to drive PWM signals and acquire voltages 
  *			 and currents
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
#include "stm32f3xx_hal.h"
#include "HRTIM_pwm_config_param.h"
#include "LLC_board_config_param.h"
#include "LLC_control_param.h"
#include "LLC_PWMnCurrVoltFdbk.h"
#include "LLC_Globals.h"

/** @addtogroup DSMPS_project
  * @{
  */

/** @addtogroup PWM_Actuation_and_Sensing
  * @{
  */
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Updates HRTIM master period registers changing PWM frequency
  * @param  hhrtim: pointer to HRTIM handler
  * @param  hPWMPeriodHRTimTick: PWM period value in HRTIM ticks
  * @retval None
*/
__attribute__((section(".ccmram"))) 
void PWM_PeriodActuation(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMPeriodHRTimTick)
{   
  /* Set MASTER_CMP2 for ADCx regular sequence's trigger - it doesn't change */
//  __HAL_HRTIM_SetCompare(hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, ADCx_REGULAR_TRIGGER_POINT_HR_TICKS);

  /* Set MASTER_CMP3 for ADCy regular sequence's trigger - it doesn't change if CMP is configured in autodelayed mode */
//  __HAL_HRTIM_SetCompare(hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, (hPWMPeriodHRTimTick/2 + ADCy_REGULAR_TRIGGER_POINT_HR_TICKS));
  
  /* Set TIMA_CMP3 for ADCy regular sequence's trigger - it doesn't change if CMP is configured in autodelayed mode */
  __HAL_HRTIM_SetCompare(hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, (hPWMPeriodHRTimTick/2 + ADCy_REGULAR_TRIGGER_POINT_HR_TICKS));
   
  /* set new HRTIM Master Period */
  __HAL_HRTIM_SetPeriod(hhrtim, HRTIM_TIMERINDEX_MASTER, hPWMPeriodHRTimTick);
  
  /* set new PWM_FB_HS1_LS1_HRTIM_TIMERINDEX Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick);  
  
  /* set new PWM_FB_HS2_LS2_HRTIM_TIMERINDEX Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_FB_HS2_LS2_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick);  

}

/**
  * @brief  Updates HRTIM compare registers to generate synchronous rectification waveforms, the dead time is considered fixed at compilation time
  * @param  hhrtim: pointer to HRTIM handler
  * @param  hPWMPeriodHRTimTick: PWM period value in HRTIM ticks
  * @param  hDelaySRRising1: rising delay for SR1 waveform in HRTIM ticks
  * @param  hDelaySRFalling1: falling delay for SR1 waveform in HRTIM ticks
  * @param  hDelaySRRising2: rising delay for SR2 waveform in HRTIM ticks
  * @param  hDelaySRFalling2: falling delay for SR2 waveform in HRTIM ticks
  * @retval None
*/
__attribute__((section(".ccmram"))) 
void PWM_SynchRectActuation(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMPeriodHRTimTick, int16_t hDelaySRRising1, int16_t hDelaySRFalling1, int16_t hDelaySRRising2, int16_t hDelaySRFalling2)
{
  /* to consider if FB PWM dead time can change */ 
  uint16_t hSRRising1 = (uint16_t)((int32_t)DEAD_TIME_RISING_HR_TICKS_SIGNED + (int32_t)hDelaySRRising1); // if hDelaySRRising1 <0, must be anyway (hDelaySRRising1 + DEAD_TIME_RISING_HR_TICK_SIGNED) > HRTIM_MIN_CMP_PER_VALUE
  uint16_t hSRFalling1 = (uint16_t)((int32_t)hPWMPeriodHRTimTick/2 - (int32_t)hDelaySRFalling1);  
  uint16_t hSRRising2 = (uint16_t)((int32_t)hPWMPeriodHRTimTick/2 + (int32_t)hDelaySRRising2 + DEAD_TIME_FALLING_HR_TICKS_SIGNED); 
  uint16_t hSRFalling2 = (uint16_t)((int32_t)hPWMPeriodHRTimTick - (int32_t)hDelaySRFalling2);
  
  /* check that the falling edge is inside the period - only for negative delay */
  if(hSRFalling2 >= hPWMPeriodHRTimTick){
    hSRFalling2 -= hPWMPeriodHRTimTick;
    if(hSRFalling2 < HRTIM_MIN_CMP_PER_VALUE) hSRFalling2 = HRTIM_MIN_CMP_PER_VALUE;
  }
  else if(hSRFalling2 > HRTIM_MAX_CMP_PER_VALUE){
    hSRFalling2 = HRTIM_MAX_CMP_PER_VALUE;
  }
    
  #ifndef AUTODELAYED_ADCy_INJECTED_TRIGGER
    uint16_t hSRInjectedADCTrigger2 = (uint16_t)(hSRFalling2 + ADCy_INJECTED_TRIGGER_POINT_HR_TICKS);
    if(hSRInjectedADCTrigger2 >= hPWMPeriodHRTimTick){
      hSRInjectedADCTrigger2 -= hPWMPeriodHRTimTick;
      if(hSRInjectedADCTrigger2 < HRTIM_MIN_CMP_PER_VALUE) hSRInjectedADCTrigger2 = HRTIM_MIN_CMP_PER_VALUE;
    }
    else if(hSRInjectedADCTrigger2 > HRTIM_MAX_CMP_PER_VALUE){
      hSRInjectedADCTrigger2 = HRTIM_MAX_CMP_PER_VALUE;
    }
  #endif
      
  /* rising edge SR_HS2_LS1 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_2, hSRRising1);
  
  /* falling edge SR_HS2_LS1 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_1, hSRFalling1);
  
  /* rising edge SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_2, hSRRising2);
  
  /* falling edge SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_1, hSRFalling2);  

#ifdef ADAPTIVE_SYNCH_RECTIFICATION 
  /* blanking window SR_HS2_LS1 update - it changes only if hDelaySRRising1 changes too */ 
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_3, hSRRising1 + SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS);
  
  /* blanking window SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_3, hSRRising2 + SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS);
  
  #ifndef AUTODELAYED_ADCy_INJECTED_TRIGGER  
    /* injected trigger point for Vds2 sensing update */
    __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_4, hSRInjectedADCTrigger2);
  #endif
  
#endif
  
  /* set new HRTIM SR1 Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick);
  
  /* set new HRTIM SR2 Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick);
}

/**
  * @brief  Updates HRTIM compare registers to generate synchronous rectification waveforms, the dead times rising and falling can change and are passed to the function
  * @param  hhrtim: pointer to HRTIM handler
  * @param  hPWMPeriodHRTimTick: PWM period value in HRTIM ticks
  * @param  hDelaySRRising1: rising delay for SR1 waveform in HRTIM ticks
  * @param  hDelaySRFalling1: falling delay for SR1 waveform in HRTIM ticks
  * @param  hDelaySRRising2: rising delay for SR2 waveform in HRTIM ticks
  * @param  hDelaySRFalling2: falling delay for SR2 waveform in HRTIM ticks
  * @param  hDeadTimeRising: dead time rising of FB PWM in HRTIM ticks
  * @param  hDeadTimeFalling: dead time falling of FB PWM in HRTIM ticks
  * @retval None
*/
__attribute__((section(".ccmram"))) 
void PWM_SynchRectActuationDT(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMPeriodHRTimTick, int16_t hDelaySRRising1, int16_t hDelaySRFalling1, int16_t hDelaySRRising2, int16_t hDelaySRFalling2, int16_t hDeadTimeRising, int16_t hDeadTimeFalling)
{
  uint16_t hSRRising1 = (uint16_t)((int32_t)hDeadTimeRising + (int32_t)hDelaySRRising1); // if hDelaySRRising1 <0, must be anyway (hDelaySRRising1 + DEAD_TIME_RISING_HR_TICK_SIGNED) > HRTIM_MIN_CMP_PER_VALUE
  uint16_t hSRFalling1 = (uint16_t)((int32_t)hPWMPeriodHRTimTick/2 - (int32_t)hDelaySRFalling1);  
  uint16_t hSRRising2 = (uint16_t)((int32_t)hPWMPeriodHRTimTick/2 + (int32_t)hDelaySRRising2 + hDeadTimeFalling); 
  uint16_t hSRFalling2 = (uint16_t)((int32_t)hPWMPeriodHRTimTick - (int32_t)hDelaySRFalling2);
    
  /* check if rising edge 1 is inside the minimum threshold - it is considered  hDelaySRRising1 + hDeadTimeRising > 0 with hDeadTimeRising > 0 */
  if(hSRRising1 < HRTIM_MIN_CMP_PER_VALUE)
  {
    hSRRising1 = HRTIM_MIN_CMP_PER_VALUE;
  }
  
 
  /* check if the falling edge is inside the period */
  if(hSRFalling2 >= hPWMPeriodHRTimTick){
    hSRFalling2 = hPWMPeriodHRTimTick -1;
  }
  
  #ifndef AUTODELAYED_ADCy_INJECTED_TRIGGER
    uint16_t hSRInjectedADCTrigger2 = (uint16_t)(hSRFalling2 + ADCy_INJECTED_TRIGGER_POINT_HR_TICKS);
    if(hSRInjectedADCTrigger2 >= hPWMPeriodHRTimTick){
      hSRInjectedADCTrigger2 -= hPWMPeriodHRTimTick;
      if(hSRInjectedADCTrigger2 < HRTIM_MIN_CMP_PER_VALUE) hSRInjectedADCTrigger2 = HRTIM_MIN_CMP_PER_VALUE;
    }
//    else if(hSRInjectedADCTrigger2 > HRTIM_MAX_CMP_PER_VALUE){
//      hSRInjectedADCTrigger2 = HRTIM_MAX_CMP_PER_VALUE;
//    }
  #endif
  
  /* set new HRTIM SR1 Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick);  // probably not needed if timer is reset by HRTIM_MASTER or TIMA/B
  
  /* set new HRTIM SR2 Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick);  // probably not needed if timer is reset by HRTIM_MASTER or TIMA/B
  
  /* rising edge SR_HS2_LS1 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_2, hSRRising1);
  
  /* falling edge SR_HS2_LS1 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_1, hSRFalling1);
  
  /* rising edge SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_2, hSRRising2);
  
  /* falling edge SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_1, hSRFalling2);  

#ifdef ADAPTIVE_SYNCH_RECTIFICATION 
  /* blanking window SR_HS2_LS1 update */ // changes only if hDelaySRRising1 changes too
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_3, hSRRising1 + SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS);
  
  /* blanking window SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_3, hSRRising2 + SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS);
  
  #ifndef AUTODELAYED_ADCy_INJECTED_TRIGGER  
    /* injected trigger point for Vds2 sensing update */
    __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_4, hSRInjectedADCTrigger2);
  #endif
  
#endif

}

/**
  * @brief  Updates HRTIM compare registers to generate synchronous both rectification waveforms and main PWMs, the dead time (unique for rising and falling edge) can change and is passed to the function
  * @param  hhrtim: pointer to HRTIM handler
  * @param  hPWMPeriodHRTimTick: PWM period value in HRTIM ticks
  * @param  hDelaySRRising1: rising delay for SR1 waveform in HRTIM ticks
  * @param  hDelaySRFalling1: falling delay for SR1 waveform in HRTIM ticks
  * @param  hDelaySRRising2: rising delay for SR2 waveform in HRTIM ticks
  * @param  hDelaySRFalling2: falling delay for SR2 waveform in HRTIM ticks
  * @param  hDeadTime: dead time rising/falling of FB PWM in HRTIM ticks
  * @retval None
*/
__attribute__((section(".ccmram"))) 
void PWM_PeriodSynchRectActuationDT(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMPeriodHRTimTick, int16_t hDelaySRRising1, int16_t hDelaySRFalling1, int16_t hDelaySRRising2, int16_t hDelaySRFalling2, int16_t hDeadTime)
{
  uint16_t hSRRising1 = (uint16_t)((int32_t)hDeadTime + (int32_t)hDelaySRRising1); // if hDelaySRRising1 <0, must be anyway (hDelaySRRising1 + DEAD_TIME_RISING_HR_TICK_SIGNED) > HRTIM_MIN_CMP_PER_VALUE
  uint16_t hSRFalling1 = (uint16_t)((int32_t)hPWMPeriodHRTimTick/2 - (int32_t)hDelaySRFalling1);  
  uint16_t hSRRising2 = (uint16_t)((int32_t)hPWMPeriodHRTimTick/2 + (int32_t)hDelaySRRising2 + hDeadTime); 
  uint16_t hSRFalling2 = (uint16_t)((int32_t)hPWMPeriodHRTimTick - (int32_t)hDelaySRFalling2);
    
  /* check if rising edge 1 is inside the minimum threshold - it is considered  hDelaySRRising1 + hDeadTime > 0 with hDeadTime > 0 */
  if(hSRRising1 < HRTIM_MIN_CMP_PER_VALUE)
  {
    hSRRising1 = HRTIM_MIN_CMP_PER_VALUE;
  }
  
  /* check if the falling edge is inside the period */
  if(hSRFalling2 >= hPWMPeriodHRTimTick){
    hSRFalling2 = hPWMPeriodHRTimTick -1;
  }
  
  #ifndef AUTODELAYED_ADCy_INJECTED_TRIGGER
    uint16_t hSRInjectedADCTrigger2 = (uint16_t)(hSRFalling2 + ADCy_INJECTED_TRIGGER_POINT_HR_TICKS);
    if(hSRInjectedADCTrigger2 >= hPWMPeriodHRTimTick){
      hSRInjectedADCTrigger2 -= hPWMPeriodHRTimTick;
      if(hSRInjectedADCTrigger2 < HRTIM_MIN_CMP_PER_VALUE) hSRInjectedADCTrigger2 = HRTIM_MIN_CMP_PER_VALUE;
    }
//    else if(hSRInjectedADCTrigger2 > HRTIM_MAX_CMP_PER_VALUE){
//      hSRInjectedADCTrigger2 = HRTIM_MAX_CMP_PER_VALUE;
//    }
  #endif
      
  /* set new HRTIM Master Period */
  __HAL_HRTIM_SetPeriod(hhrtim, HRTIM_TIMERINDEX_MASTER, hPWMPeriodHRTimTick);   
  
  /* set new PWM_FB_HS1_LS1_HRTIM_TIMERINDEX Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_FB_HS1_LS1_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick);  
  
  /* set new PWM_FB_HS2_LS2_HRTIM_TIMERINDEX Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_FB_HS2_LS2_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick);  
    
  /* set new HRTIM SR1 Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick); // probably not needed if timer is reset by HRTIM_MASTER or TIMA/B
  
  /* set new HRTIM SR2 Period */
  __HAL_HRTIM_SetPeriod(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, hPWMPeriodHRTimTick); // probably not needed if timer is reset by HRTIM_MASTER or TIMA/B
  
  /* rising edge SR_HS2_LS1 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_2, hSRRising1);
  
  /* falling edge SR_HS2_LS1 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_1, hSRFalling1);
  
  /* rising edge SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_2, hSRRising2);
  
  /* falling edge SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_1, hSRFalling2);

  /* Set MASTER_CMP3 for ADCy regular sequence's trigger - it doesn't change if CMP is configured in autodelayed mode */
//  __HAL_HRTIM_SetCompare(hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, (hPWMPeriodHRTimTick/2 + ADCy_REGULAR_TRIGGER_POINT_HR_TICKS));

/* Set TIMA_CMP3 for ADCy regular sequence's trigger - it doesn't change if CMP is configured in autodelayed mode */
  __HAL_HRTIM_SetCompare(hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, (hPWMPeriodHRTimTick/2 + ADCy_REGULAR_TRIGGER_POINT_HR_TICKS));  

#ifdef ADAPTIVE_SYNCH_RECTIFICATION 
  /* blanking window SR_HS2_LS1 update */ // changes only if hDelaySRRising1 changes too
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS2_LS1_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_3, hSRRising1 + SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS);
  
  /* blanking window SR_HS1_LS2 update */
  __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_3, hSRRising2 + SYNCH_RECT_TURN_OFF_BLANKING_WINDOW_HR_TICKS);
  
  #ifndef AUTODELAYED_ADCy_INJECTED_TRIGGER  
    /* injected trigger point for Vds2 sensing update */
    __HAL_HRTIM_SetCompare(hhrtim, PWM_SR_HS1_LS2_HRTIM_TIMERINDEX, HRTIM_COMPAREUNIT_4, hSRInjectedADCTrigger2);
  #endif
  
#endif

}

/**
  * @brief  Updates HRTIM dead time registers for PWM signals
  * @param  hhrtim: pointer to HRTIM handler
  * @param  hPWMDeadTimeDTTick: Dead time value DT ticks for both rising and falling (only positive values)
  * @retval None
*/
__attribute__((section(".ccmram"))) 
void PWM_DeadTimeActuation(HRTIM_HandleTypeDef * hhrtim, uint16_t hPWMDeadTimeDTTick)
{     
  uint32_t hrtim_hs1_ls1_dtr = hhrtim->Instance->sTimerxRegs[PWM_FB_HS1_LS1_HRTIM_TIMERINDEX].DTxR;
  uint32_t hrtim_hs2_ls2_dtr = hhrtim->Instance->sTimerxRegs[PWM_FB_HS2_LS2_HRTIM_TIMERINDEX].DTxR;
  
  /* Clear only deadtime values */
  hrtim_hs1_ls1_dtr &= ~(HRTIM_DTR_DTR | HRTIM_DTR_DTF);
  hrtim_hs2_ls2_dtr &= ~(HRTIM_DTR_DTR | HRTIM_DTR_DTF);
  
  /* set new rising and falling dead times */
  hrtim_hs1_ls1_dtr |= hPWMDeadTimeDTTick + (hPWMDeadTimeDTTick << 16);
  hrtim_hs2_ls2_dtr |= hPWMDeadTimeDTTick + (hPWMDeadTimeDTTick << 16);
  
  /* Update the HRTIM registers */  
  hhrtim->Instance->sTimerxRegs[PWM_FB_HS1_LS1_HRTIM_TIMERINDEX].DTxR = hrtim_hs1_ls1_dtr;
  hhrtim->Instance->sTimerxRegs[PWM_FB_HS2_LS2_HRTIM_TIMERINDEX].DTxR = hrtim_hs2_ls2_dtr;
    
}

/**
  * @brief  Updates HRTIM burst mode period and compare registers
  * @param  hhrtim: pointer to HRTIM handler
  * @param  hHRTIMBurstModePeriod: number of HRTIM periods -1 that composes the burst mode period (idle + run)
  * @param  hHRTIMBurstModeIdle: number of HRTIM periods -1 that composes the idle duration
  * @retval None
*/
void PWM_UpdateBurstModeParams(HRTIM_HandleTypeDef * hhrtim, uint16_t hHRTIMBurstModePeriod, uint16_t hHRTIMBurstModeIdle)
{  
  /* A write into the HRTIM_BMPER period register disables the update temporarily, until the
  HRTIM_BMCMP compare register is written, to ensure the consistency of the two registers
  when they are modified. */
  
  /* Set the burst mode period */
  hhrtim->Instance->sCommonRegs.BMPER = hHRTIMBurstModePeriod;
  /* Set the burst mode compare value */
  hhrtim->Instance->sCommonRegs.BMCMPR = hHRTIMBurstModeIdle; 
}

/**
  * @brief  Updates Fan TIM compare register with new value of duty cycle
  * @param  htim: pointer to Fan TIM handler
  * @param  hPulseDurationTimTick: duration pulse in TIM ticks
  * @retval None
*/
void PWM_FanActuation(TIM_HandleTypeDef * htim, uint32_t Channel, uint16_t hPulseDurationTimTick)
{
   switch (Channel)
  {
    case TIM_CHANNEL_1:
    {  
      /* Set the Capture Compare Register value */
       htim->Instance->CCR1 = hPulseDurationTimTick;
       break;
    }
    case TIM_CHANNEL_2:
    {  
      /* Set the Capture Compare Register value */
       htim->Instance->CCR2 = hPulseDurationTimTick;
       break;
    }
    case TIM_CHANNEL_3:
    {  
      /* Set the Capture Compare Register value */
       htim->Instance->CCR3 = hPulseDurationTimTick;
       break;
    }
    case TIM_CHANNEL_4:
    {  
      /* Set the Capture Compare Register value */
       htim->Instance->CCR4 = hPulseDurationTimTick;
       break;
    }
    default:
      break;
  }
}
  

/**
  * @brief  return ADC Vds measure of low side SR MOSFETs in 12 bit format [0:4095] 
  * @param  SR_Leg can be SR_LEG1 or SR_LEG2
  * @retval ADC Vds temperature measure 
  *
  */
uint16_t PWM_GetVdsSRMeasure(DCDC_SRLeg_t SRLeg)
{
  if (SRLeg == SR_LEG1){
    return (HAL_ADCEx_InjectedGetValue(&AdcHandle1, ADC_INJECTED_RANK_1));
  }
  else{  //(SRLeg == SR_LEG2)
    return (HAL_ADCEx_InjectedGetValue(&AdcHandle2, ADC_INJECTED_RANK_1));
  }
}

/**
  * @brief  return ADC measure of passed current value in Ampere considering the calibrated value for current offset
  * @param  current value in Ampere
  * @retval ADC measure of the passed value
  *
  */
uint16_t PWM_ConvertCurrentValue(float fCurrentA)
{
  uint16_t hCurrentMeasure = 0;
  
  hCurrentMeasure = ((uint16_t)((((fCurrentA * HALL_IC_SENSITIVITY_mV_A)/1000)*4095)/3.3f) + hHallIcOffsetCalib );        /* converts a current in Ampere to ADC measure - considering offset */
    
  return hCurrentMeasure;
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************** (C) COPYRIGHT 2017 STMicroelectronics *******************/
