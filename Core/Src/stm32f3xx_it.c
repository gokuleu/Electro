/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @author  System Lab
  * @version V1.1.0
  * @date    31-May-2018
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_it.h"
#include "LLC_control_param.h"
#include "LLC_board_config_param.h"
#include "LLC_Globals.h"    
#include "Digital_Filters.h"
#include "LLC_PWMnCurrVoltFdbk.h"
#include "StateMachine.h"  
#include "Control_Layer.h"  

    
/** @addtogroup DSMPS_project
  * @{
  */

/** @addtogroup Interrupt_handlers
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define VOUT_NUM_BIT_FILTER             ((uint8_t)3)    /**< number of bit for output voltage's digital filtering */
#define VIN_NUM_BIT_FILTER              ((uint8_t)3)    /**< number of bit for input voltage's digital filtering */    
#define TEMPERATURE_NUM_BIT_FILTER      ((uint8_t)8)    /**< number of bit for Temperature's digital filtering */
    

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* turn on fault LED without binking */
  LED_On(&FaultLED);
  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  if(hDSMPS_StartUp_TimeLeft !=0) hDSMPS_StartUp_TimeLeft--;
  if(hDSMPS_Wait_TimeLeft !=0) hDSMPS_Wait_TimeLeft--;  
  
#ifdef DSMPS_COMMUNICATION
  if(hCommunicationTimeLeft !=0) hCommunicationTimeLeft--;
#endif
  
#ifdef VOUT_UNDERVOLTAGE_PROTECTION
   if(hDSMPS_UndervoltageValidation_TimeLeft !=0) hDSMPS_UndervoltageValidation_TimeLeft--;
#endif
   
  LED_DecreaseBlinkCounter(&StatusLED);
  LED_DecreaseBlinkCounter(&FaultLED);
  HAL_IncTick();   
}

/******************************************************************************/
/*                 STM32F3xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) , for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f3xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles HRTIM TIMC interrupt request.
  * @param  None
  * @retval None
  *
  * NOT USED
  */
void HRTIM1_TIMC_IRQHandler(void)
{  
  /* HRTIM IrqHandler */
//  HAL_HRTIM_IRQHandler(&hhrtim, HRTIM_TIMERINDEX_TIMER_C);
  /* Clear IT flag directly to speed-up the code */
  __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_IT_CMP4);  
}

/**
  * @brief  This function handles TIM16 interrupt request.
  * @param  None
  * @retval None
  *
  * Schedules temperature acquisition and fan speed control
  */
void TIM16_IRQHandler(void)
{      
  /* TIM6 IrqHandler */
//  HAL_TIM_IRQHandler(&TimTempFanCtrlHandle);
  /* Clear IT flag directly to speed-up the code */  
  __HAL_TIM_CLEAR_IT(&TimTempFanCtrlHandle, TIM_IT_UPDATE);

  /* filtered temperature measures */
  hTemperatureFiltered = uSimpleDigitalLowPassFilter(DCDC_MeasureStruct.hTemperature, &hTemperatureFiltered, TEMPERATURE_NUM_BIT_FILTER);
  
  /* update Temperature measure */
  fTemperatureDeg = TEMPERATURE_ADC2DEG(hTemperatureFiltered);
  
  /* simple low pass digital filter of input voltage for input UVP and OVP */
  hVinVoltageFiltered = uSimpleDigitalLowPassFilter(DCDC_MeasureStruct.hVin, &hVinVoltageFiltered, VIN_NUM_BIT_FILTER);
    
  /* update Config Param if UI_COMMUNICATION is not defined and changes are made via debugger */ 
  CTR_UpdateConfigParam(&DCDC_ConfigParamStruct, &hhrtim);
  
  /* check if the converter is in run state for fan, SR and burst mode driving */ 
  if(STM_GetStateMachineStatus() == DSMPS_RUN)
  {
  
#ifdef FAN_PWM_DRIVING
    if(DCDC_ConfigParamStruct.bFanPWMDrivingEnabled == TRUE)
    {
      /* regulate fan speed depending only on load current */
    //  CTR_FanSpeedRegulationLoad(DCDC_MeasureStruct.hIout);  
      /* regulate fan speed depending on temperature and load current */
      CTR_FanSpeedRegulationTempLoad(fTemperatureDeg, DCDC_MeasureStruct.hIout, &FanPwmHandle);
//      CTR_FanSpeedRegulationTempLoadDis(&DCDC_ConfigParamStruct, fTemperatureDeg, DCDC_MeasureStruct.hIout, &FanPwmHandle);
//      
      /* run fan at fixed speed with a duty cycle equal to FAN_PWMDutyCycle */
    //  PWM_FanActuation(&FanPwmHandle, FAN_PWM_TIM_CHANNEL, FAN_PWMDutyCycle); 
    }
#endif
    
//#ifdef LIGHT_LOAD_BURST_MODE
//    /* Light load burst mode control ------------------------------------- */
//    if(DCDC_ConfigParamStruct.bBurstModeEnabled == TRUE)
//    {
//      /* manage burst mode for light load using the output current measure */
//      CTR_BurstModeControl(DCDC_MeasureStruct.hIout, &hhrtim);
//    }
//#endif
  
#ifdef AUTOMATIC_SR_TURN_ON_OFF
    if(DCDC_ConfigParamStruct.bSREnabled == TRUE)
    {
      /* manage automatic turn-on/off for SR depending on the load current */
      CTR_SRAutomaticTurnOnOffLoad(DCDC_MeasureStruct.hIout, &hhrtim);
    }
#endif
  
  }
}

/**
  * @brief  This function handles TIM6 and DAC1 interrupt request.
  * @param  None
  * @retval None
  *
  * Schedules Vout control loop, adaptive SR (if defined) and registers update
  */
__attribute__((section(".ccmram"))) 
void TIM6_DAC1_IRQHandler(void)
{
   /* debug set GPIO pin */
//  HAL_GPIO_WritePin(FAULT_LED_GPIO_PORT, FAULT_LED_GPIO_PIN, GPIO_PIN_SET);
  /* TIM6 Irq Handler */
//  HAL_TIM_IRQHandler(&TimVoutControlHandle);
  /* Clear IT flag directly to speed-up the code */
  __HAL_TIM_CLEAR_IT(&TimVoutControlHandle, TIM_IT_UPDATE);
    
  /*** ------------- VOUT CONTROL LOOP ------------- ***/
  /* simple low pass digital filter on Vout voltage */
  hVoutVoltageFiltered = uSimpleDigitalLowPassFilter(DCDC_MeasureStruct.hVout, &hVoutVoltageFiltered, VOUT_NUM_BIT_FILTER);   
  
  /************* debug conversion factors ******************/
//  static float debugVoutV = 0;
//  static float debugVinV = 0;
//  static float debugIoutA = 0;

//  debugVoutV = OUT_VOLT_ADC_2_VOLT_VALUE(hVoutVoltageFiltered);
//  debugVinV = IN_VOLT_ADC_2_VOLT_VALUE(hVinVoltageFiltered); //DCDC_MeasureStruct.hVin);
//  debugIoutA = HALL_IC_ADC_VALUE2AMP(DCDC_MeasureStruct.hIout);  
  /*********************************************************/
  
#ifdef GATE_DRIVER_BOOTSTRAP_PRECHARGE
  /* check is precharge state is enabled */
  if(FullBridgeDriverPrechargeStruct.bPrechargeEnabled == TRUE){
    /* perform bootstrap precharge routine */
    CTR_GateDriverBootstrapCapPrecharge(&hhrtim);
  }
#endif
  
  if(bDCDC_OutputEnabled == TRUE){ // to check if this condition is needed

    /* frequency ramp-up during DSMPS_START start state */
    if(STM_GetStateMachineStatus() == DSMPS_START){
#ifdef FREQUENCY_RAMP_UP
      /* ramp-up of PWM period in closed loop from HRTIM_MIN_PWM_PERIOD_START_UP to HRTIM_MAX_PWM_PERIOD_START_UP, it is interrupted when Vout reaches VOUT_MIN_CLOSED_LOOP */
      if(DCDC_ConfigParamStruct.bOpenLoopEnabled == FALSE){
        if(hPWMPeriod < HRTIM_MAX_PWM_PERIOD_START_UP){
          if (hStartUpFreqUpdateCounter == 0){
            hPWMPeriod += HRTIM_START_UP_DELTA_PERIOD;
            hStartUpFreqUpdateCounter = START_UP_COUNTER_PRESCALER_RATIO;
          }
          else{
            hStartUpFreqUpdateCounter--;
          }
        }
        else{ // hPWMPeriod > HRTIM_MAX_PWM_PERIOD_START_UP
          hPWMPeriod = HRTIM_MAX_PWM_PERIOD_START_UP;
        }
      }
      else /* Open loop operation: ramp-up of PWM period from HRTIM_MIN_PWM_PERIOD_START_UP to HRTIM_MIN_PWM_PERIOD, and it is not interrupted */
      {
        if(hPWMPeriod < HRTIM_MIN_PWM_PERIOD){
          if (hStartUpFreqUpdateCounter == 0){
            hPWMPeriod += HRTIM_START_UP_DELTA_PERIOD;
            hStartUpFreqUpdateCounter = START_UP_COUNTER_PRESCALER_RATIO;
          }
          else{
            hStartUpFreqUpdateCounter--;
          }
        }
        else{ // hPWMPeriod > HRTIM_MIN_PWM_PERIOD
          hPWMPeriod = HRTIM_MIN_PWM_PERIOD; //open loop frequency set at minimum period after ramp-up
        }
      }
#endif
     }
    
    /* Close control loop if start-up is completed */   
    else if(STM_GetStateMachineStatus() == DSMPS_RUN){
      
#ifdef LIGHT_LOAD_BURST_MODE
    /* Light load burst mode control ------------------------------------- */
    if(DCDC_ConfigParamStruct.bBurstModeEnabled == TRUE)
    {
      /* manage burst mode for light load using the output current measure */
      CTR_BurstModeControl(DCDC_MeasureStruct.hIout, &hhrtim);
    }
#endif
    
      /* Set frequency in open or closed loop ------------------------------- */
      if(DCDC_ConfigParamStruct.bOpenLoopEnabled == TRUE)
      {
        /* calc PWM period in open loop mode */
        hPWMPeriod = CTR_SetPWMFrequency(DCDC_ConfigParamStruct.wOpenLoopFreq_Hz); // can be made once in open loop?
      }
      else // closed loop operation
      {
        /* perform Vout control loop */
        hPWMPeriod = CTR_ExecuteVoltageLoop(hVout_Reference, hVoutVoltageFiltered); 
      }      

      /* SR rising and falling edges calculation ------------------------------ */
      if(bDCDC_AdaptiveSynchRectEnabled == TRUE) 
      {
        /* if Adaptive SR is enabled together with output SR, calc adaptive delays, otherwise set fallig edgs at init value */
        if(bDCDC_SynchRectOutputEnabled == TRUE)  
        {
          /* calculation of adaptive SR falling edges */
          CTR_AdaptiveSynchRectEdgeCalculation();
        }
        else{
          hSR_DelayFalling1 = SYNCH_RECT_DELAY_FALLING1_INIT_HR_TICKS;                    
          hSR_DelayFalling2 = SYNCH_RECT_DELAY_FALLING2_INIT_HR_TICKS;
        }
      }
      else // bDCDC_AdaptiveSynchRectEnabled == FALSE
      {
        /* calculation of fixed SR rising and falling edges */
//        CTR_SetSRFixedDelays(&DCDC_ConfigParamStruct); // already done in CTR_UpdateConfigParam()
      }
    
    }
   
/* Update HRTIM registers ----------------------------------------------------*/

  /* Disable Update of Master timer and all timers used */
  HAL_HRTIM_UpdateDisable(&hhrtim, (HRTIM_TIMERUPDATE_MASTER | PWM_FB_HS1_LS1_HRTIM_TIMERUPDATE | PWM_FB_HS2_LS2_HRTIM_TIMERUPDATE | PWM_SR_HS2_LS1_HRTIM_TIMERUPDATE | PWM_SR_HS1_LS2_HRTIM_TIMERUPDATE));

  /* check if dead time has to be updated */
  if(bDCDC_DeadTimeChanged == TRUE)
  {
    /* update deadtime value if it is changed */
    PWM_DeadTimeActuation(&hhrtim, hPWMDeadTime);
    /* reset dead time changed flag */
    bDCDC_DeadTimeChanged = FALSE;
  }    
  /* update PWM period */
//  PWM_PeriodActuation(&hhrtim, hPWMPeriod);
      
  /* update SR PWMs: if rising/falling edge are calculated only when output are enabled, at start-up the values won't be correct -> overcurrent fault */
  /* update SR waveforms - DT is fixed */
//    PWM_SynchRectActuation(&hhrtim, hPWMPeriod, hSR_DelayRising1, hSR_DelayFalling1, hSR_DelayRising2, hSR_DelayFalling2);
  /* update SR waveforms depending also on the last value of dead time (rising and falling DT are the same) */
//  PWM_SynchRectActuationDT(&hhrtim, hPWMPeriod, hSR_DelayRising1, hSR_DelayFalling1, hSR_DelayRising2, hSR_DelayFalling2, hPWMDeadTimeHRticks, hPWMDeadTimeHRticks);
  /* actuation of main and SR PWM signals with a variable dead time */
  PWM_PeriodSynchRectActuationDT(&hhrtim, hPWMPeriod, hSR_DelayRising1, hSR_DelayFalling1, hSR_DelayRising2, hSR_DelayFalling2, hPWMDeadTimeHRticks);
  /* Enable Update of Master timer and all timers used - in this manner all registers are updated together at next MASTER repetition event */
  HAL_HRTIM_UpdateEnable(&hhrtim, (HRTIM_TIMERUPDATE_MASTER | PWM_FB_HS1_LS1_HRTIM_TIMERUPDATE | PWM_FB_HS2_LS2_HRTIM_TIMERUPDATE | PWM_SR_HS2_LS1_HRTIM_TIMERUPDATE | PWM_SR_HS1_LS2_HRTIM_TIMERUPDATE));
  
 }
 
//  /* Enable Update of Master timer and all timers used - in this manner all registers are updated together at next MASTER repetition event */
//  HAL_HRTIM_UpdateEnable(&hhrtim, (HRTIM_TIMERUPDATE_MASTER | PWM_FB_HS1_LS1_HRTIM_TIMERUPDATE | PWM_FB_HS2_LS2_HRTIM_TIMERUPDATE | PWM_SR_HS2_LS1_HRTIM_TIMERUPDATE | PWM_SR_HS1_LS2_HRTIM_TIMERUPDATE));
    
  /* debug reset GPIO pin */
  HAL_GPIO_WritePin(DEBUG_DIGITAL_OUT1_GPIO_PORT, DEBUG_DIGITAL_OUT1_GPIO_PIN, GPIO_PIN_RESET);
  
  /* Clear IT flag directly to speed-up the code */
//  __HAL_TIM_CLEAR_IT(&TimVoutControlHandle, TIM_IT_UPDATE);
}

/**
* @brief  This function handles COMP_OC interrupt request.
* @param  None
* @retval None
*/
void COMP_OC_IRQHandler(void)
{    
  /* Overcurrent protection managed in HRTIM1_FLT_IRQHandler()*/
  /* COMPx IrqHandler */
  HAL_COMP_IRQHandler(&CompOCHandle);
}

/**
* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
void ADCx_DMA_IRQHandler(void)
{
  if(__HAL_DMA_GET_TC_FLAG_INDEX(AdcHandle1.DMA_Handle) == DMA_FLAG_TC1){
      
    /* debug set GPIO pin */
//    HAL_GPIO_WritePin(DEBUG_DIGITAL_OUT1_GPIO_PORT, DEBUG_DIGITAL_OUT1_GPIO_PIN, GPIO_PIN_SET);
    
    /* DMA IrqHandler */
  //  HAL_DMA_IRQHandler(AdcHandle1.DMA_Handle);
    /* clear directly the TC flag to speed-up the code */
    /* Clear the transfer complete flag */
    __HAL_DMA_CLEAR_FLAG(AdcHandle1.DMA_Handle, DMA_FLAG_TC1);
    /* debug reset GPIO pin */
//    HAL_GPIO_WritePin(DEBUG_DIGITAL_OUT1_GPIO_PORT, DEBUG_DIGITAL_OUT1_GPIO_PIN, GPIO_PIN_RESET);
  }
}

/**
  * @brief  This function handles DMA interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is related to DMA stream used for USART data transmission     
  */
void USARTx_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
  
  // TO MOVE TO THE COMMUNICATION FILE
  /* received status from primary MCU */
//  PFC_Received_Status = ReceivePrimaryStatus();
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is related to DMA stream used for USART data reception    
  */
void USARTx_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}

/**
  * @brief  This function handles USARTx interrupt request.
  * @param  None
  * @retval None
  */
void USARTx_IRQHandler(void)
{
  HAL_UART_IRQHandler(& UartHandle);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
