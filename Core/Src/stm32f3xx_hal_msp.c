/**
  ******************************************************************************
  * @file    stm32f3xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    24-May-2017
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "stm32f3xx_hal.h"
#include "LLC_board_config_param.h"
#include "LLC_control_param.h"
#include "UI_UART_Interface.h"

/** @addtogroup DSMPS_project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* blocking function */
    while(1)
    {
    }
}

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */
/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  static DMA_HandleTypeDef  hdma_adc1;
  static DMA_HandleTypeDef  hdma_adc2;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock */
  ADCx_CHANNEL_GPIO_CLK_ENABLE();
  /* ADCx Periph clock enable */
  ADCx_CLK_ENABLE();
  /* Enable DMA clock */
  DMAx_CLK_ENABLE();
  
  /* Enable GPIO clock */
  ADCy_CHANNEL_GPIO_CLK_ENABLE();
  /* ADC1 Periph clock enable */
  ADCy_CLK_ENABLE();
  
#ifdef DSMPS_CONTROL_BOARD  // also port C is used
  __GPIOC_CLK_ENABLE();
#endif
  
  /*##-2- Configure peripheral GPIO ##########################################*/
  /* ADC Channel GPIO pin configuration - Vout */
  GPIO_InitStruct.Pin = ADCx_VOUT_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_VOUT_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
  
  /* ADC Channel GPIO pin configuration - Vbus */
  GPIO_InitStruct.Pin = ADCx_VBUS_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_VBUS_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
  
  /* ADC Channel GPIO pin configuration - Iout */
  GPIO_InitStruct.Pin = ADCx_IOUT_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_IOUT_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
  
  /* ADC Channel GPIO pin configuration - Ires */
  GPIO_InitStruct.Pin = ADCy_IRES_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCy_IRES_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
  
  /* ADC Channel GPIO pin configuration - IresAvg */
  GPIO_InitStruct.Pin = ADCy_IRES_AVG_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCy_IRES_AVG_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
  
  /* ADC Channel GPIO pin configuration - Temperature */
  GPIO_InitStruct.Pin = ADCy_TEMP_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCy_TEMP_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION
  /* ADC Channel GPIO pin configuration - Vds_SR1*/
  GPIO_InitStruct.Pin = ADCx_VDS1_CHANNEL;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_VDS1_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
  
  /* ADC Channel GPIO pin configuration - Vds_SR2*/
  GPIO_InitStruct.Pin = ADCy_VDS2_CHANNEL;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCy_VDS2_CHANNEL_GPIO_PORT, &GPIO_InitStruct); 
#endif         

  /*##-3- Configure DMA #################################################*/
  if(hadc->Instance == ADCx)
  {
    hdma_adc1.Instance = ADCx_DMA_CHANNEL; 
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_adc1.Init.MemInc = DMA_MINC_DISABLE; // 1 conversion
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE; // 3 conversions
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    /*##-4- Select Callbacks functions called after Transfer complete and Transfer error */
//    hdma_adc1.XferCpltCallback  = TransferComplete;
//    DmaHandle.XferErrorCallback = TransferError;

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      /* ADC initialization Error */
      Error_Handler();
    } 
    
    /* Associate the initialized DMA handle to the the ADC handle */
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
    
      /* NVIC configuration for DMA transfer complete interrupt - if needed */
//      HAL_NVIC_SetPriority(ADCx_DMA_IRQn, 0, 0);   
//      HAL_NVIC_EnableIRQ(ADCx_DMA_IRQn);
    
#ifdef VOUT_ANALOG_WATCHDOG_ENABLED
          /* ADC Irq - to verify trigger point with EOS convertion Irq */
      HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);   
      HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
      /* Enable AWD1 Irq */
      __HAL_ADC_ENABLE_IT(hadc, ADC_IT_AWD1);
      /* Enable EOS Irq */
//      __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
#endif
      /* ADC Irq - to verify trigger point with EOS convertion Irq */
//      HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);   
//      HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
      /* Enable EOS Irq */
//      __HAL_ADC_ENABLE_IT(hadc, ADC_IT_EOS);
      /* Enable EOS Irq */
//      __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
      
      
  }
/****************************************************************/
  
  else if (hadc->Instance == ADCy){
    {
//      __HAL_DMA_REMAP_CHANNEL_ENABLE(ADCy_DMA_REMAP_CH); // comment if remap is not needed - see RM pag 180
        
      hdma_adc2.Instance = ADCy_DMA_CHANNEL; 
      hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
      hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
//      hdma_adc2.Init.MemInc = DMA_MINC_DISABLE; // 1 conversion
      hdma_adc2.Init.MemInc = DMA_MINC_ENABLE; // 3 conversions
      hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
      hdma_adc2.Init.Mode = DMA_CIRCULAR;
      hdma_adc2.Init.Priority = DMA_PRIORITY_VERY_HIGH;

      /*##-4- Select Callbacks functions called after Transfer complete and Transfer error */
//      hdma_adc2.XferCpltCallback  = TransferComplete;
//      DmaHandle.XferErrorCallback = TransferError;

      if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
      {
        /* ADC initialization Error */
        Error_Handler();
      } 
      
      /* Associate the initialized DMA handle to the the ADC handle */
      __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);
  
//      /* NVIC configuration for DMA transfer complete interrupt - if needed */
//      HAL_NVIC_SetPriority(ADCy_DMA_IRQn, 2, 0);   
//      HAL_NVIC_EnableIRQ(ADCy_DMA_IRQn);
    }
  }
  
}

/**
  * @brief ADC MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  static DMA_HandleTypeDef  hdma_adc;
  
  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADCx GPIO pin */
  HAL_GPIO_DeInit(ADCx_VOUT_CHANNEL_GPIO_PORT, ADCx_VOUT_CHANNEL_PIN);
  /* De-initialize the ADCy GPIO pin */
  HAL_GPIO_DeInit(ADCy_TEMP_CHANNEL_GPIO_PORT, ADCy_TEMP_CHANNEL_PIN);
  
  /*##-3- Disable the DMA Streams ############################################*/
  /* De-Initialize the DMA Stream associate to transmission process */
  HAL_DMA_DeInit(&hdma_adc); 
    
  /*##-4- Disable the NVIC for DMA ###########################################*/
//  HAL_NVIC_DisableIRQ(ADCx_DMA_IRQn);
}


/**
* @brief  HAL_HRTIM_MspInit
* @param  None
* @retval None
*/
void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef * hhrtim)
{
  static GPIO_InitTypeDef GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/  
  
  /* Configure HRTIM outputs */  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;;  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;;  
  GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
  
  /* Configure HRTIM output: PWM_FB_HS1 */
  GPIO_InitStruct.Pin = PWM_FB_HS1_GPIO_PIN;  
  HAL_GPIO_Init(PWM_FB_HS1_GPIO_PORT, &GPIO_InitStruct);

  /* Configure HRTIM output: PWM_FB_LS1 */
  GPIO_InitStruct.Pin = PWM_FB_LS1_GPIO_PIN;
  HAL_GPIO_Init(PWM_FB_LS1_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure HRTIM output: PWM_FB_HS2 */
  GPIO_InitStruct.Pin = PWM_FB_HS2_GPIO_PIN;
  HAL_GPIO_Init(PWM_FB_HS2_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure HRTIM output: PWM_FB_LS2 */
  GPIO_InitStruct.Pin = PWM_FB_LS2_GPIO_PIN;
  HAL_GPIO_Init(PWM_FB_LS2_GPIO_PORT, &GPIO_InitStruct);
  
#ifdef SYNCH_RECTIFICATION
  /* Configure HRTIM output: PWM_SR_HS1 */
  GPIO_InitStruct.Pin = PWM_SR_HS1_GPIO_PIN;
  HAL_GPIO_Init(PWM_SR_HS1_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure HRTIM output: PWM_SR_LS1 */
  GPIO_InitStruct.Pin = PWM_SR_LS1_GPIO_PIN;
  HAL_GPIO_Init(PWM_SR_LS1_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure HRTIM output: PWM_SR_HS2 */
  GPIO_InitStruct.Pin = PWM_SR_HS2_GPIO_PIN;
  HAL_GPIO_Init(PWM_SR_HS2_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure HRTIM output: PWM_SR_LS2 */
  GPIO_InitStruct.Pin = PWM_SR_LS2_GPIO_PIN;
  HAL_GPIO_Init(PWM_SR_LS2_GPIO_PORT, &GPIO_InitStruct);
#endif
  
#ifdef OVERCURRENT_PROTECTION
  /* Configure HRTIM output: SCOUT - Not used */
//  GPIO_InitStruct.Alternate = GPIO_AF12_HRTIM1;
//  GPIO_InitStruct.Pin = HRTIM_SCOUT_GPIO_PIN;
//  HAL_GPIO_Init(HRTIM_SCOUT_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure Global Fault interrupt channel in NVIC */
  HAL_NVIC_SetPriority(HRTIM1_FLT_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(HRTIM1_FLT_IRQn);
#endif
  /*##-3- Configure the NVIC #################################################*/
  /* Configure and enable HRTIM TIMERC interrupt channel in NVIC */
//  HAL_NVIC_SetPriority(HRTIM1_TIMC_IRQn, 0, 1);
//  HAL_NVIC_EnableIRQ(HRTIM1_TIMC_IRQn);
//  __HAL_HRTIM_TIMER_ENABLE_IT(hhrtim, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_IT_CMP4);
   
  /* Configure and enable HRTIM Master interrupt channel in NVIC */
//  HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, 0, 1);
//  HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
}


/**
  * @brief COMP MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for COMP interrupt request enable
  * @param hcomp: COMP handle pointer
  * @retval None
  */
void HAL_COMP_MspInit(COMP_HandleTypeDef* hcomp)
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  
#ifdef OVERCURRENT_PROTECTION 
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  COMP_OC_GPIO_CLK_ENABLE();
  /* COMP Periph clock enable */
  COMP_OC_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* COMP GPIO pin configuration */
  GPIO_InitStruct.Pin = COMP_OC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;  
  HAL_GPIO_Init(COMP_OC_GPIO_PORT, &GPIO_InitStruct);
  
  #ifdef DEBUG_COMP_OC_OUT
    /* Debug COMP2 Out */
    GPIO_InitStruct.Pin = COMP_OC_OUT_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = COMP_OC_OUT_AF;
    HAL_GPIO_Init(COMP_OC_OUT_GPIO_PORT, &GPIO_InitStruct);
  #endif
#endif
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION 
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  COMP_VDS_SR1_GPIO_CLK_ENABLE();
  /* COMP Periph clock enable */
  COMP_VDS_SR1_CLK_ENABLE();
  COMP_VDS_SR2_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* COMP_VDS_SR1 GPIO pin configuration */
  GPIO_InitStruct.Pin = COMP_VDS_SR1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;  
  HAL_GPIO_Init(COMP_VDS_SR1_GPIO_PORT, &GPIO_InitStruct);
  
  /* COMP_VDS_SR2 GPIO pin configuration */
  GPIO_InitStruct.Pin = COMP_VDS_SR2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;  
  HAL_GPIO_Init(COMP_VDS_SR2_GPIO_PORT, &GPIO_InitStruct);
  
  // set config for channel 2
  
  #ifdef DEBUG_COMP_SR_OUT
    /* Debug COMP_VDS_SR1 Out */
    GPIO_InitStruct.Pin = COMP_VDS_SR1_OUT_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = COMP_VDS_SR1_OUT_AF;
    HAL_GPIO_Init(COMP_VDS_SR1_OUT_GPIO_PORT, &GPIO_InitStruct);
    
    /* Debug COMP_VDS_SR2 Out */
    GPIO_InitStruct.Pin = COMP_VDS_SR2_OUT_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = COMP_VDS_SR2_OUT_AF;
    HAL_GPIO_Init(COMP_VDS_SR2_OUT_GPIO_PORT, &GPIO_InitStruct);
  #endif
    
    // set config for output of channel 2
    
  /*##-3- Configure the NVIC for COMP_OC ########################################*/
  /* NVIC for COMP_OC */
//  HAL_NVIC_SetPriority(COMP_OC_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(COMP_OC_IRQn);
#endif
  
}

/**
  * @brief  DeInitializes the COMP MSP.
  * @param  hcomp: pointer to a COMP_HandleTypeDef structure that contains
  *         the configuration information for the specified COMP.  
  * @retval None
  */
void HAL_COMP_MspDeInit(COMP_HandleTypeDef* hcomp)
{
  /*##-1- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the COMPx GPIO pin */
  HAL_GPIO_DeInit(COMP_OC_GPIO_PORT, COMP_OC_PIN);
                
  /*##-2- Disable the NVIC for COMP ###########################################*/
  HAL_NVIC_DisableIRQ(COMP_OC_IRQn);
}

/**
  * @brief DAC MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hdac: DAC handle pointer
  * @retval None
  */
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
#ifdef OVERCURRENT_PROTECTION 
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  DAC_OC_CHANNEL_GPIO_CLK_ENABLE();
  /* DAC Periph clock enable */
  DAC_OC_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin = DAC_OC_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC_OC_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
#endif 
  
#ifdef ADAPTIVE_SYNCH_RECTIFICATION
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  DAC_SR_CHANNEL_GPIO_CLK_ENABLE();
  /* DAC Periph clock enable */
  DAC_SR_CLK_ENABLE();
  
  /*##-2A- Configure peripheral GPIO ##########################################*/  
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin = DAC_SR_CHANNEL1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC_SR_CHANNEL1_GPIO_PORT, &GPIO_InitStruct);
  
  /*##-2B- Configure peripheral GPIO ##########################################*/  
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin = DAC_SR_CHANNEL2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC_SR_CHANNEL2_GPIO_PORT, &GPIO_InitStruct);
#endif
  
}

/**
  * @brief  DeInitializes the DAC MSP.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.  
  * @retval None
  */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  /* Enable DAC reset state */
  DAC_OC_FORCE_RESET();
  
  /* Release DAC from reset state */
  DAC_OC_RELEASE_RESET();
}

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  #ifdef UI_COMMUNICATION
    UI_UART_MspInit(huart);
  #endif
  
  /************* DSMPS B2B COMMUNICATION **************************/
//  static DMA_HandleTypeDef hdma_tx;
//  static DMA_HandleTypeDef hdma_rx;
//  
//  GPIO_InitTypeDef  GPIO_InitStruct;
//  
//  /*##-1- Enable peripherals and GPIO Clocks #################################*/
//  /* Enable GPIO clock */
//  USARTx_TX_GPIO_CLK_ENABLE();
//  USARTx_RX_GPIO_CLK_ENABLE();
//  
//  /* Enable USARTx clock */
//  USARTx_CLK_ENABLE(); 
//  /* Enable DMAx clock */
//  DMAx_CLK_ENABLE();   
//  
//  /*##-2- Configure peripheral GPIO ##########################################*/  
//  /* UART TX GPIO pin configuration  */
//  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
//  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull      = GPIO_PULLUP;
//  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
//  GPIO_InitStruct.Alternate = USARTx_TX_AF;
//    
//  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
//    
//  /* UART RX GPIO pin configuration  */
//  GPIO_InitStruct.Pin = USARTx_RX_PIN;
//  GPIO_InitStruct.Alternate = USARTx_RX_AF;
//    
//  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
//    
//  /*##-3- Configure the DMA streams ##########################################*/
//  /* Configure the DMA handler for Transmission process */
//  hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
//  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
//  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
//  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
//  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
//  hdma_tx.Init.Mode                = DMA_NORMAL; 
//  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
//  
//  HAL_DMA_Init(&hdma_tx);   
//  
//  /* Associate the initialized DMA handle to the UART handle */
//  __HAL_LINKDMA(huart, hdmatx, hdma_tx);
//    
//  /* Configure the DMA handler for reception process */
//  hdma_rx.Instance = USARTx_RX_DMA_CHANNEL; 
//  hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//  hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//  hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
//  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//  hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//  hdma_rx.Init.Mode = DMA_CIRCULAR;
//  hdma_rx.Init.Priority = DMA_PRIORITY_LOW; //DMA_PRIORITY_VERY_HIGH;
//
//  HAL_DMA_Init(&hdma_rx);
//    
//  /* Associate the initialized DMA handle to the the UART handle */
//  __HAL_LINKDMA(huart, hdmarx, hdma_rx);
//    
//  /*##-4- Configure the NVIC for DMA #########################################*/   
//  /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
//  HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 8, 0);
//  HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
//    
//  /* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
//  HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 9, 0);   
//  HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
//  
//  /* NVIC for USARTx */
////  HAL_NVIC_SetPriority(USARTx_IRQn, 8, 1);
////  HAL_NVIC_EnableIRQ(USARTx_IRQn);  
  /**********************************************************************/
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  #ifdef UI_COMMUNICATION
    UI_UART_MspDeInit(huart);
  #endif
    
/************* DSMPS B2B COMMUNICATION **************************/    
//  static DMA_HandleTypeDef hdma_tx;
//  static DMA_HandleTypeDef hdma_rx;
//
//  /*##-1- Reset peripherals ##################################################*/
//  USARTx_FORCE_RESET();
//  USARTx_RELEASE_RESET();
//
//  /*##-2- Disable peripherals and GPIO Clocks #################################*/
//  /* Configure UART Tx as alternate function  */
//  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
//  /* Configure UART Rx as alternate function  */
//  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
//   
//  /*##-3- Disable the DMA Streams ############################################*/
//  /* De-Initialize the DMA Stream associate to transmission process */
//  HAL_DMA_DeInit(&hdma_tx); 
//  /* De-Initialize the DMA Stream associate to reception process */
//  HAL_DMA_DeInit(&hdma_rx);
//  
//  /*##-4- Disable the NVIC for DMA ###########################################*/
//  HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn);
//  HAL_NVIC_DisableIRQ(USARTx_DMA_RX_IRQn);
//  
//  /*##-5- Disable the NVIC for UART ##########################################*/
//  HAL_NVIC_DisableIRQ(USARTx_IRQn);
    /**********************************************************************/
}

/**
  * @brief  Initializes the TIM PWM MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{    
  
#ifdef FAN_PWM_DRIVING
  GPIO_InitTypeDef      GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIM6 Periph clock enable */
//  __TIM16_CLK_ENABLE();
  /* FAN PWM TIM Periph clock enable */
  FAN_PWM_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/ 
  /* TIM GPIO pin configuration - Debug output blanking window */
  GPIO_InitStruct.Pin = FAN_PWM_GPIO_PIN; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL; //GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = FAN_PWM_TIM_AF; 
  HAL_GPIO_Init(FAN_PWM_GPIO_PORT, &GPIO_InitStruct); 

//  HAL_NVIC_SetPriority(TIM6_DAC1_IRQn, 7, 0);
//  HAL_NVIC_EnableIRQ(TIM6_DAC1_IRQn);
  
//  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 7, 0);
//  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);  
#endif

}

/**
  * @brief  DeInitializes TIM PWM MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Disable peripherals and GPIO Clocks #################################*/
  /* FAN PWM TIM Periph clock enable */
  FAN_PWM_CLK_DISABLE();  
}

/**
  * @brief  Initializes the TIM Base MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /* TIM6 Periph clock enable */
  __TIM6_CLK_ENABLE();
  
  /* TIM16 Periph clock enable */
  __TIM16_CLK_ENABLE();
   
  /* Configure and enable TIM6 interrupt channel in NVIC */
  HAL_NVIC_SetPriority(TIM6_DAC1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC1_IRQn);
  
  HAL_NVIC_SetPriority(TIM16_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(TIM16_IRQn);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
