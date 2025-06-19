#include <CAN.h>
#include "main.h"

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
extern CAN_HandleTypeDef hcan;
extern uint16_t ADC_VAL[7];

uint8_t RxData[8];
uint8_t TxData[8];

void Send_on_CAN(){
	  uint32_t time_count = HAL_GetTick();
	  static uint32_t Prev_time = 0;
	  if(time_count - Prev_time > 100){
		  	Prev_time = time_count;

		  	TxData[0] = ADC_VAL[0]&0xff;
		  	TxData[1] = (ADC_VAL[0]&0xff00)>>8;
		  	TxData[2] = ADC_VAL[1]&0xff;
		  	TxData[3] = (ADC_VAL[1]&0xff00)>>8;
		  	TxData[4] = ADC_VAL[2]&0xff;
		  	TxData[5] = (ADC_VAL[2]&0xff00)>>8;
		  	TxData[6] = ADC_VAL[3]&0xff;
		  	TxData[7] = (ADC_VAL[3]&0xff00)>>8;

		  	Transmit_On_CAN(0x123, TxData);
		  }
}

void Transmit_On_CAN(uint16_t id, uint8_t data[8]){
	CAN_TxHeaderTypeDef TxHeader = {
			.StdId = id,
			.IDE = CAN_ID_STD,
		    .RTR = CAN_RTR_DATA,
		    .DLC = 8	,
		    .TransmitGlobalTime = DISABLE
	};

	uint32_t txMailbox;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0){}
	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, (uint8_t*)data , &txMailbox) != HAL_OK){
		Error_Handler();
	}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	if (HAL_CAN_GetRxMessage (CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
		Error_Handler();
	}

	if(RxHeader.StdId == 0x001){

	}

}

