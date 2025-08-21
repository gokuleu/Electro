#include <CAN.h>
#include "main.h"

#define NTC_PULL_UP_RESISTOR 10000
#define Adc_max_COUNT 4095
#define T_AMBIENT 25
#define KELVIN_TO_CELSIUS 273.15
#define BETA_VALUE 3984
#define TEMP_AVG 1
#define deadtime_timerperiod 1/64 
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
extern CAN_HandleTypeDef hcan;
extern uint16_t ADC_VAL_1[7];
extern float rms_adc;


uint8_t RxData[8];
uint8_t TxData[8];
// float temp_out;

// float moving_Temperature_measured_fun_M( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
// {
//     static float Prev_current_val;
//     float Bus_Current_Error_value;
//     Bus_Current_Error_value = (current_val - Prev_current_val);
//     Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
//     current_val = Prev_current_val ;
//     return current_val ;
// }

void Send_on_CAN(){
	  uint32_t time_count = HAL_GetTick();
	  static uint32_t Prev_time = 0;
//	  uint8_t tempxxxx;
	  uint16_t rms_input_voltage=0;
			  rms_input_voltage=rms_adc;
	  if(time_count - Prev_time > 100){
		  	Prev_time = time_count;
		  	// double resistance = (ADC_VAL_1[4] * NTC_PULL_UP_RESISTOR)/(Adc_max_COUNT - ADC_VAL_1[4]);
		  	//      double temp_K = resistance/NTC_PULL_UP_RESISTOR;
		  	//      temp_K = log(temp_K);
		  	//      temp_K /= BETA_VALUE;
		  	//      temp_K += 1.0/(T_AMBIENT + KELVIN_TO_CELSIUS);
		  	//      temp_K = 1.0/temp_K;
		  	//      temp_K -= KELVIN_TO_CELSIUS;
		  	//    temp_out = moving_Temperature_measured_fun_M(temp_K, TEMP_AVG);
		  	//  tempxxxx=temp_out;


		  	TxData[0] = ADC_VAL_1[0]&0xff;
		  	TxData[1] = (ADC_VAL_1[0]&0xff00)>>8;
		  	TxData[2] = ADC_VAL_1[1]&0xff;
		  	TxData[3] = (ADC_VAL_1[1]&0xff00)>>8;
		  	TxData[4] = ADC_VAL_1[7]&0xff;
		  	TxData[5] = (ADC_VAL_1[7]&0xff00)>>8;
//		  	TxData[6] = ADC_VAL_1[3]&0xff;
//		  	TxData[7] = tempxxxx;

		  	Transmit_On_CAN(0x18FF50E5, TxData);
			TxData[0] = (rms_input_voltage)&0xff;
			TxData[1] = ((rms_input_voltage)&0xff00)>>8;
			TxData[2] = ADC_VAL_1[1]&0xff;
			TxData[3] = (ADC_VAL_1[1]&0xff00)>>8;
			TxData[4] = ADC_VAL_1[7]&0xff;
			TxData[5] = (ADC_VAL_1[7]&0xff00)>>8;
			TxData[6] = ADC_VAL_1[3]&0xff;
//			TxData[7] = tempxxxx;
		  	Transmit_On_CAN(0x18FF50E6, TxData);
		  }
}

void Transmit_On_CAN(uint32_t id, uint8_t data[8]){
	CAN_TxHeaderTypeDef TxHeader = {
			.ExtId = id,
			.IDE = CAN_ID_EXT,
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
	if(RxData[2]==0x01){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}


	if(RxHeader.StdId == 0x4FF){
		float rx_data_1=RxData[0]; 
		float rx_data_2=RxData[1]; //percent
		// uint16_t data =((RxData[1]<<8))|((RxData[0]));
		uint16_t data=4096000000.0f/(rx_data_1*2000.0f);
		// float percent = RxData[1]*0.005;
		// float deadtime_1= (float)(((float)(1/(rx_data_1*2*RxData[1]*1000)))/(deadtime_timerperiod));  // x percent
		float deadtime_1= (float)((320.0f*rx_data_2)/(rx_data_1));  // x percent
		uint16_t deadtime=deadtime_1;
		// uint16_t deadtime = (float)(percent*data);
		HRTIM1->sMasterRegs.MPER = data;
		
		// Read-modify-write to preserve other bits
uint32_t temp = HRTIM1_TIMA->DTxR;

temp &= ~HRTIM_DTR_DTR_Msk;           // Clear rising bits
temp |= (deadtime & 0x1FF);            // Set new rising time

temp &= ~HRTIM_DTR_DTF_Msk;           // Clear falling bits
temp |= ((deadtime & 0x1FF) << 16);    // Set new falling time

HRTIM1_TIMA->DTxR = temp;

temp=0;
// Read-modify-write to preserve other bits
temp = HRTIM1_TIMB->DTxR;

temp &= ~HRTIM_DTR_DTR_Msk;           // Clear rising bits
temp |= (deadtime & 0x1FF);            // Set new rising time

temp &= ~HRTIM_DTR_DTF_Msk;           // Clear falling bits
temp |= ((deadtime & 0x1FF) << 16);    // Set new falling time

HRTIM1_TIMB->DTxR = temp;


	}

}

