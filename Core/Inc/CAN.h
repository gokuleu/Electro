/*
 * CAN.h
 *
 *  Created on: Jun 7, 2025
 *      Author: Srivatsen-TRN0149
 */

#include "main.h"
//#include "stm32f3xx_hal_can.h"
#include "stm32f3xx_hal.h"

#ifndef INC_CAN_H_
#define INC_CAN_H_

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern CAN_HandleTypeDef hcan;

void Transmit_On_CAN(uint32_t id, uint8_t data[8]);
void Send_on_CAN();

#endif /* INC_CAN_H_ */

