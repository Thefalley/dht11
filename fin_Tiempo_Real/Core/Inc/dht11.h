/*
 * dht11.h
 *
 *  Created on: Jan 5, 2024
 *      Author: pabme
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "stm32f4xx_hal.h"

typedef struct {
  GPIO_TypeDef* GPIO_Port;
  uint16_t GPIO_Pin;
} DHT11_TypeDef;

void DHT11_Init(DHT11_TypeDef* dht11);
HAL_StatusTypeDef DHT11_ReadData(DHT11_TypeDef* dht11, uint8_t* humidity, uint8_t* temperature);


#endif /* INC_DHT11_H_ */
