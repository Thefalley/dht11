/*
 * adc.h
 *
 *  Created on: Jan 5, 2024
 *      Author: pabme
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"



uint32_t readAnalogA0(void);
uint32_t getWaterLevel (uint32_t val_agua);


#endif /* INC_ADC_H_ */
