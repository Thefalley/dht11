/*
 * adc.c
 *
 *  Created on: Jan 5, 2024
 *      Author: pabme
 */

#include "adc.h"

#define MAX_AGUA 800 // cero de mas
#define MIN_AGUA 400// y = ax - b
#define M 4
#define B 200 // Calculado papel (0 = (100/460)*400 + b)


extern ADC_HandleTypeDef hadc1;

uint32_t readAnalogA0(void){
	uint32_t agua = 0;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 20);
	agua = HAL_ADC_GetValue(&hadc1);

	return agua;
}

uint32_t getWaterLevel (uint32_t val_agua){
	// 100 agua max
	// 0 agua min
	uint32_t resul;

	// Foto geogebra
	resul = -0.25*val_agua +B;

	// Limitado a dos digitos
	if (resul == 100){
		resul = 99;
	}

	return resul ;
}

uint32_t leer_Analog_1(void){
	uint32_t gas = 0;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 20);
	gas = HAL_ADC_GetValue(&hadc1);

	return gas;
}



