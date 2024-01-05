/*
 * dht11.c
 *
 *  Created on: Jan 5, 2024
 *      Author: pabme
 */
#include "cmsis_os.h"
#include <stdio.h>
#include "dht11.h"

extern TIM_HandleTypeDef htim11;
extern osEventFlagsId_t flag_Event;

void DHT11_Delay(uint32_t uSeg) {
	htim11.Init.Period = 84 - 1; // no cabiar (1us con el prescaler de 1)
	// Maximo valor de 65536
	htim11.Init.Prescaler = uSeg - 1  ;

	osEventFlagsClear(flag_Event, 0x01); // Iniciar valor de flag

	HAL_TIM_Base_Start_IT(&htim11); // iniciar timer

	osEventFlagsWait(flag_Event, 0x01, osFlagsWaitAll, osWaitForever);

	// Detener el temporizador
	HAL_TIM_Base_Stop_IT(&htim11);

	// Limpiar el flag de evento
	osEventFlagsClear(flag_Event, 0x01);
}


void DHT11_Init(DHT11_TypeDef* dht11) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //__HAL_RCC_GPIOx_CLK_ENABLE(); // Reemplaza 'x' con el puerto GPIO que estás utilizando

  GPIO_InitStruct.Pin = dht11->GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(dht11->GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(dht11->GPIO_Port, dht11->GPIO_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef DHT11_ReadData(DHT11_TypeDef* dht11, uint8_t* humidity, uint8_t* temperature) {

  // Configurar modo salida
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = dht11->GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  // Iniciar la comunicación con el sensor DHT11
  HAL_GPIO_WritePin(dht11->GPIO_Port, dht11->GPIO_Pin, GPIO_PIN_RESET);
  DHT11_Delay(18005); // Mantén bajo durante al menos 18 ms

  // Cambiar a la fase de lectura
  HAL_GPIO_WritePin(dht11->GPIO_Port, dht11->GPIO_Pin, GPIO_PIN_SET);
  DHT11_Delay(1); // Espera antes de leer la respuesta del sensor

  // Configurar el pin como entrada
  GPIO_InitStruct.Pin = dht11->GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(dht11->GPIO_Port, &GPIO_InitStruct);

  // Leer la respuesta del sensor (40 bits de datos)
  uint8_t data[5] = {0};
  for (int i = 0; i < 40; ++i) {
    DHT11_Delay(50); // Espera para cada bit

    if (HAL_GPIO_ReadPin(dht11->GPIO_Port, dht11->GPIO_Pin) == GPIO_PIN_RESET) {
      // Bit bajo detectado, registra el tiempo de baja
      DHT11_Delay(29);

      if (HAL_GPIO_ReadPin(dht11->GPIO_Port, dht11->GPIO_Pin) == GPIO_PIN_RESET) {
        // Bit '0' detectado
        data[i / 8] <<= 1; // Desplaza a la izquierda
      } else {
        // Bit '1' detectado
        data[i / 8] <<= 1;
        data[i / 8] |= 1; // Establece el bit menos significativo a 1
        DHT11_Delay(41); // Compensa los 70 usegundos
      }
    }
  }

  // Verificar la suma de comprobación
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    *humidity = data[0];
    *temperature = data[2];

    return HAL_OK;
  } else {
    return HAL_ERROR;
  }
}

void formatSensorData(uint8_t* buffer, uint8_t hum, uint8_t tem) {
	uint8_t label_hum[3] = "HUM";
	uint8_t label_tem[3] = "TEM";
    // Utiliza sprintf para formatear el valor en la cadena
	 sprintf((char*)buffer, "%s = %02u%%  %s = %02u%%\n\r", label_hum, hum, label_tem, tem);
}
