#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f10x.h"

#define DHT11_OK      0
#define DHT11_ERROR   1

void DHT11_Init(void);
uint8_t DHT11_ReadData(uint8_t *temperature, uint8_t *humidity);

#endif
