#ifndef __DS1302_H
#define __DS1302_H

#include "stm32f10x.h"

typedef struct
{
	uint8_t year;
	uint8_t month;
	uint8_t date;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t week;
} DS1302_TimeTypeDef;

void DS1302_Init(void);
void DS1302_SetDateTime(const DS1302_TimeTypeDef *time);
void DS1302_GetDateTime(DS1302_TimeTypeDef *time);
uint8_t DS1302_IsClockHalted(void);
void DS1302_StartClock(void);

#endif
