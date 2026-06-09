#ifndef __DS18B20_H
#define __DS18B20_H

#include "stm32f10x.h"

#define DS18B20_TEMP_ERROR  (-32768)

void ds18b20_init(void);
short get_tempetature(void);

#endif
