#ifndef __STEP_COUNTER_H
#define __STEP_COUNTER_H

#include "stm32f10x.h"

void StepCounter_Init(void);
void StepCounter_Update(int16_t ax, int16_t ay, int16_t az,
						int16_t gx, int16_t gy, int16_t gz);
uint32_t StepCounter_GetCount(void);
void StepCounter_Reset(void);
void StepCounter_GetSensorData(int16_t *ax, int16_t *ay, int16_t *az,
								int16_t *gx, int16_t *gy, int16_t *gz);

#endif
