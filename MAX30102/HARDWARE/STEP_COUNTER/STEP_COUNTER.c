#include "STEP_COUNTER.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS      0xD0

/*
 * Step detection:
 * 1. Use |ax| + |ay| + |az| as an orientation-independent motion signal.
 * 2. Track gravity/pose with a slow baseline filter.
 * 3. Smooth the high-pass motion signal.
 * 4. Use an adaptive threshold and count one step only after a peak falls back.
 *
 * This is better for a wrist band than gyro angle integration, because normal
 * arm rotation should not immediately become a step.
 */
#define STEP_THRESHOLD_MIN        700
#define STEP_THRESHOLD_MAX       4500
#define STEP_PEAK_MAX           16000
#define STEP_MIN_INTERVAL          12
#define STEP_MAX_CANDIDATE         45
#define STEP_STARTUP_SAMPLES       25

#define TIM2_PRESCALER       7199
#define TIM2_PERIOD           199

static volatile uint32_t g_step_count;
static int32_t g_baseline;
static int32_t g_motion_avg;
static int32_t g_smooth;
static int32_t g_peak;
static uint16_t g_tick;
static uint16_t g_last_step_tick;
static uint8_t g_candidate_ticks;
static uint8_t g_in_candidate;
static uint8_t g_startup_count;

static volatile int16_t g_ax, g_ay, g_az;
static volatile int16_t g_gx, g_gy, g_gz;

static int32_t Abs32(int32_t value)
{
	return value < 0 ? -value : value;
}

static int32_t Clamp32(int32_t value, int32_t min_value, int32_t max_value)
{
	if (value < min_value) return min_value;
	if (value > max_value) return max_value;
	return value;
}

static void I2C_WaitEvent(I2C_TypeDef *I2Cx, uint32_t event)
{
	uint32_t timeout = 10000;
	while (I2C_CheckEvent(I2Cx, event) != SUCCESS)
	{
		if (--timeout == 0) break;
	}
}

static void I2C_WriteReg(uint8_t reg, uint8_t data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

	I2C_SendData(I2C2, reg);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);

	I2C_SendData(I2C2, data);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTOP(I2C2, ENABLE);
}

static uint8_t I2C_ReadReg(uint8_t reg)
{
	uint8_t data;

	I2C_GenerateSTART(I2C2, ENABLE);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

	I2C_SendData(I2C2, reg);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTART(I2C2, ENABLE);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);

	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	data = I2C_ReceiveData(I2C2);

	I2C_AcknowledgeConfig(I2C2, ENABLE);

	return data;
}

static void I2C_ReadBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t i;

	I2C_AcknowledgeConfig(I2C2, ENABLE);

	I2C_GenerateSTART(I2C2, ENABLE);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

	I2C_SendData(I2C2, reg);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTART(I2C2, ENABLE);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
	I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

	for (i = 0; i < len; i++)
	{
		if (i == (uint8_t)(len - 1))
		{
			I2C_AcknowledgeConfig(I2C2, DISABLE);
			I2C_GenerateSTOP(I2C2, ENABLE);
		}
		I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
		buf[i] = I2C_ReceiveData(I2C2);
	}

	I2C_AcknowledgeConfig(I2C2, ENABLE);
}

static void MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
							  int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t data[14];

	I2C_ReadBytes(MPU6050_ACCEL_XOUT_H, data, 14);

	*ax = (int16_t)((data[0] << 8) | data[1]);
	*ay = (int16_t)((data[2] << 8) | data[3]);
	*az = (int16_t)((data[4] << 8) | data[5]);
	*gx = (int16_t)((data[8] << 8) | data[9]);
	*gy = (int16_t)((data[10] << 8) | data[11]);
	*gz = (int16_t)((data[12] << 8) | data[13]);
}

void TIM2_IRQHandler(void)
{
	int16_t ax, ay, az, gx, gy, gz;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz);

		g_ax = ax; g_ay = ay; g_az = az;
		g_gx = gx; g_gy = gy; g_gz = gz;

		StepCounter_Update(ax, ay, az, gx, gy, gz);
	}
}

void StepCounter_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	g_step_count = 0;
	g_baseline = 0;
	g_motion_avg = STEP_THRESHOLD_MIN;
	g_smooth = 0;
	g_peak = 0;
	g_tick = 0;
	g_last_step_tick = 0;
	g_candidate_ticks = 0;
	g_in_candidate = 0;
	g_startup_count = 0;
	g_ax = g_ay = g_az = 0;
	g_gx = g_gy = g_gz = 0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2 | RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 50000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);

	I2C_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	I2C_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	I2C_WriteReg(MPU6050_SMPLRT_DIV,  0x09);
	I2C_WriteReg(MPU6050_CONFIG,      0x06);
	I2C_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
	I2C_WriteReg(MPU6050_ACCEL_CONFIG,0x18);

	TIM_TimeBaseInitStructure.TIM_Period        = TIM2_PERIOD;
	TIM_TimeBaseInitStructure.TIM_Prescaler     = TIM2_PRESCALER;
	TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM2, ENABLE);
}

void StepCounter_Update(int16_t ax, int16_t ay, int16_t az,
						int16_t gx, int16_t gy, int16_t gz)
{
	int32_t mag;
	int32_t motion;
	int32_t threshold;
	uint16_t interval;

	(void)gx; (void)gy; (void)gz;

	g_tick++;

	mag = Abs32(ax) + Abs32(ay) + Abs32(az);
	if (g_baseline == 0)
	{
		g_baseline = mag;
		return;
	}

	g_baseline = (g_baseline * 31 + mag) / 32;
	motion = mag - g_baseline;
	g_smooth = (g_smooth * 3 + motion) / 4;
	g_motion_avg = (g_motion_avg * 31 + Abs32(g_smooth)) / 32;

	if (g_startup_count < STEP_STARTUP_SAMPLES)
	{
		g_startup_count++;
		return;
	}

	threshold = Clamp32(g_motion_avg * 2, STEP_THRESHOLD_MIN, STEP_THRESHOLD_MAX);

	if (!g_in_candidate)
	{
		if (g_smooth > threshold)
		{
			g_in_candidate = 1;
			g_candidate_ticks = 0;
			g_peak = g_smooth;
		}
		return;
	}

	g_candidate_ticks++;
	if (g_smooth > g_peak)
	{
		g_peak = g_smooth;
	}

	if (g_candidate_ticks > STEP_MAX_CANDIDATE)
	{
		g_in_candidate = 0;
		return;
	}

	if (g_smooth < (threshold / 2))
	{
		interval = (uint16_t)(g_tick - g_last_step_tick);
		if (g_peak < STEP_PEAK_MAX &&
			(g_last_step_tick == 0 || interval >= STEP_MIN_INTERVAL))
		{
			g_step_count++;
			g_last_step_tick = g_tick;
		}
		g_in_candidate = 0;
	}
}

uint32_t StepCounter_GetCount(void)
{
	uint32_t count;

	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
	count = g_step_count;
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	return count;
}

void StepCounter_Reset(void)
{
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
	g_step_count = 0;
	g_last_step_tick = 0;
	g_in_candidate = 0;
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void StepCounter_GetSensorData(int16_t *ax, int16_t *ay, int16_t *az,
								int16_t *gx, int16_t *gy, int16_t *gz)
{
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
	*ax = g_ax; *ay = g_ay; *az = g_az;
	*gx = g_gx; *gy = g_gy; *gz = g_gz;
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}
