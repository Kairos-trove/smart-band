/*************************************************************************************
Wiring

MAX30102:
	VCC -> 3.3V, GND -> GND, SCL -> PB7, SDA -> PB8, INT -> PB9
OLED:
	VCC -> 3.3V, GND -> GND, SCK -> PA5, SDA -> PA6
MPU6050:
	VCC -> 3.3V, GND -> GND, SCL -> PB10, SDA -> PB11
DS18B20 body temperature:
	VCC -> 3.3V, GND -> GND, DQ -> PA8
DHT11 environment temperature/humidity:
	VCC -> 3.3V, GND -> GND, DATA -> PA7
DS1302 RTC:
	VCC -> 3.3V, GND -> GND, CE/RST -> PB12, SCLK -> PB13, IO -> PB14
Buzzer:
	VCC -> 3.3V, GND -> GND, I/O -> PA12
Page key:
	KEY -> PA0, other side -> GND
**************************************************************************************/
#include "delay.h"
#include "sys.h"
#include "max30102.h"
#include "algorithm.h"
#include "OLED.h"
#include "ds18b20.h"
#include "dht11.h"
#include "ds1302.h"
#include "Buzzer.h"
#include "STEP_COUNTER.h"

#define SAMPLE_COUNT          500
#define SAMPLE_STEP           100

#define HR_LOW_ALARM          50
#define HR_HIGH_ALARM         120
#define SPO2_LOW_ALARM        94
#define BODY_TEMP_LOW_ALARM   350
#define BODY_TEMP_HIGH_ALARM  380

#define FALL_ACC_THRESHOLD    5500
#define UNSAFE_GYRO_DELTA     12000

#define KEY_GPIO              GPIOA
#define KEY_PIN               GPIO_Pin_0
#define KEY_RCC               RCC_APB2Periph_GPIOA

static uint32_t aun_ir_buffer[SAMPLE_COUNT];
static uint32_t aun_red_buffer[SAMPLE_COUNT];
static int32_t n_ir_buffer_length = SAMPLE_COUNT;
static int32_t n_spo2;
static int8_t ch_spo2_valid;
static int32_t n_heart_rate;
static int8_t ch_hr_valid;

static int16_t AX, AY, AZ, GX, GY, GZ;
static uint8_t TempBytes[6];
static uint8_t DisplayPage = 0;

static void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(KEY_RCC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = KEY_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(KEY_GPIO, &GPIO_InitStructure);
}

static uint8_t Key_Scan(void)
{
	if (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == Bit_RESET)
	{
		delay_ms(20);
		if (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == Bit_RESET)
		{
			while (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == Bit_RESET);
			return 1;
		}
	}
	return 0;
}

static int32_t Abs32(int32_t value)
{
	return value < 0 ? -value : value;
}

static void ReadMax30102Sample(uint16_t index)
{
	while (MAX30102_INT == 1);
	max30102_FIFO_ReadBytes(REG_FIFO_DATA, TempBytes);
	aun_red_buffer[index] = ((uint32_t)(TempBytes[0] & 0x03) << 16) |
							((uint32_t)TempBytes[1] << 8) |
							TempBytes[2];
	aun_ir_buffer[index] = ((uint32_t)(TempBytes[3] & 0x03) << 16) |
						   ((uint32_t)TempBytes[4] << 8) |
						   TempBytes[5];
}

static void InitMax30102Samples(void)
{
	uint16_t i;

	for (i = 0; i < SAMPLE_COUNT; i++)
	{
		ReadMax30102Sample(i);
	}

	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length,
		aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
}

static void UpdateMax30102Samples(uint8_t *heart_rate, uint8_t *spo2)
{
	uint16_t i;

	for (i = SAMPLE_STEP; i < SAMPLE_COUNT; i++)
	{
		aun_red_buffer[i - SAMPLE_STEP] = aun_red_buffer[i];
		aun_ir_buffer[i - SAMPLE_STEP] = aun_ir_buffer[i];
	}

	for (i = SAMPLE_COUNT - SAMPLE_STEP; i < SAMPLE_COUNT; i++)
	{
		ReadMax30102Sample(i);
	}

	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length,
		aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

	if (ch_hr_valid && ch_spo2_valid &&
		n_heart_rate >= 30 && n_heart_rate <= 220 &&
		n_spo2 >= 70 && n_spo2 <= 100)
	{
		*heart_rate = (uint8_t)n_heart_rate;
		*spo2 = (uint8_t)n_spo2;
	}
	else
	{
		*heart_rate = 0;
		*spo2 = 0;
	}
}

static uint16_t ReadBodyTempX10(void)
{
	short raw;

	raw = get_tempetature();
	if (raw < 0)
	{
		return 0;
	}

	return (uint16_t)((raw * 10) / 16);
}

static uint8_t IsFallDetected(void)
{
	int32_t mag = Abs32(AX) + Abs32(AY) + Abs32(AZ);

	if (mag > FALL_ACC_THRESHOLD)
	{
		return 1;
	}
	return 0;
}

static uint8_t IsUnsafeMotion(void)
{
	if (Abs32(GX) > UNSAFE_GYRO_DELTA ||
		Abs32(GY) > UNSAFE_GYRO_DELTA ||
		Abs32(GZ) > UNSAFE_GYRO_DELTA)
	{
		return 1;
	}
	return 0;
}

static char *GetPostureText(void)
{
	if (Abs32(AZ) > Abs32(AX) && Abs32(AZ) > Abs32(AY))
	{
		return "UP";
	}
	if (Abs32(AX) > Abs32(AY))
	{
		return AX > 0 ? "SIDE-R" : "SIDE-L";
	}
	return AY > 0 ? "FACE-UP" : "FACE-DN";
}

static uint8_t IsRtcTimeValid(const DS1302_TimeTypeDef *time)
{
	if (time->month < 1 || time->month > 12) return 0;
	if (time->date < 1 || time->date > 31) return 0;
	if (time->hour > 23) return 0;
	if (time->minute > 59) return 0;
	if (time->second > 59) return 0;
	if (time->week < 1 || time->week > 7) return 0;
	return 1;
}

static void DisplayData(const DS1302_TimeTypeDef *time,
						uint8_t env_temp,
						uint8_t env_humi,
						uint16_t body_temp_x10,
						uint8_t heart_rate,
						uint8_t spo2,
						char *posture,
						uint8_t fall,
						uint8_t unsafe,
						uint8_t alarm,
						uint32_t steps)
{
	OLED_Clear();

	OLED_Printf(0, 0, OLED_8X16, "20%02d-%02d-%02d %02d:%02d",
		time->year, time->month, time->date, time->hour, time->minute);

	if (DisplayPage == 0)
	{
		OLED_Printf(0, 16, OLED_8X16, "ENV: %02d C", env_temp);
		OLED_Printf(0, 32, OLED_8X16, "HUM: %02d %%", env_humi);
		OLED_Printf(0, 48, OLED_8X16, "BODY:%02d.%01d C", body_temp_x10 / 10, body_temp_x10 % 10);
	}
	else if (DisplayPage == 1)
	{
		if (heart_rate)
			OLED_Printf(0, 16, OLED_8X16, "HR: %03d BPM", heart_rate);
		else
			OLED_Printf(0, 16, OLED_8X16, "HR: -- BPM");

		if (spo2)
			OLED_Printf(0, 32, OLED_8X16, "SpO2: %02d %%", spo2);
		else
			OLED_Printf(0, 32, OLED_8X16, "SpO2: -- %%");

		OLED_Printf(0, 48, OLED_8X16, "STATUS: %s",
			alarm ? (unsafe ? "RISK" : "ERR") : "OK");
	}
	else
	{
		OLED_Printf(0, 16, OLED_8X16, "STEP: %06lu", (unsigned long)steps);
		OLED_Printf(0, 32, OLED_8X16, "POSE: %s", posture);
		OLED_Printf(0, 48, OLED_8X16, "FALL: %s", fall ? "YES" : "NO");
	}

	OLED_Update();
}

int main(void)
{
	uint8_t heart_rate = 0;
	uint8_t spo2 = 0;
	uint8_t env_temp = 0;
	uint8_t env_humi = 0;
	uint8_t fall = 0;
	uint8_t unsafe = 0;
	uint8_t alarm = 0;
	uint8_t display_warning = 0;
	uint16_t body_temp_x10 = 0;
	DS1302_TimeTypeDef now;
	DS1302_TimeTypeDef rtc_read;
	DS1302_TimeTypeDef default_time = {26, 6, 9, 22, 03, 0, 7};

	NVIC_Configuration();
	delay_init();

	OLED_Init();
	OLED_ShowString(0, 0, "Smart band init", OLED_8X16);
	OLED_ShowString(0, 16, "Please wait...", OLED_8X16);
	OLED_Update();

	max30102_init();
	ds18b20_init();
	DHT11_Init();
	DS1302_Init();
	delay_ms(10);
	Buzzer_init();
	Key_Init();
	StepCounter_Init();

	DS1302_GetDateTime(&now);
	if (DS1302_IsClockHalted() || !IsRtcTimeValid(&now))
	{
		DS1302_SetDateTime(&default_time);
		DS1302_GetDateTime(&now);
	}
	else
	{
		DS1302_StartClock();
	}

	StepCounter_GetSensorData(&AX, &AY, &AZ, &GX, &GY, &GZ);

	InitMax30102Samples();

	while (1)
	{
		UpdateMax30102Samples(&heart_rate, &spo2);

		DS1302_GetDateTime(&rtc_read);
		if (!IsRtcTimeValid(&rtc_read))
		{
			DS1302_GetDateTime(&rtc_read);
		}
		if (IsRtcTimeValid(&rtc_read))
		{
			now = rtc_read;
		}
		DHT11_ReadData(&env_temp, &env_humi);
		body_temp_x10 = ReadBodyTempX10();

		StepCounter_GetSensorData(&AX, &AY, &AZ, &GX, &GY, &GZ);

		fall = IsFallDetected();
		unsafe = IsUnsafeMotion();
		if (Key_Scan())
		{
			DisplayPage = (DisplayPage + 1) % 3;
		}

		alarm = fall || unsafe;

		display_warning = alarm ||
			(heart_rate != 0 && (heart_rate < HR_LOW_ALARM || heart_rate > HR_HIGH_ALARM)) ||
			(spo2 != 0 && spo2 < SPO2_LOW_ALARM);

		if (alarm)
		{
			Buzzer_on();
		}
		else
		{
			Buzzer_off();
		}

		DisplayData(&now, env_temp, env_humi, body_temp_x10, heart_rate, spo2,
			GetPostureText(), fall, unsafe, display_warning, StepCounter_GetCount());
	}
}

