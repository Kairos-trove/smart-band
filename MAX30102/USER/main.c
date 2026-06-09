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
#define MAX30102_PART_ID      0x15
#define MAX30102_MIN_SIGNAL   10000
#define MAX30102_HOLD_LIMIT   6

#define HR_LOW_ALARM          50
#define HR_HIGH_ALARM         120
#define SPO2_LOW_ALARM        94
#define BODY_TEMP_LOW_ALARM   350
#define BODY_TEMP_HIGH_ALARM  380
#define BODY_TEMP_OFFSET_X10  30
#define BODY_TEMP_MIN_X10     200
#define BODY_TEMP_MAX_X10     450
#define BODY_TEMP_WARMUP_READS 3
#define BODY_TEMP_MAX_STEP_X10 2

#define FALL_ACC_DELTA        6000
#define UNSAFE_GYRO_DELTA     12000

#define KEY_GPIO              GPIOA
#define KEY_PIN               GPIO_Pin_0
#define KEY_RCC               RCC_APB2Periph_GPIOA

#define RTC_INIT_MARK0        0xB6
#define RTC_INIT_MARK1        0x6B

static uint32_t aun_ir_buffer[SAMPLE_COUNT];
static uint32_t aun_red_buffer[SAMPLE_COUNT];
static int32_t n_ir_buffer_length = SAMPLE_COUNT;
static int32_t n_spo2;
static int8_t ch_spo2_valid;
static int32_t n_heart_rate;
static int8_t ch_hr_valid;

static int16_t AX, AY, AZ, GX, GY, GZ;
static int16_t Last_AX, Last_AY, Last_AZ;
static uint8_t TempBytes[6];
static uint8_t DisplayPage = 0;
static uint8_t Max30102Ok = 1;
static uint8_t Max30102FingerOn = 0;
static uint8_t Max30102Hold = 0;
static uint8_t LastHeartRate = 0;
static uint8_t LastSpo2 = 0;
static int16_t BodyTempFilteredX10 = -1;
static uint8_t BodyTempValidReads = 0;

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
	static uint8_t key_ready = 1;

	if (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == Bit_RESET)
	{
		delay_ms(5);
		if (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == Bit_RESET)
		{
			if (key_ready)
			{
				key_ready = 0;
				return 1;
			}
		}
	}
	else
	{
		key_ready = 1;
	}
	return 0;
}

static void PageKey_Task(void)
{
	if (Key_Scan())
	{
		DisplayPage = !DisplayPage;
	}
}

static int32_t Abs32(int32_t value)
{
	return value < 0 ? -value : value;
}

static uint8_t ReadMax30102Sample(uint16_t index)
{
	uint32_t timeout = 15000;
	uint32_t red;
	uint32_t ir;
	uint8_t timed_out = 0;

	while (MAX30102_INT == 1)
	{
		PageKey_Task();
		if (timeout == 0)
		{
			timed_out = 1;
			break;
		}
		timeout--;
		delay_us(1);
	}
	max30102_FIFO_ReadBytes(REG_FIFO_DATA, TempBytes);
	red = ((uint32_t)(TempBytes[0] & 0x03) << 16) |
		  ((uint32_t)TempBytes[1] << 8) |
		  TempBytes[2];
	ir = ((uint32_t)(TempBytes[3] & 0x03) << 16) |
		 ((uint32_t)TempBytes[4] << 8) |
		 TempBytes[5];

	aun_red_buffer[index] = red;
	aun_ir_buffer[index] = ir;

	if (timed_out)
	{
		delay_ms(10);
	}

	if ((red < MAX30102_MIN_SIGNAL && ir < MAX30102_MIN_SIGNAL) ||
		(red == 0x3FFFF && ir == 0x3FFFF))
	{
		return 0;
	}
	return 1;
}

static uint8_t Max30102_Check(void)
{
	uint8_t part_id;

	part_id = max30102_Bus_Read(REG_PART_ID);
	if (part_id == MAX30102_PART_ID)
	{
		return 1;
	}
	return 0;
}

static void InitMax30102Samples(void)
{
	uint16_t i;
	uint16_t valid_count = 0;
	uint8_t fail_count = 0;

	if (!Max30102_Check())
	{
		Max30102Ok = 0;
		ch_hr_valid = 0;
		ch_spo2_valid = 0;
		return;
	}

	for (i = 0; i < SAMPLE_COUNT; i++)
	{
		if (ReadMax30102Sample(i))
		{
			valid_count++;
			fail_count = 0;
		}
		else
		{
			fail_count++;
			if (fail_count >= 20)
			{
				break;
			}
		}
	}

	if (valid_count > (SAMPLE_COUNT / 2))
	{
		maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length,
			aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
	}
	else
	{
		Max30102Ok = 0;
		ch_hr_valid = 0;
		ch_spo2_valid = 0;
	}
}

static void UpdateMax30102Samples(uint8_t *heart_rate, uint8_t *spo2)
{
	uint16_t i;
	uint16_t valid_count = 0;

	if (!Max30102Ok)
	{
		max30102_init();
		if (!Max30102_Check())
		{
			Max30102FingerOn = 0;
			LastHeartRate = 0;
			LastSpo2 = 0;
			*heart_rate = 0;
			*spo2 = 0;
			return;
		}
		Max30102Ok = 1;
	}

	for (i = SAMPLE_STEP; i < SAMPLE_COUNT; i++)
	{
		aun_red_buffer[i - SAMPLE_STEP] = aun_red_buffer[i];
		aun_ir_buffer[i - SAMPLE_STEP] = aun_ir_buffer[i];
	}

	for (i = SAMPLE_COUNT - SAMPLE_STEP; i < SAMPLE_COUNT; i++)
	{
		if (ReadMax30102Sample(i))
		{
			valid_count++;
		}
	}

	if (valid_count > (SAMPLE_STEP / 2))
	{
		Max30102Ok = 1;
		Max30102FingerOn = 1;
		maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length,
			aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
	}
	else
	{
		Max30102Ok = 0;
		Max30102FingerOn = 0;
		Max30102Hold = 0;
		LastHeartRate = 0;
		LastSpo2 = 0;
		ch_hr_valid = 0;
		ch_spo2_valid = 0;
	}

	if (!Max30102FingerOn)
	{
		*heart_rate = 0;
		*spo2 = 0;
		return;
	}

	if (ch_hr_valid && n_heart_rate >= 30 && n_heart_rate <= 220)
	{
		*heart_rate = (uint8_t)n_heart_rate;
		LastHeartRate = *heart_rate;
		Max30102Hold = 0;
	}
	else
	{
		if (LastHeartRate != 0 && Max30102Hold < MAX30102_HOLD_LIMIT)
		{
			*heart_rate = LastHeartRate;
			Max30102Hold++;
		}
		else
		{
			*heart_rate = 0;
		}
	}

	if (ch_spo2_valid && n_spo2 >= 70 && n_spo2 <= 100)
	{
		*spo2 = (uint8_t)n_spo2;
		LastSpo2 = *spo2;
	}
	else
	{
		*spo2 = LastSpo2;
	}
}

static int16_t ReadBodyTempX10(void)
{
	short raw;

	raw = get_tempetature();
	if (raw == DS18B20_TEMP_ERROR)
	{
		return -1;
	}

	return (int16_t)((raw * 10) / 16);
}

static int16_t UpdateBodyTempFilter(int16_t raw_temp_x10)
{
	int16_t corrected;
	int16_t previous;
	int16_t target;

	if (raw_temp_x10 < BODY_TEMP_MIN_X10 || raw_temp_x10 > BODY_TEMP_MAX_X10)
	{
		return BodyTempFilteredX10;
	}

	corrected = raw_temp_x10 + BODY_TEMP_OFFSET_X10;
	if (corrected > BODY_TEMP_MAX_X10)
	{
		corrected = BODY_TEMP_MAX_X10;
	}

	if (BodyTempValidReads < BODY_TEMP_WARMUP_READS)
	{
		BodyTempValidReads++;
		if (BodyTempFilteredX10 < 0)
		{
			BodyTempFilteredX10 = corrected;
		}
		else
		{
			BodyTempFilteredX10 = (BodyTempFilteredX10 + corrected) / 2;
		}
		return -1;
	}

	if (BodyTempFilteredX10 < 0)
	{
		BodyTempFilteredX10 = corrected;
	}
	else
	{
		previous = BodyTempFilteredX10;
		target = (BodyTempFilteredX10 * 7 + corrected) / 8;
		if (target > previous + BODY_TEMP_MAX_STEP_X10)
		{
			BodyTempFilteredX10 = previous + BODY_TEMP_MAX_STEP_X10;
		}
		else if (target + BODY_TEMP_MAX_STEP_X10 < previous)
		{
			BodyTempFilteredX10 = previous - BODY_TEMP_MAX_STEP_X10;
		}
		else
		{
			BodyTempFilteredX10 = target;
		}
	}

	return BodyTempFilteredX10;
}

static uint8_t IsFallDetected(void)
{
	if (Abs32((int32_t)AX - Last_AX) > FALL_ACC_DELTA ||
		Abs32((int32_t)AY - Last_AY) > FALL_ACC_DELTA ||
		Abs32((int32_t)AZ - Last_AZ) > FALL_ACC_DELTA)
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
						int16_t body_temp_x10,
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
		OLED_Printf(0, 16, OLED_8X16, "ENV:%02dC %02d%%", env_temp, env_humi);

		if (heart_rate != 0)
		{
			OLED_Printf(0, 32, OLED_8X16, "HR:%03d BPM", heart_rate);
		}
		else
		{
			OLED_Printf(0, 32, OLED_8X16, "HR:--- BPM");
		}

		if (spo2 != 0)
		{
			OLED_Printf(0, 48, OLED_8X16, "SpO2:%02d%%", spo2);
		}
		else
		{
			OLED_Printf(0, 48, OLED_8X16, "SpO2:--%%");
		}
	}
	else
	{
		if (body_temp_x10 >= 0)
		{
			OLED_Printf(0, 16, OLED_8X16, "BODY:%02d.%01dC", body_temp_x10 / 10, body_temp_x10 % 10);
		}
		else
		{
			OLED_Printf(0, 16, OLED_8X16, "BODY:--.-C");
		}
		OLED_Printf(88, 16, OLED_8X16, alarm ? (unsafe ? "RISK" : "ERR") : "OK");

		OLED_Printf(0, 32, OLED_8X16, "STEP:%06lu", (unsigned long)steps);

		OLED_Printf(0, 48, OLED_8X16, "P:%s", posture);
		OLED_Printf(72, 48, OLED_8X16, "F:%s", fall ? "YES" : "NO");
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
	int16_t body_temp_x10 = -1;
	int16_t body_raw_x10 = -1;
	DS1302_TimeTypeDef now;
	DS1302_TimeTypeDef rtc_read;
	DS1302_TimeTypeDef default_time = {26, 6, 9, 16, 45, 0, 1};

	NVIC_Configuration();
	delay_init();

	OLED_Init();
	OLED_ShowString(0, 0, "Smart band init", OLED_8X16);
	OLED_ShowString(0, 16, "Please wait...", OLED_8X16);
	OLED_Update();

	OLED_Clear();
	OLED_ShowString(0, 0, "Init MAX30102", OLED_8X16);
	OLED_Update();
	max30102_init();

	OLED_Clear();
	OLED_ShowString(0, 0, "Init DS18B20", OLED_8X16);
	OLED_Update();
	ds18b20_init();

	OLED_Clear();
	OLED_ShowString(0, 0, "Init DHT11", OLED_8X16);
	OLED_Update();
	DHT11_Init();

	OLED_Clear();
	OLED_ShowString(0, 0, "Init DS1302", OLED_8X16);
	OLED_Update();
	DS1302_Init();
	delay_ms(10);

	OLED_Clear();
	OLED_ShowString(0, 0, "Init IO", OLED_8X16);
	OLED_Update();
	Buzzer_init();
	Key_Init();

	OLED_Clear();
	OLED_ShowString(0, 0, "Init MPU6050", OLED_8X16);
	OLED_Update();
	StepCounter_Init();

	DS1302_GetDateTime(&now);
	if (DS1302_IsClockHalted() ||
		!IsRtcTimeValid(&now) ||
		DS1302_ReadRam(0) != RTC_INIT_MARK0 ||
		DS1302_ReadRam(1) != RTC_INIT_MARK1)
	{
		DS1302_SetDateTime(&default_time);
		DS1302_WriteRam(0, RTC_INIT_MARK0);
		DS1302_WriteRam(1, RTC_INIT_MARK1);
		DS1302_GetDateTime(&now);
	}
	else
	{
		DS1302_StartClock();
	}

	StepCounter_GetSensorData(&AX, &AY, &AZ, &GX, &GY, &GZ);
	Last_AX = AX;
	Last_AY = AY;
	Last_AZ = AZ;

	OLED_Clear();
	OLED_ShowString(0, 0, "MAX sample...", OLED_8X16);
	OLED_ShowString(0, 16, "Wait 1-5 sec", OLED_8X16);
	OLED_Update();
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
		body_raw_x10 = ReadBodyTempX10();
		body_temp_x10 = UpdateBodyTempFilter(body_raw_x10);

		Last_AX = AX;
		Last_AY = AY;
		Last_AZ = AZ;
		StepCounter_GetSensorData(&AX, &AY, &AZ, &GX, &GY, &GZ);

		fall = IsFallDetected();
		unsafe = IsUnsafeMotion();
		PageKey_Task();

		alarm = fall || unsafe ||
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
			GetPostureText(), fall, unsafe, alarm, StepCounter_GetCount());
	}
}
