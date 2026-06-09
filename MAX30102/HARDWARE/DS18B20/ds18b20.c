#include "ds18b20.h"
#include "delay.h"

#define DS18B20_GPIO GPIOA
#define DS18B20_PIN  GPIO_Pin_8
#define DS18B20_RCC  RCC_APB2Periph_GPIOA

static void ds18b20_mode_output(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = DS18B20_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(DS18B20_GPIO, &GPIO_InitStructure);
}

static void ds18b20_mode_input(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = DS18B20_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DS18B20_GPIO, &GPIO_InitStructure);
}

static uint8_t ds18b20_reset(void)
{
	uint16_t timeout;

	ds18b20_mode_output();
	GPIO_ResetBits(DS18B20_GPIO, DS18B20_PIN);
	delay_us(600);
	GPIO_SetBits(DS18B20_GPIO, DS18B20_PIN);
	ds18b20_mode_input();

	timeout = 1000;
	while (GPIO_ReadInputDataBit(DS18B20_GPIO, DS18B20_PIN))
	{
		if (timeout-- == 0) return 0;
		delay_us(1);
	}

	timeout = 1000;
	while (!GPIO_ReadInputDataBit(DS18B20_GPIO, DS18B20_PIN))
	{
		if (timeout-- == 0) return 0;
		delay_us(1);
	}

	return 1;
}

static void ds18b20_write_bit(uint8_t bit)
{
	ds18b20_mode_output();
	GPIO_ResetBits(DS18B20_GPIO, DS18B20_PIN);
	if (bit)
	{
		delay_us(2);
		GPIO_SetBits(DS18B20_GPIO, DS18B20_PIN);
		delay_us(70);
	}
	else
	{
		delay_us(70);
		GPIO_SetBits(DS18B20_GPIO, DS18B20_PIN);
		delay_us(2);
	}
}

static uint8_t ds18b20_read_bit(void)
{
	uint8_t bit;

	ds18b20_mode_output();
	GPIO_ResetBits(DS18B20_GPIO, DS18B20_PIN);
	delay_us(2);
	GPIO_SetBits(DS18B20_GPIO, DS18B20_PIN);
	ds18b20_mode_input();
	delay_us(12);
	bit = GPIO_ReadInputDataBit(DS18B20_GPIO, DS18B20_PIN);
	delay_us(55);

	return bit;
}

static void ds18b20_write_byte(uint8_t data)
{
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		ds18b20_write_bit(data & 0x01);
		data >>= 1;
	}
}

static uint8_t ds18b20_read_byte(void)
{
	uint8_t i;
	uint8_t data = 0;

	for (i = 0; i < 8; i++)
	{
		if (ds18b20_read_bit())
		{
			data |= (uint8_t)(1 << i);
		}
	}

	return data;
}

void ds18b20_init(void)
{
	RCC_APB2PeriphClockCmd(DS18B20_RCC, ENABLE);
	ds18b20_mode_output();
	GPIO_SetBits(DS18B20_GPIO, DS18B20_PIN);
	ds18b20_reset();
}

short get_tempetature(void)
{
	uint8_t tl, th;
	short raw;

	if (!ds18b20_reset())
	{
		return DS18B20_TEMP_ERROR;
	}
	ds18b20_write_byte(0xCC);
	ds18b20_write_byte(0x44);
	delay_ms(750);

	if (!ds18b20_reset())
	{
		return DS18B20_TEMP_ERROR;
	}
	ds18b20_write_byte(0xCC);
	ds18b20_write_byte(0xBE);

	tl = ds18b20_read_byte();
	th = ds18b20_read_byte();
	raw = (short)((th << 8) | tl);

	if (raw == 0 || raw == 0x0550 || raw == (short)0xFFFF)
	{
		return DS18B20_TEMP_ERROR;
	}

	return raw;
}
