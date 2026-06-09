#include "dht11.h"
#include "delay.h"

#define DHT11_GPIO       GPIOA
#define DHT11_PIN        GPIO_Pin_7
#define DHT11_RCC        RCC_APB2Periph_GPIOA

static void DHT11_SetOutput(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = DHT11_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DHT11_GPIO, &GPIO_InitStructure);
}

static void DHT11_SetInput(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = DHT11_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DHT11_GPIO, &GPIO_InitStructure);
}

static void DHT11_WriteBit(uint8_t bit)
{
	GPIO_WriteBit(DHT11_GPIO, DHT11_PIN, (BitAction)bit);
}

static uint8_t DHT11_ReadBit(void)
{
	return GPIO_ReadInputDataBit(DHT11_GPIO, DHT11_PIN);
}

static uint8_t DHT11_WaitLevel(uint8_t level, uint16_t timeout_us)
{
	while (DHT11_ReadBit() == level)
	{
		if (timeout_us-- == 0)
		{
			return DHT11_ERROR;
		}
		delay_us(1);
	}
	return DHT11_OK;
}

void DHT11_Init(void)
{
	RCC_APB2PeriphClockCmd(DHT11_RCC, ENABLE);
	DHT11_SetOutput();
	DHT11_WriteBit(1);
}

uint8_t DHT11_ReadData(uint8_t *temperature, uint8_t *humidity)
{
	uint8_t data[5] = {0};
	uint8_t i, j;

	DHT11_SetOutput();
	DHT11_WriteBit(0);
	delay_ms(20);
	DHT11_WriteBit(1);
	delay_us(30);
	DHT11_SetInput();

	if (DHT11_WaitLevel(1, 100) != DHT11_OK) return DHT11_ERROR;
	if (DHT11_WaitLevel(0, 100) != DHT11_OK) return DHT11_ERROR;
	if (DHT11_WaitLevel(1, 100) != DHT11_OK) return DHT11_ERROR;

	for (i = 0; i < 5; i++)
	{
		for (j = 0; j < 8; j++)
		{
			if (DHT11_WaitLevel(0, 70) != DHT11_OK) return DHT11_ERROR;
			delay_us(40);
			data[i] <<= 1;
			if (DHT11_ReadBit())
			{
				data[i] |= 0x01;
			}
			if (DHT11_WaitLevel(1, 100) != DHT11_OK) return DHT11_ERROR;
		}
	}

	if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
	{
		return DHT11_ERROR;
	}

	*humidity = data[0];
	*temperature = data[2];
	return DHT11_OK;
}
