#include "ds1302.h"
#include "delay.h"

#define DS1302_GPIO        GPIOB
#define DS1302_RCC         RCC_APB2Periph_GPIOB
#define DS1302_CE_PIN      GPIO_Pin_12
#define DS1302_SCLK_PIN    GPIO_Pin_13
#define DS1302_IO_PIN      GPIO_Pin_14

#define DS1302_REG_SEC     0x80
#define DS1302_REG_MIN     0x82
#define DS1302_REG_HOUR    0x84
#define DS1302_REG_DATE    0x86
#define DS1302_REG_MONTH   0x88
#define DS1302_REG_WEEK    0x8A
#define DS1302_REG_YEAR    0x8C
#define DS1302_REG_CTRL    0x8E
#define DS1302_REG_CHARGER 0x90
#define DS1302_REG_BURST   0xBE

static uint8_t DS1302_Tim2WasEnabled = 0;

static void DS1302_IO_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = DS1302_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DS1302_GPIO, &GPIO_InitStructure);
}

static void DS1302_IO_Input(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = DS1302_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DS1302_GPIO, &GPIO_InitStructure);
}

static void DS1302_Lock(void)
{
	DS1302_Tim2WasEnabled = 0;
	if (RCC->APB1ENR & RCC_APB1Periph_TIM2)
	{
		DS1302_Tim2WasEnabled = (uint8_t)((TIM2->CR1 & TIM_CR1_CEN) != 0);
		TIM_Cmd(TIM2, DISABLE);
	}
}

static void DS1302_Unlock(void)
{
	if ((RCC->APB1ENR & RCC_APB1Periph_TIM2) && DS1302_Tim2WasEnabled)
	{
		TIM_Cmd(TIM2, ENABLE);
	}
}

static void DS1302_Delay(void)
{
	delay_us(2);
}

static uint8_t DS1302_DecToBcd(uint8_t dec)
{
	return (uint8_t)(((dec / 10) << 4) | (dec % 10));
}

static uint8_t DS1302_BcdToDec(uint8_t bcd)
{
	return (uint8_t)((bcd >> 4) * 10 + (bcd & 0x0F));
}

static void DS1302_WriteByteRaw(uint8_t data)
{
	uint8_t i;

	DS1302_IO_Output();
	for (i = 0; i < 8; i++)
	{
		GPIO_WriteBit(DS1302_GPIO, DS1302_IO_PIN, (BitAction)(data & 0x01));
		DS1302_Delay();
		GPIO_SetBits(DS1302_GPIO, DS1302_SCLK_PIN);
		DS1302_Delay();
		GPIO_ResetBits(DS1302_GPIO, DS1302_SCLK_PIN);
		data >>= 1;
	}
}

static uint8_t DS1302_ReadByteRaw(void)
{
	uint8_t i;
	uint8_t data = 0;

	DS1302_IO_Input();
	for (i = 0; i < 8; i++)
	{
		if (GPIO_ReadInputDataBit(DS1302_GPIO, DS1302_IO_PIN))
		{
			data |= (uint8_t)(1 << i);
		}
		GPIO_SetBits(DS1302_GPIO, DS1302_SCLK_PIN);
		DS1302_Delay();
		GPIO_ResetBits(DS1302_GPIO, DS1302_SCLK_PIN);
		DS1302_Delay();
	}

	return data;
}

static void DS1302_WriteReg(uint8_t command, uint8_t data)
{
	DS1302_Lock();
	GPIO_ResetBits(DS1302_GPIO, DS1302_SCLK_PIN);
	GPIO_SetBits(DS1302_GPIO, DS1302_CE_PIN);
	DS1302_WriteByteRaw(command);
	DS1302_WriteByteRaw(data);
	GPIO_ResetBits(DS1302_GPIO, DS1302_CE_PIN);
	DS1302_Unlock();
}

static uint8_t DS1302_ReadReg(uint8_t command)
{
	uint8_t data;

	DS1302_Lock();
	GPIO_ResetBits(DS1302_GPIO, DS1302_SCLK_PIN);
	GPIO_SetBits(DS1302_GPIO, DS1302_CE_PIN);
	DS1302_WriteByteRaw(command | 0x01);
	data = DS1302_ReadByteRaw();
	GPIO_ResetBits(DS1302_GPIO, DS1302_CE_PIN);
	DS1302_Unlock();

	return data;
}

static void DS1302_ReadBurst(uint8_t *data)
{
	uint8_t i;

	DS1302_Lock();
	GPIO_ResetBits(DS1302_GPIO, DS1302_SCLK_PIN);
	GPIO_SetBits(DS1302_GPIO, DS1302_CE_PIN);
	DS1302_WriteByteRaw(DS1302_REG_BURST | 0x01);
	for (i = 0; i < 8; i++)
	{
		data[i] = DS1302_ReadByteRaw();
	}
	GPIO_ResetBits(DS1302_GPIO, DS1302_CE_PIN);
	DS1302_IO_Output();
	GPIO_SetBits(DS1302_GPIO, DS1302_IO_PIN);
	DS1302_Unlock();
}

void DS1302_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(DS1302_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = DS1302_CE_PIN | DS1302_SCLK_PIN | DS1302_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DS1302_GPIO, &GPIO_InitStructure);

	GPIO_ResetBits(DS1302_GPIO, DS1302_CE_PIN);
	GPIO_ResetBits(DS1302_GPIO, DS1302_SCLK_PIN);
	GPIO_SetBits(DS1302_GPIO, DS1302_IO_PIN);

	DS1302_WriteReg(DS1302_REG_CTRL, 0x00);
	DS1302_WriteReg(DS1302_REG_CHARGER, 0x00);
}

void DS1302_SetDateTime(const DS1302_TimeTypeDef *time)
{
	DS1302_WriteReg(DS1302_REG_CTRL, 0x00);
	DS1302_WriteReg(DS1302_REG_SEC, 0x80);
	DS1302_WriteReg(DS1302_REG_MIN, DS1302_DecToBcd(time->minute));
	DS1302_WriteReg(DS1302_REG_HOUR, DS1302_DecToBcd(time->hour));
	DS1302_WriteReg(DS1302_REG_DATE, DS1302_DecToBcd(time->date));
	DS1302_WriteReg(DS1302_REG_MONTH, DS1302_DecToBcd(time->month));
	DS1302_WriteReg(DS1302_REG_WEEK, DS1302_DecToBcd(time->week));
	DS1302_WriteReg(DS1302_REG_YEAR, DS1302_DecToBcd(time->year));
	DS1302_WriteReg(DS1302_REG_SEC, DS1302_DecToBcd(time->second) & 0x7F);
	DS1302_WriteReg(DS1302_REG_CTRL, 0x80);
}

void DS1302_GetDateTime(DS1302_TimeTypeDef *time)
{
	uint8_t data[8];

	DS1302_ReadBurst(data);
	time->second = DS1302_BcdToDec(data[0] & 0x7F);
	time->minute = DS1302_BcdToDec(data[1]);
	time->hour = DS1302_BcdToDec(data[2] & 0x3F);
	time->date = DS1302_BcdToDec(data[3]);
	time->month = DS1302_BcdToDec(data[4]);
	time->week = DS1302_BcdToDec(data[5]);
	time->year = DS1302_BcdToDec(data[6]);
}

uint8_t DS1302_IsClockHalted(void)
{
	return (uint8_t)((DS1302_ReadReg(DS1302_REG_SEC) & 0x80) != 0);
}

void DS1302_StartClock(void)
{
	uint8_t sec;

	DS1302_WriteReg(DS1302_REG_CTRL, 0x00);
	sec = DS1302_ReadReg(DS1302_REG_SEC);
	DS1302_WriteReg(DS1302_REG_SEC, sec & 0x7F);
	DS1302_WriteReg(DS1302_REG_CTRL, 0x80);
}
