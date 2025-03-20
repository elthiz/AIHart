#include <string.h>

#include "led.h"

#include "main.h"
#include "setting.h"
#include "i2c.h"


#define LED_ENABLE 							1
#define LED_DISABLE 						0

uint8_t delayCnt = 0;
uint8_t ledI2C[] = {0b00000010, 0b00111111, 0b00111111};
uint8_t ledLocal[] = {0b00000010, 0b00111111, 0b00111111};

void ledSet(LED_TYPE type, uint8_t state);

typedef struct LedData
{
	GPIO_TypeDef* port;
	uint16_t pin;
	LED_MODE mode;
	uint32_t startCounter;
	uint32_t secondCounter;
	uint8_t isMultiColor;
	uint8_t color;
	uint16_t flag;
	void ( *ledFunction )( LED_TYPE type );
} LedData;

enum
{
	CH1 = 0,
	CH2 = 1,
	CH3 = 2,
	CH4 = 3,
	CH5 = 4,
	CH6 = 5,
} LED_I2C_PIN;

LedData ledData[] =
{
	{ LED_STATUS_GPIO_Port,	LED_STATUS_Pin,  	MODE_OFF, 	0,	0,	0 	},
	{ LED_RUN_GPIO_Port,	LED_RUN_Pin,   		MODE_OFF, 	0,	0,	0 	},
	{ LED_ALARM_GPIO_Port,	LED_ALARM_Pin, 		MODE_OFF, 	0,	0,	0 	},

	{ NULL,					CH1, 				MODE_OFF, 	0,	0,	1 	},
	{ NULL,					CH2, 				MODE_OFF, 	0,	0,	1 	},
	{ NULL,					CH3, 				MODE_OFF, 	0,	0,	1 	},
	{ NULL,					CH4, 				MODE_OFF, 	0,	0,	1 	},
	{ NULL,					CH5, 				MODE_OFF, 	0,	0,	1 	},
	{ NULL,					CH6, 				MODE_OFF, 	0,	0,	1 	},
};

void ledSet(LED_TYPE type, uint8_t state)
{
	if ( ledData[type].isMultiColor )
	{
		if (state)
		{
			ledLocal[1] = ( ledLocal[1] & ~(1 << ledData[type].pin ) ) | ( ( ( ledData[type].color >> 0 ) & 1 ) << ledData[type].pin );
			ledLocal[2] = ( ledLocal[2] & ~(1 << ledData[type].pin ) ) | ( ( ( ledData[type].color >> 1 ) & 1 ) << ledData[type].pin );
		}
		else
		{
			ledLocal[1] |= (1 << ledData[type].pin);
			ledLocal[2] |= (1 << ledData[type].pin);
		}
	}
	else
	{
		HAL_GPIO_WritePin( ledData[type].port, ledData[type].pin, state);
	}
};

void ledOff( LED_TYPE type )
{
	ledSet(type, LED_DISABLE);
}

void ledSingle( LED_TYPE type )
{
	if( ledData[type].flag == 0 )
	{
		ledData[type].flag = 1;
		ledData[type].startCounter = HAL_GetTick();

		ledSet(type, LED_ENABLE);
		ledData[type].secondCounter = HAL_GetTick() + 200;
	}
	else
	{
		if( ledData[type].secondCounter <= HAL_GetTick() )
		{
			if( ledData[type].flag == 1 )
			{
				ledData[type].flag = 2;
				ledData[type].secondCounter += 1000;
				ledSet(type, LED_DISABLE);
			}
			else if( ledData[type].flag == 2 )
			{
				ledData[type].flag = 0;
			}
		}
	}
}

void ledDouble( LED_TYPE type )
{
	if( ledData[type].flag == 0 )
	{
		ledData[type].flag = 1;
		ledData[type].startCounter = HAL_GetTick();

		ledSet(type, LED_ENABLE);
		ledData[type].secondCounter = HAL_GetTick() + 200;
	}
	else
	{
		if( ledData[type].secondCounter <= HAL_GetTick() )
		{
			if( ledData[type].flag == 1 )
			{
				ledData[type].flag = 2;
				ledData[type].secondCounter += 200;
				ledSet(type, LED_DISABLE);
			}
			else if( ledData[type].flag == 2 )
			{
				ledData[type].flag = 3;
				ledData[type].secondCounter += 200;
				ledSet(type, LED_ENABLE);
			}
			else if( ledData[type].flag == 3 )
			{
				ledData[type].flag = 4;
				ledData[type].secondCounter += 1000;
				ledSet(type, LED_DISABLE);
			}
			else if( ledData[type].flag == 4 )
			{
				ledData[type].flag = 0;
			}
		}
	}
}

void ledTriple( LED_TYPE type )
{
	if( ledData[type].flag == 0 )
	{
		ledData[type].flag = 1;
		ledData[type].startCounter = HAL_GetTick();

		ledSet(type, LED_ENABLE);
		ledData[type].secondCounter = HAL_GetTick() + 200;
	}
	else
	{
		if(ledData[type].secondCounter <= HAL_GetTick())
		{
			if( ledData[type].flag == 1 )
			{
				ledData[type].flag = 2;
				ledData[type].secondCounter += 200;
				ledSet(type, LED_DISABLE);
			}
			else if( ledData[type].flag == 2 )
			{
				ledData[type].flag = 3;
				ledData[type].secondCounter += 200;
				ledSet(type, LED_ENABLE);
			}
			else if( ledData[type].flag == 3 )
			{
				ledData[type].flag = 4;
				ledData[type].secondCounter += 200;
				ledSet(type, LED_DISABLE);
			}
			else if( ledData[type].flag == 4 )
			{
				ledData[type].flag = 5;
				ledData[type].secondCounter += 200;
				ledSet(type, LED_ENABLE);
			}
			else if( ledData[type].flag == 5 )
			{
				ledData[type].flag = 6;
				ledData[type].secondCounter += 1000;
				ledSet(type, LED_DISABLE);
			}
			else if( ledData[type].flag == 6 )
			{
				ledData[type].flag = 0;
			}
		}
	}
}

void ledBlink( LED_TYPE type )
{
	if( ledData[type].flag == 0 )
	{
		ledData[type].flag = 1;
		ledData[type].startCounter = HAL_GetTick();

		ledSet(type, LED_ENABLE);
		ledData[type].secondCounter = HAL_GetTick() + 200;
	}
	else
	{
		if(ledData[type].secondCounter <= HAL_GetTick())
		{
			if( ledData[type].secondCounter <= HAL_GetTick() )
			{
				if( ledData[type].flag == 1 )
				{
					ledData[type].flag = 2;
					ledData[type].secondCounter += 200;
					ledSet(type, LED_DISABLE);
				}
				else if( ledData[type].flag == 2 )
				{
					ledData[type].flag = 0;
				}
			}
		}
	}
}

void ledFlick( LED_TYPE type )
{
	if( ledData[type].flag == 0 )
	{
		ledData[type].flag = 1;
		ledData[type].startCounter = HAL_GetTick();

		ledSet(type, LED_ENABLE);
		ledData[type].secondCounter = HAL_GetTick() + 50;
	}
	else
	{
		if(ledData[type].secondCounter <= HAL_GetTick())
		{
			if( ledData[type].secondCounter <= HAL_GetTick() )
			{
				if( ledData[type].flag == 1 )
				{
					ledData[type].flag = 2;
					ledData[type].secondCounter += 50;
					ledSet(type, LED_DISABLE);
				}
				else if( ledData[type].flag == 2 )
				{
					ledData[type].flag = 0;
				}
			}
		}
	}
}

void ledOn( LED_TYPE type )
{
	ledSet(type, LED_ENABLE);
}

void ledInit(void)
{
	/* Перенести из GPIO */
	if ( HAL_I2C_IsDeviceReady( &hi2c1, XL9535_I2C_ADDRESS, 1, 100) == HAL_OK )
	{
		uint8_t configurationData[] = {0b00000110, 0b00000000, 0b00000000};
		HAL_I2C_Master_Transmit(&hi2c1, XL9535_I2C_ADDRESS, configurationData, sizeof(configurationData), 100);
	}
	else
	{
		/* TODO: FAULT РАСШИРИТЕЛЬ ПОРТОВ НЕ НАЙДЕН */
	}
}

void ledProcess(void)
{
	if( ledData[LED_STAT].ledFunction != NULL ) { ledData[LED_STAT].ledFunction(LED_STAT); }
	if( ledData[LED_RUN].ledFunction != NULL ) { ledData[LED_RUN].ledFunction(LED_RUN); }
	if( ledData[LED_ALRM].ledFunction != NULL ) { ledData[LED_ALRM].ledFunction(LED_ALRM); }

	if( ledData[LED_CH1].ledFunction != NULL ) { ledData[LED_CH1].ledFunction(LED_CH1); }
	if( ledData[LED_CH2].ledFunction != NULL ) { ledData[LED_CH2].ledFunction(LED_CH2); }
	if( ledData[LED_CH3].ledFunction != NULL ) { ledData[LED_CH3].ledFunction(LED_CH3); }
	if( ledData[LED_CH4].ledFunction != NULL ) { ledData[LED_CH4].ledFunction(LED_CH4); }
	if( ledData[LED_CH5].ledFunction != NULL ) { ledData[LED_CH5].ledFunction(LED_CH5); }
	if( ledData[LED_CH6].ledFunction != NULL ) { ledData[LED_CH6].ledFunction(LED_CH6); }

	if ( ( HAL_GetTick() - LED_DELAY_TRANSMIT ) > delayCnt )
	{
		delayCnt = HAL_GetTick();
		memcpy(ledI2C, ledLocal, sizeof(ledI2C));
		HAL_I2C_Master_Transmit_DMA( &hi2c1, XL9535_I2C_ADDRESS, ledI2C, sizeof(ledI2C) );
	}
}

void setLedMode( LED_TYPE type, LED_MODE mode, LED_COLOR color )
{
	ledData[type].mode = mode;
	ledData[type].flag = 0;

	if ( ledData[type].isMultiColor )
	{
		ledData[type].color = color;
	}

	if( mode == MODE_OFF )
	{ ledData[type].ledFunction = ledOff; }
	else if( mode == MODE_SINGLE )
	{ ledData[type].ledFunction = ledSingle; }
	else if( mode == MODE_DOUBLE )
	{ ledData[type].ledFunction = ledDouble; }
	else if( mode == MODE_TRIPLE )
	{ ledData[type].ledFunction = ledTriple; }
	else if( mode == MODE_BLINK )
	{ ledData[type].ledFunction = ledBlink; }
	else if( mode == MODE_FLICK )
	{ ledData[type].ledFunction = ledFlick; }
	else if( mode == MODE_ON )
	{ ledData[type].ledFunction = ledOn; }

}
