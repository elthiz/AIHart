#ifndef INC_LED_H_
#define INC_LED_H_

#include <stdint.h>

typedef enum
{
	LED_STAT	= 0,
	LED_RUN		= 1,
	LED_ALRM	= 2,

	LED_CH1		= 3,
	LED_CH2		= 4,
	LED_CH3		= 5,
	LED_CH4		= 6,
	LED_CH5		= 7,
	LED_CH6		= 8,
} LED_TYPE;

typedef enum
{
	MODE_OFF	= 0,	// All time OFF
	MODE_SINGLE	= 1,	// 200ms - ON, 1000ms - OFF, Repeat
	MODE_DOUBLE	= 2,	// 200ms - ON, 200ms - OFF, 200ms - ON, 1000ms - OFF, Repeat
	MODE_TRIPLE	= 3,	// 200ms - ON, 200ms - OFF, 200ms - ON, 200ms - OFF, 200ms - ON, 1000ms - OFF
	MODE_BLINK	= 4,	// 200ms - ON, 200ms - OFF, Repeat
	MODE_FLICK	= 5,	// 50ms - ON, 50ms - OFF, Repeat
	MODE_ON		= 6		// All timer ON
} LED_MODE;

typedef enum
{
	COLOR_GREEN 	= 0b01,
    COLOR_RED 		= 0b10,
	COLOR_YELLOW 	= 0b00,
	COLOR_NONE 		= 0b11,
} LED_COLOR;



void ledInit( void );
void ledProcess( void );
void setLedMode( LED_TYPE type, LED_MODE mode, LED_COLOR color);


#endif /* INC_LED_H_ */
