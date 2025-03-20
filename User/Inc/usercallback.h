#ifndef INC_USERCALLBACK_H_
#define INC_USERCALLBACK_H_

typedef enum
{
	SPI1_TX_RX_CPT = 0,
	SPI2_TX_RX_CPT = 1,
	SPI2_TX_CPT    = 2,
	SPI2_RX_CPT    = 3,
} TYPE_CALLBACK;

void registerCallback(void ( *callback )(), TYPE_CALLBACK type );

#endif
