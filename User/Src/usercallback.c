#include "usercallback.h"

#include "spi.h"

void ( *arrayCallback[4] )();

void registerCallback(void ( *callback )(), TYPE_CALLBACK type )
{
	arrayCallback[type] = callback;
}

/* Callback при завершении */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if ( hspi == &hspi1 )
	{
		arrayCallback[SPI1_TX_RX_CPT]();
	}
	else
	if ( hspi == &hspi2 )
	{
		arrayCallback[SPI2_TX_RX_CPT]();
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if ( hspi == &hspi2 )
	{
		arrayCallback[SPI2_TX_CPT]();
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if ( hspi == &hspi2 )
	{
		arrayCallback[SPI2_RX_CPT]();
	}
}
