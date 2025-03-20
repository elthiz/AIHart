#include "moduledata.h"
#include "setting.h"
#include "usart.h"
#include "usercan.h"
#include "tim.h"
#include "string.h"

#include "led.h"

/* Мьютекс берется каждый раз, когда происходит любое действие (прием/передача) на любом из каналов
 * Освобождается при завершении операции */
volatile uint8_t flagBusy = 0;

/* Номер канала, на котором была совершена последняя операция */
volatile uint8_t activeCH = 0;

volatile uint8_t flagToTransmitPDO = 0;

volatile uint8_t timerTickCounter = 0;

/* Структора под отдельный канал */
typedef struct HartData
{
	uint32_t *pointerRxBuff;
	uint32_t *pointerTxBuff;

	uint16_t *pointerRxSize;
	uint16_t *pointerTxSize;

	uint8_t localHartFlagTxEn;
	uint8_t localHartFlagRxEn;
	uint8_t localHartFlagTxCompleted;
	uint8_t localHartFlagRxCompleted;

	uint8_t tickForTimeout;

	uint8_t muxValue;
} HartData;

HartData hartData[6] =
{
	{ userData.rxBuffCH1, userData.txBuffCH1, &userData.sizeRxBuffer[0], &userData.sizeTxBuffer[0] },
	{ userData.rxBuffCH2, userData.txBuffCH2, &userData.sizeRxBuffer[1], &userData.sizeTxBuffer[1] },
	{ userData.rxBuffCH3, userData.txBuffCH3, &userData.sizeRxBuffer[2], &userData.sizeTxBuffer[2] },
	{ userData.rxBuffCH4, userData.txBuffCH4, &userData.sizeRxBuffer[3], &userData.sizeTxBuffer[3] },
	{ userData.rxBuffCH5, userData.txBuffCH5, &userData.sizeRxBuffer[4], &userData.sizeTxBuffer[4] },
	{ userData.rxBuffCH6, userData.txBuffCH6, &userData.sizeRxBuffer[5], &userData.sizeTxBuffer[5] },
};

uint8_t hart_request3[] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x82, 0xA6, 0x18, 0x51, 0x62, 0xA7, 0x03,
		0x00, 0xAB, 0x00,
};


void hartInit()
{
	/* Таблица мультиплексора
	 * 0 выход = 111
	 * 1 выход = 110
	 * 2 выход = 100
	 * 3 выход = 010
	 * 4 выход = 001
	 * 5 выход = 000
	 * Самый старший бит необходимо инвертировать!
	 * */
	hartData[0].muxValue = 0b011;
	hartData[1].muxValue = 0b010;
	hartData[2].muxValue = 0b000;
	hartData[3].muxValue = 0b110;
	hartData[4].muxValue = 0b101;
	hartData[5].muxValue = 0b100;

	hartData[0].tickForTimeout = 30;
	hartData[1].tickForTimeout = 30;
	hartData[2].tickForTimeout = 30;
	hartData[3].tickForTimeout = 30;
	hartData[4].tickForTimeout = 120;
	hartData[5].tickForTimeout = 30;
}

void hartProcess()
{
	for (int numCH = 0; numCH < AI_CH_NUM; numCH++)
	{
		if ( hartData[numCH].localHartFlagTxEn != userData.hartFlagTxEn[numCH] )
		{
			hartData[numCH].localHartFlagTxEn = userData.hartFlagTxEn[numCH];
			flagToTransmitPDO = 1;
		}

		if ( hartData[numCH].localHartFlagRxEn != userData.hartFlagRxEn[numCH] )
		{
			hartData[numCH].localHartFlagRxEn = userData.hartFlagRxEn[numCH];
			flagToTransmitPDO = 1;
		}

		if ( hartData[numCH].localHartFlagTxCompleted != userData.hartFlagTxCompleted[numCH] )
		{
			hartData[numCH].localHartFlagTxCompleted = userData.hartFlagTxCompleted[numCH];
			flagToTransmitPDO = 1;
		}

		if ( hartData[numCH].localHartFlagRxCompleted != userData.hartFlagRxCompleted[numCH] )
		{
			hartData[numCH].localHartFlagRxCompleted = userData.hartFlagRxCompleted[numCH];
			flagToTransmitPDO = 1;
		}
	}

	if ( flagToTransmitPDO )
	{
		usercanSendPDO4();
		flagToTransmitPDO = 0;
	}

	if (flagBusy)
	{
		return;
	}
	else
	{
		if ( hartData[activeCH].localHartFlagTxEn && ( !hartData[activeCH].localHartFlagTxCompleted ) )
		{
			HAL_TIM_Base_Stop_IT(&htim5);
			flagBusy = 1;
			HAL_GPIO_WritePin(MUX_0_GPIO_Port, MUX_0_Pin, 1);
			HAL_GPIO_WritePin(MUX_1_GPIO_Port, MUX_1_Pin, 1);
			HAL_GPIO_WritePin(MUX_2_GPIO_Port, MUX_2_Pin, 1);
			HAL_GPIO_WritePin(UART_RTS_GPIO_Port, UART_RTS_Pin, GPIO_PIN_RESET);
			//HAL_UART_Transmit_IT(&huart1, (uint8_t*)hartData[activeCH].pointerTxBuff, *hartData[activeCH].pointerTxSize);
			HAL_UART_Transmit_IT(&huart1, hart_request3, 16);
			return;
		}
		if ( hartData[activeCH].localHartFlagRxEn && ( !hartData[activeCH].localHartFlagRxCompleted) )
		{
			HAL_TIM_Base_Stop_IT(&htim5);
			HAL_UART_DMAStop(&huart1);
			flagBusy = 1;
			HAL_GPIO_WritePin(MUX_0_GPIO_Port, MUX_0_Pin, (1 << 0) & hartData[activeCH].muxValue);
			HAL_GPIO_WritePin(MUX_1_GPIO_Port, MUX_1_Pin, (1 << 1) & hartData[activeCH].muxValue);
			HAL_GPIO_WritePin(MUX_2_GPIO_Port, MUX_2_Pin, (1 << 2) & hartData[activeCH].muxValue);
			memset(hartData[activeCH].pointerRxBuff, 0, SIZE_HART_BUFF);
			timerTickCounter = 0;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)hartData[activeCH].pointerRxBuff, SIZE_HART_BUFF);
			HAL_TIM_Base_Start_IT(&htim5);
		}
		else
		{
			if (activeCH == 5)
			{
				activeCH = 0;
			}
			else
			{
				activeCH++;
			}
		}
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	(*hartData[activeCH].pointerRxSize) = Size;
	userData.hartFlagRxCompleted[activeCH] = 1;
	hartData[activeCH].localHartFlagRxCompleted = 1;

	flagBusy = 0;
	flagToTransmitPDO = 1;

	HAL_TIM_Base_Stop_IT(&htim5);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	userData.hartFlagTxCompleted[activeCH] = 1;
	hartData[activeCH].localHartFlagTxCompleted = 1;

	flagBusy = 0;
	flagToTransmitPDO = 1;

	HAL_GPIO_WritePin(UART_RTS_GPIO_Port, UART_RTS_Pin, GPIO_PIN_SET);
}

void hartTimeout()
{
	if (__HAL_DMA_GET_COUNTER(huart1.hdmarx) < SIZE_HART_BUFF)
	{
		timerTickCounter = 0;
		HAL_TIM_Base_Stop_IT(&htim5);
		return;
	}

	if (timerTickCounter == hartData[activeCH].tickForTimeout)
	{
		flagBusy = 0;
		HAL_TIM_Base_Stop_IT(&htim5);

		if (activeCH == 5)
		{
			activeCH = 0;
		}
		else
		{
			activeCH++;
		}
	}
	else
	{
		timerTickCounter += 1;
	}
}

