#include "ai.h"

#include "setting.h"
#include "usercan.h"
#include "moduledata.h"
#include "usercallback.h"
#include "flash.h"

#include "spi.h"
#include "tim.h"
#include "led.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "crc.h"

/* ________________________ ENUM'S ________________________ */
/* Режимы работы блока AI. */
enum AI_MODE
{
	/* Режим - работа. Получение выборок с АЦП, рассчет значений тока. */
	AI_WORKING 					= 	0,
	/* Режим - калибровка. Калибровка по отдельному каналу, обновление значение фильтров. */
	AI_CALIBRATION 				= 	1,
};

/* Режимы работы калибровки. */
enum CALIBRATION_MODE
{
	/* Режим - ожидание. Ожидание команд, обновление индикации, обновление значений фильтров. */
	CALIBRATION_WAIT 			=   0,
	/* Режим - сэмплирование. Получение выборок с N-канала, запись среднего в массив калибровочных значений по величине тока. */
	CALIBRATION_SAMPLING		=	1,
	/* Режим - рассчет. Рассчет коэфициентов по средним выборком всей шкалы 4-20Ма. */
	CALIBRATION_CALC			= 	2,
	/* Режим - сохранение. Сохранение коэфициентов и значений фильтров на флешку. */
	CALIBRATION_SAVE 			= 	3,
};

/* Статусы флешки */
typedef enum FLASH_STATUS
{
	/* Статус - в работе. С флешкой выполняются операции. */
	FLASH_AT_WORK 				=   1,
	/* Статус - завершено. Значения успешно прочитаны. */
	FLASH_DONE 					= 	2,
} FLASH_STATUS;

/* Операции флешки */
enum FLASH_OPERATION
{
	/* Операция - сохранить. Сохранение данных на флешку. */
	FLASH_SAVE 					=   1,
	/* Операция - чтение. Чтение данных в буфер флешки. */
	FLASH_READ					= 	2,
	/* Операция - получение прочитанных данных. Чтение данных из буфера RX флешки. */
	FLASH_GET_READED			=	3,
	/* Операция - установка W/R режима. */
	FLASH_WRITE_MODE			=	4,
	/* Операция - Очистка сектора флешки. */
	FLASH_ERASE_SECTOR			=	5,
	/* Операция - вернуться к исходным значениям. Возвращает все переменные в состояние по умолчанию */
	FLASH_DEFAULT				=	6,
};

/* Маппинг каналов:
 * Входные IIN подключены сзадом наоборот к АЦП.
 * 1 канал - 6 вход АЦП.
 * 2 канал - 5 вход АЦП и т.д.
 * Помимо этого АЦП возвращает выборку с N-канала только на N+2 итерации. */
enum CH_MAP
{
	CH1 						=	4,
	CH2 						= 	5,
	CH3 						= 	0,
	CH4 						= 	1,
	CH5 						= 	2,
	CH6 						= 	3,
};

/* ________________________ FUNCTION'S PROTOTYPE ________________________ */
void aiSPITxRxCallback( void );
void aiCalibrationMa( uint8_t channel, uint16_t ma );
void aiCalibrationSaveData( void );
FLASH_STATUS aiReadCalibrationData( void );
uint16_t calcMedian( uint16_t sample, uint16_t* ptrToArray, uint8_t* pos );
double calcAverage( uint16_t sample, uint16_t* ptrToArray, uint32_t* ptrToSum, uint8_t* ptrToPos );
void aiWorking( void );
void updateLed( void );
void aiWaitCalibration( void );
void aiCalcCalibration( uint8_t channel );
void aiUpdateMode( void );
void aiReset( void );

/* ________________________ VARIABLE ________________________ */
/* Текущий режим работы блока AI. */
uint8_t aiMode = AI_WORKING;
/* Флаг, отправлен ли конфиг на АЦП. */
uint8_t isConfigSended = 0;
/* Флаг, получена ли выборка с АЦП. */
volatile uint8_t isSampleReceived = 0;
/* Флаг, нужно ли обновить индикацию. */
uint8_t isLedUpdate = 1;
/* Размер массива под фильтрацию скользящим средним. */
uint8_t filterAvgSize = DEFAULT_FILTER_AVERAGE_SIZE;
/* Значение коэфициента под фильтрацию экспонентой. */
float filterExpCurrent = DEFAULT_FILTER_EXP;
/* Локальная переменная канала под калибровку (нужна для индикации калибровочного канала). */
uint8_t calibrationCh = CALIBRATION_NO_CHANNEL;
/* Флаг индикации при записи на флешку. */
uint8_t isSaveLedEnabled = 0;

/* ________________________ STRUCT ________________________ */
typedef struct AiData
{
	/* Конфиг канала в АЦП. */
	uint8_t config[2];
	/* Выборка АЦП. */
	uint8_t sample[2];
	/* Канал с учетом маппинга. */
	uint8_t realCh;
	/* Указатель на данные userData. */
	uint16_t *ptrToUserData;
	/* Текущая позиция в массиве среднего под фильтрацию. */
	uint8_t avgCurrentPos;
	/* Массив среднего под фильтрацию. */
	uint16_t avgCurrentArr[SIZE_ARRAY_AVERAGE];
	/* Среднее по всем выборкам. */
	uint32_t avgCurrent;
	/* Значение тока с полной точностью. */
	double currentFull;
	/* Коэфициент калибровки x^2*a. */
	double coefA;
	/* Коэфициент калибровки x*b. */
	double coefB;
	/* Коэфициент калибровки x+-c. */
	double coefC;
	/* Значение тока. */
	uint16_t current;
	/* Массив под медианную фильтрацию. */
	uint16_t medianCurrentArr[SIZE_ARRAY_MEDIAN];
	/* Текущая позиция в массиве среднего. */
	uint8_t medianCurrentPos;
	/* Флаг короткого замыкания по значению тока. */
	uint8_t isKZ;
	/* Флаг, что ток укладывается в рабочий диапазон тока. */
	uint8_t isOK;
	/* Флаг, что обрыв линии по току. */
	uint8_t isFall;
} AiData;

typedef struct AiCalibrationData
{
	/* Сумма всех выборок на определенное значение тока при одной калибровке. */
	double sumCalibration;
	/* Среднее значение выборок. */
	uint16_t adcValue[SIZE_ARRAY_RANGE_MA];
	/* Выборка АЦП. */
	uint8_t sample[2];
} AiCalibrationData;

typedef struct AiDataFlash
{
	/* Коэфициент калибровки x^2*a. */
	double coefA[AI_CH_NUM];
	/* Коэфициент калибровки x*b. */
	double coefB[AI_CH_NUM];
	/* Коэфициент калибровки x+-c. */
	double coefC[AI_CH_NUM];
	/* Значение шага экспонециального фильтра. */
	double filterExpCurrent;
	/* Контрольная сумма. */
	uint32_t crc;
	/* Размер массива под фильтрацию средним. */
	uint8_t filterAvgSize;
} AiDataFlash;

typedef struct AiDataLed
{
	/* Канал индикации. */
	LED_TYPE type;
	/* Режим инидикации. */
	LED_MODE mode;
	/* Цвет индикации. */
	LED_COLOR color;
	/* Режим инидикации в рабочем режиме. */
	LED_MODE modeWorking;
	/* Цвет индикации в рабочем режиме. */
	LED_COLOR colorWorking;
} AiDataLed;

/* ________________________ INIT STRUCT ________________________ */
/* Пример конфигураций, отправляемых в АЦП:
*     		cfg		    INCC	INx		BW		REF		SEQ		RB
			 1  		111  	001 	1 		110 	00 		0
			 1  		110  	010 	0 		110 	00 		0
*/
AiData aiData[AI_CH_NUM] =
{
	{ { 0b11111101, 0b11000000 }, { 0, 0 }, CH1, &userData.dataCH1CH2[1] },
	{ { 0b11111011, 0b11000000 }, { 0, 0 }, CH2, &userData.dataCH1CH2[3] },
	{ { 0b11111001, 0b11000000 }, { 0, 0 }, CH3, &userData.dataCH3CH4[1] },
	{ { 0b11110111, 0b11000000 }, { 0, 0 }, CH4, &userData.dataCH3CH4[3] },
	{ { 0b11110101, 0b11000000 }, { 0, 0 }, CH5, &userData.dataCH5CH6[1] },
	{ { 0b11110011, 0b11000000 }, { 0, 0 }, CH6, &userData.dataCH5CH6[3] },
};

AiDataLed aiDataLed[AI_CH_NUM] =
{
	{ LED_CH1, MODE_OFF, COLOR_YELLOW, MODE_OFF, COLOR_YELLOW },
	{ LED_CH2, MODE_OFF, COLOR_YELLOW, MODE_OFF, COLOR_YELLOW },
	{ LED_CH3, MODE_OFF, COLOR_YELLOW, MODE_OFF, COLOR_YELLOW },
	{ LED_CH4, MODE_OFF, COLOR_YELLOW, MODE_OFF, COLOR_YELLOW },
	{ LED_CH5, MODE_OFF, COLOR_YELLOW, MODE_OFF, COLOR_YELLOW },
	{ LED_CH6, MODE_OFF, COLOR_YELLOW, MODE_OFF, COLOR_YELLOW },
};

AiCalibrationData aiCalibrationData[AI_CH_NUM] = {};

AiDataFlash aiDataFlash = {};

/**
  * @brief Инициализация модуля AI, чтение и проверка калибровочных данных из памяти.
  */
void aiInit( void )
{
	/* Флаг, что флешка в рабочем состоянии. */
	uint8_t flashValid = 1;
	/* Флаг, что данные прочитались верно. */
	uint8_t dataValid = 1;
	/* Счетчик попыток чтения из флешки. */
	uint16_t readAttemptCnt = 0;
	/* CRC данных из флешки. */
	uint32_t crc = 0;

	/* Регистрация колбэков. */
	registerCallback( aiSPITxRxCallback, SPI1_TX_RX_CPT );

	/* Чтение данных из памяти. */
	while ( ( aiReadCalibrationData() != FLASH_DONE ) )
	{
		/* Если достигли лимита на попытки чтения. */
		if (readAttemptCnt == FLASH_MAX_ATTEMPT_READ)
		{
			/* Обнуляем флаг, что флешка в рабочем состоянии. */
			flashValid = 0;
			break;
		}
		/* Увеличиваем счетчик ошибок. */
		readAttemptCnt++;
	};
	/* Если флешка в рабочем состоянии. */
	if ( flashValid )
	{
		/* Если есть CRC. */
		if ( ( aiDataFlash.crc != 0 ) || ( aiDataFlash.crc != 0xFF ) )
		{
			/* Сохраняем прочитанную CRC. */
			crc = aiDataFlash.crc;
			/* Обнуляем CRC в структуре для рассчета CRC без самой CRC. */
			aiDataFlash.crc = 0;
			/* Рассчитываем CRC по прочитанным данным. */
			uint32_t crcReadedData = HAL_CRC_Calculate(&hcrc, (uint32_t*)( (uint8_t*)&aiDataFlash ), sizeof(AiDataFlash) / sizeof(uint32_t) );
			/* Если CRC не совпадают. */
			if ( crc != crcReadedData )
			{
				/* Выставление индицаии ошибки CRC. */
				for ( uint8_t channel = 0; channel < AI_CH_NUM; channel++ )
				{
					aiDataLed[channel].colorWorking = COLOR_YELLOW;
					aiDataLed[channel].modeWorking = MODE_FLICK;

					aiDataLed[channel].color = COLOR_YELLOW;
					aiDataLed[channel].mode = MODE_FLICK;
				}
				/* Выставляем флаг, что данные не валидны. */
				dataValid = 0;
			}
		}
		else
		{
			/* CRC нет - выставляем индикацию неверной CRC. */
			for ( uint8_t channel = 0; channel < AI_CH_NUM; channel++ )
			{
				aiDataLed[channel].colorWorking = COLOR_YELLOW;
				aiDataLed[channel].modeWorking = MODE_BLINK;

				aiDataLed[channel].color = COLOR_YELLOW;
				aiDataLed[channel].mode = MODE_BLINK;
			}
			/* Выставляем флаг, что данные не валидны. */
			dataValid = 0;
		}
	}
	else
	{
		/* Флешка не в рабочем состоянии - выставляем индикацию ошибки флешки. */
		for ( uint8_t channel = 0; channel < AI_CH_NUM; channel++ )
		{
			aiDataLed[channel].colorWorking = COLOR_RED;
			aiDataLed[channel].modeWorking = MODE_BLINK;

			aiDataLed[channel].color = COLOR_RED;
			aiDataLed[channel].mode = MODE_BLINK;
		}
		/* Выставляем флаг, что данные не валидны. */
		dataValid = 0;
	}
	/* Если данные валидны. */
	if ( dataValid )
	{
		for ( uint8_t channel = 0; channel < AI_CH_NUM; channel++ )
		{
			/* Переносим коэфициенты. */
			aiData[channel].coefA = aiDataFlash.coefA[channel];
			aiData[channel].coefB = aiDataFlash.coefB[channel];
			aiData[channel].coefC = aiDataFlash.coefC[channel];
			/* Выставляем индикацию, что всё ОК. */
			aiDataLed[channel].colorWorking = COLOR_GREEN;
			aiDataLed[channel].modeWorking = MODE_ON;

			aiDataLed[channel].color = COLOR_GREEN;
			aiDataLed[channel].mode = MODE_ON;
		}
		/* Если вдруг размер массива под фильтрацию средним меньше 1, устанавливаем значение по умолчанию. */
		if ( aiDataFlash.filterAvgSize < 1 )
		{
			filterAvgSize = DEFAULT_FILTER_AVERAGE_SIZE;
		}
		else
		{
			filterAvgSize = aiDataFlash.filterAvgSize;
		}
		/* Переносим настройки экспонециальной фильтрации. */
		filterExpCurrent = aiDataFlash.filterExpCurrent;
	}
	else
	{
		/* Если данные битые, устанавлиаем коэфициенты и фильтры по умолчанию. */
		for ( uint8_t channel = 0; channel < AI_CH_NUM; channel++ )
		{
			aiData[channel].coefA = DEFAULT_CALIBRATION_COEF_A;
			aiData[channel].coefB = DEFAULT_CALIBRATION_COEF_B;
			aiData[channel].coefC = DEFAULT_CALIBRATION_COEF_C;
		}
		filterAvgSize = DEFAULT_FILTER_AVERAGE_SIZE;
		filterExpCurrent = DEFAULT_FILTER_EXP;
	}
	/* Выставляем флаг, что необходимо обновить индикацию. */
	isLedUpdate = 1;
	/* Обновляем индикацию. */
	updateLed();
};


/**
  * @brief Проверка и обновления режима работы блока AI.
  */
void aiUpdateMode( void )
{
	/* Если режим изменился. */
	if (userData.aiMode != aiMode)
	{
		/* Обнуляем флаги. */
		isSampleReceived = 0;
		isConfigSended = 0;
		/* Выставляем флаг обновления инидкации. */
		isLedUpdate = 1;
		/* Обновляем режим работы. */
		aiMode = userData.aiMode;

		if ( aiMode == AI_WORKING )
		{
			/* Если режим работы сменился на WORKING */
			/* Восстанавливаем последнюю сохраненную индикацию режима WORKING. */
			aiDataLed[0].color = aiDataLed[0].colorWorking;
			aiDataLed[1].color = aiDataLed[1].colorWorking;
			aiDataLed[2].color = aiDataLed[2].colorWorking;
			aiDataLed[3].color = aiDataLed[3].colorWorking;
			aiDataLed[4].color = aiDataLed[4].colorWorking;
			aiDataLed[5].color = aiDataLed[5].colorWorking;

			aiDataLed[0].mode = aiDataLed[0].modeWorking;
			aiDataLed[1].mode = aiDataLed[1].modeWorking;
			aiDataLed[2].mode = aiDataLed[2].modeWorking;
			aiDataLed[3].mode = aiDataLed[3].modeWorking;
			aiDataLed[4].mode = aiDataLed[4].modeWorking;
			aiDataLed[5].mode = aiDataLed[5].modeWorking;
		}
		else
		if ( aiMode == AI_CALIBRATION )
		{
			/* Если режим работы сменился на CALIBRATION. */
			/* Сбрасываем канал под калибровку. */
			calibrationCh = CALIBRATION_NO_CHANNEL;
			/* Сбрасываем флаг индикации. */
			isSaveLedEnabled = 0;
			/* Сбрасываем индикацию. */
			aiDataLed[0].color = COLOR_GREEN;
			aiDataLed[1].color = COLOR_GREEN;
			aiDataLed[2].color = COLOR_GREEN;
			aiDataLed[3].color = COLOR_GREEN;
			aiDataLed[4].color = COLOR_GREEN;
			aiDataLed[5].color = COLOR_GREEN;

			aiDataLed[0].mode = MODE_OFF;
			aiDataLed[1].mode = MODE_OFF;
			aiDataLed[2].mode = MODE_OFF;
			aiDataLed[3].mode = MODE_OFF;
			aiDataLed[4].mode = MODE_OFF;
			aiDataLed[5].mode = MODE_OFF;
		}
	}
}

/**
  * @brief Обновление индикации каналов.
  */
void updateLed( void )
{
	/* Если индикацию необходимо обновить. */
	if ( isLedUpdate )
	{
		/* Обнуляем флаг необходимости обновления индикации. */
		isLedUpdate = 0;
		/* Обновляем индикацию каналов. */
		setLedMode(aiDataLed[0].type, aiDataLed[0].mode, aiDataLed[0].color);
		setLedMode(aiDataLed[1].type, aiDataLed[1].mode, aiDataLed[1].color);
		setLedMode(aiDataLed[2].type, aiDataLed[2].mode, aiDataLed[2].color);
		setLedMode(aiDataLed[3].type, aiDataLed[3].mode, aiDataLed[3].color);
		setLedMode(aiDataLed[4].type, aiDataLed[4].mode, aiDataLed[4].color);
		setLedMode(aiDataLed[5].type, aiDataLed[5].mode, aiDataLed[5].color);
	}
}

/**
  * @brief Сбрасывание всех каналов (массивов среднего, значений среднего, медианной фильтрации).
  */
void aiReset( void )
{
	for ( uint8_t ch = 0; ch < AI_CH_NUM; ch++ )
	{
		/* Сбрасываем медианный фильтр. */
		aiData[ch].medianCurrentPos = 0;
		memset( aiData[ch].medianCurrentArr, 0, SIZE_ARRAY_MEDIAN * sizeof(uint16_t) );
		/* Сбрасываем средне-скользящий фильтр. */
		memset( aiData[ch].avgCurrentArr, 0, SIZE_ARRAY_AVERAGE * sizeof(uint16_t) );
		aiData[ch].avgCurrentPos = 0;
		/* Сбрасываем значения тока. */
		aiData[ch].current = 0;
		aiData[ch].currentFull = 0;
		aiData[ch].avgCurrent = 0;
	}
}

/**
  * @brief Главная точка работы блока AI.
  */
void aiProcess( void )
{
	/* Проверяем, не изменился ли режим работы блока AI. */
	aiUpdateMode();

	if ( aiMode == AI_WORKING )
	{
		/* Если режим WORKING. */
		aiWorking();
	}
	else
	if ( aiMode == AI_CALIBRATION)
	{
		/* Если режим CALIBRATION. */
		if (userData.calibrationMode == CALIBRATION_SAMPLING)
		{
			/* Высчитываем калибрацию для канала под значение тока. */
			aiCalibrationMa(userData.calibrationCh, userData.calibrationMa);
		}
		else
		if (userData.calibrationMode == CALIBRATION_WAIT)
		{
			/* Режим ожидания команды. */
			aiWaitCalibration();
		}
		else
		if (userData.calibrationMode == CALIBRATION_CALC)
		{
			/* Режим высчитывания коэфициентов */
			aiCalcCalibration(userData.calibrationCh);
		}
		else
		if (userData.calibrationMode == CALIBRATION_SAVE)
		{
			/* Режим сохранения значений. */
			aiCalibrationSaveData();
		}
	}
	updateLed();
};

/**
  * @brief WORKING режим, вычисление значений тока.
  */
void aiWorking( void )
{
	/* Текущий канал (по порядку, не по маппингу). */
	static uint8_t channel = 0;
	/* Медианное значение тока. */
	static uint16_t medianCurrent = 0;
	/* Среднее значение тока. */
	static double avgCurrent = 0;

	/* Если запрос на получение выборки к АЦП отправлен. */
	if ( isConfigSended )
	{
		/* Если выборка с АЦП получена. */
		if ( isSampleReceived )
		{
			/* Поднимаем чип-селект. */
			HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
			/* Очищаем флаги. */
			isSampleReceived = 0;
			isConfigSended = 0;
			/* Считаем медианное по сырой выборки из АЦП. */
			medianCurrent = calcMedian( ( aiData[channel].sample[0] << 8 ) | aiData[channel].sample[1], aiData[channel].medianCurrentArr, &aiData[channel].medianCurrentPos );
			/* Считаем среднее по сырым выборкам канала. */
			avgCurrent = calcAverage( medianCurrent, aiData[channel].avgCurrentArr, &aiData[channel].avgCurrent, &aiData[channel].avgCurrentPos );
			/* Приводим выборку к идеальной выборке. */
			avgCurrent = aiData[channel].coefA * (avgCurrent * avgCurrent) + ( avgCurrent * aiData[channel].coefB ) + aiData[channel].coefC;
			/* Рассчитываем ток по приведенной выборке. */
			avgCurrent = LOWER_SAMPLE_BIAS + (avgCurrent - ADC_IDEAL_MA4) * CURRENT_STEP;
			/* Фильтруем экспонециальным фильтром и обновляем значение тока. */
			aiData[channel].currentFull += (avgCurrent - aiData[channel].currentFull) * filterExpCurrent;
			aiData[channel].current = aiData[channel].currentFull;

			if ( (aiData[channel].current > CURRENT_FALL_VALUE && aiData[channel].current < CURRENT_KZ_VALUE ) && !aiData[channel].isOK )
			{
				aiDataLed[channel].color = COLOR_GREEN;
				aiDataLed[channel].mode = MODE_ON;
				aiDataLed[channel].colorWorking = COLOR_GREEN;
				aiDataLed[channel].modeWorking = MODE_ON;
				aiData[channel].isFall = 0;
				aiData[channel].isKZ = 0;
				aiData[channel].isOK = 1;

				isLedUpdate = 1;
			}
			else
			if ( aiData[channel].current < CURRENT_FALL_VALUE && !aiData[channel].isFall )
			{
				aiDataLed[channel].color = COLOR_YELLOW;
				aiDataLed[channel].mode = MODE_ON;
				aiDataLed[channel].colorWorking = COLOR_YELLOW;
				aiDataLed[channel].modeWorking = MODE_ON;
				aiData[channel].isFall = 1;
				aiData[channel].isKZ = 0;
				aiData[channel].isOK = 0;

				isLedUpdate = 1;
			}
			else
			if ( aiData[channel].current > CURRENT_KZ_VALUE && !aiData[channel].isKZ )
			{
				aiDataLed[channel].color = COLOR_RED;
				aiDataLed[channel].mode = MODE_ON;
				aiDataLed[channel].colorWorking = COLOR_RED;
				aiDataLed[channel].modeWorking = MODE_ON;
				aiData[channel].isFall = 0;
				aiData[channel].isKZ = 1;
				aiData[channel].isOK = 0;

				isLedUpdate = 1;
			}


			/* Если текущий канал АЦП последний. */
			if ( channel == 5 )
			{
				/* Обнуляем счетчик каналов. */
				channel = 0;
			}
			else
			{
				/* Инкрементируем счетчик каналов. */
				channel++;
			}
		}
		else
		{
			/* Если выборка с АЦП не получена. */
			return;
		}

	}
	else
	{
		/* Если запрос не отправлен. */
		/* Выставляем флаг, что запрос к АЦП отправлен. */
		isConfigSended = 1;
		/* Опускаем чип-селект АЦП. */
		HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
		/* Отправляем конфиг канала, получаем данные в канал по маппингу. */
		HAL_SPI_TransmitReceive_DMA(&hspi1, aiData[channel].config, aiData[aiData[channel].realCh].sample, 2);
	}
}

/**
  * @brief  Режим ожидания команды под калибрацию.
  */
void aiWaitCalibration( void )
{
	/* Если канал изменился. */
	if ( calibrationCh != userData.calibrationCh )
	{
		/* Обновляем локальную переменную канала. */
		calibrationCh = userData.calibrationCh;
		/* Если канал задан неверно. */
		if ( ( calibrationCh < 0 ) || ( calibrationCh > ( AI_CH_NUM - 1 ) ) )
		{
			/* Включаем аварийную индикацю. */
			for ( uint8_t ch = 0; ch < AI_CH_NUM; ch++ )
			{
				aiDataLed[ch].color = COLOR_RED;
				aiDataLed[ch].mode = MODE_SINGLE;
			}
			isLedUpdate = 1;
		}
		else
		{
			/* Выключаем индикацию каналов. */
			for ( uint8_t ch = 0; ch < AI_CH_NUM; ch++ )
			{
				aiDataLed[ch].color = COLOR_GREEN;
				aiDataLed[ch].mode = MODE_OFF;
			}
			/* Подсвечиваем рабочий канал. */
			aiDataLed[calibrationCh].color = COLOR_GREEN;
			aiDataLed[calibrationCh].mode = MODE_BLINK;
			isLedUpdate = 1;
		}
	}
	/* Если изменилось значение размера массива под фильтрацию средним. */
	if ( filterAvgSize != userData.filterAvgSize )
	{
		/* Проверяем на допустимый диапазон. */
		if ( ( userData.filterAvgSize > 1 ) && ( userData.filterAvgSize < SIZE_ARRAY_AVERAGE ) )
		{
			/* Обновляем значение размера массива среднего под фильтрацию. */
			filterAvgSize = userData.filterAvgSize;
			/* Сбрасываем значения каналов. */
			aiReset();
		}
	}
	/* Если изменилось значение шага фильтрацией экспонентой. */
	if ( filterExpCurrent != userData.filterExpCurrent )
	{
		/* Если шаг больше 0 и меньше 1. */
		if ( ( userData.filterExpCurrent > 0 ) && ( userData.filterExpCurrent <= 1 ) )
		{
			/* Обновляем шаг фильтра. */
			filterExpCurrent = userData.filterExpCurrent;
			/* Сбрасываем значения каналов. */
			aiReset();
		}
	}
}

/**
  * @brief  Вычисление инкрементального скользящего среднего в массиве.
  * @param  sample:		новая выборка АЦП.
  * @param  ptrToArray: указатель на массив среднего.
  * @param  ptrToSum:	указатель на сумму среднего.
  * @param  ptrToPos:	указатель на актуальную позицию в массиве среднего.
  * @retval вычисленное среднее.
  */
double calcAverage( uint16_t sample, uint16_t* ptrToArray, uint32_t* ptrToSum, uint8_t* ptrToPos )
{
	/* Если мы достигли края конца массива под среднее. */
	if (++(*ptrToPos) == filterAvgSize)
	{
		/* Обнуляем переменную позиции. */
		*ptrToPos = 0;
	}
	/* Вычитаем предыдущие значение выборки для вычисления скользящего среднего. */
	*ptrToSum -= ptrToArray[*ptrToPos];
	/* Добавляем текущее значение выборки для вычисления скользящего среднего. */
	*ptrToSum += sample;
	/* Меняем предыдущее значение выборки на текущее. */
	ptrToArray[*ptrToPos] = sample;
	/* Новое значение тока по скользящему среднему. */
	return (*ptrToSum) / filterAvgSize;
}

/**
  * @brief  Вычисление медианного из трех значений.
  * @param  sample:		новая выборка АЦП.
  * @param  ptrToArray: указатель на массив медианного.
  * @param  ptrToPos:	указатель на актуальную позицию в массиве медианного.
  * @retval вычисленное медианное.
  */
uint16_t calcMedian( uint16_t sample, uint16_t* ptrToArray, uint8_t* ptrToPos )
{
	/* Добавляем новую выборку вместо последней старой. */
	ptrToArray[*ptrToPos] = sample;
	/* Если мы достигли конца массива под медианное. */
	if (*ptrToPos == 2)
	{
		/* Обнуляем счетчик позиции. */
		*ptrToPos = 0;
	}
	else
	{
		/* Инкрементируем счетчик позиции. */
		(*ptrToPos)++;
	}
	/* Не спрашивайте... */
	return
		(ptrToArray[0] < ptrToArray[1])
		? ((ptrToArray[1] < ptrToArray[2])
		? ptrToArray[1] : ((ptrToArray[2] < ptrToArray[0])
		? ptrToArray[0] : ptrToArray[2])) : ((ptrToArray[0] < ptrToArray[2])
		? ptrToArray[0] : ((ptrToArray[2] < ptrToArray[1])
		? ptrToArray[1] : ptrToArray[2]));
}

/**
  * @brief Калибровка тока.
  * @param  ch:		номер канала, который калибруется.
  * @param  ma:		опорное значение для калибровки.
  */
void aiCalibrationMa( uint8_t ch, uint16_t ma )
{
	/* Началась ли калибровка. */
	static uint8_t isStarted = 0;
	/* Счетчик полученных выборок для калибрации. */
	static uint32_t calibrationCnt = 0;

	/* Если введено неверное значение ma. */
	if ( ( ma < MA4 ) || (ma > MA20) )
	{
		userData.calibrationMode = CALIBRATION_WAIT;
		return;
	}

	/* Если введено неверное значение канала. */
	if ( ( ch < 0 ) || (ch > ( AI_CH_NUM - 1) ) )
	{
		userData.calibrationMode = CALIBRATION_WAIT;
		return;
	}

	/* Если калибрация еще не запустилась. */
	if ( !isStarted )
	{
		/* Выставляем флаг, что калибрация началась. */
		isStarted = 1;
		/* Обнуляем массив медианного. */
		memset(aiData[ch].medianCurrentArr, 0, 6);
		/* Обнуляем позицию медианного. */
		aiData[ch].medianCurrentPos = 0;
		/* Выставляем индикацию канала. */
		aiDataLed[ch].color = COLOR_YELLOW;
		aiDataLed[ch].mode = MODE_FLICK;
		isLedUpdate = 1;
	}

	if ( isConfigSended )
	{
		if ( isSampleReceived )
		{
			/* Поднимаем чип-селект. */
			HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
			/* Очищаем флаги. */
			isSampleReceived = 0;
			isConfigSended = 0;
			/* Проверка, прошли ли выборки, которые не учитываются. */
			if ( calibrationCnt < NUM_NOT_TAKEN_SAMPLE_CALIBRATION )
			{
				return;
			}
			/* Прибавляем к сумме калибрации медианное. */
			aiCalibrationData[ch].sumCalibration += calcMedian( (aiCalibrationData[ch].sample[0] << 8 ) | aiCalibrationData[ch].sample[1], aiData[ch].medianCurrentArr, &aiData[ch].medianCurrentPos);
			/* Если серднее вычислено полностью. */
			if (calibrationCnt == (SIZE_ARR_CALIBRATION + NUM_NOT_TAKEN_SAMPLE_CALIBRATION) - 1 )
			{
				/* Вычисляем среднее выборок. */
				aiCalibrationData[ch].adcValue[ma - 4] = (double) aiCalibrationData[ch].sumCalibration / SIZE_ARR_CALIBRATION;
				/* Обнуляем сумму для среднего. */
				aiCalibrationData[ch].sumCalibration = 0;
				/* Обнуляем счетчик полученных выборок. */
				calibrationCnt = 0;
				/* Обнуляем флаг, что калибрация работает. */
				isStarted = 0;
				/* Обнуляем массив медианного. */
				memset(aiData[ch].medianCurrentArr, 0, 6);
				/* Обнуляем позицию медианного. */
				aiData[ch].medianCurrentPos = 0;

				aiDataLed[ch].color = COLOR_GREEN;
				aiDataLed[ch].mode = MODE_BLINK;
				isLedUpdate = 1;

				/* Переходим в режим ожидания. */
				userData.calibrationMode = CALIBRATION_WAIT;
			}
		}
		else
		{
			return;
		}
	}
	else
	{
		/* Выставляем флаг, что запрос к АЦП отправлен. */
		isConfigSended = 1;
		/* Увеличиваем счетчик выборок калибрации. */
		calibrationCnt++;
		HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
		/* Отправляем конфигурацию канала. */
		HAL_SPI_TransmitReceive_DMA(&hspi1, aiData[ch].config, aiCalibrationData[ch].sample, 2);
		return;
	}
}

void aiCalcCalibration( uint8_t channel )
{
	/* Если введено неверное значение канала. */
	if ( ( channel < 0 ) || ( channel > ( AI_CH_NUM - 1 ) ) )
	{
		userData.calibrationMode = CALIBRATION_WAIT;
		return;
	}
	/* Массив с идеальными значениям выборок АЦП. */
	static const uint16_t idealADCValue[] =
	{
		ADC_IDEAL_MA4,
		ADC_IDEAL_MA5,
		ADC_IDEAL_MA6,
		ADC_IDEAL_MA7,
		ADC_IDEAL_MA8,
		ADC_IDEAL_MA9,
		ADC_IDEAL_MA10,
		ADC_IDEAL_MA11,
		ADC_IDEAL_MA12,
		ADC_IDEAL_MA13,
		ADC_IDEAL_MA14,
		ADC_IDEAL_MA15,
		ADC_IDEAL_MA16,
		ADC_IDEAL_MA17,
		ADC_IDEAL_MA18,
		ADC_IDEAL_MA19,
		ADC_IDEAL_MA20,
	};

	double sum_x = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0;
	double sum_y = 0, sum_xy = 0, sum_x2y = 0;

	for (uint8_t i = 0; i < SIZE_ARRAY_RANGE_MA; i++) {
		sum_x += aiCalibrationData[channel].adcValue[i];
		sum_x2 += (double) aiCalibrationData[channel].adcValue[i] * aiCalibrationData[channel].adcValue[i];
		sum_x3 += (double) aiCalibrationData[channel].adcValue[i] * aiCalibrationData[channel].adcValue[i] * aiCalibrationData[channel].adcValue[i];
		sum_x4 += (double) aiCalibrationData[channel].adcValue[i] * aiCalibrationData[channel].adcValue[i] * aiCalibrationData[channel].adcValue[i] * aiCalibrationData[channel].adcValue[i];
		sum_y += (double) idealADCValue[i];
		sum_xy += (double) aiCalibrationData[channel].adcValue[i] * idealADCValue[i];
		sum_x2y += (double) aiCalibrationData[channel].adcValue[i] * aiCalibrationData[channel].adcValue[i] * idealADCValue[i];
	}

	/* Матрица коэффициентов для системы уравнений. */
	double A = sum_x4, B = sum_x3, C = sum_x2;
	double D = sum_x3, E = sum_x2, F = sum_x;
	double G = sum_x2, H = sum_x, I = SIZE_ARRAY_RANGE_MA;

	/* Правая часть. */
	double J = sum_x2y, K = sum_xy, L = sum_y;

	/* Вычисляем определитель */
    double denominator = A * (E * I - H * F) - B * (D * I - G * F) + C * (D * H - E * G);

	/* Проверка на нулевой определитель (система не может быть решена). */
	if (denominator == 0)
	{
		/* Выставляем индикацию ошибки вычисления коэфициентов. */
		for ( uint8_t channel = 0; channel < AI_CH_NUM; channel++ )
		{
			aiDataLed[channel].color = COLOR_RED;
			aiDataLed[channel].mode = MODE_ON;
		}
		isLedUpdate = 1;
		userData.calibrationMode = CALIBRATION_WAIT;
		return;
	}

	/* Вычисляем коэфициенты. */
	aiDataFlash.coefA[channel] = (J * (E * I - H * F) - K * (D * I - G * F) + L * (D * H - E * G)) / denominator;
	aiDataFlash.coefB[channel] = (A * (K * I - H * L) - B * (J * I - G * L) + C * (J * H - K * G)) / denominator;
	aiDataFlash.coefC[channel] = (A * (E * L - K * F) - B * (D * L - J * F) + C * (D * K - E * J)) / denominator;
	/* Переносим коэфициенты в структуру AI, чтобы результат калибровки можно было увидеть сразу. */
	aiData[channel].coefA = aiDataFlash.coefA[channel];
	aiData[channel].coefB = aiDataFlash.coefB[channel];
	aiData[channel].coefC = aiDataFlash.coefC[channel];
	/* Сбрасываем значения каналов. */
	aiReset();
	userData.calibrationMode = CALIBRATION_WAIT;
}

/**
  * @brief  Получение значений из flash.
  * @retval Статус flash.
  */
FLASH_STATUS aiReadCalibrationData( void )
{
	static uint8_t operation = FLASH_READ;
	/* Адрес памяти, с которым ведется работа. */
	static uint32_t address = FLASH_CALIBRATION_ADR;

	/* Если флешка занята. */
	if (flashGetStatus() == FLASH_BUSY)
	{
		return FLASH_AT_WORK;
	}
	/* Если операция = чтение данных. */
	if ( operation == FLASH_READ )
	{
		/* Отправляем команду чтения. */
		flashReadData( address, sizeof(AiDataFlash) );
		operation = FLASH_GET_READED;
		return FLASH_AT_WORK;
	}
	/* Если операция = получить запрошенные данные. */
	if ( operation == FLASH_GET_READED )
	{
		/* Получаем данные. */
		flashGetReadedData((uint8_t*) &aiDataFlash);
		operation = FLASH_DEFAULT;
	}
	if ( operation == FLASH_DEFAULT )
	{
		operation = FLASH_READ;
		address = 0x0;
		return FLASH_DONE;
	}

	return FLASH_AT_WORK;
};

/**
  * @brief Сохранение коэфициентов и настроек фильтрации на flash.
  */
void aiCalibrationSaveData( void )
{
	/* Следующая операция с флешкой. */
	static uint8_t operation = FLASH_ERASE_SECTOR;
	/* Адрес, куда записывать данные. */
	static uint32_t address = FLASH_CALIBRATION_ADR;
	/* Статус флешки. */
	static uint8_t statusFlash = FLASH_BUSY;
	/* CRC. */
	static uint32_t crc = 0;

	/* Если индикация записи в флешку не была включена. */
	if ( !isSaveLedEnabled )
	{
		/* Включаем индикацию записи. */
		for ( uint8_t ch = 0; ch < AI_CH_NUM; ch++ )
		{
			aiDataLed[ch].color = COLOR_YELLOW;
			aiDataLed[ch].mode = MODE_FLICK;
		}
		isLedUpdate = 1;
		isSaveLedEnabled = 1;
	}

	/* Обнуляем CRC. */
	aiDataFlash.crc = 0;
	/* Сохраняем значения фильтрации. */
	aiDataFlash.filterAvgSize = filterAvgSize;
	aiDataFlash.filterExpCurrent = filterExpCurrent;

	/* Получаем байтовый указатель на сохраняемую структуру. */
	uint8_t* ptrToDataFlash = (uint8_t*)&aiDataFlash;
	/* Считаем crc. */
	crc = HAL_CRC_Calculate( &hcrc, (uint32_t*)ptrToDataFlash, sizeof(AiDataFlash)/sizeof(uint32_t) );
	/* Записывам crc в структуру. */
	aiDataFlash.crc = crc;

	/* Получаем статус флешки. */
	statusFlash = flashGetStatus();
	/* Если флешка занята. */
	if (statusFlash == FLASH_BUSY)
	{
		return;
	}
	/* Если флешка свободна на чтение. */
	if (statusFlash == FLASH_FREE_R)
	{
		flashSetWriteMode();
	}
	/* Если флешка свободна на чтение и запись. */
	if (statusFlash == FLASH_FREE_RW)
	{
		/* Если операция = очистить сектор. */
		if ( operation == FLASH_ERASE_SECTOR )
		{
			/* Очищаем сектор. */
			flashEraseSector(address);
			operation = FLASH_SAVE;
			return;
		}
		/* Если операция = сохранить данные на флешку. */
		if ( operation == FLASH_SAVE )
		{
			/* Записываем данные на флешку. */
			flashWriteData( address, ptrToDataFlash, sizeof(AiDataFlash) );
			operation = FLASH_DEFAULT;
			return;
		}
		/* Если операция = сбросить состояния к изначальному. */
		if ( operation == FLASH_DEFAULT )
		{
			/* Возвращаем адрес к исходному. */
			address = FLASH_CALIBRATION_ADR;
			/* Возвращаем операцию к исходной. */
			operation = FLASH_ERASE_SECTOR;
			/* Сбрасываем режим калибрации в wait. */
			userData.calibrationMode = CALIBRATION_WAIT;
			/* Меняем канал калибрации на по умолчанию. */
			calibrationCh = CALIBRATION_NO_CHANNEL;
			/* Сбрасываем флаг включенной индикации записи во флешку. */
			isSaveLedEnabled = 1;
			isLedUpdate = 1;
			/* Обнуляем crc. */
			crc = 0;
			return;
		}
	}
}


void aiSPITxRxCallback( void )
{
	/* Выставляем флаг, что данные с АЦП приняты. */
	isSampleReceived = 1;
}
