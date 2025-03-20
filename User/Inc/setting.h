#ifndef INC_SETTING_H_
#define INC_SETTING_H_

/* ________________________ LED ________________________ */
/* Адрес микросхемы под индикацию. */
#define XL9535_I2C_ADDRESS 					0x42
/* Задержка перед отправкой состояния каналов индикации на микросхему. */
#define LED_DELAY_TRANSMIT 					10
/* Оффсет диодов на расширители портов. */
#define LED_I2C_OFFSET						3

/* ________________________ CALIBRATION ________________________ */
/* Размер массива под калибрацию. */
#define SIZE_ARR_CALIBRATION 				100000
/* Кол-во неучитывающихся выборок при калибрации. */
#define NUM_NOT_TAKEN_SAMPLE_CALIBRATION 	10
/* Максимальный азмер массива под фильтрацию средним тока. */
#define SIZE_ARRAY_AVERAGE					200
/* Размер массива под медианную фильтрацию тока. */
#define SIZE_ARRAY_MEDIAN					3
/* Кол-во точке калибровок. */
#define SIZE_ARRAY_RANGE_MA					17
/* Значение калибровочного канала, когда канал не выбран. */
#define CALIBRATION_NO_CHANNEL				0xFF
/* Значение по умолчанию для коэфициента А. */
#define DEFAULT_CALIBRATION_COEF_A			-4.180287149356397e-008
/* Значение по умолчанию для коэфициента B. */
#define DEFAULT_CALIBRATION_COEF_B			1.1461199229165544
/* Значение по умолчанию для коэфициента C. */
#define DEFAULT_CALIBRATION_COEF_C			43.661028945484645

/* ________________________ VALUE OF IDEAL ADC ________________________ */
#define ADC_IDEAL_MA4						16352
#define ADC_IDEAL_MA5						18160
#define ADC_IDEAL_MA6						19984
#define ADC_IDEAL_MA7						21808
#define ADC_IDEAL_MA8						23616
#define ADC_IDEAL_MA9						25440
#define ADC_IDEAL_MA10						27248
#define ADC_IDEAL_MA11						29072
#define ADC_IDEAL_MA12						30880
#define ADC_IDEAL_MA13						32704
#define ADC_IDEAL_MA14						34528
#define ADC_IDEAL_MA15						36336
#define ADC_IDEAL_MA16						38160
#define ADC_IDEAL_MA17						39968
#define ADC_IDEAL_MA18						41792
#define ADC_IDEAL_MA19						43600
#define ADC_IDEAL_MA20						45424

/* ________________________ AI ________________________ */
/* Кол-во каналов. */
#define AI_CH_NUM							6
/* Значение тока при КЗ. */
#define CURRENT_KZ_VALUE					20500
/* Значение тока при обрыве линии. */
#define CURRENT_FALL_VALUE					3600
/* Ток в 4 милиампера. */
#define MA4									4
/* Ток в 20 милиамперов. */
#define MA20								20
/* Шаг тока при идеальных значениях АЦП. */
#define CURRENT_STEP						0.55035773252614199229
/* Нижняя граница тока 4-20(милиампер) в микроамперах. */
#define LOWER_SAMPLE_BIAS					4000

/* ________________________ FILTER ________________________ */
/* Значение экспонециального фильтра по умолчанию. */
#define DEFAULT_FILTER_EXP					0.1
/* Значение фильтра под скользящее среднее по умолчанию. */
#define DEFAULT_FILTER_AVERAGE_SIZE			30

/* ________________________ HART ________________________ */
#define SIZE_HART_BUFF						284

/* ________________________ FLASH ________________________ */
/* Размер буферов флешки: 256 - данные + 4 - команда = 260 байт. */
#define SIZE_FLASH_BUFFER					260
/* Максимальное количество обращений к флешке до ошибки чтения. */
#define FLASH_MAX_ATTEMPT_READ				10000
/* Адрес, с которого берутся калибровочные данные с флешки. */
#define FLASH_CALIBRATION_ADR				0x00



#endif
