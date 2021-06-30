#ifndef MEM_DEVICE_H__
#define MEM_DEVICE_H__

#include <stdint.h>
//#include "main.h"


//*************************************************************
//Параметры прибора по умолчанию
//*************************************************************
#define name_device				"SIAM MODEMv2\0"
#define ver_device				"1.0.42nRf\0"
#define DEVICE					0x1401
#define MEM_MODEL				0x000c 
#define DEVICE_NUM				1

#define DEFAULT_LOAD_NULL		1300.
#define DEFAULT_LOAD_FACTOR		0.15
#define DEFAULT_ACCEL_M_G		900.
#define DEFAULT_ACCEL_NULL		1500.
#define DEFAULT_ACCEL_P_G		2100.
#define DEFAULT_TEMP_NULL		277.746
#define DEFAULT_TEMP_FACTOR		0.17193

#define DEFAULT_ONTIMER			0
#define DEFAULT_FLIDLE			1
#define DEFAULT_IDLETIMER		180

//*************************************************************
//Флаги управлени прибором
//*************************************************************
typedef enum
{
	DEV_NULL=0
	,DEV_START						//запуск измерения
	,DEV_INIT						//инициализация
	,DEV_Ctrl_3
	,DEV_Ctrl_4
	,DEV_OFF						// выключение датчика
} CtrlRegState;

//*************************************************************
//Флаги статуса прибора
//*************************************************************
typedef enum
{
	DEV_EMPTY=0						// датчик свободен
	,DEV_BUSY						// старт измерения
	,DEV_CALC						// измерение завершено, идет расчет
	,DEV_State_3
	,DEV_READY						//исследование завершено, но не экспортировано
	,DEV_ERROR						//исследование завершено с ошибкой, но не экспортировано
	,DEV_PREPAR						//подготовка оптической системы
	,DEV_VALVETEST					//тест клапанов
} StateRegState;

//*************************************************************
//Флаги ошибок измерения прибора
//*************************************************************
typedef enum
{
	ERR_ADC=0
	,ERR_ALIGN_VEL
	,ERR_ALIGN_POS
	,ERR_PERIOD
	,ERR_TRAVEL_MIN
	,ERR_TRAVEL_MAX
	,ERR_PERIOD_MIN
	,ERR_PERIOD_MAX
	,ERR_GATE_PERIOD
} ErrorRegState;

//*************************************************************
//1.	Общие регистры(0x00000000)
//*************************************************************
#define DeviceReg_ADDR				0x00000000
typedef struct {
	uint16_t device;
	uint16_t mem_model;
	uint32_t name_addr;
	uint16_t name_size;
	uint32_t device_num;
} __attribute__((aligned(2),packed)) DeviceReg;

//*************************************************************
//1.1.	имя прибора
//*************************************************************
#define NAME_ADDR					0x00000400
typedef struct {
	char Name[30];
} __attribute__((aligned(2),packed)) NameBlock;

//*************************************************************
//2.	Информационные регистры(0x00001000)(r)
//*************************************************************
#define InfoReg_ADDR				0x00001000
typedef struct {
	uint32_t ver_addr;
	uint16_t ver_size;
} __attribute__((aligned(2),packed)) InfoReg;

//*************************************************************
//2.1.	версия прошивки
//*************************************************************
#define VER_ADDR					0x00000500
typedef struct {
	char Ver[30];
} __attribute__((aligned(2),packed)) VerBlock;

//*************************************************************
//3.	 Параметры измерения(0x00008000)(r/w)
//*************************************************************
#define CURRENT_PARAM_ADDR			0x00008000
typedef struct{
	uint16_t Rod;
	uint32_t DynPeriod;
	uint16_t ApertNumber;
	uint16_t Imtravel;
	uint16_t ModelPump;
}__attribute__((aligned(2),packed)) CURRENT_PARAM;

//*************************************************************
//4.	Энергонезависимые параметры(0x00008100)
//*************************************************************
#define StaticParamReg_ADDR			0x00008100
typedef struct{
	float 		LoadNull;
	float 		LoadFactor;
	float 		AccelNull;
	float 		AccelPg;
	uint32_t 	TimeON;
	float 		TempNull;
	float 		TempFactor;
	float 		AccelMg;
	uint16_t	BlockAutoOFF;
	uint16_t	TimeOFF;
}__attribute__((aligned(2),packed)) StaticParamReg;

//*************************************************************
//5.	Текущие параметры(0x00008400)
//*************************************************************
#define CurrParamReg_ADDR			0x00008400
typedef struct{
	uint16_t 	Acc;
	int16_t		Temp;
	float Load;
	float Accel;
}__attribute__((aligned(2),packed)) CurrParamReg;

//*************************************************************
//6.	Операционные регистры(0x00008800)
//*************************************************************
#define OperReg_ADDR				0x00008800
typedef struct{
	uint16_t	CtrlReg;
	uint16_t	StatReg;
	uint8_t		ErrorReg;	
}__attribute__((aligned(2),packed)) OperReg;

//*************************************************************
//7.	Операционные регистры программирования(0x00008900)
//*************************************************************
#define ProgramReg_ADDR				0x00008900
typedef struct{
	uint8_t		UpgradeInit[16];
	uint16_t	UpgradeStart;
}__attribute__((aligned(2),packed)) ProgramReg;

//*************************************************************
//8.	Формат отчета(0x80000000)
//*************************************************************
#define DYN_REPORT_ADDR				0x80000000
typedef struct{
	uint16_t	max_weight;
	uint16_t	min_weight;
	uint16_t	travel;
	uint16_t	period;
	uint16_t	step;
	uint16_t	weight_discr;
	uint16_t	time_discr;
}__attribute__((aligned(2),packed)) DYN_REPORT;

//*************************************************************
//9.	Формат графика динамограммы (0x81000000)
//*************************************************************
#define DYN_FILE_ADDR				0x81000000
typedef struct{
	uint8_t	din_file_buf[2000];
}__attribute__((aligned(2),packed)) DYN_FILE;

//*************************************************************
//10.	Формат графика ускорения(0x83000000)
//*************************************************************
#define ACCEL_FILE_ADDR				0x83000000
typedef struct{
	uint8_t	accel_file_buf[4000];
}__attribute__((aligned(2),packed)) ACCEL_FILE;

//*************************************************************
//11.	С адреса 0xFF000000 – память программ (буфер) 
//*************************************************************
#define ProgramBuf_ADDR				0xFF000000
typedef struct{
	uint8_t	programm[49151];
}__attribute__((aligned(2),packed)) ProgramBuf;
//*************************************************************
typedef struct{
	DeviceReg				mDeviceReg;
	NameBlock				gNameBlock;
	InfoReg					mInfoReg;
	VerBlock				gVerBlock;
	CurrParamReg			mCurrParamReg;
	OperReg					gOperReg;
	StaticParamReg			mStaticParamReg;
	CURRENT_PARAM			mCurrentParam;
}__attribute__((aligned(2),packed)) DeviceHoldingReg;
extern DeviceHoldingReg		gDeviceHoldingReg;
//*************************************************************
typedef struct{
	uint32_t				Num;
	StaticParamReg			mStaticParamReg;
}__attribute__((aligned(4),packed)) DeviceParam;
//*************************************************************

extern DYN_REPORT			mDYN_REPORT;
extern DYN_FILE				mDYN_FILE;
extern ACCEL_FILE			mACCEL_FILE;

extern uint8_t flag_save;

void mem_device_init(void);
void Save_Param(void);
#endif
