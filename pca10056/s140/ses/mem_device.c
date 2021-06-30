#include "mem_device.h"
//#include "main.h"
#include "ProtokolSiam.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\mem_application.h"

DYN_REPORT mDYN_REPORT = {
	3700,
	1200,
	4143,
	10,
	10,
	1,
	1000
};

DeviceParam defalt_DeviceParam = {
	DEVICE_NUM,						//номер прибора
	DEFAULT_LOAD_NULL,				//НКП нагрузки, мВ
	DEFAULT_LOAD_FACTOR,			//РКП нагрузки, мВ/кг
	DEFAULT_ACCEL_NULL,				//0g акселерометра, мВ
	DEFAULT_ACCEL_P_G,				// 1g акселерометра, мВ
	DEFAULT_ONTIMER,				//интервал включений
	DEFAULT_TEMP_NULL,				//смещение нуля, гр. С
	DEFAULT_TEMP_FACTOR,			// гр.С/дискрета
	DEFAULT_ACCEL_M_G,				//-1g акселерометра, мВ	
	DEFAULT_FLIDLE,					//флаг запрета выключения по времени
	DEFAULT_IDLETIMER,				//таймер выключения по времени, сек
};

DYN_FILE mDYN_FILE;
ACCEL_FILE mACCEL_FILE;

DeviceHoldingReg		gDeviceHoldingReg;

uint8_t flag_save = 0;

/*void mem_device_init(void)
{
	LoadConfig();

	memset(&gDeviceHoldingReg, 0, sizeof(gDeviceHoldingReg));

	memcpy(&gDeviceHoldingReg.gNameBlock.Name, name_device, strlen(name_device));
	memcpy(&gDeviceHoldingReg.gVerBlock.Ver, ver_device, strlen(ver_device));

	gDeviceHoldingReg.mDeviceReg.device				= DEVICE;
	gDeviceHoldingReg.mDeviceReg.mem_model			= MEM_MODEL;
	gDeviceHoldingReg.mDeviceReg.name_addr			= NAME_ADDR;
	gDeviceHoldingReg.mDeviceReg.name_size			= strlen(gDeviceHoldingReg.gNameBlock.Name);
	gDeviceHoldingReg.mDeviceReg.device_num			= static_param.Num;

	gDeviceHoldingReg.mInfoReg.ver_addr				= VER_ADDR;
	gDeviceHoldingReg.mInfoReg.ver_size				= strlen(gDeviceHoldingReg.gVerBlock.Ver);

	memcpy(&gDeviceHoldingReg.mStaticParamReg, &static_param.mStaticParamReg, sizeof(StaticParamReg));

	gDeviceHoldingReg.gOperReg.CtrlReg				= DEV_INIT;
	gDeviceHoldingReg.gOperReg.StatReg				= DEV_EMPTY;
	gDeviceHoldingReg.gOperReg.ErrorReg				= 0;

	memset(&gHoldingRegModem, 0, sizeof(gHoldingRegModem));
	gHoldingRegModem.mSysUID.HW_ID					= 0x1305;
	gHoldingRegModem.mSysUID.HW_VERSION				= 0x0067;
	gHoldingRegModem.mSysUID.SW_ID					= 0x1000;
	gHoldingRegModem.mSysUID.SW_VERSION				= 0x0505;
}*/

//-----------------------------------------------------------------------------
uint8_t* find_reg_siam(uint8_t addr_device, uint32_t addr_reg, uint16_t len, eSiamRegisterMode eMode){
//	if(sBL_Cfg.mModbusAddress == addr_device)
//	{
		if(SIAM_REG_WRITE == eMode)
			//ApplyCfg();
		GETREG_region(REG_HOLDING_START, REG_HOLDING_NREGS*2, addr_reg, len, HoldingRegBL, HReg);
//	}
	return (uint8_t*)NULL;
}

//-----------------------------------------------------------------------------
