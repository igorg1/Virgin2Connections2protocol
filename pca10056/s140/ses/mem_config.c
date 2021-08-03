#include "mem_config.h"
#include "Utils.h"
#include <nrf_sdm.h>
#include <stddef.h>
#include "nrf_delay.h"
#include "main.h"

void APPCfg_Load(void);
void APPCfg_Save(void);

void BLCfg_Load(void);

void StartApp(void);

//#define vector_table_addr 0x40000 //0x2c000
DeviceParam				static_param;
uint16_t				gPendingApplyCfg=0;
uint8_t					gCurrentPasswd[8] = {1,2,3,4,5,6,7,8};
uint32_t				gCurrentCRC=0;


UartCfg					gCurrentUartCfg;

void ApplyCfg(void)
{
	gPendingApplyCfg = ccApply;
	NRF_LOG_INFO("ApplyCfg");
}

void BLCfg_SetDefault(void)
{
	memset((void*)&sBL_UID, 0, sizeof(SysUID) );
	memset((void*)&sBL_LockCfg, 0, sizeof(LockCfg) );
	memset((void*)&sBL_Cfg, 0, sizeof(BLConfig) );

	sBL_UID = *(SysUID*)gBLSysUID_Addr;
	sBL_Cfg.mModbusAddress = 127;

	NRF_LOG_INFO("Set Defalt BootLoader Config");
}

void BLCfg_Load(void)
{
	NRF_LOG_ERROR("Load BLCFG");
	const BL_Storage* bl_stor = (const BL_Storage*)Flash_LoadStor(FLASH_BLCFG_START_ADDR
										, FLASH_BLCFG_END_ADDR, sizeof(BL_Storage));
	if(bl_stor && 0==memcmp(gBLSysUID_Addr, &bl_stor->mBLParam.mSysUID, sizeof(SysUID)) )
	{
		sBL_UID = *(SysUID*)gBLSysUID_Addr;
		sBL_LockCfg = bl_stor->mBLParam.mLockCfg;
		sBL_Cfg = bl_stor->mBLParam.mBLConfig;

		if(!sBL_Cfg.mModbusAddress)
			sBL_Cfg.mModbusAddress=127;
	}
	else
	{
		BLCfg_SetDefault();
	}
}

void APPCfg_SetDefault(void)
{
	//memset((void*)usRegHoldingBuf, 0, sizeof(HoldingReg) );

	//sSysUID = *(SysUID*)gAPPSysUID_Addr;

	//memcpy(&sBluetoothCfg.mDefaultName, defaultNameModem, strlen(defaultNameModem));
	//sBluetoothCfg.mBtMode = defaultBtMode;
	//sBluetoothCfg.mBtTxPower = defaultBtTxPower;
	//sBluetoothCfg.mAdvInterval = defaultAdvInterval;
	//sBluetoothCfg.mAdvDuration = defaultAdvDuration;
	//sBluetoothCfg.mDisconnectTimer = defaultDisconnectTimer;

	//sProtocolCfg.mMode = defaulMode;

	//sUartCfg.mSpeedUart0 = defaultSpeedUart0;
	//sUartCfg.mUart0Timer = defaultUart0Timer;

	NRF_LOG_INFO("Set Defalt Aplication Config");
	//BLCfg_Load();
}

void APPCfg_Load(void)
{
	NRF_LOG_ERROR("Load APPCFG");
	const APP_Storage* app_stor = (const APP_Storage*)Flash_LoadStor(FLASH_APPCFG_START_ADDR
										, FLASH_APPCFG_END_ADDR, sizeof(APP_Storage));
	if(app_stor && 0==memcmp(gAPPSysUID_Addr, &app_stor->mAPPParam.mSysUID, sizeof(SysUID)) )
	{
		sSysUID = *(SysUID*)gAPPSysUID_Addr;
		sLockCfg = app_stor->mAPPParam.mLockCfg;

	//	memcpy(&sBluetoothCfg.mDefaultName,
	//					app_stor->mModemParam.mBluetoothCfg.mDefaultName,
	//					strlen((const char*)app_stor->mModemParam.mBluetoothCfg.mDefaultName));
		
	//	sBluetoothCfg.mBtMode = app_stor->mModemParam.mBluetoothCfg.mBtMode;
	//	sBluetoothCfg.mBtTxPower = app_stor->mModemParam.mBluetoothCfg.mBtTxPower;
	//	switch(sBluetoothCfg.mBtMode)
	//	{
	//		default:
	//			sBluetoothCfg.mBtMode = bt4_1cf;
		//		break;
	//		case bt4_11f: break;
	//		case bt4_12f: break;
	//		case bt4_1cf: break;
	//		case bt4_1nf: break;
	//		case bt5_c1t: break;
	//		case bt5_c2t: break;
	//		case bt5_cct: break;
	//		case bt5_cnt: break;
		//}
	//	sBluetoothCfg.mAdvInterval = app_stor->mModemParam.mBluetoothCfg.mAdvInterval;
	//	sBluetoothCfg.mAdvDuration = app_stor->mModemParam.mBluetoothCfg.mAdvDuration;
	//	sBluetoothCfg.mDisconnectTimer = app_stor->mModemParam.mBluetoothCfg.mDisconnectTimer;
		
		sProtocolCfg.mMode = app_stor->mModemParam.mProtocolCfg.mMode;
		
		sUartCfg.mSpeedUart0 = app_stor->mModemParam.mUartCfg.mSpeedUart0;
	//	sUartCfg.mUart0Timer = app_stor->mModemParam.mUartCfg.mUart0Timer;
	}
	else
	{
		APPCfg_SetDefault();
	}
	BLCfg_Load();
	//memcpy(&gCurrentBluetoothCfg, &sBluetoothCfg, sizeof(BluetoothCfg));
}

void APPCfg_Save(void)
{
	const uint32_t psz = sizeof(sBL_LockCtrl.mPasswd)/4;
	const uint32_t bl_pass_CRC
		= crc32_compute((uint8_t const *)&sBL_LockCtrl.mPasswd, psz, NULL);
	if(!sBL_LockCfg.mLock || bl_pass_CRC == sBL_LockCfg.mPasswdCRC)
	{
		BL_Storage bl_stor;
		memset(&bl_stor, 0, sizeof(BL_Storage));
		bl_stor.mBLParam.mSysUID = sBL_UID;
		bl_stor.mBLParam.mLockCfg = sBL_LockCfg;
		bl_stor.mBLParam.mBLConfig = sBL_Cfg;
		Flash_SaveStor(FLASH_BLCFG_START_ADDR, FLASH_BLCFG_END_ADDR, sizeof(BL_Storage),(uint32_t*)&bl_stor );
	}

	APP_Storage app_stor;
	memset(&app_stor, 0, sizeof(APP_Storage));
	app_stor.mAPPParam.mSysUID = sSysUID;
	app_stor.mAPPParam.mLockCfg = sLockCfg;

	//memcpy(&app_stor.mModemParam.mBluetoothCfg.mDefaultName,
	//				&sBluetoothCfg.mDefaultName,
	//				strlen((const char*)sBluetoothCfg.mDefaultName));
	//app_stor.mModemParam.mBluetoothCfg.mBtMode = sBluetoothCfg.mBtMode;
	//app_stor.mModemParam.mBluetoothCfg.mBtTxPower = sBluetoothCfg.mBtTxPower;
	//app_stor.mModemParam.mBluetoothCfg.mAdvInterval = sBluetoothCfg.mAdvInterval;
	//app_stor.mModemParam.mBluetoothCfg.mAdvDuration = sBluetoothCfg.mAdvDuration;
	//app_stor.mModemParam.mBluetoothCfg.mDisconnectTimer = sBluetoothCfg.mDisconnectTimer;
	
	//app_stor.mModemParam.mProtocolCfg.mMode = sProtocolCfg.mMode;
	
	//app_stor.mModemParam.mUartCfg.mSpeedUart0 = sUartCfg.mSpeedUart0;
	//app_stor.mModemParam.mUartCfg.mUart0Timer = sUartCfg.mUart0Timer;

	Flash_SaveStor(FLASH_APPCFG_START_ADDR, FLASH_APPCFG_END_ADDR, sizeof(APP_Storage),(uint32_t*)&app_stor );
	NRF_LOG_INFO("APPCfg_Save");
}

void Config_Init(void)
{
	APPCfg_Load();

	ApplyCfg();
}
void LockCtrl_Configurate()
{
	const uint16_t min_addr=offsetof(HoldingReg, mSysCtrl.mCommand) / 2;
	if(sLockCtrl.mLockCmd && min_addr>sLockCtrl.mLockCmd)
		sLockCtrl.mLockCmd=min_addr;

	if(sLockCfg.mLock)
	{
		if(sLockCtrl.mLockCmd)// if try already lock
		{
			sLockCtrl.mLockCmd=sLockCfg.mLock;
		}
		else
		{
			const uint32_t psz = sizeof(sLockCtrl.mPasswd);
			if(0!=memcmp(&gCurrentPasswd, &sLockCtrl.mPasswd,sizeof(sLockCtrl.mPasswd)) )
			{
				gCurrentCRC = crc32_compute((uint8_t const *)&sLockCtrl.mPasswd, psz, NULL);
				memcpy(&gCurrentPasswd, &sLockCtrl.mPasswd,sizeof(sLockCtrl.mPasswd));
			}
			if(gCurrentCRC == sLockCfg.mPasswdCRC)
			{
				sLockCtrl.mLockCmd=sLockCfg.mLock=0;
				sLockCfg.mPasswdCRC=0;
				gCurrentCRC = 0;
			}
			else
				sLockCtrl.mLockCmd=sLockCfg.mLock;
		}
	}
	else
	{
		if(sLockCtrl.mLockCmd) // try lock
		{
			const uint32_t psz = sizeof(sLockCtrl.mPasswd);
			gCurrentCRC = sLockCfg.mPasswdCRC
				= crc32_compute((uint8_t const *)&sLockCtrl.mPasswd, psz, NULL);
		}
		else
			sLockCtrl.mLockCmd=sLockCfg.mLock;
	}
	sSysCtrl.mStatus++;
}

void SysCtrl_Configurate(void)
{
	ret_code_t err_code;
	if(sSysCtrl.mCommand)
	{
		switch(sSysCtrl.mCommand)
		{
		default: break;
		case SCAN:
		//	APPCfg_SetDefault();
                      scan_start();   
			break;
		case ccSave:
		//	sLockCfg.mLock = sLockCtrl.mLockCmd;
		//	APPCfg_Save();
			break;
		case ccApply:
		//	ApplyCfg();
			break;
		case ccReboot:
			//bsp_board_led_off(1);
		//	nrf_delay_ms(200);
		//	err_code = sd_nvic_SystemReset();
		//	APP_ERROR_CHECK(err_code);
			break;
		}
		sSysCtrl.mStatus++;
		sSysCtrl.mCommand=0;
	}
}

void BtMode_Configurate(void)
{
	if(!gPendingApplyCfg)
		return;
	//if(0==memcmp(&gCurrentBluetoothCfg, &sBluetoothCfg, sizeof(BluetoothCfg)))
	//	return;
	//BtModeUpdate();
	//memcpy(&gCurrentBluetoothCfg, &sBluetoothCfg, sizeof(BluetoothCfg));
}

void Uart_Configurate(void)
{
	if(!gPendingApplyCfg)
		return;
	//if(0==memcmp(&gCurrentUartCfg, &sUartCfg, sizeof(UartCfg)))
	//	return;
	//Uart0_UpdateCfg();
	//memcpy(&gCurrentUartCfg, &sUartCfg, sizeof(UartCfg));
}

void Config_Handler(void)
{
	if(!gPendingApplyCfg)
		return;
	gPendingApplyCfg=0;
	LockCtrl_Configurate();
	if(!sLockCfg.mLock)
	{
		SysCtrl_Configurate();
//		Prot_Configurate(sBL_Cfg.mModbusAddress);
		BtMode_Configurate();
		Uart_Configurate();
	}
	gPendingApplyCfg=0;
}
