#ifndef MEM_CONFIG_H__
#define MEM_CONFIG_H__

#include "stdint.h"
#include "mem_device.h"
#include "mem_modem.h"
#include "mem_application.h"




typedef struct{
	BLParam					mBLParam;
	uint8_t mReserved[ 256	- sizeof(uint32_t)//version confit in Flash
							- sizeof(uint32_t)//crc
							- sizeof(BLParam)
							];
} __attribute__((aligned(4),packed)) BL_Storage;

typedef struct{
	APPParam				mAPPParam;
	ModemParam				mModemParam;
	uint8_t mReserved[ 256	- sizeof(uint32_t)//version confit in Flash
							- sizeof(uint32_t)//crc
							- sizeof(APPParam)
							- sizeof(ModemParam)
							];
        
} __attribute__((aligned(4),packed)) APP_Storage;

extern DeviceParam				static_param;
extern DeviceParam 				defalt_DeviceParam;

void SaveConfig(void);
uint8_t LoadConfig(void);

void Config_Init(void);
void Config_Handler(void);

#endif
