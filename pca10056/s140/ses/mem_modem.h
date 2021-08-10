#ifndef MEM_MODEM_H__
#define MEM_MODEM_H__

#include <stdint.h>
#include "mem_commstruct.h"

#define defaulMode SiamMaster
#define defaultSpeedUart0			baud_115200

typedef enum
{
  SiamMaster = 0,
  ModBusMaster,
  SiamSlave,
  ModBusSlave,
  prNULL
} ProtocolMode;

typedef enum
{
	 baud_9600=0
	,baud_14400
	,baud_19200
	,baud_38400
	,baud_57600
	,baud_115200
	,baud_230400
	,baud_460800
	,baud_921600
} UartSpeed;

typedef struct
{
  UartSpeed mSpeedUart0;
  int16_t mUart0Timer;
} __attribute__ ((aligned (2), packed)) UartCfg;

typedef struct
{
  ProtocolMode mMode; // siam | modbus
} __attribute__ ((aligned (2))) ProtocolCfg;

//---------------------------------------
typedef struct
{
  // BluetoothCfg	mBluetoothCfg;
  ProtocolCfg mProtocolCfg;
  UartCfg mUartCfg;
} __attribute__ ((aligned (2))) ModemParam;

#endif