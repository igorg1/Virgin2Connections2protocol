#define defaulMode SiamMaster

typedef enum
{
  SiamMaster = 0,
  ModBusMaster,
  SiamSlave,
  ModBusSlave,
  prNULL
} ProtocolMode;

typedef struct
{
  //UartSpeed mSpeedUart0;
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
} __attribute__ ((aligned (4))) ModemParam;