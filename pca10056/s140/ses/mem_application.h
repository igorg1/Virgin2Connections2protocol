#ifndef MEM_APPLICATION_H__
#define MEM_APPLICATION_H__

#include "mem_commstruct.h"
#include <stddef.h>
#include <stdint.h>
//#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\mem_modem.h"
#include "ble_gap.h"
#include "mem_modem.h"
//#include "main.h"

typedef struct
{
  uint16_t id;
  char name[20];
  uint8_t mac_addr[6];
} WORKINGSCANVAL;

typedef struct
{
  // App----------------------
  SysUID mSysUID;
  LockCtrl mLockCtrl;
  SysCtrl mSysCtrl;
  // BL-----------------------
  BLConfig mBL_Cfg;
  SysUID mBL_UID;
  LockCtrl mBL_LockCtrl;
  // Modem--------------------
  ModemParam mModemParam;
  // Device-------------------
  uint16_t cntScanVal;
  WORKINGSCANVAL workingScanVal[62];
  // protected - inaccessible from modbus
  LockCfg mLockCfg;
  LockCfg mBL_LockCfg;
} __attribute__((aligned(2))) HoldingReg;
//-----------------------------------------------------------------------------
#define REG_HOLDING_START (0)
#define REG_HOLDING_NREGS \
  (uint16_t)((sizeof(HoldingReg) - sizeof(LockCfg) * 2) / 2)

extern uint16_t *usRegHoldingBuf;
extern HoldingReg HReg;

#define BEGIN_EDIT_REG ((uint16_t)(offsetof(HoldingReg, mLockCtrl)) / 2)
#define END_EDIT_REG REG_HOLDING_NREGS

// shortcuts
#define sSysUID HReg.mSysUID
#define sLockCtrl HReg.mLockCtrl
#define sSysCtrl HReg.mSysCtrl

#define sBL_UID HReg.mBL_UID
#define sBL_LockCtrl HReg.mBL_LockCtrl
#define sBL_Cfg HReg.mBL_Cfg

#define sLockCfg HReg.mLockCfg
#define sBL_LockCfg HReg.mBL_LockCfg

#define sProtocolCfg HReg.mModemParam.mProtocolCfg
#define sUartCfg HReg.mModemParam.mUartCfg

#define sProtocolCfg HReg.mModemParam.mProtocolCfg
#define sUartCfg HReg.mModemParam.mUartCfg

//#define s_cntScanVal HReg.cntScanVal;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
typedef struct
{
  SysUID mSysUID;
  LockCfg mLockCfg;
  BLConfig mBLConfig;
} __attribute__((aligned(4), packed)) BLParam;

extern uint32_t *gBLSysUID_Addr;
//-----------------------------------------------------------------------------
typedef struct
{
  SysUID mSysUID;
  LockCfg mLockCfg;
} __attribute__((aligned(4), packed)) APPParam;

extern uint32_t *gAPPSysUID_Addr;
//-----------------------------------------------------------------------------
void ApplyCfg(void);

#endif