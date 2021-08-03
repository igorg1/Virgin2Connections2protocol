#include "mem_application.h"
//#include "main.h"
//#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ProtokolMbStruct.h"
#include "ProtokolMbStruct.h"
#include <string.h>

#define MYBSWAP16(v) ((uint16_t) (v) >> 8) | ((uint16_t) (v) << 8)

HoldingReg HReg;
uint16_t *usRegHoldingBuf = (uint16_t *)&HReg;

const SysUID gAPPSysUID = { 1, 257, 10, 513 };
uint32_t *gAPPSysUID_Addr = (uint32_t *)&gAPPSysUID;

const SysUID gBLSysUID = { 1, 257, 0, 1 };
uint32_t *gBLSysUID_Addr = (uint32_t *)&gBLSysUID;

static inline unsigned short
bswap16 (unsigned short a)
{
  return (a << 8) | (a >> 8);
}

eMBErrorCode
eMBRegHoldingCB (uint8_t *pucRegBuffer, uint16_t usAddress, uint16_t usNRegs,
    eMBRegisterMode eMode)
{
#if REG_HOLDING_START != 0
  if (usAddress < REG_HOLDING_START)
    return MB_ENOREG;
#endif
  --usAddress;          // 0 based start address
  usNRegs += usAddress; // 0 based end address
  if (usNRegs > REG_HOLDING_START + REG_HOLDING_NREGS)
    return MB_ENOREG;
  uint16_t *cellBuf = (uint16_t *)pucRegBuffer;
  uint16_t *reg = usRegHoldingBuf + usAddress - REG_HOLDING_START;
  switch (eMode)
    {
    default:
      return MB_ENOREG;
    case MB_REG_READ:
      while (usAddress++ < usNRegs)
        *cellBuf++ = bswap16 (*reg++);
      break;
    case MB_REG_WRITE:
      if (usAddress < BEGIN_EDIT_REG)
        usAddress = BEGIN_EDIT_REG;
      if (usNRegs > END_EDIT_REG)
        usNRegs = END_EDIT_REG;

      if (usAddress < usNRegs)
        {
          // ApplyCfg();
          while (usAddress++ < usNRegs)
            {
              if (sLockCfg.mLock && usAddress > sLockCfg.mLock)
                break;
              *reg++ = bswap16 (*cellBuf++);
            }
        }
      break;
    } // switch(eMode)
  return MB_ENOERR;
}