#ifndef PROTOKOL_MB_H__
#define PROTOKOL_MB_H__
//-----------------------------------------------------------------------------
#include "ProtokolMbStruct.h"
#include "exchenge.h"
//-----------------------------------------------------------------------------
void DoReceive_Modbus (void *prot, void *buf, uint16_t len);
void OnTxCompleate_Modbus (void *);
//-----------------------------------------------------------------------------
void Modbus_Hanndler (ProtInstanse *prot, Exchange *ex);
//-----------------------------------------------------------------------------
eMBErrorCode eMBRegHoldingCB (uint8_t *pucRegBuffer, uint16_t usAddress,
    uint16_t usNRegs, eMBRegisterMode eMode);
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------