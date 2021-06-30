#ifndef PROTOKOL_H__
#define PROTOKOL_H__
//-----------------------------------------------------------------------------
#include <stdint.h>
//-----------------------------------------------------------------------------
#define MAX_PROT_LEN 256
//-----------------------------------------------------------------------------
typedef enum
{
  NotInit = 0,
  Receiving,
  FrameReceived,
  FrameError,
  ResponseReady,
  Transmiting,
  TxCompleate
} ProtokolState;
//-----------------------------------------------------------------------------
typedef enum
{
  prSlave = 0,
  prMaster
} ProtokolType;
//-----------------------------------------------------------------------------
typedef struct
{
  uint8_t addr;
  ProtokolType sProtokolType;
  uint8_t sExType;
  ProtokolState state;
  uint8_t mBuf[MAX_PROT_LEN * 2];
  uint16_t mLen;
  uint16_t idxIn;
  uint16_t idxOut;
  void *queue_adrr;
} ProtInstanse;
//-----------------------------------------------------------------------------
void Prot_Init (void);
void Prot_Handler (void);
void Prot_Configurate (uint16_t addr);
uint16_t usMBCRC16 (uint8_t *pucFrame, uint32_t usLen);
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------