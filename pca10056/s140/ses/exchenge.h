#ifndef EXCHENGE_H__
#define EXCHENGE_H__

#include <stdint.h>

typedef void (*gEnableFn) (int);
typedef void (*gExchangeFn) (void *, void *, uint16_t);
typedef void (*gCompleateFn) (void *);

typedef uint16_t (*gSendFn) (uint8_t *, uint16_t);
uint16_t SendData (gSendFn DoSend, uint16_t mtu, uint8_t *data, uint16_t len);

typedef enum
{
  exSlave = 0,
  exMaster
} ExchangeType;

typedef struct
{
  uint8_t sExType;
  gCompleateFn OnTxCompleate;
  gEnableFn EnableRx;
  gEnableFn EnableTx;

  gExchangeFn TransmitFn;
  gExchangeFn OnReceiveFn;
} Exchange;

#endif