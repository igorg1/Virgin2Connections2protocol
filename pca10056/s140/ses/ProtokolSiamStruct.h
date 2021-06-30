#ifndef PROTOKOL_SIAM_STRUCT_H__
#define PROTOKOL_SIAM_STRUCT_H__

#include "Protokol.h"
#include <stdint.h>

typedef enum
{
  eNoError = 0,
  eWrongAddr,
  eWrongSize,
  eWrongData,
  eWrongWritingReg,
  eWrongCrc
} eSiamError;

#endif