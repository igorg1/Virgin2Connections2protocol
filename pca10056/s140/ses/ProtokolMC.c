#include "ProtokolMb.h"
#include "nrf_log.h"
#include <string.h>

void RTUSend (ProtInstanse *mb);

//**********************************************************************************
eMBException
prveMBError2Exception (eMBErrorCode eErrorCode)
{
  eMBException eStatus;

  switch (eErrorCode)
    {
    case MB_ENOERR:
      eStatus = MB_EX_NONE;
      break;

    case MB_ENOREG:
      eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
      break;

    case MB_ETIMEDOUT:
      eStatus = MB_EX_SLAVE_BUSY;
      break;

    default:
      eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
      break;
    }

  return eStatus;
}
//**********************************************************************************
eMBException
eMBFuncReadHoldingRegister (uint8_t *pucFrame, uint16_t *usLen)
{
  uint16_t usRegAddress;
  uint16_t usRegCount;
  uint8_t *pucFrameCur;

  eMBException eStatus = MB_EX_NONE;
  eMBErrorCode eRegStatus;

  if (*usLen == (MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN))
    {
      usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
      usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
      usRegAddress++;

      usRegCount = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8);
      usRegCount |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1]);

      /* Check if the number of registers to read is valid. If not
       * return Modbus illegal data value exception.
       */
      if ((usRegCount >= 1) && (usRegCount <= MB_PDU_FUNC_READ_REGCNT_MAX))
        {
          /* Set the current PDU data pointer to the beginning. */
          pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
          *usLen = MB_PDU_FUNC_OFF;

          /* First byte contains the function code. */
          *pucFrameCur++ = MB_FUNC_READ_HOLDING_REGISTER;
          *usLen += 1;

          /* Second byte in the response contain the number of bytes. */
          *pucFrameCur++ = (uint8_t) (usRegCount * 2);
          *usLen += 1;

          /* Make callback to fill the buffer. */
          eRegStatus = eMBRegHoldingCB (
              pucFrameCur, usRegAddress, usRegCount, MB_REG_READ);
          /* If an error occured convert it into a Modbus exception. */
          if (eRegStatus != MB_ENOERR)
            {
              eStatus = prveMBError2Exception (eRegStatus);
            }
          else
            {
              *usLen += usRegCount * 2;
            }
        }
      else
        {
          eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
  else
    {
      /* Can't be a valid request because the length is incorrect. */
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  return eStatus;
}
//**********************************************************************************
eMBException
eMBFuncWriteMultipleHoldingRegister (uint8_t *pucFrame, uint16_t *usLen)
{
  uint16_t usRegAddress;
  uint16_t usRegCount;
  uint8_t ucRegByteCount;

  eMBException eStatus = MB_EX_NONE;
  eMBErrorCode eRegStatus;

  if (*usLen >= (MB_PDU_FUNC_WRITE_MUL_SIZE_MIN + MB_PDU_SIZE_MIN))
    {
      usRegAddress =
          (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8);
      usRegAddress |=
          (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1]);
      usRegAddress++;

      usRegCount =
          (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF] << 8);
      usRegCount |=
          (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF + 1]);

      ucRegByteCount = pucFrame[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

      if ((usRegCount >= 1) &&
          (usRegCount <= MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX) &&
          (ucRegByteCount == (uint8_t) (2 * usRegCount)))
        {
          /* Make callback to update the register values. */
          eRegStatus =
              eMBRegHoldingCB (&pucFrame[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF],
                  usRegAddress, usRegCount, MB_REG_WRITE);

          /* If an error occured convert it into a Modbus exception. */
          if (eRegStatus != MB_ENOERR)
            {
              eStatus = prveMBError2Exception (eRegStatus);
            }
          else
            {
              /* The response contains the function code, the starting
               * address and the quantity of registers. We reuse the
               * old values in the buffer because they are still valid.
               */
              *usLen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
            }
        }
      else
        {
          eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
  else
    {
      /* Can't be a valid request because the length is incorrect. */
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  return eStatus;
}
//**********************************************************************************
void
ErrorResponseMB (
    ProtInstanse *mb, uint8_t ucFunctionCode, eMBException eException)
{
  mb->mLen = MB_RTU_PDU_ADDR_OFF;

  mb->mBuf[mb->mLen++] = mb->addr;
  mb->mBuf[mb->mLen++] = (uint8_t) (ucFunctionCode | MB_FUNC_ERROR);
  mb->mBuf[mb->mLen] = eException;
}
//**********************************************************************************
void
FrameProcess_Modbus (ProtInstanse *pr)
{
  uint16_t qty = 0;
  uint16_t rcv_crc = 0;

  /*	for(; pr->idxIn<pr->idxOut; pr->idxIn++)
          {
                  if(0 == memcmp(&pr->addr, pr->mBuf + pr->idxIn, 1))
                  break;
          }
  */
  if (sizeof (pr->mBuf) / 2 < pr->idxIn)
    {
      memcpy (&pr->mBuf, &pr->mBuf[pr->idxIn], pr->idxOut - pr->idxIn);
      pr->idxOut = pr->idxOut - pr->idxIn;
      pr->idxIn = 0;
    }

  if (pr->idxIn == pr->idxOut)
    {
      pr->state = NotInit;
      return;
    }

  if (5 > pr->idxOut - pr->idxIn)
    {
      pr->state = NotInit;
      return;
    }

  if (pr->sProtokolType == prMaster)
    {
      switch (pr->mBuf[pr->idxIn + 1])
        {
        default:
          pr->idxIn += 1;
          return;
        case 0x03:
          qty = 8;
          break;
        case 0x10:
          qty = (uint16_t)pr->mBuf[pr->idxIn + 6] + 9;
          break;
        case 0x17:
          qty = (uint16_t)pr->mBuf[pr->idxIn + 10] + 13;
          break;
        }
    }

  if (pr->sProtokolType == prSlave)
    {
      switch (pr->mBuf[pr->idxIn + 1])
        {
        default:
          pr->idxIn += 1;
          return;
        case 0x03:
          qty = 5 + pr->mBuf[pr->idxIn + 2];
          break;
        case 0x83:
          qty = 5;
          break;
        case 0x10:
          qty = 8;
          break;
        case 0x90:
          qty = 5;
          break;
        case 0x17:
          qty = 5 + pr->mBuf[pr->idxIn + 2];
          break;
        case 0x97:
          qty = 5;
          break;
        }
    }

  rcv_crc = usMBCRC16 (&pr->mBuf[pr->idxIn], qty);
  if (rcv_crc != 0)
    {
      if (qty < pr->idxOut)
        pr->idxIn += 1;
      return;
    }

  memcpy (&pr->mBuf, &pr->mBuf[pr->idxIn], qty); // pr->idxOut - pr->idxIn);
  pr->mLen = qty;
  pr->idxIn = 0;
  pr->idxOut = 0;

  //	NRF_LOG_HEXDUMP_DEBUG(pr->mBuf, pr->mLen);

  rcv_crc = usMBCRC16 (pr->mBuf, pr->mLen);
  const uint8_t rcv_address = pr->mBuf[MB_RTU_PDU_ADDR_OFF];

  if (MB_RTU_PDU_SIZE_MIN > pr->mLen || 0 != rcv_crc)
    //		|| rcv_address != pr->addr )
    {
      pr->state = NotInit;
      return;
    }

  uint16_t pusLength =
      (uint16_t) (pr->mLen - MB_RTU_PDU_PDU_OFF - MB_RTU_PDU_SIZE_CRC);
  uint8_t *pucFrame = (uint8_t *)&pr->mBuf[MB_RTU_PDU_PDU_OFF];
  const uint8_t ucFunctionCode = pucFrame[MB_PDU_FUNC_OFF];
  eMBException eException = MB_EX_ILLEGAL_FUNCTION;

  if (rcv_address == pr->addr)
    {
      __disable_irq ();
      switch (ucFunctionCode)
        {
        default:
          break;
        case MB_FUNC_READ_HOLDING_REGISTER:
          eException = eMBFuncReadHoldingRegister (pucFrame, &pusLength);
          break;
        case MB_FUNC_WRITE_MULTIPLE_REGISTERS:
          eException =
              eMBFuncWriteMultipleHoldingRegister (pucFrame, &pusLength);
          break;
        }
      if (pr->sProtokolType == prMaster)
        pr->sExType = 1;
      if (pr->sProtokolType == prSlave)
        pr->sExType = 0;
      __enable_irq ();
    }
  else
    {
      NRF_LOG_DEBUG ("ResponseReady len: %d", pr->mLen);
      pr->state = ResponseReady;
      if (pr->sProtokolType == prMaster)
        pr->sExType = 0;
      if (pr->sProtokolType == prSlave)
        pr->sExType = 1;
      return;
    }

  if (eException)
    ErrorResponseMB (pr, ucFunctionCode, eException);
  else
    pr->mLen = pusLength;

  pr->state = ResponseReady;
  RTUSend (pr);
}
//**********************************************************************************
void
OnTxCompleate_Modbus (void *prot)
{
  ProtInstanse *mb_prot = (ProtInstanse *)prot;
  mb_prot->state = TxCompleate;
}
//**********************************************************************************
void
RTUSend (ProtInstanse *mb)
{
  uint16_t usCRC16;

  /* First byte before the Modbus-PDU is the slave address. */
  uint8_t *pucSndBufferCur = (uint8_t *)mb->mBuf;
  uint16_t usSndBufferCount = 1;

  /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
  pucSndBufferCur[MB_RTU_PDU_ADDR_OFF] = mb->addr;
  usSndBufferCount += mb->mLen;

  /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
  usCRC16 = usMBCRC16 ((uint8_t *)pucSndBufferCur, usSndBufferCount);

  *(uint16_t *)&mb->mBuf[usSndBufferCount] = usCRC16;
  usSndBufferCount += 2;

  mb->mLen = usSndBufferCount;

  NRF_LOG_HEXDUMP_DEBUG (mb->mBuf, mb->mLen);
}
//**********************************************************************************
void
Modbus_Hanndler (ProtInstanse *prot, Exchange *ex)
{
  switch (prot->state)
    {
    default:
      break;
    case FrameReceived:
      ex->EnableRx (0);
      FrameProcess_Modbus (prot);
      if (prot->state == NotInit)
        {
          ex->EnableRx (1);
          break;
        }
      if (prot->state != ResponseReady)
        break;
    case ResponseReady:
      if (prot->sExType == ex->sExType)
        {
          ex->EnableTx (1);
          ex->TransmitFn (prot, prot->mBuf, prot->mLen);
          prot->state = NotInit;
        }
      break;
    case FrameError:
      break;
    case TxCompleate:
      ex->EnableTx (0);
      ex->EnableRx (1);
      memset (&prot->mBuf, 0, sizeof (prot->mBuf));
      prot->mLen = 0;
      prot->idxIn = 0;
      prot->idxOut = 0;
      prot->state = NotInit;
      break;
    }
}
//**********************************************************************************