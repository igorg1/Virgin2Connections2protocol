#include "ProtokolSiam.h"
#include "ble_types.h"
#include "main.h"
#include "nrf_log.h"
#include <string.h>

UART_ASSIGN connCurrent[62];
extern uint8_t cntAll;
extern bool isConnectedBle;
extern uint16_t bleConnNumber;
extern uint16_t bleReturnedConnNumber;

//-----------------------------------------------------------------------------
eSiamError ProcessWriteCmdSiam(ProtInstanse *pr, uint8_t *buf, uint8_t len, uint8_t addr_device, uint32_t addr_reg, uint16_t qty) {
  uint16_t crc = usMBCRC16(pr->mBuf + 2, 8);
  if (crc != *(uint16_t *)&pr->mBuf[10])
    return eWrongCrc;

  const uint16_t data_crc = usMBCRC16(pr->mBuf + 12, qty);
  if (data_crc != *(uint16_t *)&pr->mBuf[len - 2])
    return eWrongCrc;

  uint8_t *reg = find_reg_siam(addr_device, addr_reg, qty, SIAM_REG_WRITE);
  if (!reg)
    return eWrongAddr;

  memcpy(&reg[0], &buf[0] + 12, qty);

  pr->mLen = 12;
  pr->state = ResponseReady;
  return eNoError;
}
//-----------------------------------------------------------------------------
eSiamError ProcessReadCmdSiam(ProtInstanse *pr, uint8_t *buf, uint8_t len, uint8_t addr_device, uint32_t addr_reg, uint16_t qty) {
  uint16_t crc = usMBCRC16(pr->mBuf + 2, pr->mLen - 4);
  if (crc != *(uint16_t *)&pr->mBuf[10])
    return eWrongCrc;

  if (MAX_PROT_LEN * 16 < qty + 12 + 2) {
    return eWrongSize;
  }

  uint8_t *reg = find_reg_siam(addr_device, addr_reg, qty, SIAM_REG_READ);
  if (!reg)
    return eWrongAddr;

  memcpy(&buf[0] + len, &reg[0], qty);

  crc = usMBCRC16(buf + len, qty);

  memcpy(buf + len + qty, &crc, sizeof(crc));

  pr->mLen = len + qty + sizeof(crc);
  pr->state = ResponseReady;
  return eNoError;
}
//-----------------------------------------------------------------------------
void ErrorResponseSiam(ProtInstanse *pr, eSiamError eException) {
  //	memcpy(&pr->mBuf[0]+12+2, &crc, 2);
  //pr->mBuf[2] |=01-127
  pr->mBuf[3] |= 0x80;
  pr->mLen = 14;                             //12
  memcpy(&pr->mBuf[0] + 10, &eException, 2); //
  uint16_t crc = usMBCRC16((uint8_t *)&pr->mBuf[2], 10);
  memcpy(pr->mBuf + 12, &crc, 2); //+
  pr->state = ResponseReady;
}

void change_address(ProtInstanse *pr) {
  uint8_t IDscan;
  ret_code_t err_code;
  uint8_t tempAddress=0;
  uint16_t crc;
  switch (pr->sProtokolType) {
  case prMaster:
    if (true == (pr->mBuf[2] & 0x01)) {
      IDscan = (pr->mBuf[2] + 1) / 2;
      pr->mBuf[2] = 0x01;
    } else {
      IDscan = pr->mBuf[2] / 2;
      pr->mBuf[2] = 0x7F;
    }
    for (size_t i = 0; i < 62; i++) {//if work without saved flash record - &TODO condition don't check daved record
      if (HReg.workingScanVal[i].id == IDscan) {
        connCurrent[i].id_scan = IDscan;
        break;
      }
    }
    for (size_t i = 0; i < 62; i++) {
      if (connCurrent[i].id_scan == IDscan) {
        if (connCurrent[i].connHandler == BLE_CONN_HANDLE_INVALID) {
          isConnectedBle = false;
          err_code = sd_ble_gap_connect(&connCurrent[i].addr,
              &connCurrent[i].p_scan_params, &connCurrent[i].p_conn_params,
              connCurrent[i].con_cfg_tag);
          APP_ERROR_CHECK(err_code);
          // do {
          //__WFE();
          //  sd_app_evt_wait ();
          // } while (!isConnectedBle);
          while (!isConnectedBle)
            ;
          connCurrent[i].connHandler = bleConnNumber;
        }
        break;
      }
    }
    pr->sconn_handle = bleConnNumber;//IDscan; //@TODO - add in ProtInstanse scann_handle - DONE
     crc = usMBCRC16((uint8_t *)&pr->mBuf[2], 8);
    memcpy(pr->mBuf + 10, &crc, 2); //+
    break;
  case prSlave:
  NRF_LOG_INFO("test");
    for (size_t i=0;i<62;i++){
      if (connCurrent[i].connHandler==bleReturnedConnNumber){
        switch(pr->mBuf[2]){
        case 0x01:
        pr->mBuf[2]=(connCurrent[i].id_scan*2)-1;
        break;
        case 0x7F:
        pr->mBuf[2]=connCurrent[i].id_scan*2;
        break;
        default:
        break;
        }
        break;
      }
    }
     crc = usMBCRC16((uint8_t *)&pr->mBuf[2], 8);
    memcpy(pr->mBuf + 10, &crc, 2);
    break;
  default:
    break;
  }
}
//-----------------------------------------------------------------------------
void FrameProcess_Saim(ProtInstanse *pr) {
  uint16_t qty = 0;
  uint16_t rcv_crc = 0;

  for (; pr->idxIn < pr->idxOut; pr->idxIn++) {
    if (0 == memcmp("\r\n", pr->mBuf + pr->idxIn, 2))
      break;
  }

  if (sizeof(pr->mBuf) / 2 < pr->idxIn) {
    memcpy(&pr->mBuf, &pr->mBuf[pr->idxIn], pr->idxOut - pr->idxIn);
    pr->idxOut = pr->idxOut - pr->idxIn;
    pr->idxIn = 0;
  }

  if (pr->idxIn == pr->idxOut) {
    pr->state = NotInit;
    return;
  }

  if (12 > pr->idxOut - pr->idxIn) {
    pr->state = NotInit;
    return;
  }

  uint8_t crc_size = 0;
  switch (pr->mBuf[pr->idxIn + 3]) {
  default:
    crc_size = 12;
    break;
  case 0x01:
    crc_size = 12;
    break;
  case 0x02:
    crc_size = 12;
    break;
  case 0x81:
    crc_size = 14;
    break;
  case 0x82:
    crc_size = 14;
    break;
  }
  rcv_crc = usMBCRC16(&pr->mBuf[pr->idxIn + 2], crc_size - 2);
  if (rcv_crc != 0) {
    pr->idxIn += 2;
    return;
  }

  if (pr->sProtokolType == prMaster) {
    switch (pr->mBuf[pr->idxIn + 3]) {
    default:
      pr->state = NotInit;
      return;
    case 0x01:
      pr->mLen = 12;
      break;
    case 0x02:
      qty = *(uint16_t *)&pr->mBuf[pr->idxIn + 8];
      if (qty + 2 > pr->idxOut - pr->idxIn - 12) {
        pr->state = NotInit;
        return;
      }
      pr->mLen = 12 + qty + 2;
      break;
    }
  }

  if (pr->sProtokolType == prSlave) {
    switch (pr->mBuf[pr->idxIn + 3]) {
    default:
      pr->state = NotInit;
      return;
    case 0x01:
      qty = *(uint16_t *)&pr->mBuf[pr->idxIn + 8];
      if (qty + 2 > pr->idxOut - pr->idxIn - 12) {
        pr->state = NotInit;
        return;
      }
      pr->mLen = 12 + qty + 2;
      break;
    case 0x02:
      pr->mLen = 12;
      break;
    case 0x81:
      pr->mLen = 14;
      break;
    case 0x82:
      pr->mLen = 14;
      break;
    }
  }

  memcpy(&pr->mBuf, &pr->mBuf[pr->idxIn], pr->mLen);
  pr->idxIn = 0;
  pr->idxOut = 0;

  const uint8_t ucFunctionCode = pr->mBuf[3];

  const uint8_t addr_device = *(uint8_t *)&pr->mBuf[2];
  const uint32_t addr_reg = *(uint32_t *)&pr->mBuf[4];
  qty = *(uint16_t *)&pr->mBuf[8];
  
  eSiamError eException = eNoError;
  NRF_LOG_INFO("ProtcolType %d\n\n", pr->sProtokolType);
  if ((addr_device == pr->addr)&&(pr->sProtokolType == prMaster)) {
    //__disable_irq();
    switch (ucFunctionCode) {
    default:
      break;
    case 0x01:
      eException = ProcessReadCmdSiam(pr, pr->mBuf, pr->mLen, addr_device, addr_reg, qty);
      break;
    case 0x02:
      eException = ProcessWriteCmdSiam(pr, pr->mBuf, pr->mLen, addr_device, addr_reg, qty);
      break;
    }
    if (pr->sProtokolType == prMaster)
      pr->sExType = 1;
    if (pr->sProtokolType == prSlave)
      pr->sExType = 0;
    //__enable_irq();
  } else {
    NRF_LOG_DEBUG("ResponseReady len: %d", pr->mLen);
    change_address(pr);
    pr->state = ResponseReady;
    if (pr->sProtokolType == prMaster) //@TODO
      pr->sExType = 0;
    if (pr->sProtokolType == prSlave)
      pr->sExType = 1;
    return;
  }

  if (eException)
    ErrorResponseSiam(pr, eException);

  NRF_LOG_HEXDUMP_DEBUG(pr->mBuf, pr->mLen);
}
//-----------------------------------------------------------------------------
void OnTxCompleate_Siam(void *pr) {
  ProtInstanse *mb_prot = (ProtInstanse *)pr;
  mb_prot->state = TxCompleate;
}
//-----------------------------------------------------------------------------
void Siam_Handler(ProtInstanse *pr, Exchange *ex) {
  switch (pr->state) {
  default:
    break;
  case FrameReceived:
    ex->EnableRx(0);
    FrameProcess_Saim(pr);
  //  NRF_LOG_INFO("Here");
    if (pr->state == NotInit) {
      ex->EnableRx(1);
      break;
    }
    if (pr->state != ResponseReady)
      break;
  case ResponseReady:
    if (pr->sExType == ex->sExType) {
      ex->EnableTx(1);
 //     NRF_LOG_INFO("Here1");
      ex->TransmitFn(pr, pr->mBuf, pr->mLen); //передача
      pr->state = NotInit;
    }
    break;
  case FrameError:
    break;
  case TxCompleate:
    ex->EnableTx(0);
    ex->EnableRx(1);
    memset(&pr->mBuf, 0, sizeof(pr->mBuf));
    pr->mLen = 0;
    pr->idxIn = 0;
    pr->idxOut = 0;
    pr->state = NotInit;
    break;
  }
}

//-----------------------------------------------------------------------------