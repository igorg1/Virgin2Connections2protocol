#include "Protokol.h"
#include "ProtokolMb.h"
#include "ProtokolSiam.h"
#include "exchenge.h"
#include "mem_modem.h"
#include "mem_application.h"
#include "nrf_log.h"

//#include "ble_phy_handler.h"
#include "nrf_queue.h"
//#include "uart.h"
//#include "ble_modbus.h"
//#include "ble_phy_handler.h"

//**********************************************************************************
typedef struct
{
  uint8_t *p_data;
  uint32_t length;
} buffer_t;
//**********************************************************************************
typedef void (*ProtokolHandler) ();
ProtokolHandler gProtokolHandler; //Указатель на функцию Handlera протокола
//**********************************************************************************
ProtInstanse prot_inst_ble_mb;

ProtInstanse prot_inst_siam_master;//Uart replace
ProtInstanse prot_inst_mb_master;//Uart replace

ProtInstanse prot_inst_slave;

ProtocolMode sProtocolMode;
uint8_t sProtocolAddr;
//**********************************************************************************
Exchange ex_blemb;
Exchange ex_blesiam;

Exchange ex_uart0;
//**********************************************************************************
void
DoReceive (void *prot, void *input_buf, uint16_t buf_size)
{
  ret_code_t ret;
  buffer_t buf_queue;
  uint8_t queue_size = 0;

  ProtInstanse *pr = (ProtInstanse *)prot;
  pr->queue_adrr = input_buf;

  if (NULL != pr->queue_adrr)
    queue_size = (uint8_t)nrf_queue_max_utilization_get (pr->queue_adrr);
  NRF_LOG_DEBUG ("queue_size: %d", queue_size);
  if (queue_size)
    {
      for (uint8_t i = 1; i <= queue_size; i++)
        {
          ret = nrf_queue_pop (pr->queue_adrr, &buf_queue);
          APP_ERROR_CHECK (ret);

          memcpy (&pr->mBuf[pr->idxOut], buf_queue.p_data, buf_queue.length);
          pr->mLen = pr->idxOut += buf_queue.length;
          NRF_LOG_INFO ("DoReceive buf_size: %d", pr->mLen);
        }
    }
  else
    {
      pr->state = NotInit;
      return;
    }

  pr->state = FrameReceived;
}
//**********************************************************************************
void
Siam_Master (void) //Трансляция протокола SIAM через модем
{
  Siam_Handler (
      &prot_inst_siam_master, &ex_uart0); //Запросы от мастера передаем слейву
  Siam_Handler (
      &prot_inst_slave, &ex_blesiam); //Запросы от слейва передаем мастеру
}

void
ModBus_Master (void) //Трансляция протокола MODBUS через модем
{
  Modbus_Hanndler (
      &prot_inst_mb_master, &ex_uart0); //Запросы от мастера передаем слейву
  Modbus_Hanndler (
      &prot_inst_slave, &ex_blemb); //Запросы от слейва передаем мастеру
}

void
Siam_Slave (void) //Возврат протокола SIAM обратно в UART
{
  Siam_Handler (&prot_inst_slave, &ex_uart0);
}

void
ModBus_Slave (void) //Возврат протокола MODBUS обратно в UART
{
  Modbus_Hanndler (&prot_inst_slave, &ex_uart0);
}
//**********************************************************************************
void
Prot_Init (void)
{
  sProtocolAddr = sBL_Cfg.mModbusAddress;
  sProtocolMode = sProtocolCfg.mMode;

  /*****Master****/
  prot_inst_siam_master.addr = sProtocolAddr; // sBL_Cfg.mModbusAddress;
  prot_inst_siam_master.sProtokolType = prMaster;
  prot_inst_siam_master.state = NotInit;
  ex_blesiam.OnReceiveFn = DoReceive;
  ex_blesiam.OnTxCompleate = OnTxCompleate_Siam;
  ex_blesiam.sExType = 1;
  BLE_Siam_Register (&prot_inst_siam_master, &ex_blesiam);

  prot_inst_mb_master.addr = sProtocolAddr; // sBL_Cfg.mModbusAddress;
  prot_inst_mb_master.sProtokolType = prMaster;
  prot_inst_mb_master.state = NotInit;
  ex_blemb.OnReceiveFn = DoReceive;
  ex_blemb.OnTxCompleate = OnTxCompleate_Modbus;
  ex_blemb.sExType = 1;
  BLE_Mb_Register (&prot_inst_mb_master, &ex_blemb);

  /*****Slave****/
  prot_inst_slave.addr = sProtocolAddr; // sBL_Cfg.mModbusAddress;
  prot_inst_slave.state = NotInit;

  ex_uart0.OnReceiveFn = DoReceive;
  switch (sProtocolMode)
    {
    default:
      ex_uart0.sExType = 0;
      prot_inst_slave.sProtokolType = prSlave;
      ex_uart0.OnTxCompleate = OnTxCompleate_Siam;

      gProtokolHandler = Siam_Master;

      break;
    case SiamMaster:
      ex_uart0.sExType = 0;
      prot_inst_slave.sProtokolType = prSlave;
      ex_uart0.OnTxCompleate = OnTxCompleate_Siam;

      gProtokolHandler = Siam_Master;

      break;
    case ModBusMaster:
      ex_uart0.sExType = 0;
      prot_inst_slave.sProtokolType = prSlave;
      ex_uart0.OnTxCompleate = OnTxCompleate_Modbus;

      gProtokolHandler = ModBus_Master;

      break;
    case SiamSlave:
      ex_uart0.sExType = 1;
      prot_inst_slave.sProtokolType = prMaster;
      ex_uart0.OnTxCompleate = OnTxCompleate_Siam;

      gProtokolHandler = Siam_Slave;

      break;
    case ModBusSlave:
      ex_uart0.sExType = 1;
      prot_inst_slave.sProtokolType = prMaster;
      ex_uart0.OnTxCompleate = OnTxCompleate_Modbus;

      gProtokolHandler = ModBus_Slave;

      break;
    case prNULL:
      ex_uart0.sExType = 1;
      prot_inst_slave.sProtokolType = prMaster;
      ex_uart0.OnTxCompleate = OnTxCompleate_Modbus;

      gProtokolHandler = NULL;

      break;
    }
  Uart0_Register (&prot_inst_slave, &ex_uart0);
}
//**********************************************************************************
void
Prot_Handler (void)
{
  gProtokolHandler (); //указатель на хандлер

  Siam_Handler (
      &prot_inst_siam_master, &ex_blesiam); //Запросы от мастера отдаем мастеру
  Modbus_Hanndler (
      &prot_inst_mb_master, &ex_blemb); //Запросы от мастера отдаем мастеру
}
//**********************************************************************************