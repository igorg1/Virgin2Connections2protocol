#include "uart.h"
//#include "nrf_libuarte_async.h"
#include "nrf_queue.h"
//#include "mem_config.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ProtokolMbStruct.h "

ProtokolInstanse *uarte1_buf_data = NULL;
//**********************************************************************************
void
uart_clear_buf (
    nrf_libuarte_async_t *p_libuarte, nrf_libuarte_async_evt_t *p_evt)
{
  nrf_libuarte_async_rx_free (
      p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
}
//**********************************************************************************

void Uart0_Register(void* protocol, Exchange* ex)
{
	memset(&uarte0_config, 0, sizeof(uarte0_config));

	//uarte0_config.tx_pin		= PIN_TXD0;
	//uarte0_config.rx_pin		= PIN_RXD0;
	//uarte0_config.baudrate		= NRF_UARTE_BAUDRATE_57600;
	//uarte0_config.parity		= NRF_UARTE_PARITY_EXCLUDED;
	//uarte0_config.hwfc			= NRF_UARTE_HWFC_DISABLED;
	//uarte0_config.timeout_us	= 10000;
	//uarte0_config.int_prio		= APP_IRQ_PRIORITY_LOW;

	//Uart0_UpdateCfg();
	
	gOnUartReceive0 = ex->OnReceiveFn;
	gOnUartTxCompleate0 = ex->OnTxCompleate;

	ex->TransmitFn = Sent0;
	ex->EnableRx = EnableRxUart0;
	ex->EnableTx = EnableTxUart0;
	gProtocolInstance0 = protocol;

	ex->EnableRx(1);
	//timer_uart0 = sUartCfg.mUart0Timer;
}
// gExchangeFn gOnUartReceive0 = NULL;
// gCompleateFn gOnUartTxCompleate0 = NULL;
// void *gProtocolInstance0 = NULL;
// int gEnableRxUart0 = 0;
// int gEnableTxUart0 = 0;