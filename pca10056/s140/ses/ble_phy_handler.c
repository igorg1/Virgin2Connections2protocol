#include "ble_phy_handler.h"
#include "nrf_queue.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_siam.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_modbus.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_phy.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\components\ble\ble_link_ctx_manager\ble_link_ctx_manager.h"
#include "nrf_log.h"

//**********************************************************************************
//**********************************************************************************
//**********************************************************************************
/**@brief Структура очереди.
 *
 * @details
 *
 * @param[in]
 */
 /**@brief Структура очереди.
 *
 * @details
 *
 * @param[in]
 */
 typedef struct {
	uint8_t * p_data;
	uint32_t length;
} buffer_t;
//**********************************************************************************
//**********************************************************************************
//**********************************************************************************

//**********************************************************************************
//**********************************************************************************
//**********************************************************************************
gExchangeFn gOnBleReceiveSiam = NULL;
gCompleateFn gOnBleTxCompleteSiam = NULL;
void* gProtokolBleSiamInstans = NULL;
uint32_t* gServiceBleSiamFn = 0;
uint32_t* gConnHandlerSiam = 0;
int gEnableRxBleSiam=0;
int gEnableTxBleSiam=0;

NRF_QUEUE_DEF(buffer_t, mBleSiam_buf_queue, 10, NRF_QUEUE_MODE_NO_OVERFLOW);

void siam_data_handler(ble_siam_evt_t * p_evt)
{
	ret_code_t ret;
	buffer_t buf;

	switch(p_evt->type){
		case BLE_SIAM_EVT_RX_DATA:
			if((gOnBleReceiveSiam!= NULL))//&&gEnableRxUart0)
			{
			//Uart0_Enable();
			//timer_ble = sBluetoothCfg.mDisconnectTimer;
			buf.p_data = (uint8_t *)p_evt->params.rx_data.p_data;
			buf.length = p_evt->params.rx_data.length;
			NRF_LOG_DEBUG("BLE rx data len: %d", p_evt->params.rx_data.length);
			ret = nrf_queue_push(&mBleSiam_buf_queue, &buf);
			APP_ERROR_CHECK(ret);
			gOnBleReceiveSiam(gProtokolBleSiamInstans, (void*)&mBleSiam_buf_queue, 0);
			
			}
			break;
		case BLE_SIAM_EVT_TX_RDY:
			if(gOnBleTxCompleteSiam!=NULL)
			{
				gOnBleTxCompleteSiam(gProtokolBleSiamInstans);
			}
			break;
		case BLE_SIAM_EVT_COMM_STARTED:
			break;
		case BLE_SIAM_EVT_COMM_STOPPED:
			break;
		default:
			break;
	}
}

uint32_t ble_siam_data_send(ble_siam_t * p_nus,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_siam_client_context_t * p_client;

    VERIFY_PARAM_NOT_NULL(p_nus);

    err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage, conn_handle, (void *) &p_client);
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > BLE_SIAM_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_nus->tx_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}


uint16_t siam_data_send(uint8_t* data, uint16_t len)
{
	uint32_t err_code = ble_siam_data_send((void*)gServiceBleSiamFn, data, &len, *gConnHandlerSiam);
	if ((err_code != NRF_ERROR_INVALID_STATE) &&
		(err_code != NRF_ERROR_RESOURCES) &&
		(err_code != NRF_ERROR_NOT_FOUND))
	{
		APP_ERROR_CHECK(err_code);
		return 0;
	}
	return len;
}

void EnableRxBleSiam(int enable)
{
	gEnableRxBleSiam=enable;
}
void EnableTxBleSiam(int enable)
{
	gEnableTxBleSiam=enable;
}

void Ble_SIAM_SEND(void* protocol, void* data, uint16_t len)
{
	if(gEnableTxBleSiam)
	{
		SendData(siam_data_send, m_ble_siam_max_data_len, (uint8_t*)data, len );
	}
//	return sent;
}

void BLE_Siam_Register(void* protocol, Exchange* ex)
{
	gOnBleReceiveSiam = ex->OnReceiveFn;
	gOnBleTxCompleteSiam = ex->OnTxCompleate;

	ex->TransmitFn = Ble_SIAM_SEND;
	ex->EnableRx = EnableRxBleSiam;
	ex->EnableTx = EnableTxBleSiam;
	gProtokolBleSiamInstans = protocol;

	ex->EnableRx(1);
	Ble_Siam_ServReg((uint32_t*)&gServiceBleSiamFn, (uint32_t*)&gConnHandlerSiam);
}
//**********************************************************************************
//**********************************************************************************
//**********************************************************************************
gExchangeFn gOnBleReceiveMb = NULL;
gCompleateFn gOnBleTxCompleteMb = NULL;
void* gProtokolBleMbInstans = NULL;
uint32_t* gServiceBleMbFn = 0;
uint32_t* gConnHandlerMb = 0;
int gEnableRxBleMb=0;
int gEnableTxBleMb=0;

NRF_QUEUE_DEF(buffer_t, mBleMb_buf_queue, 10, NRF_QUEUE_MODE_NO_OVERFLOW);

void modbus_data_handler(ble_modbus_evt_t * p_evt)
{
	ret_code_t ret;
	buffer_t buf;

	switch(p_evt->type){
		case BLE_MODBUS_EVT_RX_DATA:
			if((gOnBleReceiveMb!= NULL))//&&gEnableRxUart0)
			{
			//Uart0_Enable();
			buf.p_data = (uint8_t *)p_evt->params.rx_data.p_data;
			buf.length = p_evt->params.rx_data.length;
			ret = nrf_queue_push(&mBleMb_buf_queue, &buf);
			APP_ERROR_CHECK(ret);
			gOnBleReceiveMb(gProtokolBleMbInstans, (void*)&mBleMb_buf_queue, 0);
			}
			break;
		case BLE_MODBUS_EVT_TX_RDY:
			if(gOnBleTxCompleteMb!=NULL)
			{
				gOnBleTxCompleteMb(gProtokolBleMbInstans);
			}
			break;
		case BLE_MODBUS_EVT_COMM_STARTED:
			break;
		case BLE_MODBUS_EVT_COMM_STOPPED:
			break;
		default:
			break;
	}
}

uint32_t ble_modbus_data_send(ble_modbus_t * p_modbus,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_modbus_client_context_t * p_client;

    VERIFY_PARAM_NOT_NULL(p_modbus);

    err_code = blcm_link_ctx_get(p_modbus->p_link_ctx_storage, conn_handle, (void *) &p_client);
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > BLE_MODBUS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_modbus->tx_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}



uint16_t modbus_data_send(uint8_t* data, uint16_t len)
{
	uint32_t err_code = ble_modbus_data_send((void*)gServiceBleMbFn, data, &len, *gConnHandlerMb);
	if ((err_code != NRF_ERROR_INVALID_STATE) &&
		(err_code != NRF_ERROR_RESOURCES) &&
		(err_code != NRF_ERROR_NOT_FOUND))
	{
		APP_ERROR_CHECK(err_code);
		return 0;
	}
	return len;
}

void EnableRxBleMb(int enable)
{
	gEnableRxBleMb=enable;
}
void EnableTxBleMb(int enable)
{
	gEnableTxBleMb=enable;
}

void Ble_MODBUS_SEND(void* protocol, void* data, uint16_t len)
{
	if(gEnableTxBleMb)
	{
		SendData(modbus_data_send, m_ble_siam_max_data_len, (uint8_t*)data, len );
	}
}

void BLE_Mb_Register(void* protocol, Exchange* ex)
{
	gOnBleReceiveMb = ex->OnReceiveFn;
	gOnBleTxCompleteMb = ex->OnTxCompleate;

	ex->TransmitFn = Ble_MODBUS_SEND;
	ex->EnableRx = EnableRxBleMb;
	ex->EnableTx = EnableTxBleMb;
	gProtokolBleMbInstans = protocol;

	ex->EnableRx(1);
	Ble_Mb_ServReg((uint32_t*)&gServiceBleMbFn, (uint32_t*)&gConnHandlerMb);
}
//**********************************************************************************
