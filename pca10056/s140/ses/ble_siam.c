#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_siam.h"
#include "nrf_log.h"

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_siam_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_siam_evt_t              evt;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];
    ble_siam_client_context_t * p_client = NULL;

    err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage,
                                 p_ble_evt->evt.gap_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gap_evt.conn_handle);
    }

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_nus->tx_handles.cccd_handle,
                                      &gatts_val);

    if ((err_code == NRF_SUCCESS)     &&
        (p_nus->data_handler != NULL) &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        if (p_client != NULL)
        {
            p_client->is_notification_enabled = true;
        }

        memset(&evt, 0, sizeof(ble_siam_evt_t));
        evt.type        = BLE_SIAM_EVT_COMM_STARTED;
        evt.p_nus       = p_nus;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_nus->data_handler(&evt);
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_siam_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                    err_code;
    ble_siam_evt_t                 evt;
    ble_siam_client_context_t    * p_client;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    memset(&evt, 0, sizeof(ble_siam_evt_t));
    evt.p_nus       = p_nus;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;

    if ((p_evt_write->handle == p_nus->tx_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (p_client != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_client->is_notification_enabled = true;
                evt.type                          = BLE_SIAM_EVT_COMM_STARTED;
            }
            else
            {
                p_client->is_notification_enabled = false;
                evt.type                          = BLE_SIAM_EVT_COMM_STOPPED;
            }

            if (p_nus->data_handler != NULL)
            {
                p_nus->data_handler(&evt);
            }

        }
    }
    else if ((p_evt_write->handle == p_nus->rx_handles.value_handle) &&
             (p_nus->data_handler != NULL))
    {
        evt.type                  = BLE_SIAM_EVT_RX_DATA;
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;

        p_nus->data_handler(&evt);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(ble_siam_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_siam_evt_t              evt;
    ble_siam_client_context_t * p_client;

    err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
        return;
    }

    if ((p_client->is_notification_enabled) && (p_nus->data_handler != NULL))
    {
        memset(&evt, 0, sizeof(ble_siam_evt_t));
        evt.type        = BLE_SIAM_EVT_TX_RDY;
        evt.p_nus       = p_nus;
        evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_nus->data_handler(&evt);
    }
}



void ble_siam_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_siam_t * p_nus = (ble_siam_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_nus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_nus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_hvx_tx_complete(p_nus, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

