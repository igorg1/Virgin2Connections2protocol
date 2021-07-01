#include "ble_phy.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_modbus.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_phy_handler.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_siam.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"

BLE_SIAM_DEF (
    m_siam, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
BLE_MODBUS_DEF (
    m_modbus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
//NRF_BLE_QWR_DEF (m_qwr); /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle =
    BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
uint16_t m_ble_siam_max_data_len =
    BLE_GATT_ATT_MTU_DEFAULT -
    3; /**< Maximum length of data (in bytes) that can be transmitted to the
          peer by the Nordic UART service module. */

/**@brief Function for Ble Disconnect.
 *
 * @details ...
 *
 * @warning ...
 * @warning ..
 */
void
ble_disconnect (void)
{
  uint32_t err_code;
  err_code = sd_ble_gap_disconnect (
      m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  APP_ERROR_CHECK (err_code);
  NRF_LOG_INFO ("BLE disconnect");
}

static void
nrf_qwr_error_handler (uint32_t nrf_error)
{
  APP_ERROR_HANDLER (nrf_error);
}

/**@brief Function for initializing services that will be used by the
 * application.
 *
 * @details Инициализация сервисов
 *
 */
void
Ble_Siam_ServReg (uint32_t *serv, uint32_t *conn)
{
  *serv = (uint32_t)&m_siam;
  *conn = (uint32_t)&m_conn_handle;
}
void
Ble_Mb_ServReg (uint32_t *serv, uint32_t *conn)
{
  *serv = (uint32_t)&m_modbus;
  *conn = (uint32_t)&m_conn_handle;
}

static void
services_init (void)
{
  uint32_t err_code;
  ble_siam_init_t siam_init;
  ble_modbus_init_t modbus_init;
  //	ble_dis_init_t			dis_init;
  //	ble_dis_sys_id_t		sys_id;
  nrf_ble_qwr_init_t qwr_init = { 0 };

  // Initialize Queued Write Module.
  //qwr_init.error_handler = nrf_qwr_error_handler;

 // err_code = nrf_ble_qwr_init (&m_qwr, &qwr_init);
 // APP_ERROR_CHECK (err_code);

  //Инициализация сервиса BLE_SIAM.
  memset (&siam_init, 0, sizeof (siam_init));
  siam_init.data_handler = siam_data_handler;
  err_code = ble_siam_init (&m_siam, &siam_init);
  APP_ERROR_CHECK (err_code);

  //Инициализация сервиса BLE_MODBUS.
  memset (&modbus_init, 0, sizeof (modbus_init));
  modbus_init.data_handler = modbus_data_handler;
  err_code = ble_modbus_init (&m_modbus, &modbus_init);
  APP_ERROR_CHECK (err_code);

  //Инициализация информационного сервиса.
  //	memset(&dis_init, 0, sizeof(dis_init));
  //	ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
  //	ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);
  //	sys_id.manufacturer_id				= MANUFACTURER_ID;
  //	sys_id.organizationally_unique_id	= ORG_UNIQUE_ID;
  //	dis_init.p_sys_id					= &sys_id;
  //	dis_init.dis_char_rd_sec			= SEC_OPEN;
  //	err_code = ble_dis_init(&dis_init);
  //	APP_ERROR_CHECK(err_code);
}