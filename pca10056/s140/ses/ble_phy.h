#ifndef BLE_PHY_H__
#define BLE_PHY_H__

#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_hci.h"

extern uint16_t   m_ble_siam_max_data_len;// = BLE_GATT_ATT_MTU_DEFAULT - 3;		/**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

void Ble_Siam_ServReg(uint32_t* serv, uint32_t* conn);
void Ble_Mb_ServReg(uint32_t* serv, uint32_t* conn);

#endif