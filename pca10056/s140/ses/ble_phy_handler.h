#ifndef BLE_PHY_HANDLER_H__
#define BLE_PHY_HANDLER_H__

//#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_phy.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\exchenge.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_siam.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_modbus.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\components\ble\ble_link_ctx_manager\ble_link_ctx_manager.h"
#include "ble_nus_c.h"


void siam_data_handler(ble_siam_evt_t * p_evt);				//Обработчик событий сервиса BLE_SIAM
void modbus_data_handler(ble_modbus_evt_t * p_evt);		//Обработчик событий сервиса BLE_MODBUS

void BLE_Mb_Register(void* protocol, Exchange* ex);		//Регистрация протокола
void BLE_Siam_Register(void* protocol, Exchange* ex);	//Регистрация протокола	

uint16_t modbus_data_send(uint8_t* data, uint16_t len);
extern ble_nus_c_t * g_ble_nus_c_SIAM;
#endif