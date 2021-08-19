#ifndef MAIN_H__
#define MAIN_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "sdk_config.h"
#include "nordic_common.h"

#include "ble_phy.h"
#include "mem_config.h"
#include "mem_application.h"
#include "mem_device.h"
#include "mem_modem.h"
#include "Protokol.h"
//#include "ProtokolMb.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ProtokolMB.h"
#include "ProtokolSiam.h"
#include "Utils.h"
#include "uart.h"
#include "exchenge.h"
#include "ProtokolMbStruct.h"
#include "ble_phy_handler.h"
#include "mem_application.h"


void scan_start(void);
uint16_t readFlashDEE(void);
void checkChangeID (uint16_t id);
void saveScannedDev (void);

#endif //MAIN_H__