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
#include "ProtokolMb.h"
#include "ProtokolSiam.h"
#include "Utils.h"
#include "uart.h"


void scan_start(void);
uint8_t readFlashDEE(void);
void checkChangeID (uint16_t id);
void saveScannedDev (void);

#endif //MAIN_H__