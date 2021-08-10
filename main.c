/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update
 * for such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "app_error.h"
#include "app_timer.h"
//#include "app_uart.h"
#include "app_util.h"
#include "ble.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_nus_c.h"
#include "bsp_btn_ble.h"
#include "libraries/atomic_flags/nrf_atflags.h"
#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\main.h"
#include "nrf_drv_clock.h"
#include "nrf_libuarte_async.h"
#include "nrf_queue.h"
#include <bsp.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\Protokol.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ProtokolMbStruct.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\Utils.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ble_phy_handler.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\exchenge.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\mem_application.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\mem_config.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\uart.h"
#include "nrf_ble_qwr.h"
#include "C:\NordicCurrentSDK\nRF5SDK1702d674dde\nRF5_SDK_17.0.2_d674dde\examples\ble_app_uart_c_Jury\ble_app_uart_c\pca10056\s140\ses\ProtokolSiam.h"

extern UART_ASSIGN connCurrent[62];

typedef struct
{
  uint16_t id_scan;
  ble_gap_addr_t peer_addr;
  uint8_t ttt;
  ble_gap_conn_params_t p_conn_params;
  ble_gap_scan_params_t p_scan_params;
  char name[20];
  uint8_t con_cfg_tag;
  uint8_t conHandler;
  uint8_t nameForNothing[64 - 2 * sizeof(uint32_t) // version confit in Flash
                         //- sizeof(uint32_t)    // crc
                         - sizeof(uint16_t) - sizeof(ble_gap_addr_t) - sizeof(uint8_t) - sizeof(ble_gap_scan_params_t) - sizeof(ble_gap_conn_params_t) - (20 * sizeof(char)) - sizeof(uint8_t) - sizeof(uint8_t)];

} SCANVAL;

extern HoldingReg HReg;
SCANVAL scanVal;
SCANVAL *readDEE;
//SCANVAL *dynScanSaveVal;
SCANVAL flashedDevicesArray[62];
uint8_t dynArrayCnt = 0; //do not change manually
uint8_t scanMyCnt = 0;   //do not change manually
uint8_t cntAll = 0;      //scanMyCnt+read from Flash count
volatile uint8_t workingPosition = 2;
const APP_Storage *app_stor = NULL;
const uint8_t siamID[2] = {0x0D, 0x0A};
uint8_t modem1Str[12] = {0x0d, 0x0a, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02,
    0x00, 0x90, 0x67}; // 0d0a01010000000002009067
uint8_t modem1StrMB[8] = {0x7F, 0x03, 0x00, 0x00, 0x00, 0x01, 0x8E,
    0x14}; // 7F03000000018E14
uint8_t modem2Str[12] = {0x0d, 0x0a, 0x7F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02,
    0x00, 0x16, 0xCF}; // 0d0a7F0100000000020016CF

uint16_t oldHRegID = NULL;
static bool firstScreen = false;
static int8_t cntForConnect = 1;
uint16_t savedFlashCnt = 0;
uint8_t readFlashDEE(void);
/* LibUARTE section begin */
NRF_LIBUARTE_ASYNC_DEFINE(libuarte0, 0, 1, NRF_LIBUARTE_PERIPHERAL_NOT_USED,
    NRF_LIBUARTE_PERIPHERAL_NOT_USED, 1024, 3); // UARTE0 is Siam
nrf_libuarte_async_t *gLibuarte0 = (nrf_libuarte_async_t *)&libuarte0;
NRF_LIBUARTE_ASYNC_DEFINE(libuarte1, 1, 2, NRF_LIBUARTE_PERIPHERAL_NOT_USED,
    NRF_LIBUARTE_PERIPHERAL_NOT_USED, 1024, 3); // UARTE1 is MB

static volatile bool m_loopback_phase0; // Siam
static volatile bool m_loopback_phase1; // MB

typedef struct
{
  uint8_t *p_data;
  uint32_t length;
} buffer_t;

NRF_QUEUE_DEF(buffer_t, m_buf_queue0, 10, NRF_QUEUE_MODE_NO_OVERFLOW); // Siam
NRF_QUEUE_DEF(buffer_t, m_buf_queue1, 10, NRF_QUEUE_MODE_NO_OVERFLOW); // MB

/* LibUARTE section end */

#define APP_BLE_CONN_CFG_TAG                                          \
  1 /**< Tag that refers to the BLE stack configuration set with @ref \
       sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO                                                 \
  3 /**< BLE observer priority of the application. There is no need to modify \
       this value. */

//#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE                                           \
  BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service \
                                (vendor specific). */

#define ECHOBACK_BLE_UART_DATA                                            \
  0 /**< Echo the UART data that is received over the Nordic UART Service \
       (NUS) back to the sender. */
#define SCAN_DURATION_WHITELIST \
  1000 /**< Duration of the scanning in units of 10 milliseconds. */
#define BLE_GAP_SCAN_BUFFER_EXTENDED_MIN (255)
#define NRF_BLE_SCAN_ACTIVE_SCANNING \
  1 /**< 0 -- passive scanning, 1 -- active scanning. */
BLE_NUS_C_ARRAY_DEF_SIAM(m_ble_nus_c_SIAM,
    NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< BLE Nordic UART Service
                                   (NUS) client instances. */
ble_nus_c_t *g_ble_nus_c_SIAM = (ble_nus_c_t *)&m_ble_nus_c_SIAM;
BLE_NUS_C_ARRAY_DEF_MB(m_ble_nus_c_MB,
    NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< BLE Nordic UART Service
                                     (NUS) client instances. */

NRF_BLE_GATT_DEF(m_gatt);        /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc); /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);        /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, /**< BLE GATT Queue instance. */
    NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len =
    BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH -
    HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be
                      transmitted to the peer by the Nordic UART service
                      module. */

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid = {.uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE};

static ble_uuid_t const m_nus_uuid_MB = {.uuid = BLE_UUID_NUS_SERVICE_MB,
    .type = NUS_SERVICE_UUID_TYPE};

#define BLE_UUID_NUS2_SERVICE 0xFFFF // 0x1101
static ble_uuid_t const m_nus2_uuid = {.uuid = BLE_UUID_NUS2_SERVICE,
    .type = 0xFF}; // 0x16

/**< Scan parameters requested for scanning and connection. BLE5/4 now */
static ble_gap_scan_params_t m_scan_param = {
    .active =
        0, // NRF_BLE_SCAN_ACTIVE_SCANNING, /* Disable the acvtive scanning */
    .interval = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout = SCAN_DURATION_WHITELIST,
    // .scan_phys = BLE_GAP_PHY_CODED,//BLE_GAP_PHY_1MBPS,
    .scan_phys = BLE_GAP_PHY_CODED | BLE_GAP_PHY_1MBPS,
    .report_incomplete_evts = 0,
    .extended = 1,
};

nrf_libuarte_async_config_t nrf_libuarte_async_config0 = {.tx_pin =
                                                              TX_PIN_NUMBER,
    .rx_pin = RX_PIN_NUMBER,
    .baudrate = NRF_UARTE_BAUDRATE_115200,
    .parity = NRF_UARTE_PARITY_EXCLUDED,
    .hwfc = NRF_UARTE_HWFC_DISABLED,
    .timeout_us = 1000,
    .int_prio = 5};

gExchangeFn gOnUartReceive0 = NULL;
gCompleateFn gOnUartTxCompleate0 = NULL;
void *gProtocolInstance0 = NULL;
int gEnableRxUart0 = 0;
int gEnableTxUart0 = 0;

void uart_clear_buf(
    nrf_libuarte_async_t *p_libuarte, nrf_libuarte_async_evt_t *p_evt) {
  nrf_libuarte_async_rx_free(
      p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
}
//**********************************************************************************
bool uart0_tx_complete = false;

uint16_t
uarte0_data_send(uint8_t *data, uint16_t len) {
  uart0_tx_complete = false;

  nrf_libuarte_async_tx(gLibuarte0, data, len);

  do {
    __WFE();
  } while (!uart0_tx_complete);
  return len;
}

void EnableRxUart0(int enable) {
  gEnableRxUart0 = enable;
}
void EnableTxUart0(int enable) {
  gEnableTxUart0 = enable;
}

void Sent0(void *protocol, void *data, uint16_t len) {
  if (gEnableTxUart0) {
    SendData(uarte0_data_send, MB_RTU_PDU_SIZE_MAX, (uint8_t *)data, len);
  }
}

void uart_event_handler0(void *context, nrf_libuarte_async_evt_t *p_evt);

uint8_t
Uart0_Enable(void) {

  ret_code_t err_code;
  if (!libuarte0.p_libuarte->uarte->ENABLE) {
    err_code =
        nrf_libuarte_async_init(&libuarte0, &nrf_libuarte_async_config0,
            uart_event_handler0, (void *)&libuarte0);
    APP_ERROR_CHECK(err_code);
    nrf_libuarte_async_enable(gLibuarte0);
    NRF_LOG_INFO("Uart0_Enable");
    return 1;
  }
  return 0;
}

uint8_t
Uart0_Disable(void) {
  if (libuarte0.p_libuarte->uarte->ENABLE) {
    nrf_libuarte_async_uninit(&libuarte0);
    NRF_LOG_INFO("Uart0_Disable");
    return 1;
  }
  return 0;
}

void Uart0_Register(void *protocol, Exchange *ex) {
  memset(&nrf_libuarte_async_config0, 0, sizeof(nrf_libuarte_async_config0));

  // uarte0_config.tx_pin		= PIN_TXD0;
  // uarte0_config.rx_pin		= PIN_RXD0;
  // uarte0_config.baudrate		= NRF_UARTE_BAUDRATE_57600;
  // uarte0_config.parity		= NRF_UARTE_PARITY_EXCLUDED;
  // uarte0_config.hwfc			= NRF_UARTE_HWFC_DISABLED;
  // uarte0_config.timeout_us	= 10000;
  // uarte0_config.int_prio		= APP_IRQ_PRIORITY_LOW;

  // Uart0_UpdateCfg();

  gOnUartReceive0 = ex->OnReceiveFn;
  gOnUartTxCompleate0 = ex->OnTxCompleate;

  ex->TransmitFn = Sent0;
  ex->EnableRx = EnableRxUart0;
  ex->EnableTx = EnableTxUart0;
  gProtocolInstance0 = protocol;

  ex->EnableRx(1);
  // timer_uart0 = sUartCfg.mUart0Timer;
}

void uart_event_handler0(void *context, nrf_libuarte_async_evt_t *p_evt) // Siam
{
  nrf_libuarte_async_t *p_libuarte = (nrf_libuarte_async_t *)context;
  ret_code_t ret;
  uint32_t ret_val;
  buffer_t buf;

  switch (p_evt->type) {
  case NRF_LIBUARTE_ASYNC_EVT_ERROR:
    break;
  case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
    //      if (false == memcmp (p_evt->data.rxtx.p_data, siamID, 2))
    //       {
    //         ret_val = ble_nus_c_string_send (&m_ble_nus_c_SIAM[0],
    //            p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
    //      }
    //    else
    //       {
    //        ret_val = ble_nus_c_string_send (&m_ble_nus_c_MB[0],
    //            p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
    //      }

    //  // ret = nrf_libuarte_async_tx (
    //   //     p_libuarte, p_evt->data.rxtx.p_data,
    //   p_evt->data.rxtx.length);
    //   // if (ret_val == NRF_ERROR_BUSY)//ret
    //   //  {
    //   //    buffer_t buf = {
    //   //     .p_data = p_evt->data.rxtx.p_data,
    //   //      .length = p_evt->data.rxtx.length,
    //   //    };
    if ((gOnUartReceive0 != NULL)) //&&gEnableRxUart0)
    {
      buf.p_data = p_evt->data.rxtx.p_data;
      buf.length = p_evt->data.rxtx.length;
      NRF_LOG_INFO("uart rx data len: %d", p_evt->data.rxtx.length);
      ret = nrf_queue_push(&m_buf_queue0, &buf);
      APP_ERROR_CHECK(ret);
      gOnUartReceive0(
          gProtocolInstance0, (void *)&m_buf_queue0, 0); //@TODO
    }
    uart_clear_buf(p_libuarte, p_evt);
    // }
    //  else
    //   {
    //      APP_ERROR_CHECK (ret_val);
    //    }

  //  m_loopback_phase0 = true;
    break;
  case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
    uart0_tx_complete = true;
    // if(false==memcmp(p_evt->data.rxtx.p_data,siamID,2))
    // {
    //     ret_val = ble_nus_c_string_send (&m_ble_nus_c_SIAM[0],
    //         p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
    //             }
    // else
    // {
    //     ret_val = ble_nus_c_string_send (&m_ble_nus_c_MB[0],
    //         p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
    // }
    // if (m_loopback_phase0)
    //   {
    //      m_loopback_phase0 = false;
    //      nrf_libuarte_async_rx_free (
    //           p_libuarte, p_evt->data.rxtx.p_data,
    //           p_evt->data.rxtx.length);
    //      if (!nrf_queue_is_empty (&m_buf_queue0))
    //        {
    //          buffer_t buf;
    //          ret = nrf_queue_pop (&m_buf_queue0, &buf);
    //          APP_ERROR_CHECK (ret);
    // UNUSED_RETURN_VALUE (
    //     nrf_libuarte_async_tx (p_libuarte, buf.p_data,
    //     buf.length));
    //         }

    //  }
    if (gOnUartTxCompleate0 != NULL) {
      gOnUartTxCompleate0(gProtocolInstance0);
    }
    break;
  default:
    break;
  }
}

void uart_event_handler1(void *context, nrf_libuarte_async_evt_t *p_evt) // MB
{
  nrf_libuarte_async_t *p_libuarte = (nrf_libuarte_async_t *)context;
  ret_code_t ret;

  switch (p_evt->type) {
  case NRF_LIBUARTE_ASYNC_EVT_ERROR:
    break;
  case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
    ret = nrf_libuarte_async_tx(
        p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
    if (ret == NRF_ERROR_BUSY) {
      buffer_t buf = {
          .p_data = p_evt->data.rxtx.p_data,
          .length = p_evt->data.rxtx.length,
      };

      ret = nrf_queue_push(&m_buf_queue1, &buf);
      APP_ERROR_CHECK(ret);
    } else {
      APP_ERROR_CHECK(ret);
    }
    m_loopback_phase1 = true;
    break;
  case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
    if (m_loopback_phase1) {
      nrf_libuarte_async_rx_free(
          p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
      if (!nrf_queue_is_empty(&m_buf_queue1)) {
        buffer_t buf;
        ret = nrf_queue_pop(&m_buf_queue1, &buf);
        APP_ERROR_CHECK(ret);
        UNUSED_RETURN_VALUE(
            nrf_libuarte_async_tx(p_libuarte, buf.p_data, buf.length));
      }
    }
    break;
  default:
    break;
  }
}

static uint8_t scan_buffer_data[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN];
static ble_data_t scan_buffer = {scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN};

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final
 * product. You need to analyze how your product is supposed to react in case
 * of assert.
 * @warning On assert from the SoftDevice, the system can only recover on
 * reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went
 * wrong.
 */
static void
nus_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function to start scanning. */
void scan_start(void) {
  ret_code_t ret;

  // ret = nrf_ble_scan_start (&m_scan);
  ret = sd_ble_gap_scan_start(&m_scan_param, &scan_buffer);
  APP_ERROR_CHECK(ret);
  if ((ret != NRF_ERROR_INVALID_STATE) && (ret != NRF_SUCCESS))

  {

    NRF_LOG_ERROR("sd_ble_gap_scan_start returned 0x%x", ret);
  }

  NRF_LOG_INFO("Starting scan.");

  //  ret = bsp_indication_set (BSP_INDICATE_SCANNING);
  //  APP_ERROR_CHECK (ret);

  ret = bsp_indication_set(BSP_INDICATE_SCANNING);
  APP_ERROR_CHECK(ret);
}

/**@brief Function for handling Scanning Module events.
 */
uint8_t mac[6] = {0x2b, 0x4b, 0x41, 0x87, 0xd0, 0xf1};
// uint8_t mac[6] = {0xf1, 0xd0, 0x87, 0x41, 0x4b, 0x2b};
uint8_t id_connect = NULL;
uint8_t cntConnect = 0;
const uint8_t type024[21] = {0x24, 0x44, 0xD0, 0xA2, 0xD0, 0x9D, 0xD0, 0x9F,
    0xD0, 0x92, 0xD0, 0x9E, 0x20, 0xD0, 0xA1, 0xD0, 0x98, 0xD0, 0x90, 0xD0,
    0x9C};
const uint8_t type016[13] = {0x16, 0x01, 0x11, 0x54, 0x4E, 0x50, 0x56, 0x4F,
    0x20, 0x53, 0x49, 0x41, 0x4D};
const uint8_t type07[17] = {0x07, 0x7C, 0x16, 0xA5, 0x5E, 0xBA, 0x11, 0xCB,
    0x92, 0x0C, 0x49, 0x7F, 0xB8, 0x01, 0x11, 0x9A, 0x56};
uint8_t type024Cmp[21] = {0};
uint8_t type016Cmp[13] = {0};
uint8_t type07Cmp[17] = {0};
m_scan_struct scanMy[20] = {0};
m_scan_struct temp[1] = {0};
uint8_t writeScanMy[6] = {0};
uint8_t Scanned[6] = {0};
uint8_t temp_;
bool findUnique = false;

static bool firstScan = true;
static bool findMatch = false;
volatile bool withName = false;
volatile bool readWriteFlash = false;
static uint8_t forCnt = 0;
uint8_t oldForCnt = 0;

static void
scan_evt_handler(scan_evt_t const *p_scan_evt) {
  ret_code_t err_code;
  static bool isDone = false;
  int i, j;

  switch (p_scan_evt->scan_evt_id) {
  case NRF_BLE_SCAN_EVT_CONNECTING_ERROR: {
    err_code = p_scan_evt->params.connecting_err.err_code;
    APP_ERROR_CHECK(err_code);
  } break;
    // Scan is automatically stopped by the connection.
  case NRF_BLE_SCAN_EVT_CONNECTED: {

    ble_gap_evt_connected_t const *p_connected =
        p_scan_evt->params.connected.p_connected;

    NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
        p_connected->peer_addr.addr[0], p_connected->peer_addr.addr[1],
        p_connected->peer_addr.addr[2], p_connected->peer_addr.addr[3],
        p_connected->peer_addr.addr[4], p_connected->peer_addr.addr[5]);
    // if (false == isDone)
    // {
    //   isDone = true;
    // for (int i = 0; i < cntConnect; i++)
    //   {
    //     if (false ==
    //          memcmp (scanMy[i].peer_addr.addr, modem1, 6)) // My modem 2
    //        NRF_LOG_HEXDUMP_INFO (scanMy[i].peer_addr.addr, 6);
    //        err_code = sd_ble_gap_connect (&scanMy[i].peer_addr,
    //          &scanMy[i].p_scan_params, &scanMy[i].p_conn_params,
    //           scanMy[i].con_cfg_tag);
    //           cntConnect=0;

    //     }
    // }
  } break;
  case NRF_BLE_SCAN_EVT_NOT_FOUND: {
    if (true == firstScan) {
      firstScan = false;
      for (i = 0; i < 20; i++) {
        memset(&scanMy[i], 0, sizeof(scanMy[0]));
      }
    }

    printf("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
        temp[0].peer_addr.addr[0], temp[0].peer_addr.addr[1],
        temp[0].peer_addr.addr[2], temp[0].peer_addr.addr[3],
        temp[0].peer_addr.addr[4], temp[0].peer_addr.addr[5]);
    printf(temp[0].data);
    printf("\r\n");
    printf("is "
           "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%"
           "02x%02x%02x\r\n",
        temp[0].data[0], temp[0].data[1], temp[0].data[2], temp[0].data[3],
        temp[0].data[4], temp[0].data[5], temp[0].data[6], temp[0].data[7],
        temp[0].data[8], temp[0].data[9], temp[0].data[10],
        temp[0].data[11], temp[0].data[12], temp[0].data[13],
        temp[0].data[14], temp[0].data[15], temp[0].data[16],
        temp[0].data[17]);
    printf("\r\n\r\n");
    // memmove (type024Cmp, temp[0].data + 16, 21);
    // memmove (type016Cmp, temp[0].data + 4, 13);
    //  memmove (type07Cmp, temp[0].data + 16, 17);//20
    //        printf("type07 is
    //        %02x%02x%02x%02x%02x%02x\r\n",type07Cmp[0],type07Cmp[1],type07Cmp[2],type07Cmp[3],type07Cmp[4],type07Cmp[5]);
    //   if ((false == memcmp (type024Cmp, type024, 21)) ||
    //       (false == memcmp (type016Cmp, type016, 13)) ||
    //       (false == memcmp (type07Cmp, type07, 16)))//17
    //     {
    // For name
    // if ((false == memcmp (type024Cmp, type024, 21)))
    //    {
    //      memmove (temp[0].name, temp[0].data + 3, 10);
    //     }
    //   else if ((false == memcmp (type016Cmp, type016, 13)))
    //     {
    //       memmove (temp[0].name, temp[0].data + 19, 12);
    //     }
    //    else if ((false == memcmp (type07Cmp, type07, 17)))
    //      {
    //        memmove (temp[0].name, temp[0].data + 15, 17);
    //      }

    //{
    if ((cntConnect == 0) && (withName == true)) {
      memcpy(&scanMy[cntConnect], &temp, sizeof(temp));
      cntConnect++;
      withName = false;
    } else {
      memmove(Scanned, &temp[0].peer_addr.addr, 6);
      findMatch = false;
      for (int j = 0; j <= cntConnect; j++) {
        memmove(&writeScanMy, scanMy[j].peer_addr.addr, 6);
        if (false == memcmp(Scanned, writeScanMy, 6))
          findMatch = true;
      }
      if ((!findMatch) && (withName == true)) {
        memcpy(&scanMy[cntConnect], temp, sizeof(temp));
        cntConnect++;
        withName = false;
      }
    }
    //}
  } break;

  case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT: {
    printf("Scan timed out.\r\n");
    printf("Found next devices:\r\n");
    printf("N\t\t\t\t\t\t Name\t\t\t\t\t\t\t\t\t Mac Address\r\n");
    for (i = 0; i < 20; i++) // arraySize of ScanMy
    {
      if (scanMy[i].peer_addr.addr[0] == 0x0) {
        temp_ = i;
        break;
      }
    }
    for (j = 0; j < temp_; j++) {
      printf("%d\t\t\t\t\t\t %s\t\t\t\t\t\t\t\t 0x%x%x%x%x%x%x \r\n", j,
          scanMy[j].name, scanMy[j].peer_addr.addr[0],
          scanMy[j].peer_addr.addr[1], scanMy[j].peer_addr.addr[2],
          scanMy[j].peer_addr.addr[3], scanMy[j].peer_addr.addr[4],
          scanMy[j].peer_addr.addr[5]);
    }
    // scan_start();
    firstScan = true;
    cntConnect = 0;
    findMatch = false;
    withName = false;
    //   NRF_LOG_INFO ("Scan Done, save on Flash Memory");
    for (i = 0; i < 20; i++) // arraySize of ScanMy
    {
      if (scanMy[i].peer_addr.addr[0] == 0x0) //i now how many in ScanMy
        break;
    }
    scanMyCnt = i;
    // HReg.cntScanVal += scanMyCnt;
    findUnique = false;
    if (cntAll == 0)
      oldForCnt = scanMyCnt;
    else
      oldForCnt = cntAll;
    for (i = 0; i < scanMyCnt; i++) {
      findUnique = false;
      for (int j = 0; j < oldForCnt; j++) {
        if (false == memcmp(&HReg.workingScanVal[j].mac_addr, scanMy[i].peer_addr.addr, 6)) {
          findUnique = true;
        }
      }
      if (!findUnique) {
      //HReg
        memcpy(&HReg.workingScanVal[cntAll].mac_addr, scanMy[i].peer_addr.addr, 6);
        memcpy(&HReg.workingScanVal[cntAll].name, scanMy[i].name, 20);

      //ConCurrent
        memcpy(&connCurrent[cntAll].addr,scanMy[i].peer_addr.addr, 6);
        cntAll++;
      }
    }
    HReg.cntScanVal = cntAll;

    NRF_LOG_INFO("Read from Flash memory\r\n");

    //readWriteFlash = true;

    //        const uint8_t modem3[6] = { 0x28, 0xEC, 0xCB, 0xAB, 0xEA,
    //        0xE9 };
    // const uint8_t modem1[6] = { 0x7C, 0x60, 0x72, 0x8C, 0xAD, 0xF6 };
    //  for (int i = 0; i < cntConnect; i++)
    //   {
    //     if (false ==
    //         memcmp (scanMy[i].peer_addr.addr, modem3, 6)) // My modem 3
    //       {
    //         NRF_LOG_HEXDUMP_INFO (scanMy[i].peer_addr.addr, 6);
    //         err_code = sd_ble_gap_connect (&scanMy[i].peer_addr,
    //             &scanMy[i].p_scan_params, &scanMy[i].p_conn_params,
    //             scanMy[i].con_cfg_tag);
    //       }
    //  }
  } break;

  default:
    break;
  }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void
scan_init(void) {
  ret_code_t err_code;
  nrf_ble_scan_init_t init_scan;
  // ble_gap_scan_params_t scan_param;
  memset(&init_scan, 0, sizeof(init_scan));
  init_scan.p_scan_param = &m_scan_param;
  // memset (&scan_param, 0, sizeof (scan_param));
  init_scan.connect_if_match = false;
  init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

  err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
  APP_ERROR_CHECK(err_code);

  //  err_code = nrf_ble_scan_filter_set (&m_scan, SCAN_UUID_FILTER,
  //  &m_nus_uuid); APP_ERROR_CHECK (err_code);

  err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus2_uuid);
  APP_ERROR_CHECK(err_code);

  err_code =
      nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the
 * database discovery module. Depending on the UUIDs that are discovered, this
 * function forwards the events to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void
db_disc_handler(ble_db_discovery_evt_t *p_evt) {

  NRF_LOG_INFO("uuid 0x%x; type %x",
      p_evt->params.discovered_db.srv_uuid.uuid,
      p_evt->params.discovered_db.srv_uuid.type);
  ble_nus_c_on_db_disc_evt_SIAM(&m_ble_nus_c_SIAM[p_evt->conn_handle], p_evt);
  ble_nus_c_on_db_disc_evt_MB(&m_ble_nus_c_MB[p_evt->conn_handle], p_evt);
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client
 * events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS
 * client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void
ble_nus_c_evt_handler(
    ble_nus_c_t *p_ble_nus_c, ble_nus_c_evt_t const *p_ble_nus_evt) {
  ret_code_t err_code;

  switch (p_ble_nus_evt->evt_type) {
  case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
    NRF_LOG_INFO("Discovery complete. TX type %x",
        p_ble_nus_c->handles.nus_tx_handle);
    err_code = ble_nus_c_handles_assign(
        p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
    APP_ERROR_CHECK(err_code);

    err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Connected to device with Nordic UART Service, handle: "
                 "%x, evt_handle: %x",
        p_ble_nus_c->conn_handle, p_ble_nus_evt->conn_handle);
    break;

  case BLE_NUS_C_EVT_NUS_TX_EVT:
    NRF_LOG_INFO("Receive N \t%x\r\n", p_ble_nus_c->conn_handle);
    err_code = nrf_libuarte_async_tx(
        &libuarte0, p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
      //ble_nus_chars_received_uart_print_SIAM (
      //    p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
    //    ble_nus_chars_received_uart_print_MB (
    //      p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
    break;

  case BLE_NUS_C_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected.");
    // scan_start ();
    break;
  }
}
/**@snippet [Handling events from the ble_nus_c module] */

/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool
shutdown_handler(nrf_pwr_mgmt_evt_t event) {
  ret_code_t err_code;

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  switch (event) {
  case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
    break;

  default:
    break;
  }

  return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(
    shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */

static void
ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  ret_code_t err_code;
  ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    err_code = ble_nus_c_handles_assign(
        &m_ble_nus_c_MB[p_ble_evt->evt.gap_evt.conn_handle],
        p_ble_evt->evt.gap_evt.conn_handle, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ble_nus_c_handles_assign(
        &m_ble_nus_c_SIAM[p_ble_evt->evt.gap_evt.conn_handle],
        p_ble_evt->evt.gap_evt.conn_handle, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);

    // start discovery of services. The NUS Client waits for a discovery
    // result
    err_code = ble_db_discovery_start(
        &m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Connected. conn_handle: 0x%x", p_gap_evt->conn_handle);

    if (ble_conn_state_central_conn_count() <
        NRF_SDH_BLE_CENTRAL_LINK_COUNT) {
      // Resume scanning.
      //          scan_start ();
    }
    break;

  case BLE_GAP_EVT_DISCONNECTED:

    NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
        p_gap_evt->conn_handle, p_gap_evt->params.disconnected.reason);
    break;

  case BLE_GAP_EVT_TIMEOUT:
    if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
      NRF_LOG_INFO("Connection Request timed out.");
    }
    break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    // Pairing not supported.
    err_code =
        sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
            BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
    // Accepting parameters requested by peer.
    err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
        &p_gap_evt->params.conn_param_update_request.conn_params);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys = {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
    };
    err_code =
        sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

    //  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    //     NRF_LOG_DEBUG ("GATT Server Timeout.");
    //     err_code = sd_ble_gap_disconnect
    //     (p_ble_evt->evt.gatts_evt.conn_handle,
    //          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    //     APP_ERROR_CHECK (err_code);
    //     break;

  default:
    break;
  }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void
ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(
      m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
    NRF_LOG_INFO("ATT MTU exchange completed.");

    m_ble_nus_max_data_len =
        p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)",
        m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void) {
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_central_set(
      &m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */

void bsp_event_handler(bsp_event_t event) {
  ret_code_t err_code;
  uint32_t ret_val;
  int i;

  // uint32_t m = 0;
  // pTest1=(int *)malloc(5);
  const uint8_t modem1[6] = {0x7C, 0x60, 0x72, 0x8C, 0xAD, 0xF6};
  const uint8_t modem2[6] = {0x77, 0x8C, 0xF7, 0xDC, 0x0B, 0xEF};
  const uint8_t modem3[6] = {0x28, 0xEC, 0xCB, 0xAB, 0xEA, 0xE9};
  const uint8_t modemDDIM2[6] = {0x12, 0x02, 0x72, 0xA4, 0x16, 0x00};

  switch (event) {
    //    case BSP_EVENT_SLEEP:
    //      nrf_pwr_mgmt_shutdown (NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    //      break;

    //    case BSP_EVENT_DISCONNECT:
    //    const uint8_t modem1[6] = { 0x7C, 0x60, 0x72, 0x8C, 0xAD, 0xF6 };
    //      for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
    //        {
    //          err_code = sd_ble_gap_disconnect (m_ble_nus_c[c].conn_handle,
    //              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    //          if (err_code != NRF_ERROR_INVALID_STATE)
    //           {
    //              APP_ERROR_CHECK (err_code);
    //            }
    //        }
    //      break;

  case BSP_EVENT_KEY_0: {
    // NRF_LOG_INFO ("BSP_EVENT_KEY_0");
    NRF_LOG_INFO("Scan start");
    scan_start();
    //  for (int i = 0; i < cntConnect; i++)
    //     {
    //      if (false ==
    //          memcmp (scanMy[i].peer_addr.addr, modem2, 6)) // My modem 2
    //       {
    //         NRF_LOG_HEXDUMP_INFO (scanMy[i].peer_addr.addr, 6);
    //         err_code = sd_ble_gap_connect (&scanMy[i].peer_addr,
    //             &scanMy[i].p_scan_params, &scanMy[i].p_conn_params,
    //             scanMy[i].con_cfg_tag);
    //         APP_ERROR_CHECK (err_code);
    //        }
    //   }
  } break;

  case BSP_EVENT_KEY_1: {
    if (!firstScreen) {
      firstScreen = true;
      readFlashDEE();
    }
    cntForConnect++;
    if (cntForConnect > (savedFlashCnt - 1))
      cntForConnect = 0;
    NRF_LOG_INFO("Number For device to connect: %d", cntForConnect);

    /* begin section*/
    //   NRF_LOG_INFO("Scan Done, save on Flash Memory");
    //   for (i = 0; i < 20; i++) // arraySize of ScanMy
    //   {
    //     if (scanMy[i].peer_addr.addr[0] == 0x0)
    //       break;
    //   }
    /*end section*/
    // memset (scanVal, 0, sizeof (scanVal));
    //    scanVal = (SCANVAL *)malloc (i * sizeof (SCANVAL));
    // if (scanVal == NULL)
    //   NRF_LOG_INFO ("Error. Can't work further");
    /* begin section*/
    //  for (size_t j = 0; j < i; j++) {
    //     memset(&scanVal, 0, sizeof(SCANVAL));
    //     scanVal.id_scan = j;
    //     memcpy(scanVal.name, scanMy[j].name, 20);
    //     memcpy(&scanVal.con_cfg_tag, &scanMy[j].con_cfg_tag, 1);
    //      memcpy(&scanVal.p_conn_params, &scanMy[j].p_conn_params,
    //          sizeof(scanVal.p_conn_params));
    //     memcpy(&scanVal.addr, &scanMy[j].peer_addr.addr, 6);
    //     Flash_SaveStor(FLASH_DEECFG_START_ADDR, FLASH_DEECFG_END_ADDR,
    //         sizeof(SCANVAL), (uint32_t *)&scanVal);
    //   }
    /*end section*/
    //   Flash_SaveStor (FLASH_DEECFG_START_ADDR, FLASH_DEECFG_END_ADDR,
    //       sizeof (SCANVAL) * i, (uint32_t *)scanVal);
    //  NRF_LOG_INFO("New data was Flashed");
    //    free (scanVal);
    //   for (int i = 0; i < cntConnect; i++)
    //     {
    //       if (false ==
    //            memcmp (scanMy[i].peer_addr.addr, modem1, 6)) // My modem
    //            1
    //         {
    //           NRF_LOG_HEXDUMP_INFO (scanMy[i].peer_addr.addr, 6);
    //           err_code = sd_ble_gap_connect (&scanMy[i].peer_addr,
    //               &scanMy[i].p_scan_params, &scanMy[i].p_conn_params,
    //               scanMy[i].con_cfg_tag);
    //           APP_ERROR_CHECK (err_code);
    //        }
    //    }

  } break;
  case BSP_EVENT_KEY_2: {
    if (!firstScreen) {
      firstScreen = true;
      readFlashDEE();
    }
    cntForConnect--;
    if (cntForConnect < 0)
      cntForConnect = (savedFlashCnt - 1);
    NRF_LOG_INFO("Number For device to connect: %d", cntForConnect);

    /* begin section*/
    //  NRF_LOG_INFO ("Read from Flash memory");
    //   uint32_t endMemoryPtr = (uint32_t)Flash_LoadStor (
    //       FLASH_DEECFG_START_ADDR, FLASH_DEECFG_END_ADDR, sizeof (SCANVAL));
    //    i = 0;
    //    memset (&readDEE, 0, sizeof (SCANVAL) * 20);
    //   for (m = 0xF1000; m < endMemoryPtr; m += 0x40)
    //     {
    //       memcpy( &readDEE[i],(uint32_t*)m,sizeof(SCANVAL));
    //       i++;
    //     }
    //    NRF_LOG_INFO ("Test");
    /*end section*/

    // NRF_LOG_INFO ("BSP_EVENT_KEY_2");
    // do
    //   {
    //     ret_val =
    //         ble_nus_c_string_send (&m_ble_nus_c_SIAM[0], modem2Str, 12);
    //     if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) &&
    //         (ret_val != NRF_ERROR_INVALID_STATE))
    //       {
    //         NRF_LOG_ERROR (
    //             "Failed sending NUS message. Error 0x%x. ", ret_val);
    //         APP_ERROR_CHECK (ret_val);
    //       }

    //  }
    // while (ret_val == NRF_ERROR_BUSY);
  } break;
  case BSP_EVENT_KEY_3: {
    //  NRF_LOG_HEXDUMP_INFO(&dynScanSaveVal[cntForConnect].peer_addr.addr, 6);
    //  err_code = sd_ble_gap_connect(&dynScanSaveVal[cntForConnect].peer_addr, &dynScanSaveVal[cntForConnect].p_scan_params, &dynScanSaveVal[cntForConnect].p_conn_params, dynScanSaveVal[cntForConnect].con_cfg_tag);
    //  APP_ERROR_CHECK(err_code);
    /* begin section*/
    //   NRF_LOG_INFO("BSP_EVENT_KEY_3");

    //   for (int i = 0; i < cntConnect; i++) {
    //     if (false == memcmp(scanMy[i].peer_addr.addr, modemDDIM2,
    //                       6)) // modemDDIM180
    //      {
    //       NRF_LOG_HEXDUMP_INFO(scanMy[i].peer_addr.addr, 6);
    //        err_code = sd_ble_gap_connect(&scanMy[i].peer_addr,
    //            &scanMy[i].p_scan_params, &scanMy[i].p_conn_params,
    //            scanMy[i].con_cfg_tag);
    //        APP_ERROR_CHECK(err_code);
    //      }
    /*end section*/
  }
  // do
  //    {
  //      ret_val = ble_nus_c_string_send (&m_ble_nus_c_MB[0],
  //      modem1StrMB,
  //          8); // ble_nus_c_string_send (&m_ble_nus_c_SIAM[0],
  //          modem2Str,
  // 12);
  //      if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) &&
  //          (ret_val != NRF_ERROR_INVALID_STATE))
  //        {
  //          NRF_LOG_ERROR (
  //              "Failed sending NUS message. Error 0x%x. ", ret_val);
  //          APP_ERROR_CHECK (ret_val);
  // }
  //    }
  //  while (ret_val == NRF_ERROR_BUSY);
  // }

  break;

  default:
    break;
  }
}

/**@brief Function for initializing the UART. */

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void
nus_c_init(void) {
  ret_code_t err_code;
  ble_nus_c_init_t init;

  init.evt_handler = ble_nus_c_evt_handler;
  init.error_handler = nus_error_handler;
  init.p_gatt_queue = &m_ble_gatt_queue;
  for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++) {
    err_code = ble_nus_c_init(&m_ble_nus_c_SIAM[c], &init);
    APP_ERROR_CHECK(err_code);
    err_code = ble_nus_c_initMB(&m_ble_nus_c_MB[c], &init);
    APP_ERROR_CHECK(err_code);
  }

  // for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
  //  {
  //     err_code = ble_nus_c_initMB (&m_ble_nus_c_MB[c], &init);
  //     APP_ERROR_CHECK (err_code);
  //   }
}

/**@brief Function for initializing buttons and leds. */
static void
buttons_leds_init(void) {
  ret_code_t err_code;
  bsp_event_t startup_event;

  err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the timer. */
static void
timer_init(void) {
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module. */
static void
log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void
power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/** @brief Function for initializing the database discovery module. */
static void
db_discovery_init(void) {
  ble_db_discovery_init_t db_init;

  memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

  db_init.evt_handler = db_disc_handler;
  db_init.p_gatt_queue = &m_ble_gatt_queue;

  ret_code_t err_code = ble_db_discovery_init(&db_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next
 * event occurs.
 */
static void
idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

void readFlash(void) {
  int i, j;
  uint32_t m = 0;
  SCANVAL *arrayForTemp;
  uint8_t forTempArray = 0;
  uint32_t endMemoryPtr = (uint32_t)Flash_LoadStor(
      FLASH_DEECFG_START_ADDR, FLASH_DEECFG_END_ADDR, sizeof(SCANVAL));
  if (endMemoryPtr > FLASH_DEECFG_START_ADDR) { //something in the flash
    j = (int)(endMemoryPtr - FLASH_DEECFG_START_ADDR);
    j = j / 0x40;
    readDEE = (SCANVAL *)calloc(j, sizeof(SCANVAL));
    if (readDEE == NULL)
      NRF_LOG_INFO("Error. Can't work further");
    j = 0;
    for (m = FLASH_DEECFG_START_ADDR; m < endMemoryPtr; m += 0x40) {
      memcpy(&readDEE[j], (uint32_t *)m, sizeof(SCANVAL));
      j++; // j now how many in flash memory
    }
    dynArrayCnt = j;
    /* now find unique new element's in scanned array, if there */

    arrayForTemp = (SCANVAL *)calloc(10, sizeof(SCANVAL)); //10 elements in temp array only :)
    findUnique = false;
    for (i = 0; i < scanMyCnt; i++) //scanned array
    {
      // findUnique = false;
      for (j = 0; j < dynArrayCnt; j++) {
        if (false == memcmp(&scanMy[i].peer_addr.addr, &readDEE[j].peer_addr.addr, 6))
          findUnique = true;
      }
      if (!findUnique) //&TODO
      {
        memcpy(&arrayForTemp[forTempArray].peer_addr, &scanMy[i].peer_addr, sizeof(ble_gap_addr_t));
        memcpy(&arrayForTemp[forTempArray].name, &scanMy[i].name, 20);
        memcpy(&arrayForTemp[forTempArray].p_conn_params, &scanMy[i].p_conn_params, sizeof(ble_gap_conn_params_t));
        memcpy(&arrayForTemp[forTempArray].p_scan_params, &scanMy[i].p_scan_params, sizeof(ble_gap_scan_params_t));
        memcpy(&arrayForTemp[forTempArray].con_cfg_tag, &scanMy[i].con_cfg_tag, sizeof(uint8_t));
        forTempArray++;
      }
    }
    if (forTempArray > 0) //new founded scanned elements flashed to nonVolatile memory
    {
      for (i = 0; i < forTempArray; i++) {
        Flash_SaveStor(FLASH_DEECFG_START_ADDR, FLASH_DEECFG_END_ADDR,
            sizeof(SCANVAL), (uint32_t *)&arrayForTemp[i]);
      }
    }
  }
  //saving NEW scanning elements to EMPTY Flash
  else {
    NRF_LOG_INFO("Saving scanned devices to Flash memory\r\n");
    for (i = 0; i < 20; i++) // arraySize of ScanMy
    {
      if (scanMy[i].peer_addr.addr[0] == 0x0)
        break;
    }
    for (j = 0; j < i; j++) {
      memset(&scanVal, 0, sizeof(SCANVAL));
      // scanVal.id_scan = j;
      memcpy(scanVal.name, scanMy[j].name, 20);
      memcpy(&scanVal.con_cfg_tag, &scanMy[j].con_cfg_tag, 1);
      memcpy(&scanVal.p_conn_params, &scanMy[j].p_conn_params,
          sizeof(ble_gap_conn_params_t));
      memcpy(&scanVal.p_scan_params, &scanMy[j].p_scan_params,
          sizeof(ble_gap_scan_params_t));
      memcpy(&scanVal.peer_addr, &scanMy[j].peer_addr, sizeof(ble_gap_addr_t));
      Flash_SaveStor(FLASH_DEECFG_START_ADDR, FLASH_DEECFG_END_ADDR,
          sizeof(SCANVAL), (uint32_t *)&scanVal);
    }
  }
  free(arrayForTemp);
  free(readDEE);
  // NRFX_DELAY_US(1);
}

uint8_t readFlashDEE(void) {
  // int i, j;
  uint32_t m = 0;
  uint32_t endMemoryPtr = (uint32_t)Flash_LoadStor(
      FLASH_DEECFG_START_ADDR, FLASH_DEECFG_END_ADDR, sizeof(SCANVAL));
  if (endMemoryPtr > FLASH_DEECFG_START_ADDR) { //something in the flash
                                                //   i = (int)(endMemoryPtr - FLASH_DEECFG_START_ADDR);
                                                //   i = i / 0x40;
                                                // dynScanSaveVal = (SCANVAL *)calloc(i, sizeof(SCANVAL));
                                                // if (dynScanSaveVal == NULL)
                                                //   NRF_LOG_INFO("Error. Can't work further");
    savedFlashCnt = 0;
    for (m = FLASH_DEECFG_START_ADDR; m < endMemoryPtr; m += 0x40) {
      memcpy(&flashedDevicesArray[savedFlashCnt], (uint32_t *)m, sizeof(SCANVAL));
      //  memcpy(&dynScanSaveVal[savedFlashCnt], (uint32_t *)m, sizeof(SCANVAL));
      // NRF_LOG_INFO("%d\t\t\t\t%s", savedFlashCnt, &dynScanSaveVal[savedFlashCnt].name);
      savedFlashCnt++; // savedFlashCnt now how many in flash memory
    }
    return savedFlashCnt;
  } else if (endMemoryPtr == 0) {
    NRF_LOG_INFO("Can't found saved devices. You must first scan.");
    return 0;
  }
}

void saveScannedDev(void) {
  //sort scanned array with non-zero ID
  //if is scanned array with non-zero - erase flash
  //write scanned array
}

int main(void) {
  ret_code_t err_code;
  uint8_t cnt = 0;
  // Initialize.
  log_init();
  nrf_libuarte_async_config_t nrf_libuarte_async_config1 = {.tx_pin =
                                                                ARDUINO_0_PIN,
      .rx_pin = ARDUINO_1_PIN,
      .baudrate = NRF_UARTE_BAUDRATE_115200,
      .parity = NRF_UARTE_PARITY_EXCLUDED,
      .hwfc = NRF_UARTE_HWFC_DISABLED,
      .timeout_us = 1000,
      .int_prio = 5};
  timer_init();
  buttons_leds_init();
  db_discovery_init();
  power_management_init();
  ble_stack_init();
  flash_init(&fstorage_cfg);
  gatt_init();
  nus_c_init();
  ble_conn_state_init();
  scan_init();
  err_code = nrf_libuarte_async_init(&libuarte0, &nrf_libuarte_async_config0,
      uart_event_handler0, (void *)&libuarte0);
  APP_ERROR_CHECK(err_code);
  nrf_libuarte_async_enable(&libuarte0);
  err_code = nrf_libuarte_async_init(&libuarte1, &nrf_libuarte_async_config1,
      uart_event_handler1, (void *)&libuarte1);
  APP_ERROR_CHECK(err_code);
  nrf_libuarte_async_enable(&libuarte1);
  for (size_t i = 0; i < 62; i++) {
    memset(&flashedDevicesArray[i], 0, sizeof(SCANVAL));
  }
  for (size_t i = 0; i < 62; i++) {
    memset(&HReg.workingScanVal[i], 0, sizeof(WORKINGSCANVAL));
  }
  for (size_t i=0;i<8;i++){
    connCurrent[i].connHandler=BLE_CONN_HANDLE_INVALID;
  }
  Config_Init(); //Чтение конфигурации из Flash
  Prot_Init();   //Инициализация протоколов
  HReg.cntScanVal = cntAll;
  if (cntAll > 0) {
    for (size_t i = 0; i < cntAll; i++) {
    //HReg
      memcpy(&HReg.workingScanVal[i].mac_addr, &flashedDevicesArray[i].peer_addr.addr, 6);
      memcpy(&HReg.workingScanVal[i].name, &flashedDevicesArray[i].name, 20);
      memcpy(&HReg.workingScanVal[i].id, &flashedDevicesArray[i].id_scan, sizeof(uint16_t));
    //ConnCurrent
    memcpy(&connCurrent[i].addr, &flashedDevicesArray[i].peer_addr.addr, 6);
    memcpy(&connCurrent[i].p_conn_params, &flashedDevicesArray[i].p_conn_params, sizeof(ble_gap_conn_params_t));
    memcpy(&connCurrent[i].p_scan_params, &flashedDevicesArray[i].p_scan_params, sizeof(ble_gap_scan_params_t));
    memcpy(&connCurrent[i].id_scan, &flashedDevicesArray[i].id_scan, sizeof(uint16_t));
    memcpy(&connCurrent[i].con_cfg_tag, &flashedDevicesArray[i].con_cfg_tag, sizeof(uint8_t));
    memcpy(connCurrent[i].name, &flashedDevicesArray[i].name, 20);

    }
  }

  // Start execution.
  printf("BLE UART central example started.\r\n");
  NRF_LOG_INFO("BLE UART central example started.");
  // scan_start ();
  // sBL_Cfg.mModbusAddress = 127;

  // Enter main loop.
  while (1) {
    Prot_Handler();
    Config_Handler();
    idle_state_handle();
  }
}