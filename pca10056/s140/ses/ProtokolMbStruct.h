#ifndef PROTOKOL_MB_STRUCT_H__
#define PROTOKOL_MB_STRUCT_H__
//**********************************************************************************
#include "Protokol.h"
#include <stdint.h>
//**********************************************************************************
/* ----------------------- Defines ------------------------------------------*/
#define MB_RTU_PDU_SIZE_MIN 4   /*!< Minimum size of a Modbus RTU frame. */
#define MB_RTU_PDU_SIZE_MAX 256 /*!< Maximum size of a Modbus RTU frame. */
#define MB_RTU_PDU_SIZE_CRC 2   /*!< Size of CRC field in PDU. */
#define MB_RTU_PDU_ADDR_OFF 0   /*!< Offset of slave address in Ser-PDU. */
#define MB_RTU_PDU_PDU_OFF 1    /*!< Offset of Modbus-PDU in Ser-PDU. */

#define MB_PDU_FUNC_OFF 0 /*!< Offset of function code in PDU. */
/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_SIZE_MAX 253 /*!< Maximum size of a PDU. */
#define MB_PDU_SIZE_MIN 1   /*!< Function Code */
#define MB_PDU_FUNC_OFF 0   /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF 1   /*!< Offset for response data in PDU. */
/* ----------------------- Defines ------------------------------------------*/
#define MB_ADDRESS_BROADCAST (0) /*! Modbus broadcast address. */
#define MB_ADDRESS_MIN (1)       /*! Smallest possible slave address. */
#define MB_ADDRESS_MAX (247)     /*! Biggest possible slave address. */
#define MB_FUNC_NONE (0)
#define MB_FUNC_READ_COILS (1)
#define MB_FUNC_READ_DISCRETE_INPUTS (2)
#define MB_FUNC_WRITE_SINGLE_COIL (5)
#define MB_FUNC_WRITE_MULTIPLE_COILS (15)
#define MB_FUNC_READ_HOLDING_REGISTER (3)
#define MB_FUNC_READ_INPUT_REGISTER (4)
#define MB_FUNC_WRITE_REGISTER (6)
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS (16)
#define MB_FUNC_READWRITE_MULTIPLE_REGISTERS (23)
#define MB_FUNC_DIAG_READ_EXCEPTION (7)
#define MB_FUNC_DIAG_DIAGNOSTIC (8)
#define MB_FUNC_DIAG_GET_COM_EVENT_CNT (11)
#define MB_FUNC_DIAG_GET_COM_EVENT_LOG (12)
#define MB_FUNC_OTHER_REPORT_SLAVEID (17)
#define MB_FUNC_ERROR (128)
/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_FUNC_READ_ADDR_OFF (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READ_REGCNT_OFF (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READ_SIZE (4)
#define MB_PDU_FUNC_READ_REGCNT_MAX (0x007D)

#define MB_PDU_FUNC_WRITE_ADDR_OFF (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_VALUE_OFF (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_SIZE (4)

#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF (MB_PDU_DATA_OFF + 4)
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF (MB_PDU_DATA_OFF + 5)
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN (5)
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX (0x0078)

#define MB_PDU_FUNC_READWRITE_READ_ADDR_OFF (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF (MB_PDU_DATA_OFF + 4)
#define MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF (MB_PDU_DATA_OFF + 6)
#define MB_PDU_FUNC_READWRITE_BYTECNT_OFF (MB_PDU_DATA_OFF + 8)
#define MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF (MB_PDU_DATA_OFF + 9)
#define MB_PDU_FUNC_READWRITE_SIZE_MIN (9)
/* ----------------------- Type definitions ---------------------------------*/
//**********************************************************************************
typedef enum
{
  MB_REG_READ, /*!< Read register values and pass to protocol stack. */
  MB_REG_WRITE /*!< Update register values. */
} eMBRegisterMode;
//**********************************************************************************
typedef enum
{
  MB_ENOERR,    /*!< no error. */
  MB_ENOREG,    /*!< illegal register address. */
  MB_EINVAL,    /*!< illegal argument. */
  MB_EPORTERR,  /*!< porting layer error. */
  MB_ENORES,    /*!< insufficient resources. */
  MB_EIO,       /*!< I/O error. */
  MB_EILLSTATE, /*!< protocol stack in illegal state. */
  MB_ETIMEDOUT  /*!< timeout error occurred. */
} eMBErrorCode;
//**********************************************************************************
typedef enum
{
  MB_EX_NONE = 0x00,
  MB_EX_ILLEGAL_FUNCTION = 0x01,
  MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
  MB_EX_ILLEGAL_DATA_VALUE = 0x03,
  MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
  MB_EX_ACKNOWLEDGE = 0x05,
  MB_EX_SLAVE_BUSY = 0x06,
  MB_EX_MEMORY_PARITY_ERROR = 0x08,
  MB_EX_GATEWAY_PATH_FAILED = 0x0A,
  MB_EX_GATEWAY_TGT_FAILED = 0x0B
} eMBException;
//**********************************************************************************
typedef enum
{
  ProtokolNotInit = 0,
  UART_ModBus,
  UART_Siam,
  BLE_ModBus,
  BLE_Siam
} ProtokolPort;
//**********************************************************************************

//**********************************************************************************
typedef void (*PFn) ();

typedef struct
{
  PFn InitFn;
  PFn DeInitFn;
  uint8_t addr;
  ProtokolPort port;
  ProtokolState state;
  uint8_t buf[MB_RTU_PDU_SIZE_MAX];
  uint8_t buf_size;
} ProtokolInstanse;
//**********************************************************************************

#endif