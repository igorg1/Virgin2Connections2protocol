#ifndef PROTOKOL_SIAM_H__
#define PROTOKOL_SIAM_H__
//-----------------------------------------------------------------------------
#include "ProtokolSiamStruct.h"
#include "exchenge.h"
//-----------------------------------------------------------------------------
typedef enum
{
	SIAM_REG_READ,								/*!< Read register values and pass to protocol stack. */
	SIAM_REG_WRITE								/*!< Update register values. */
} eSiamRegisterMode;
//-----------------------------------------------------------------------------
#define GETREG_start( mb_addr_offset, addr_reg, len, type, var )											\
		if((uint32_t)(mb_addr_offset+sizeof(type))>= addr_reg+len )											\
		{																									\
			return ((uint8_t*)&var)+(addr_reg-mb_addr_offset);												\
		}
#define GETREG( mb_addr_offset, addr_reg, len, type, var )													\
		if((uint32_t)mb_addr_offset <= addr_reg && (uint32_t)(mb_addr_offset+sizeof(type))>= addr_reg+len )	\
		{																									\
			return ((uint8_t*)&var)+(addr_reg-mb_addr_offset);												\
		}
#define GETREG_region( mb_addr_start, mb_addr_end, addr_reg, len, type, var )								\
if((uint32_t)(mb_addr_start + mb_addr_end)>= addr_reg+len )													\
{																											\
	return ((uint8_t*)&var)+(addr_reg-mb_addr_start);														\
}
//-----------------------------------------------------------------------------
uint8_t* find_reg_siam(uint8_t  addr_device, uint32_t addr_reg, uint16_t len, eSiamRegisterMode eMode);
//-----------------------------------------------------------------------------
void DoReceive_Siam(void* prot, void* buf, uint16_t len);
void OnTxCompleate_Siam(void*);
//-----------------------------------------------------------------------------
void Siam_Handler(ProtInstanse* prot, Exchange* ex);
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------
