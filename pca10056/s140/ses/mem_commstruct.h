#ifndef MEM_COMMON_STRUCT_H__
#define MEM_COMMON_STRUCT_H__

#include <stdint.h>
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
typedef enum __attribute__ ((aligned(2)))
{
	ccNone = 0U
	,ccSetDefault= 11U
	,ccApply= 12U
	,ccSave =14U
	,ccReboot= 18U
	,ccStartApp= 16U
        ,SCAN=1,
        SAVE=2,
        ERASE=3
} CfgCommand;
//-----------------------------------------------------------------------------
typedef enum __attribute__ ((aligned(2)))
{
	bmApp = 0U
	,bmLoader= 1U
} BootMode;
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
typedef struct
{
	uint16_t	HW_ID;
	uint16_t	HW_VERSION;
	uint16_t	SW_ID;
	uint16_t	SW_VERSION;
} __attribute__((aligned(2))) SysUID; //8
//-----------------------------------------------------------------------------
typedef struct
{
	uint32_t	mPasswdCRC;
	uint16_t	mLock;
} __attribute__((aligned(2),packed)) LockCfg;// 6
//-----------------------------------------------------------------------------
typedef struct
{
	uint16_t	mLockCmd;
	uint8_t	mPasswd[8];
} __attribute__((aligned(2),packed)) LockCtrl;// 10
//-----------------------------------------------------------------------------
typedef struct
{
	uint16_t	mCommand;
	uint16_t	mStatus;
} __attribute__((aligned(2),packed)) SysCtrl;// 4
//-----------------------------------------------------------------------------
typedef struct
{
	uint16_t	BootMode;
	uint16_t	mModbusAddress;
	uint32_t	AppSize;
	uint16_t	AppCrc;
	uint16_t	Sd_Version;
	uint16_t	Reserved[3];
} __attribute__((aligned(2),packed)) BLConfig;// 18
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#endif
