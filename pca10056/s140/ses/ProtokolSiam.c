#include "ProtokolSiam.h"
#include <string.h>
#include "nrf_log.h"

//-----------------------------------------------------------------------------
eSiamError ProcessWriteCmdSiam(ProtInstanse* pr, uint8_t* buf, uint8_t len,  uint8_t  addr_device, uint32_t addr_reg, uint16_t qty)
{
	uint16_t crc = usMBCRC16(pr->mBuf+2, 8);
	if(crc != *(uint16_t*)&pr->mBuf[10])
		return eWrongCrc;

	const uint16_t data_crc = usMBCRC16(pr->mBuf+12, qty );
	if(data_crc != *(uint16_t*)&pr->mBuf[len-2])
		return eWrongCrc;

	uint8_t *reg = find_reg_siam(addr_device, addr_reg, qty, SIAM_REG_WRITE);
	if(!reg)
		return eWrongAddr;

	memcpy(&reg[0], &buf[0]+12, qty);

	pr->mLen = 12;
	pr->state = ResponseReady;
	return eNoError;
}
//-----------------------------------------------------------------------------
eSiamError ProcessReadCmdSiam(ProtInstanse* pr, uint8_t* buf, uint8_t len,  uint8_t  addr_device, uint32_t addr_reg, uint16_t qty)
{
	uint16_t crc = usMBCRC16(pr->mBuf+2, pr->mLen-4 );
	if(crc != *(uint16_t*)&pr->mBuf[10] )
		return eWrongCrc;

	if(MAX_PROT_LEN*16< qty+12+2)
	{
		return eWrongSize;
	}

	uint8_t* reg = find_reg_siam(addr_device, addr_reg, qty, SIAM_REG_READ);
	if(!reg)
		return eWrongAddr;

	memcpy(&buf[0]+len, &reg[0], qty);

	crc =usMBCRC16(buf+len, qty );

	memcpy(buf+len+qty, &crc, sizeof(crc));

	pr->mLen = len+qty+sizeof(crc);
	pr->state = ResponseReady;
	return eNoError;
}
//-----------------------------------------------------------------------------
void ErrorResponseSiam(ProtInstanse* pr, eSiamError eException)
{
//	memcpy(&pr->mBuf[0]+12+2, &crc, 2);

	pr->mBuf[3] |= 0x80;
	pr->mLen = 14;
	memcpy(&pr->mBuf[0]+10, &eException, 2);
	uint16_t crc =usMBCRC16((uint8_t*)&pr->mBuf[2], 10);
	memcpy(pr->mBuf+12, &crc, 2);
	pr->state = ResponseReady;
}
//-----------------------------------------------------------------------------
void FrameProcess_Saim(ProtInstanse* pr)
{
	uint16_t qty = 0;
	uint16_t rcv_crc = 0;

	for(; pr->idxIn<pr->idxOut; pr->idxIn++)
	{
		if(0 == memcmp("\r\n", pr->mBuf + pr->idxIn, 2))
			break;
	}

	if(sizeof(pr->mBuf)/2 < pr->idxIn)
	{
		memcpy(&pr->mBuf, &pr->mBuf[pr->idxIn], pr->idxOut - pr->idxIn);
		pr->idxOut = pr->idxOut - pr->idxIn;
		pr->idxIn = 0;
	}

	if(pr->idxIn == pr->idxOut)
	{
		pr->state = NotInit;
		return;	
	}

	if(12 > pr->idxOut - pr->idxIn)
	{
		pr->state = NotInit;
		return;
	}

	uint8_t crc_size = 0;
	switch(pr->mBuf[pr->idxIn + 3])
	{
		default:
			crc_size=12;
			break;			
		case 0x01:
			crc_size=12;
			break;
		case 0x02:
			crc_size=12;
			break;
		case 0x81:
			crc_size=14;
			break;
		case 0x82:
			crc_size=14;
			break;
	}
	rcv_crc = usMBCRC16(&pr->mBuf[pr->idxIn + 2], crc_size-2);
	if( rcv_crc != 0){
		pr->idxIn += 2;
		return;
	}

	if(pr->sProtokolType == prMaster)
	{
		switch(pr->mBuf[pr->idxIn+3])
		{
			default:
				pr->state = NotInit;
				return;
			case 0x01:
				pr->mLen = 12;
				break;
			case 0x02:
				qty = *(uint16_t*)&pr->mBuf[pr->idxIn + 8];
				if(qty + 2 > pr->idxOut - pr->idxIn - 12)
				{
					pr->state = NotInit;
					return;
				}
				pr->mLen = 12 + qty + 2;
				break;
		}		
	}
	
	if(pr->sProtokolType == prSlave)
	{
		switch(pr->mBuf[pr->idxIn+3])
		{
			default:
				pr->state = NotInit;
				return;
			case 0x01:
				qty = *(uint16_t*)&pr->mBuf[pr->idxIn + 8];
				if(qty + 2 > pr->idxOut - pr->idxIn - 12)
				{
					pr->state = NotInit;
					return;
				}
				pr->mLen = 12 + qty + 2;
				break;
			case 0x02:
				pr->mLen = 12;
				break;
			case 0x81:
				pr->mLen = 14;
				break;
			case 0x82:
				pr->mLen = 14;
				break;
		}		
	}

	memcpy(&pr->mBuf, &pr->mBuf[pr->idxIn], pr->mLen);
	pr->idxIn = 0;
	pr->idxOut = 0;
 
	const uint8_t ucFunctionCode = pr->mBuf[3];

	const uint8_t addr_device = *(uint8_t*)&pr->mBuf[2];
	const uint32_t addr_reg = *(uint32_t*)&pr->mBuf[4];
	qty = *(uint16_t*)&pr->mBuf[8];

	eSiamError eException = eNoError;	
	
	if(addr_device == pr->addr)
	{
		//__disable_irq();
		switch(ucFunctionCode)
		{
			default: break;
			case 0x01:
				eException = ProcessReadCmdSiam(pr, pr->mBuf, pr->mLen, addr_device, addr_reg, qty);
				break;
			case 0x02:
				eException = ProcessWriteCmdSiam(pr, pr->mBuf, pr->mLen, addr_device, addr_reg, qty);
				break;
		}
		if(pr->sProtokolType == prMaster) pr->sExType = 1;
		if(pr->sProtokolType == prSlave) pr->sExType = 0;
		//__enable_irq();
	}
	else
	{
		NRF_LOG_DEBUG("ResponseReady len: %d", pr->mLen);
		pr->state = ResponseReady;
		if(pr->sProtokolType == prMaster) pr->sExType = 0;
		if(pr->sProtokolType == prSlave) pr->sExType = 1;
		return;		
	}

	if(eException)
		ErrorResponseSiam(pr, eException);

	NRF_LOG_HEXDUMP_DEBUG(pr->mBuf, pr->mLen);
}
//-----------------------------------------------------------------------------
void OnTxCompleate_Siam(void* pr)
{
	ProtInstanse* mb_prot = (ProtInstanse*)pr;
	mb_prot->state=TxCompleate;
}
//-----------------------------------------------------------------------------
void Siam_Handler(ProtInstanse* pr, Exchange* ex)
{
	switch(pr->state)
	{
		default: break;
		case FrameReceived:
			ex->EnableRx(0);
			FrameProcess_Saim(pr);
			if(pr->state == NotInit)
			{
				ex->EnableRx(1);
				break;
			}
			if(pr->state != ResponseReady)
				break;
		case ResponseReady:
			if(pr->sExType == ex->sExType)
			{
				ex->EnableTx(1);
				ex->TransmitFn(pr, pr->mBuf, pr->mLen);
				pr->state = NotInit;
			}
			break;
		case FrameError:
			break;
		case TxCompleate:
			ex->EnableTx(0);
			ex->EnableRx(1);
			memset(&pr->mBuf, 0, sizeof(pr->mBuf));
			pr->mLen = 0;
			pr->idxIn = 0;
			pr->idxOut = 0;
			pr->state = NotInit;
			break;
	}
}
//-----------------------------------------------------------------------------
