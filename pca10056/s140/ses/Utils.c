#include "Utils.h"
//#include "nrf_fstorage.h"
//#include "nrf_fstorage_sd.h"

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage_cfg) =
{
	.evt_handler	= fstorage_evt_handler,
	.start_addr		= FLASH_DEECFG_START_ADDR,
	.end_addr		= FLASH_BLCFG_END_ADDR,
};

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage_app) =
{
	.evt_handler	= fstorage_evt_handler,
	.start_addr		= FLASH_APP_START_ADDR,
	.end_addr		= FLASH_APP_END_ADDR,
};

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
	/* While fstorage is busy, sleep and wait for an event. */
	while (nrf_fstorage_is_busy(p_fstorage))
	{
		#ifdef SOFTDEVICE_PRESENT
			(void) sd_app_evt_wait();
		#else
			__WFE();
		#endif
	}
}

void flash_erase(nrf_fstorage_t* fstorage, uint32_t start)
{
	ret_code_t err_code;

	err_code = nrf_fstorage_erase(fstorage, start, 1, NULL);
	APP_ERROR_CHECK(err_code);

	wait_for_flash_ready(fstorage);
}

void flash_write(nrf_fstorage_t* fstorage, uint32_t start, uint8_t* data, uint32_t len)
{
	ret_code_t err_code;

	NRF_LOG_INFO("Write. StartAddr: 0x%x; Data: 0x%x; Len: %d", start, data, len);
//	NRF_LOG_HEXDUMP_INFO(data, len);
//	NRF_LOG_INFO("StartADDR: 0x%x; ENDADDR: 0x%x", fstorage->start_addr, fstorage->end_addr);
	
	err_code = nrf_fstorage_write(fstorage, start, data, len, NULL);
	APP_ERROR_CHECK(err_code);

	wait_for_flash_ready(fstorage);
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
	if (p_evt->result != NRF_SUCCESS)
	{
		NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
		return;
	}

	switch (p_evt->id)
	{
		case NRF_FSTORAGE_EVT_WRITE_RESULT:
		{
			NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
						p_evt->len, p_evt->addr);
						NRF_LOG_INFO("--> DATA: 0x%x", *(uint32_t*)p_evt->addr);
		} break;

		case NRF_FSTORAGE_EVT_ERASE_RESULT:
		{
			NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
						p_evt->len, p_evt->addr);
						NRF_LOG_INFO("--> DATA: 0x%x", *(uint32_t*)p_evt->addr);
		}
		break;

		default:
			break;
	}
}

void flash_init(nrf_fstorage_t* fstorage)
{
	ret_code_t rc;
	nrf_fstorage_api_t *				p_fs_api;

	p_fs_api			= &nrf_fstorage_sd;

	rc = nrf_fstorage_init(fstorage, p_fs_api, NULL);
	APP_ERROR_CHECK(rc);
}

int16_t Flash_Erase(uint32_t start, uint32_t len)
{
	if(!start)
		return 1;
	if(!len || FLASH_END_ADDR < start+len )
		return 2;

	uint32_t page_no = (start-ADDR_FLASH_PAGE_0)/FLASH_PAGE_SIZE;
	const uint32_t page_qty = page_no + len/FLASH_PAGE_SIZE + (((len%FLASH_PAGE_SIZE)?1:0));

	for(; page_no < page_qty; page_no++)
	{
		flash_erase(&fstorage_app, page_no*FLASH_PAGE_SIZE);
	}

	return 0;
}

uint32_t Flash_GetPageWhereNot(uint32_t address, uint32_t start_addr, uint32_t end_addr)
{
	uint32_t current_page_no = (address-ADDR_FLASH_PAGE_0)/FLASH_PAGE_SIZE;

	const uint32_t start_page_no = (start_addr - ADDR_FLASH_PAGE_0)/FLASH_PAGE_SIZE;
	const uint32_t end_page_no = (end_addr - ADDR_FLASH_PAGE_0)/FLASH_PAGE_SIZE;

	current_page_no++;
	return (current_page_no>=end_page_no) ? start_page_no:current_page_no;
}

uint32_t Flash_ErasePageWhereNot(uint32_t address, uint32_t start_addr, uint32_t end_addr)
{
	uint32_t addres_to_erase = ADDR_FLASH_PAGE_0 + FLASH_PAGE_SIZE*Flash_GetPageWhereNot(address, start_addr, end_addr);
	flash_erase(&fstorage_cfg, addres_to_erase);
	return addres_to_erase;
}

uint32_t Flash_FindOrErase(uint32_t addr, uint32_t start_addr, uint32_t end_addr, uint32_t stor_len)
{
	uint32_t* curr = (uint32_t*)addr;
	while((uint32_t)curr < end_addr)
	{
		int filled_area = 0;

		const size_t len = stor_len/sizeof(uint32_t);
		for(size_t i=0; i<len; i++)
			if(0xFFFFFFFF != curr[i] )
			{
				filled_area++;
				break;
			}

		if(!filled_area)
			return (uint32_t)curr;
		curr += len;
	}

	return Flash_ErasePageWhereNot(addr, start_addr, end_addr);
}

const uint32_t* Flash_LoadStor(uint32_t start_addr, uint32_t end_addr, uint32_t data_bsize)
{
	uint32_t curr_cfg_address=0;
	uint32_t max_generation=0;
	const StorHeader* hdr;
	const uint32_t stor_bsize= sizeof(StorHeader)+data_bsize;
//	const uint32_t data32_qty= stor_bsize/sizeof(uint32_t)-1;

	while (start_addr < end_addr)
	{
		hdr = (const StorHeader*)(start_addr+data_bsize);
		if(hdr->mGeneration!=0xFFFFFFFF && hdr->mCRC!=0xFFFFFFFF)
		{
			if(hdr->mGeneration >= max_generation)
			{
				const uint32_t crc = crc32_compute((uint8_t const *)start_addr, data_bsize, NULL);
				if(hdr->mCRC == crc)
				{
					max_generation = hdr->mGeneration;
					curr_cfg_address=start_addr;
				}
			}
		}
		start_addr += stor_bsize;
	}//while
		NRF_LOG_INFO("Load Config. Addr: 0x%x", curr_cfg_address);
	return (uint32_t*)curr_cfg_address;
}

int Flash_SaveStor(uint32_t start_addr, uint32_t end_addr, uint32_t len, uint32_t* new_data)
{
	const uint32_t stor_bzise = sizeof(StorHeader) + len;
	const uint32_t* old_data = Flash_LoadStor(start_addr, end_addr, len);

//	NRF_LOG_INFO("Save: Old Data Addr: 0x%x", old_data);
	printf("Save: Old Data Addr: 0x%x", old_data);

	StorHeader new_hdr = {0,0};
	uint32_t free_area = 0;

	if(old_data)
	{
		const StorHeader* old_hdr = (const StorHeader*)(((uint32_t)old_data)+len);
		if(0==memcmp(old_data, new_data, len))
		{
	//		NRF_LOG_INFO("Save: old_data=new_data");
			printf("Save: old_data=new_data");
			return 1;
		}
		if(UINT32_MAX==old_hdr->mGeneration)
		{
			new_hdr.mGeneration = 0;
			flash_erase(&fstorage_cfg, start_addr);
			free_area = start_addr;
	//		NRF_LOG_INFO("Save: flash_erase");			
			printf("Save: flash_erase");			
		}
		else
		{
			new_hdr.mGeneration = 1 + old_hdr->mGeneration;
			free_area = Flash_FindOrErase((uint32_t)old_data, start_addr, end_addr, stor_bzise);
		}
	}
	else
	{
		free_area = Flash_FindOrErase(start_addr, start_addr, end_addr, stor_bzise);	
	}

	//NRF_LOG_INFO("Save: Find addr: 0x%x", free_area);
	printf("Save: Find addr: 0x%x", free_area);
	flash_write(&fstorage_cfg, free_area,(uint8_t*)new_data, len);

	const uint32_t crc = crc32_compute((uint8_t const *)new_data, len, NULL);
	new_hdr.mCRC = crc;

	flash_write(&fstorage_cfg, free_area+len, (uint8_t*)&new_hdr, sizeof(StorHeader));

	return 0;
}
