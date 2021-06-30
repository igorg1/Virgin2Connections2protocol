#ifndef UTILS_H__
#define UTILS_H__

#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include <stdint.h>

#define FLASH_START_ADDR 0x00000000
#define FLASH_SD_END_ADDR 0x00027000
#define FLASH_END_ADDR 0x00100000
#define FLASH_PAGE_SIZE 0x1000

#ifdef DEBUG
#define FLASH_BL_PAGE 25
#define FLASH_APP_PAGE 80
#else
#define FLASH_BL_PAGE 25
#define FLASH_APP_PAGE 100
#endif
#define FLASH_APPCGF_PAGE 3
#define FLASH_BLCGF_PAGE 2

#define FLASH_BL_SIZE (FLASH_BL_PAGE * FLASH_PAGE_SIZE)
#define FLASH_BL_START_ADDR (FLASH_START_ADDR + FLASH_SD_END_ADDR)
#define FLASH_BL_END_ADDR (FLASH_BL_START_ADDR + FLASH_BL_SIZE)

#define FLASH_APP_SIZE (FLASH_APP_PAGE * FLASH_PAGE_SIZE)
#define FLASH_APP_START_ADDR (FLASH_BL_START_ADDR + FLASH_BL_SIZE)
#define FLASH_APP_END_ADDR (FLASH_APP_START_ADDR + FLASH_APP_SIZE)

#define FLASH_BLCFG_SIZE (FLASH_BLCGF_PAGE * FLASH_PAGE_SIZE)
#define FLASH_BLCFG_START_ADDR (FLASH_END_ADDR - FLASH_BLCFG_SIZE)
#define FLASH_BLCFG_END_ADDR (FLASH_BLCFG_START_ADDR + FLASH_BLCFG_SIZE)

#define FLASH_APPCFG_SIZE (FLASH_APPCGF_PAGE * FLASH_PAGE_SIZE)
#define FLASH_APPCFG_START_ADDR (FLASH_BLCFG_START_ADDR - FLASH_APPCFG_SIZE)
#define FLASH_APPCFG_END_ADDR (FLASH_APPCFG_START_ADDR + FLASH_APPCFG_SIZE)

#define ADDR_FLASH_PAGE_0 0x00000000

typedef struct
{
  uint32_t mGeneration;
  uint32_t mCRC;
} __attribute__ ((aligned (4), packed)) StorHeader;

extern nrf_fstorage_t fstorage_cfg;
extern nrf_fstorage_t fstorage_app;

void flash_init (nrf_fstorage_t *fstorage);
static void fstorage_evt_handler (nrf_fstorage_evt_t *p_evt);
void wait_for_flash_ready (nrf_fstorage_t const *p_fstorage);

void flash_write (
    nrf_fstorage_t *fstorage, uint32_t start, uint8_t *data, uint32_t len);
void flash_erase (nrf_fstorage_t *fstorage, uint32_t start);

uint32_t Flash_FindOrErase (
    uint32_t addr, uint32_t start_addr, uint32_t end_addr, uint32_t stor_len);
const uint32_t *Flash_LoadStor (
    uint32_t address, uint32_t end_addr, uint32_t data_bsize);
int Flash_SaveStor (
    uint32_t start_addr, uint32_t end_addr, uint32_t len, uint32_t *new_data);
int16_t Flash_Erase (uint32_t start, uint32_t len);

#endif