/**
 * Winbond SPI Flash
 *
 * Copyright (c) 2018 by Jay Xu.
 * All Rights Reserved.
 */
#ifndef WINBOND_FLASH_H
#define WINBOND_FLASH_H


#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <Driver_SPI.h>

enum Winbond_Instruction
{
	WB_INS_WRITE_ENABLE = 0x06,
	WB_INS_VOLATILE_SR_WRITE_ENABLE = 0x50,
	WB_INS_WRITE_DISABLE = 0x04,
	WB_INS_READ_STATUS_1 = 0x05,
	WB_INS_WRITE_STATUS_1 = 0x01,
	WB_INS_READ_STATUS_2 = 0x35,
	WB_INS_WRITE_STATUS_2 = 0x31,
	WB_INS_READ_STATUS_3 = 0x15,
	WB_INS_WRITE_STATUS_3 = 0x11,
	WB_INS_CHIP_ERASE = 0xC7,
	WB_INS_PROGRAM_SUSPEND = 0x75,
	WB_INS_PROGRAM_RESUME = 0x7A,
	WB_INS_POWER_DOWN = 0xB9,
	WB_INS_POWER_UP = 0xAB,
	WB_INS_MANU_ID = 0x90,
	WB_INS_JEDEC_ID = 0x9F,
	WB_INS_GLOBAL_BLOCK_LOCK = 0x7E,
	WB_INS_GLOBAL_BLOCK_UNLOCK = 0x98,
	WB_INS_ENTER_QPI_MODE = 0x38,
	WB_INS_ENABLE_RESET = 0x66,
	WB_INS_RESET_DEVICE = 0x99,
	WB_INS_UNIQUE_ID = 0x4B,
	WB_INS_PAGE_PROGRAM = 0x02,
	WB_INS_QUAD_PAEG_PROGRAM = 0x32,
	WB_INS_SECTOR_ERASE_4K = 0x20,
	WB_INS_BLOCK_ERASE_32K = 0x52,
	WB_INS_BLOCK_ERASE_64K = 0xD8,
	WB_INS_READ_DATA = 0x03,
	WB_INS_FAST_READ = 0x0B,
	WB_INS_FAST_READ_DUAL_OUTPUT = 0x3B,
	WB_INS_FAST_READ_QUAD_OUTPUT = 0x6B,
	WB_INS_READ_SFDP = 0x5A,
	WB_INS_ERASE_SECURITY = 0x44,
	WB_INS_PROGRAM_SECURITY = 0x42,
	WB_INS_READ_SECURITY = 0x48,
	WB_INS_INDIVIDUAL_BLOCK_LOCK = 0x36,
	WB_INS_INDIVIDUAL_BLOCK_UNLOCK = 0x39,
	WB_INS_READ_BLOCK_LOCK = 0x3D
};

typedef struct
{
	GPIO_TypeDef *csPort;
	uint16_t csPin;
	ARM_DRIVER_SPI *spi;
} WinbondFlash_Handle;

#ifdef __cplusplus
extern "C"
{
#endif
	
	
extern void WinbondFlash_Init(WinbondFlash_Handle *hnd, ARM_DRIVER_SPI *spi, GPIO_TypeDef *csPort, uint16_t csPin);
extern void WinbondFlash_Reset(WinbondFlash_Handle *hnd);
	
extern void WB_getManufacturerID(WinbondFlash_Handle *hnd, uint8_t *manuId, uint8_t *devId);
extern void WB_getJEDECID(WinbondFlash_Handle *hnd, uint8_t *manuId, uint8_t *memType, uint8_t *capa);
extern void WB_getUniqueID(WinbondFlash_Handle *hnd, uint64_t *uid);
	
extern void WB_WriteBytes(WinbondFlash_Handle *hnd, uint32_t addr, uint8_t *data, uint8_t dataLen);
extern void WB_ReadBytes(WinbondFlash_Handle *hnd, uint32_t addr, uint8_t *data, uint8_t dataLen);	
extern uint8_t WB_ReadStatus(WinbondFlash_Handle *hnd);
extern void WB_WriteEnable(WinbondFlash_Handle *hnd);
extern void WB_EraseSector(WinbondFlash_Handle *hnd, uint32_t addr);
extern void WB_EraseChip(WinbondFlash_Handle *hnd);
extern void WB_FastRead(WinbondFlash_Handle *hnd, uint32_t addr, uint8_t *data, uint8_t dataLen);
extern void WB_PowerDown(WinbondFlash_Handle * hnd);
extern void WB_PowerUp(WinbondFlash_Handle * hnd);

#ifdef __cplusplus
}
#endif

#endif