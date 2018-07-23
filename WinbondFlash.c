/**
 * Winbond SPI Flash
 *
 * Copyright (c) 2018 by Jay Xu.
 * All Rights Reserved.
 */
#include <assert.h>
#include "WinbondFlash.h"


void spiTransfer(WinbondFlash_Handle *hnd, uint8_t *sendData, uint8_t sendLen, uint8_t *recvData, uint8_t recvLen)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	hnd->spi->Send(sendData, sendLen);
	
	if (recvData && recvLen)
	{
		hnd->spi->Receive(recvData, recvLen);
	}
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WinbondFlash_Reset(WinbondFlash_Handle *hnd)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd = WB_INS_ENABLE_RESET;
	hnd->spi->Send(&cmd, 1);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
	
	__NOP();
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	cmd = WB_INS_RESET_DEVICE;
	hnd->spi->Send(&cmd, 1);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WinbondFlash_Init(WinbondFlash_Handle *hnd, ARM_DRIVER_SPI *spi, GPIO_TypeDef *csPort, uint16_t csPin)
{
	assert(hnd);
	
	hnd->spi = spi;
	hnd->csPort = csPort;
	hnd->csPin = csPin;
	
	WinbondFlash_Reset(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_getManufacturerID(WinbondFlash_Handle *hnd, uint8_t *manuId, uint8_t *devId)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd[] = { WB_INS_MANU_ID, 0x00, 0x00, 0x00 };
	hnd->spi->Send(cmd, sizeof(cmd));
	hnd->spi->Receive(manuId, 1);
	hnd->spi->Receive(devId, 1);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_getJEDECID(WinbondFlash_Handle *hnd, uint8_t *manuId, uint8_t *memType, uint8_t *capa)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd = WB_INS_JEDEC_ID;
	hnd->spi->Send(&cmd, sizeof(cmd));
	
	hnd->spi->Receive(manuId, 1);
	hnd->spi->Receive(memType, 1);
	hnd->spi->Receive(capa, 1);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_getUniqueID(WinbondFlash_Handle *hnd, uint64_t *uid)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd[5] = { WB_INS_UNIQUE_ID, 0x00 };
	hnd->spi->Send(&cmd, sizeof(cmd));
	
	hnd->spi->Receive(uid, sizeof(uid));

	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_WriteEnable(WinbondFlash_Handle *hnd)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd = WB_INS_WRITE_ENABLE;
	hnd->spi->Send(&cmd, sizeof(cmd));
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_WriteBytes(WinbondFlash_Handle *hnd, uint32_t addr, uint8_t *data, uint8_t dataLen)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd[4] = { WB_INS_PAGE_PROGRAM };

	addr &= 0x00FFFFFF;
	
	cmd[1] = (addr >> 16) & 0xFF;
	cmd[2] = (addr >> 8) & 0xFF;
	cmd[3] = addr & 0xFF;
	hnd->spi->Send(&cmd, sizeof(cmd));
	
	hnd->spi->Send(data, dataLen);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_ReadBytes(WinbondFlash_Handle *hnd, uint32_t addr, uint8_t *data, uint8_t dataLen)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd[4] = { WB_INS_READ_DATA };
	cmd[1] = (addr >> 16) & 0xFF;
	cmd[2] = (addr >> 8) & 0xFF;
	cmd[3] = addr & 0xFF;
	
	hnd->spi->Send(cmd, sizeof(cmd));
	
	hnd->spi->Receive(data, dataLen);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

uint8_t WB_ReadStatus(WinbondFlash_Handle *hnd)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd = WB_INS_READ_STATUS_1;
	hnd->spi->Send(&cmd, sizeof(cmd));
	
	uint8_t val = 0;
	hnd->spi->Receive(&val, 1);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
	return val;
}

void WB_EraseSector(WinbondFlash_Handle *hnd, uint32_t addr)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd[4] = { WB_INS_SECTOR_ERASE_4K };
	cmd[1] = (addr >> 16) & 0xFF;
	cmd[2] = (addr >> 8) & 0xFF;
	cmd[3] = addr & 0xFF;
	
	hnd->spi->Send(cmd, sizeof(cmd));
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_EraseChip(WinbondFlash_Handle *hnd)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd = WB_INS_CHIP_ERASE;
	hnd->spi->Send(&cmd, sizeof(cmd));
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_PowerDown(WinbondFlash_Handle *hnd)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd = WB_INS_POWER_DOWN;
	hnd->spi->Send(&cmd, sizeof(cmd));
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_PowerUp(WinbondFlash_Handle *hnd)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd = WB_INS_POWER_UP;
	hnd->spi->Send(&cmd, sizeof(cmd));
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}

void WB_FastRead(WinbondFlash_Handle *hnd, uint32_t addr, uint8_t *data, uint8_t dataLen)
{
	assert(hnd);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_RESET);
	
	uint8_t cmd[5] = { WB_INS_FAST_READ };
	cmd[1] = (addr >> 16) & 0xFF;
	cmd[2] = (addr >> 8) & 0xFF;
	cmd[3] = addr & 0xFF;
	cmd[4] = 0x00;	// dummy
	
	hnd->spi->Send(cmd, sizeof(cmd));
	hnd->spi->Receive(data, dataLen);
	
	HAL_GPIO_WritePin(hnd->csPort, hnd->csPin, GPIO_PIN_SET);
}
