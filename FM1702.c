/**
 * FM1702 MIFARE reader IC driver for STM32 MCUs
 *
 * Copyright (c) 2018 by Jay Xu.
 * All Rights Reserved.
 */
#include "FM1702.h"

#include <assert.h>
#include <Driver_SPI.h>
#include <string.h>
#include <cmsis_os2.h>

#define GPIO_HIGH(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define GPIO_LOW(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)

void PCD_WriteReg(FM1702_Handle *hnd, uint8_t reg, uint8_t value)
{
	assert(hnd != NULL);
	
	//GPIO_LOW(hnd->csPort, hnd->csPin);
	hnd->spiDrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	
	reg <<= 1;
	reg &= 0x7E;
	hnd->spiDrv->Send(&reg, 1);

	hnd->spiDrv->Send(&value, 1);

	hnd->spiDrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	//GPIO_HIGH(hnd->csPort, hnd->csPin);
}

uint8_t PCD_ReadReg(FM1702_Handle *hnd, uint8_t reg)
{
	assert(hnd != NULL);
	uint8_t value = 0;
	
	//GPIO_LOW(hnd->csPort, hnd->csPin);
	hnd->spiDrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	
	reg <<= 1;
	reg |= 0x80;
	hnd->spiDrv->Send(&reg, 1);

	hnd->spiDrv->Receive(&value, 1);
	
	hnd->spiDrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	//GPIO_HIGH(hnd->csPort, hnd->csPin);
	
	return value;
}

void PCD_SetBitMask(FM1702_Handle *hnd, uint8_t reg, uint8_t mask)
{
	uint8_t value = PCD_ReadReg(hnd, reg);
	PCD_WriteReg(hnd, reg, value | mask);
}

void PCD_ClearBitMask(FM1702_Handle *hnd, uint8_t reg, uint8_t mask)
{
	uint8_t value = PCD_ReadReg(hnd, reg);
	PCD_WriteReg(hnd, reg, value & (~mask));
}

void PCD_WriteFIFO(FM1702_Handle *hnd, uint8_t *data, uint8_t len)
{
	assert(hnd != NULL);
	uint8_t reg = PCD_REG_FIFO_DATA;
	reg <<= 1;
	reg &= 0x7E;
	
	//GPIO_LOW(hnd->csPort, hnd->csPin);
	hnd->spiDrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	
	hnd->spiDrv->Send(&reg, 1);
	hnd->spiDrv->Send(data, len);
	
	hnd->spiDrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	//GPIO_HIGH(hnd->csPort, hnd->csPin);
}

uint8_t PCD_ReadFIFO(FM1702_Handle *hnd, uint8_t *data)
{
	assert(hnd && data);
	
	uint8_t len = PCD_ReadReg(hnd, PCD_REG_FIFO_LENGTH);
	
	for (int i = 0; i < len; i++)
	{
		data[i] = PCD_ReadReg(hnd, PCD_REG_FIFO_DATA);
	}
	return len;
}

void PCD_FlushFIFO(FM1702_Handle *hnd)
{
	uint8_t len = PCD_ReadReg(hnd, PCD_REG_FIFO_LENGTH);
	
	PCD_SetBitMask(hnd, PCD_REG_CONTROL, 0x01);
	
	while (PCD_ReadReg(hnd, PCD_REG_FIFO_LENGTH) != 0);
}

void PCD_AntennaOn(FM1702_Handle *hnd)
{
	assert(hnd != NULL);
	PCD_SetBitMask(hnd, PCD_REG_TX_CONTROL, 0x03);
}

void PCD_AntennaOff(FM1702_Handle *hnd)
{
	assert(hnd != NULL);
	PCD_ClearBitMask(hnd, PCD_REG_TX_CONTROL, 0x03);
}

void PCD_SetTimeOut(FM1702_Handle *hnd, uint32_t timeout)
{
	uint8_t prescaler = 4;
	
	while (timeout > 0xff)
	{
		timeout >>= 1;
		prescaler++;
	}
	if (prescaler > 0x15)
	{
		prescaler = 0x15;
		timeout = 0xff;
	}
	
	PCD_WriteReg(hnd, PCD_REG_TIMER_CLOCK, prescaler);
	PCD_WriteReg(hnd, PCD_REG_TIMER_RELOAD, timeout);
}

void PCD_Reset(FM1702_Handle *hnd)
{
	assert(hnd != NULL);
	
	GPIO_HIGH(hnd->resetPort, hnd->resetPin);
	HAL_Delay(100);
	GPIO_LOW(hnd->resetPort, hnd->resetPin);
	
	while (PCD_ReadReg(hnd, PCD_REG_COMMAND) != 0)
	{
	}
	PCD_WriteReg(hnd, PCD_REG_PAGE, 0x80);
	while (PCD_ReadReg(hnd, PCD_REG_COMMAND) != 0)
	{
	}
	PCD_WriteReg(hnd, PCD_REG_PAGE, 0x00);
}

void PCD_Init(
	FM1702_Handle *hnd,
	ARM_DRIVER_SPI *spiDrv,
	GPIO_TypeDef *chipSelectPort,
	uint16_t chipSelectPin,
	GPIO_TypeDef *resetPort,
	uint16_t resetPin,
	GPIO_TypeDef *irqPort,
	uint16_t irqPin
)
{
	if (hnd == NULL)
	{
		return;
	}
	
	hnd->csPort = chipSelectPort;
	hnd->csPin = chipSelectPin;
	hnd->resetPort = resetPort;
	hnd->resetPin = resetPin;
	hnd->irqPort = irqPort;
	hnd->irqPin = irqPin;
	hnd->spiDrv = spiDrv;
	
	PCD_Reset(hnd);
	
	// set CRC preset value as 0x6363 -- according ISO 14443-3
	PCD_WriteReg(hnd, PCD_REG_CRC_PRESET_LSB, 0x63);
	PCD_WriteReg(hnd, PCD_REG_CRC_PRESET_MSB, 0x63);
	
	PCD_WriteReg(hnd, PCD_REG_CRYPTO_SELECT, 0x00);
	PCD_WriteReg(hnd, PCD_REG_FIFO_LEVEL, 0x00);
	
	PCD_WriteReg(hnd, PCD_REG_RX_CONTROL_1, 0x73);
	PCD_WriteReg(hnd, PCD_REG_RX_CONTROL_2, 0x41);	// RX auto power down
	
	PCD_WriteReg(hnd, PCD_REG_IRQ_PIN_CONFIG, 0x01); // no Inv and CMOS output
}

FM1702_Status PCD_SoftPowerDown(FM1702_Handle *hnd)
{
	assert(hnd);
	
	PCD_SetBitMask(hnd, PCD_REG_CONTROL, 0x10);
	
	return FM1702_OK;
}

FM1702_Status PCD_SoftPowerUp(FM1702_Handle *hnd)
{
	assert(hnd);
	
	PCD_ClearBitMask(hnd, PCD_REG_CONTROL, 0x10);
	HAL_Delay(50);
	while (PCD_ReadReg(hnd, PCD_REG_CONTROL) & 0x10);	// wait until exit power down state
	
	return FM1702_OK;
}

/**
 * excute FM1702 PCD commands.
 */
FM1702_Status PCD_ExecuteCommand(
	FM1702_Handle *hnd,
	uint8_t cmd,
	uint8_t *data,
	uint8_t len,
	uint8_t waitIRQ,
	uint8_t timerCtrl,
	uint32_t timeout)
{
	assert(hnd != NULL);
	
	PCD_WriteReg(hnd, PCD_REG_INTERRUPT_EN, 0x80 | waitIRQ);
	PCD_WriteReg(hnd, PCD_REG_INTERRUPT_EN, 0x7f & (~waitIRQ));
	PCD_WriteReg(hnd, PCD_REG_INTERRUPT_RQ, 0x7f);	// clear all irq source
	
	PCD_SetBitMask(hnd, PCD_REG_CONTROL, 0x04);	// stop timer now
	
	PCD_SetTimeOut(hnd, timeout);
	PCD_WriteReg(hnd, PCD_REG_COMMAND, PCD_CMD_IDLE);	// abort running command
	PCD_FlushFIFO(hnd);
	PCD_WriteFIFO(hnd, data, len);
	
	// if not specifty timer control, start timer manually
	if (timerCtrl)
	{
		PCD_WriteReg(hnd, PCD_REG_TIMER_CONTROL, timerCtrl);
	}
	else
	{
		PCD_SetBitMask(hnd, PCD_REG_CONTROL, 0x02);
	}
	
	PCD_WriteReg(hnd, PCD_REG_COMMAND, cmd);
	
	uint32_t tickStart = osKernelGetTickCount();
	while (HAL_GPIO_ReadPin(hnd->irqPort, hnd->irqPin) != GPIO_PIN_SET)
	{
		if (osKernelGetTickCount() - tickStart > 50)
		{
			return FM1702_ERR_TIMEOUT;
		}
	}
	
	while (1)
	{
		uint8_t irq = PCD_ReadReg(hnd, PCD_REG_INTERRUPT_RQ);
		if ((irq & waitIRQ) == waitIRQ)
		{
			break;
		}
		if (irq & PCD_IRQ_TIMER)
		{
			return FM1702_ERR_TIMEOUT;
		}
	}

	uint8_t error = PCD_ReadReg(hnd, PCD_REG_ERROR_FLAG);
	uint8_t status = FM1702_OK;
	
	if (error & 0x01)
	{
		status = FM1702_ERR_COLL;
	}
	else if (error & 0x02)
	{
		status = FM1702_ERR_PARITY;
	}
	else if (error & 0x04)
	{
		status = FM1702_ERR_FRAMING;
	}
	else if (error & 0x08)
	{
		status = FM1702_ERR_CRC;
	}
	else if (error & 0x10)
	{
		status = FM1702_ERR_FIFO_OVFL;
	}
	else if (error & 0x20)
	{
		status = FM1702_ERR_ACCESS;
	}
	else if ((error & 0x40) && (cmd == PCD_CMD_LOAD_KEY || cmd == PCD_CMD_LOAD_KEY_E2))
	{
		status = FM1702_ERR_KEY;
	}
	
	return status;
}

FM1702_Status PCD_CommWithPICC(
	FM1702_Handle *hnd,
	uint8_t cmd,
	uint8_t *sendData,
	uint8_t sendLen,
	uint8_t *validBits,
	uint8_t *recvData,
	uint8_t *recvLen,
	uint8_t rxAlign,
	uint8_t waitIRQ,
	uint8_t timerCtrl,
	uint32_t timeout)
{
	assert(hnd != NULL);
	
	PCD_AntennaOn(hnd);
	uint8_t bitFraming = rxAlign << 4 | (validBits ? *validBits : 0); 
	PCD_WriteReg(hnd, PCD_REG_BIT_FRAMING, bitFraming);
	
	FM1702_Status status = PCD_ExecuteCommand(hnd, cmd, sendData, sendLen, waitIRQ, timerCtrl, timeout);
	
	if (status != FM1702_OK)
	{
		return status;
	}
	
	if (recvData && recvLen)
	{
		uint8_t n = PCD_ReadReg(hnd, PCD_REG_FIFO_LENGTH);
		//printf("len = %d ", n);
		if (*recvLen < n)
		{
			return FM1702_ERR_NO_ROOM;
		}
		*recvLen = n;
		if (validBits)
		{
			*validBits = PCD_ReadReg(hnd, PCD_REG_SECONDARY_STATUS) & 0x07;
		}
		
		n = PCD_ReadReg(hnd, PCD_REG_FIFO_LENGTH);
		//printf("len 2 = %d ", n);
		uint8_t idx = 0;
		if (rxAlign != 0)
		{
			uint8_t firstByte = PCD_ReadReg(hnd, PCD_REG_FIFO_DATA);
			uint8_t mask = (0xff << rxAlign) & 0xff;
			recvData[0] |= (recvData[0] & ~mask) | (firstByte & mask);	// apply align to recveied data
			idx++;
		}
		PCD_ReadFIFO(hnd, recvData + idx);
	}
	return FM1702_OK;
}

FM1702_Status PICC_Request(FM1702_Handle *hnd, uint8_t mode, uint8_t *atqa, uint8_t *atqaSize)
{
	assert(hnd != NULL);
	
	uint8_t validBits = 7;
	PCD_WriteReg(hnd, PCD_REG_CHANNEL_REDUNDANCY, 0x00);	// no CRC and partiy
	PCD_ClearBitMask(hnd, PCD_REG_CONTROL, 0x08);	// disable crypto1
	
	return PCD_CommWithPICC(hnd,
		PCD_CMD_TRANSCEIVE,
		&mode,	1, &validBits,
		atqa, atqaSize, 0,
		PCD_IRQ_TX | PCD_IRQ_RX | PCD_IRQ_IDLE,	// wait for TxIRQ, RxIRQ and IdleIRQ
		PCD_TC_START_TX_BEGIN | PCD_TC_STOP_RX_END, 5 * 1000);
}

FM1702_Status PICC_AntiColl(FM1702_Handle *hnd, uint8_t *uid, uint8_t *uidLen)
{
	PCD_WriteReg(hnd, PCD_REG_CHANNEL_REDUNDANCY, 0x03);	// partiy enable
	PCD_ClearBitMask(hnd, PCD_REG_CONTROL, 0x08);
	
	uint8_t validBits = 0;
	uint8_t rxAlign = 0;
	uint8_t sendBuf[7] = {0};
	sendBuf[0] = PICC_CMD_SEL_CL1;
	sendBuf[1] = 0x20;
	
	uint8_t status = FM1702_OK;
	
	for (int i = 0; i < 32; i++)
	{
		status = PCD_CommWithPICC(hnd, 
			PCD_CMD_TRANSCEIVE,
			sendBuf, 2, &validBits,
			uid, uidLen, rxAlign,
			PCD_IRQ_TX | PCD_IRQ_RX | PCD_IRQ_IDLE,
			PCD_TC_START_TX_BEGIN | PCD_TC_STOP_RX_END, 50 * 1000);
		if (status != FM1702_ERR_COLL)
		{
			break;
		}

		uint8_t collPos = PCD_ReadReg(hnd, PCD_REG_COLL_POS);
		uint8_t byteCount = collPos / 8;
		uint8_t bitCount = collPos % 8;
		
		sendBuf[1] = 0x20 | (byteCount << 4) | bitCount;
	}
	return status;
}

FM1702_Status PICC_Select(FM1702_Handle *hnd, uint8_t *uid, uint8_t uidLen, uint8_t *sak)
{
	PCD_WriteReg(hnd, PCD_REG_CHANNEL_REDUNDANCY, 0x0f);
	PCD_ClearBitMask(hnd, PCD_REG_CONTROL, 0x08);
	
	uint8_t sendBuf[17] = {0};
	sendBuf[0] = PICC_CMD_SEL_CL1;
	sendBuf[1] = 0x70;

	for (int i = 0; i < uidLen; i++)
	{
		sendBuf[i + 2] = uid[i];
		sendBuf[uidLen + 2] ^= uid[i];
	}
	uint8_t len = 2;

	return PCD_CommWithPICC(hnd,
		PCD_CMD_TRANSCEIVE,
		sendBuf, 7, NULL,
		sak, &len, 0,
		PCD_IRQ_TX | PCD_IRQ_RX | PCD_IRQ_IDLE,
		PCD_TC_START_TX_BEGIN | PCD_TC_STOP_RX_END, 50 * 1000);
}

void PCD_EncodeKey(uint8_t *rawKey, uint8_t *codedKey)
{
	assert(rawKey && codedKey);
	for (int i = 0; i < 6; i++)
	{
		uint8_t l = rawKey[i] & 0x0f;
		uint8_t h = rawKey[i] >> 4;
		
		codedKey[i * 2] = (~h << 4) | h;
		codedKey[i * 2 + 1] = (~l << 4) | l;
	}
}

FM1702_Status PCD_LoadKey(FM1702_Handle *hnd, uint8_t *key)
{
	assert(hnd != NULL);
	
	uint8_t codedKey[12] = {0};
	PCD_EncodeKey(key, codedKey);
	
	return PCD_ExecuteCommand(hnd,
		PCD_CMD_LOAD_KEY, codedKey, 12,
		PCD_IRQ_IDLE,
		PCD_TC_MANUAL, 5000);
}

FM1702_Status PICC_Authent(FM1702_Handle *hnd, uint8_t sector, uint8_t authType, uint8_t *uid, uint8_t uidLen, uint8_t *key)
{
	assert(hnd != NULL);
	
	PCD_WriteReg(hnd, PCD_REG_CHANNEL_REDUNDANCY, 0x0f);	// CRC and partiy
	PCD_ClearBitMask(hnd, PCD_REG_CONTROL, 0x08);	// disable crypto1

	uint8_t status = PCD_LoadKey(hnd, key);
	if (status != FM1702_OK)
	{
		return status;
	}
	
	uint8_t sendBuf[12] = {0};
	sendBuf[0] = authType;
	sendBuf[1] = sector * 4 + 3;
	
	for (int i = 0; i < uidLen; i++)
	{
		sendBuf[i + 2] = uid[i];
	}
	
	status = PCD_CommWithPICC(hnd,
		PCD_CMD_AUTHENT_1,
		sendBuf, 2 + uidLen, 0,
		NULL, NULL, 0,
		PCD_IRQ_IDLE,
		PCD_TC_MANUAL, 50 * 1000);
	if (status != FM1702_OK)
	{
		return status;
	}

	status = PCD_CommWithPICC(hnd,
		PCD_CMD_AUTHENT_2,
		NULL, NULL, 0,
		NULL, NULL, 0,
		PCD_IRQ_IDLE,
		PCD_TC_MANUAL, 50 * 1000);
	
	if (status != FM1702_OK)
	{
		return status;
	}
	
	uint8_t ctrl = PCD_ReadReg(hnd, PCD_REG_CONTROL);
	
	if (!(ctrl & 0x08))
	{
		return FM1702_ERR_AUTH;
	}
	
	return status;
}

FM1702_Status MIFARE_Read(FM1702_Handle *hnd, uint8_t block, uint8_t *outBuf)
{
	assert(hnd != NULL);

	PCD_WriteReg(hnd, PCD_REG_CHANNEL_REDUNDANCY, 0x0f);
	
	uint8_t sendBuf[16] = {0};
	sendBuf[0] = PICC_CMD_MF_READ;
	sendBuf[1] = block;
	
	uint8_t len = 20;
	
	uint8_t status = PCD_CommWithPICC(hnd,
		PCD_CMD_TRANSCEIVE,
		sendBuf, 2, 0,
		outBuf, &len, 0,
		PCD_IRQ_TX | PCD_IRQ_RX | PCD_IRQ_IDLE,
		PCD_TC_START_TX_BEGIN | PCD_TC_STOP_RX_END, 50 * 1000);
	
	return status;
}

FM1702_Status MIFARE_Write(FM1702_Handle *hnd, uint8_t block, uint8_t *inBuf)
{
	assert(hnd && inBuf);
	
	PCD_WriteReg(hnd, PCD_REG_CHANNEL_REDUNDANCY, 0x07);
	uint8_t len = 1;
	uint8_t ack[2] = {0};
	uint8_t sendBuf[18] = {0};
	sendBuf[0] = PICC_CMD_MF_WRITE;
	sendBuf[1] = block;
	

	uint8_t status = PCD_CommWithPICC(hnd,
		PCD_CMD_TRANSCEIVE,
		sendBuf, 2, 0,
		ack, &len, 0,
		PCD_IRQ_TX | PCD_IRQ_RX | PCD_IRQ_IDLE,
		PCD_TC_START_TX_BEGIN | PCD_TC_STOP_RX_END, 50 * 1000);

	
	if (status != FM1702_OK)
	{
		return status;
	}

	switch (ack[0] & 0x0f)
	{
	case 0x0A:
		status = FM1702_OK;
		break;
	case 0x00:
		status = FM1702_ERR_AUTH;
		break;
	default:
		status = FM1702_ERR_MISC;
		break;
	}

	status = PCD_CommWithPICC(hnd,
		PCD_CMD_TRANSCEIVE,
		inBuf, 16, 0,
		ack, &len, 0,
		PCD_IRQ_TX | PCD_IRQ_RX | PCD_IRQ_IDLE,
		PCD_TC_START_TX_BEGIN | PCD_TC_STOP_RX_END, 50 * 1000);
	
	if (status != FM1702_OK)
	{
		return status;
	}

	switch (ack[0] & 0x0f)
	{
	case 0x0A:
		status = FM1702_OK;
		break;
	case 0x00:
		status = FM1702_ERR_AUTH;
		break;
	default:
		status = FM1702_ERR_MISC;
		break;
	}
	
	return status;
}

void MIFARE_SetAccessBits(uint8_t *accessBitsBuf, uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3)
{
	const uint8_t c1 = ((g3 & 0x04) << 1) | ((g2 & 0x04) << 0) | ((g1 & 0x04) >> 1) | ((g0 & 0x04) >> 2);
	const uint8_t c2 = ((g3 & 0x02) << 2) | ((g2 & 0x02) << 1) | ((g1 & 0x02) << 0) | ((g0 & 0x02) >> 1);
	const uint8_t c3 = ((g3 & 0x01) << 3) | ((g2 & 0x01) << 2) | ((g1 & 0x01) << 1) | ((g0 & 0x01) << 0);
		
	accessBitsBuf[0] = (~c2 & 0x0f) << 4 | (~c1 & 0x0f);
	accessBitsBuf[1] = c1 << 4 | (~c3 & 0x0f);
	accessBitsBuf[2] = c3 << 4 | c2;
}

FM1702_Status PICC_Halt(FM1702_Handle *hnd)
{
	assert(hnd);
	PCD_WriteReg(hnd, PCD_REG_CHANNEL_REDUNDANCY, 0x07);
	
	uint8_t sendBuf[2] = {0};
	sendBuf[0] = PICC_CMD_HLTA;
	sendBuf[1] = 0x00;
	
	return PCD_CommWithPICC(hnd,
		PCD_CMD_TRANSMIT,
		sendBuf, 2, 0,
		NULL, NULL, 0,
		PCD_IRQ_TX | PCD_IRQ_IDLE,
		PCD_TC_START_TX_BEGIN, 50 * 1000);
}
