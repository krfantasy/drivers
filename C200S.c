/**
 * FPC1102 finger print module
 *
 * Copyright (c) 2018 by Jay Xu.
 * All Rights Reserved.
 */
#include <assert.h>
#include <Driver_USART.h>
#include <string.h>

#include "C200S.h"

extern ARM_DRIVER_USART Driver_UART1;

void C200S_Init(C200S_Handle *hnd, ARM_DRIVER_USART *usartDrv, GPIO_TypeDef *irqPort, uint16_t irqPin)
{
	assert(hnd != NULL);
	
	hnd->usart = usartDrv;
	hnd->irqPort = irqPort;
	hnd->irqPin = irqPin;
	
	hnd->usart->Initialize(NULL);
	hnd->usart->PowerControl(ARM_POWER_FULL);
	hnd->usart->Control(ARM_USART_MODE_ASYNCHRONOUS |
		ARM_USART_DATA_BITS_8 |
		ARM_USART_PARITY_NONE |
		ARM_USART_STOP_BITS_1 |
		ARM_USART_FLOW_CONTROL_NONE, 57600);
	
	hnd->usart->Control(ARM_USART_CONTROL_TX, 1);
	hnd->usart->Control(ARM_USART_CONTROL_RX, 1);
	
}

static inline bool executeCommand(C200S_Handle *hnd,
	uint8_t cmd,
	uint8_t *sendData,
	uint16_t sendLen,
	uint8_t *readData,
	uint16_t *readLen)
{
	assert(hnd);
	
	uint8_t buf[20] = {0};
	buf[0] = 0xEF;
	buf[1] = 0x01;
	for (uint8_t i = 0; i < 4; i++)
	{
		buf[i + 2] = 0xFF;
	}
	buf[6] = 0x01;
	uint16_t realSendLen = sendLen + 2 + 1;
	buf[7] = (realSendLen >> 8) & 0xFF;
	buf[8] = realSendLen & 0xFF;
	buf[9] = cmd;

	for (uint16_t i = 0; i < sendLen; i++)
	{
		buf[10 + i] = sendData[i];
	}
	
	uint16_t checksum = 0;
	for (int i = 6; i < 10 + sendLen; i++)
	{
		checksum += buf[i];
	}
	buf[10 + sendLen] = (checksum >> 8) & 0xFF;
	buf[10 + sendLen + 1] = checksum & 0xFF;
	for (int i = 0; i < 10 + sendLen + 2; i++)
	{
		printf("%02x ", buf[i]);
	}
	printf("\r\n");
	
	hnd->usart->Send(buf, 10 + sendLen + 2);
	while (hnd->usart->GetStatus().tx_busy);

	memset(buf, 0, sizeof(buf));
	uint8_t recvLen = readLen ? *readLen : 0;
	hnd->usart->Receive(buf, 12 + recvLen);
	while (hnd->usart->GetStatus().rx_busy);

	for (int i = 0; i < 12 + recvLen; i++)
	{
		printf("%02x ", buf[i]);
	}
	printf("\r\n");
	
	if (readData && readLen)
	{
		uint16_t len = (buf[7] << 8) | buf[8];
		printf("len = %d\r\n", len);
		if (len - 3 > *readLen)
		{
			*readLen = len;
			return false;
		}
		*readLen = len;
		for (uint16_t i = 0; i < *readLen; i++)
		{
			readData[i] = buf[10 + i];
		}
	}
	
	return (buf[9] == 0x00);
}

bool C200S_GetFingerPrint(C200S_Handle *hnd)
{
	uint8_t data = 0x01;
	return executeCommand(hnd, C200S_CMD_GET_IMAGE, NULL, 0, NULL, 0);
}

bool C200S_GenerateFeatures(C200S_Handle *hnd, uint8_t step)
{
	return executeCommand(hnd, C200S_CMD_GEN_FEATURES, &step, sizeof(step), NULL, 0);
}

bool C200S_MergeFeatures(C200S_Handle *hnd)
{
	return executeCommand(hnd, C200S_CMD_MERGE_FEATURES, NULL, 0, NULL, 0);
}

bool C200S_StoreFingerPrint(C200S_Handle *hnd, uint8_t idx)
{
	uint8_t data[3] = {0x01, 0x00, idx};
	
	return executeCommand(hnd, C200S_CMD_STORE, data, sizeof(data), NULL, 0);
}

bool C200S_SearchFingerPrint(C200S_Handle *hnd, uint16_t *idx, uint16_t *similarity)
{
	uint8_t data[] = {0x01, 0x00, 0x00, 0x00, 0x64};
	uint8_t readData[4] = {0};
	uint16_t readLen = 4;
	bool err = executeCommand(hnd, C200S_CMD_SEARCH, data, sizeof(data), readData, &readLen);
	*idx = (readData[0] << 8) | readData[1];
	*similarity = (readData[2] << 8) | readData[3];
	return err;
}

bool C200S_EnterSleepMode(C200S_Handle *hnd)
{
	return executeCommand(hnd, C200S_CMD_SLEEP, NULL, 0, NULL, NULL);
}

bool C200S_DeleteFingerPrint(C200S_Handle *hnd, uint16_t idx)
{
	uint8_t data[2] = {0};
	data[0] = (idx >> 8) & 0xFF;
	data[1] = idx & 0xFF;
	return executeCommand(hnd, C200S_CMD_DELETE, data, 2, NULL, NULL);
}

int32_t C200S_GetFingerPrintCount(C200S_Handle *hnd)
{
	uint8_t readData[2] = {0};
	uint16_t readLen = 2;
	bool err = executeCommand(hnd, C200S_CMD_COUNT, NULL, 0, readData, &readLen);
	if (!err)
	{
		return -1;
	}
	return (readData[0] << 8) | readData[1];
}

bool C200S_AddFingerPrint(C200S_Handle *hnd, const uint8_t id)
{
	bool status = false;
	for (int i = 1; i <= 4; i++)
	{
		status = C200S_GetFingerPrint(hnd);

		if (!status)
		{
			printf("get finger print failed\r\n");
			goto End;
		}
		
		status = C200S_GenerateFeatures(hnd, i);
		
		if (!status)
		{
			printf("generate features failed\r\n");
			goto End;
		}
	}
	
	status = C200S_MergeFeatures(hnd);
	if (!status)
	{
		printf("merge features failed\r\n");
		goto End;
	}
	
	C200S_StoreFingerPrint(hnd, id);
	if (!status)
	{
		printf("store finger print failed\r\n");
		goto End;
	}
	printf("store finger print done\r\n");	
End:
		return status;
}

bool C200S_FindFingerPrint(C200S_Handle *hnd, uint16_t *idx, uint16_t *sim)
{
	bool status = false;
	status = C200S_GetFingerPrint(hnd);

	if (!status)
	{
		printf("get finger print failed\r\n");
		goto End;
	}
	status = C200S_GenerateFeatures(hnd, 1);
	
	if (!status)
	{
		printf("generate features failed\r\n");
		goto End;
	}
	
	status = C200S_SearchFingerPrint(hnd, idx, sim);
	
	if (!status)
	{
		printf("finger print not found\r\n");
		goto End;
	}

	
End:
		return status;
}