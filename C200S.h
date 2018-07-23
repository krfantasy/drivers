/**
 * FPC1102 finger print sensor module
 *
 * Copyright (c) 2018 by Jay Xu.
 * All Rights Reserved.
 */
#ifndef C200S_H
#define C200S_H

#include <stm32f0xx_hal.h>
#include <Driver_USART.h>

typedef struct
{
	ARM_DRIVER_USART *usart;
	GPIO_TypeDef *irqPort;
	uint16_t irqPin;
} C200S_Handle;

enum C200S_Command
{
	C200S_CMD_GET_IMAGE = 0x01,
	C200S_CMD_GEN_FEATURES = 0x02,
	C200S_CMD_MERGE_FEATURES = 0x05,
	C200S_CMD_STORE = 0x06,
	C200S_CMD_SEARCH = 0x04,
	C200S_CMD_DELETE = 0x0C,
	C200S_CMD_CLEAR = 0x0D,
	C200S_CMD_COUNT = 0x1D,
	C200S_CMD_SLEEP = 0x33
};

typedef enum
{
	C200S_OK = 0,
	C200S_FAILED = 1,
	C200S_ERR_DATA_LOST = 2,
	C200S_ERR_NO_IMAGE = 3,
	C200S_ERR_NOT_FOUND = 4
} C200S_Status;

#ifdef __cplusplus
extern "C"
{
#endif

extern void C200S_Init(
	C200S_Handle *hnds,
	ARM_DRIVER_USART *usartDrv,
	GPIO_TypeDef *irqPort,
	uint16_t irqPin);


extern bool C200S_GetFingerPrint(C200S_Handle *hnd);
extern bool C200S_GenerateFeatures(C200S_Handle *hnd, const uint8_t step);
extern bool C200S_MergeFeatures(C200S_Handle *hnd);
extern bool C200S_StoreFingerPrint(C200S_Handle *hnd, const uint8_t storeIdx);

extern bool C200S_SearchFingerPrint(C200S_Handle *hnd, uint16_t *idx, uint16_t *similarity);
extern bool C200S_EnterSleepMode(C200S_Handle *hnd);
extern bool C200S_DeleteFingerPrint(C200S_Handle *hnd, const uint16_t idx);
extern int32_t C200S_GetFingerPrintCount(C200S_Handle *hnd);


extern bool C200S_AddFingerPrint(C200S_Handle *hnd, const uint8_t id);
extern bool C200S_FindFingerPrint(C200S_Handle *hnd, uint16_t *idx, uint16_t *sim);

#ifdef __cplusplus
}
#endif

#endif