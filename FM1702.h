/**
 * FM1702 MIFARE reader IC driver for STM32 MCUs
 *
 * Copyright (c) 2018 by Jay Xu.
 * All Rights Reserved.
 */
#ifndef FM1702_H
#define FM1702_H

#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <Driver_SPI.h>

enum PCD_Register
{
	PCD_REG_PAGE = 0x00,
	PCD_REG_COMMAND = 0x01,
	PCD_REG_FIFO_DATA = 0x02,
	PCD_REG_PRIMARY_STATUS = 0x03,
	PCD_REG_FIFO_LENGTH = 0x04,
	PCD_REG_SECONDARY_STATUS = 0x05,
	PCD_REG_INTERRUPT_EN = 0x06,
	PCD_REG_INTERRUPT_RQ = 0x07,
	
	PCD_REG_CONTROL = 0x09,
	PCD_REG_ERROR_FLAG = 0x0A,
	PCD_REG_COLL_POS = 0x0B,
	PCD_REG_TIMER_VALUE = 0x0C,
	PCD_REG_CRC_RESULT_LSB = 0x0D,
	PCD_REG_CRC_RESULT_MSB = 0x0E,
	PCD_REG_BIT_FRAMING = 0x0F,
	
	PCD_REG_TX_CONTROL = 0x11,
	PCD_REG_CW_CONDUCTANCE = 0x12,
	PCD_REG_MOD_WIDTH = 0x15,

	PCD_REG_RX_CONTROL_1 = 0x19,
	PCD_REG_DECODER_CONTROL = 0x1A,
	PCD_REG_BIT_PHASE = 0x1B,
	PCD_REG_RX_THRESHOLD = 0x1C,
	PCD_REG_RX_CONTROL_2 = 0x1E,
	PCD_REG_CLOCK_Q_CONTROL = 0x1F,
	
	PCD_REG_RX_WAIT = 0x21,
	PCD_REG_CHANNEL_REDUNDANCY = 0x22,
	PCD_REG_CRC_PRESET_LSB = 0x23,
	PCD_REG_CRC_PRESET_MSB = 0x24,
	
	PCD_REG_FIFO_LEVEL = 0x29,
	PCD_REG_TIMER_CLOCK = 0x2A,
	PCD_REG_TIMER_CONTROL = 0x2B,
	PCD_REG_TIMER_RELOAD = 0x2C,
	PCD_REG_IRQ_PIN_CONFIG = 0x2D,
	
	PCD_REG_CRYPTO_SELECT = 0x31
};

enum PCD_Command
{
	PCD_CMD_START_UP = 0x3F,
	PCD_CMD_IDLE = 0x00,
	PCD_CMD_TRANSMIT = 0x1A,
	PCD_CMD_RECEIVE = 0x16,
	PCD_CMD_TRANSCEIVE = 0x1E,
	PCD_CMD_WRITE_E2 = 0x01,
	PCD_CMD_READ_E2 = 0x03,
	PCD_CMD_LOAD_KEY_E2 = 0x0B,
	PCD_CMD_LOAD_KEY = 0x19,
	PCD_CMD_AUTHENT_1 = 0x0C,
	PCD_CMD_AUTHENT_2 = 0x14,
	PCD_CMD_LOAD_CONFIG = 0x07,
	PCD_CMD_CALC_CRC = 0x12
};

enum PICC_Command
{
	PICC_CMD_REQA = 0x26,
	PICC_CMD_WUPA = 0x52,
	PICC_CMD_SEL_CL1 = 0x93,
	PICC_CMD_SEL_CL2 = 0x95,
	PICC_CMD_SEL_CL3 = 0x97,
	PICC_CMD_HLTA = 0x50,
	
	PICC_CMD_MF_AUTH_KEY_A = 0x60,
	PICC_CMD_MF_AUTH_KEY_B = 0x61,
	PICC_CMD_MF_READ = 0x30,
	PICC_CMD_MF_WRITE = 0xA0,
	PICC_CMD_MF_DECREMENT = 0xC0,
	PICC_CMD_MF_INCREMENT = 0xC1,
	PICC_CMD_MF_RESTORE = 0xC2,
	PICC_CMD_MF_TRANSFER = 0xB0
};

enum PCD_Timer_Control
{
	PCD_TC_MANUAL = 0x00,
	PCD_TC_START_TX_BEGIN = 0x01,
	PCD_TC_START_TX_END = 0x02,
	PCD_TC_STOP_RX_BEGIN = 0x04,
	PCD_TC_STOP_RX_END = 0x08
};

enum PCD_IRQ
{
	PCD_IRQ_NONE = 0x00,
	PCD_IRQ_LO_ALERT = 0x01,
	PCD_IRQ_HI_ALERT = 0x02,
	PCD_IRQ_IDLE = 0x04,
	PCD_IRQ_RX = 0x08,
	PCD_IRQ_TX = 0x10,
	PCD_IRQ_TIMER = 0x20
};

typedef enum
{
	FM1702_OK = 0,
	FM1702_ERR_INIT = 1,
	FM1702_ERR_TIMEOUT = 2,
	FM1702_ERR_NO_ROOM = 3,
	FM1702_ERR_KEY = 4,
	FM1702_ERR_ACCESS = 5,
	FM1702_ERR_FIFO_OVFL = 6,
	FM1702_ERR_CRC = 7,
	FM1702_ERR_FRAMING = 8,
	FM1702_ERR_PARITY = 9,
	FM1702_ERR_COLL = 10,
	FM1702_ERR_AUTH = 11,
	FM1702_ERR_MISC = 12,
	FM1702_ERR_UID = 13,
	FM1702_ERR_TAG_TYPE = 14
} FM1702_Status;

typedef struct
{
	GPIO_TypeDef *csPort;	// chip select port
	GPIO_TypeDef *resetPort;	// pin of chip select
	GPIO_TypeDef *irqPort;
	ARM_DRIVER_SPI *spiDrv;	// SPI driver
	uint16_t csPin;	// 
	uint16_t resetPin;
	uint16_t irqPin;
} FM1702_Handle;

typedef struct
{
	uint8_t uid[10];
	uint8_t uidSize;
	uint8_t sak;
} ISO14443_UID;


#ifdef __cplusplus
extern "C"
{
#endif

extern void PCD_Init(
	FM1702_Handle *hnd,
	ARM_DRIVER_SPI *spiDrv,
	GPIO_TypeDef *chipSelectPort,
	uint16_t chipSelectPin,
	GPIO_TypeDef *resetPort,
	uint16_t resetPin,
	GPIO_TypeDef *irqPort,
	uint16_t irqPin
);

extern void PCD_WriteReg(FM1702_Handle * hnd, uint8_t reg, uint8_t value);
extern uint8_t PCD_ReadReg(FM1702_Handle * hnd, uint8_t reg);
extern void PCD_WriteFIFO(FM1702_Handle *hnd, uint8_t *data, uint8_t len);
extern uint8_t PCD_ReadFIFO(FM1702_Handle *hnd, uint8_t *data);


/**
 * preform REQA or WUPA command to PICC
 */
extern FM1702_Status PICC_Request(FM1702_Handle *hnd, uint8_t mode, uint8_t *atqa, uint8_t *atqaSize);

/**
 * preform anti collision loop for PICC
 */
extern FM1702_Status PICC_AntiColl(FM1702_Handle *hnd, uint8_t *uid, uint8_t *uidLen);

/**
 * 
 */
extern FM1702_Status PICC_Select(FM1702_Handle *hnd, uint8_t *uid, uint8_t uidLen, uint8_t *sak);
extern FM1702_Status PICC_Authent(FM1702_Handle *hnd, uint8_t sector, uint8_t authType, uint8_t *uid, uint8_t uidLen, uint8_t *key);
extern FM1702_Status MIFARE_Read(FM1702_Handle *hnd, uint8_t block, uint8_t *outBuf);
extern FM1702_Status MIFARE_Write(FM1702_Handle *hnd, uint8_t block, uint8_t *inBuf);

extern FM1702_Status PICC_Halt(FM1702_Handle *hnd);

extern FM1702_Status PCD_SoftPowerDown(FM1702_Handle *hnd);
extern FM1702_Status PCD_SoftPowerUp(FM1702_Handle *hnd);

#ifdef __cplusplus
}
#endif

#endif
