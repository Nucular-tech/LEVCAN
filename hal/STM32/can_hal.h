//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#include "stdint.h"
#include "levcan.h"

#pragma once

#ifndef CAN1
#define CAN1 CAN
#endif

#ifdef ConnectivityLine
#define CAN_FilterSize 28
#else
#define CAN_FilterSize 14
#endif

#define CAN_ForceEXID
//#define CAN_ForceSTID

#define CAN_FIFO_0		0
#define CAN_FIFO_1		1

enum {
	CAN_Request, CAN_Data
};

typedef enum {
	CANH_Ok, CANH_QueueFull, CANH_QueueEmpty, CANH_Fail
} CAN_Status;

enum {
	CANH_EC_No_Error,
	CANH_EC_Stuff_Error,
	CANH_EC_Form_Error,
	CANH_EC_Acknowledgment_Error,
	CANH_EC_Bit_recessive_Error,
	CANH_EC_Bit_dominant_Error,
	CANH_EC_CRC_Error,
	CANH_EC_Set_by_software
};
//hardware level struct
typedef union {
	//union
	uint32_t ToUint32;
	//11 bit
	struct {
		unsigned Transmit11b :1;
		unsigned Request11b :1;
		unsigned ExtensionID11b :1;
		unsigned reserved :18; //just skip those, use next one
		unsigned STID :11;
	}__attribute__((packed));
	//29 bit
	struct {
		unsigned Transmit :1;
		unsigned Request :1;
		unsigned ExtensionID :1;
		unsigned EXID :29;
	}__attribute__((packed));
} CAN_IR; //identifier register

void CAN_InitFromClock(uint32_t PCLK, uint32_t bitrate_khz, uint16_t sjw, uint16_t sample_point);
void CAN_Init(uint32_t BTR);
void CAN_Start(void);

void CAN_FiltersClear(void);
void CAN_FilterEditOn(void);
LC_Return_t CAN_CreateFilterIndex(CAN_IR reg, uint16_t fifo);
LC_Return_t CAN_CreateFilterMask(CAN_IR reg, CAN_IR mask, uint8_t fifo);
LC_Return_t CAN_CreateFilterMaskPosition(CAN_IR reg, CAN_IR mask, uint8_t fifo, uint8_t position);
void CAN_FilterEditOff(void);

LC_Return_t CAN_Send(CAN_IR index, uint32_t *data, uint16_t length);
LC_Return_t CAN_Receive(CAN_IR *index, uint32_t *data, uint16_t *length);

LC_Return_t LC_HAL_Send(LC_HeaderPacked_t header, uint32_t *data, uint8_t length);
LC_Return_t LC_HAL_CreateFilterMasks(LC_HeaderPacked_t *reg, LC_HeaderPacked_t *mask, uint16_t count);
LC_Return_t LC_HAL_TxHalfFull();
#ifdef LEVCAN_USE_RTOS_QUEUE
void txCanTask(void *);
#endif
extern volatile int CAN_ERR;
extern void *LEVCAN_Node_Drv;
