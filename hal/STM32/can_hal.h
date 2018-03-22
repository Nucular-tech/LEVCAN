/*
 * CAN driver library by V.Sukhoparov
 */

#include "stm32f4xx.h"

#pragma once

#ifdef PCBV2_2
#define CAN_HAL_BTR 0x001b0002
#endif
#ifdef PCBV2_1
#define CAN_HAL_BTR 0x001a0002
#endif

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

void CAN_Init(void);
void CAN_Start(void);

void CAN_FiltersClear(void);
void CAN_FilterEditOn(void);
CAN_Status CAN_CreateFilterIndex(CAN_IR reg, uint16_t fifo);
CAN_Status CAN_CreateFilterMask(CAN_IR reg, CAN_IR mask, uint8_t fifo);
void CAN_FilterEditOff(void);

//CAN_Status CAN_SendIndex(uint32_t index32);
CAN_Status CAN_Send(uint32_t index32, uint32_t* data, uint16_t length);
CAN_Status CAN_Receive(uint32_t* index32, uint32_t* data, uint16_t* length);
