/*******************************************************************************
 * LEVCAN: Light Electric Vehicle CAN protocol [LC]
 * Copyright (C) 2020 Vasiliy Sukhoparov
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include "stdint.h"

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
CAN_Status CAN_CreateFilterIndex(CAN_IR reg, uint16_t fifo);
CAN_Status CAN_CreateFilterMask(CAN_IR reg, CAN_IR mask, uint8_t fifo);
void CAN_FilterEditOff(void);

CAN_Status CAN_Send(CAN_IR index, uint32_t *data, uint16_t length) ;
CAN_Status CAN_Receive(CAN_IR *index, uint32_t *data, uint16_t *length);
