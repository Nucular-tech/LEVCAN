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
/*
 * levcan_paramcommon.h
 *
 *  Created on: 2 февр. 2021 г.
 *      Author: VasiliSk
 */

#pragma once

typedef enum {
	LCP_AccessLvl_Any = 0, //
	LCP_AccessLvl_User = 2, //
	LCP_AccessLvl_Service = 5, //
	LCP_AccessLvl_Dev = 7, //
} LCP_AccessLvl_t; //3 bit

typedef enum {
	LCP_Normal = 0, 			//RW
	LCP_ReadOnly = 1,			//
	LCP_WriteOnly = 2, 			//
	LCP_Invalid = 3, 			//
	LCP_LiveUpdate = 1 << 2, 	//update values realtime
	LCP_LiveChange = 1 << 3, 	//apply value changes realtime
	LCP_Empty = 0xFF
} LCP_Mode_t;

typedef enum {
	LCP_Folder, LCP_Label, //null Descriptor
	LCP_Bool, //LCP_Bool_t
	LCP_Enum, //LCP_Enum_t
	LCP_Bitfield32, //LCP_Bitfield_t
	LCP_Int32, //LCP_Int32_t
	LCP_Uint32, //LCP_Uint32_t
	LCP_Int64, //LCP_Int64_t
	LCP_Uint64, //LCP_Uint64_t
	LCP_Float, //LCP_Float_t
	LCP_Double, //LCP_Double_t
	LCP_Plot2Dsimple_int32, //LCP_Plot2Dsimple_int32_t
	LCP_End = 0xFF
} LCP_Type_t;

typedef struct {
} LCP_Bool_t;

typedef struct {
	uint32_t Min;
	uint32_t Max;
} LCP_Enum_t;

typedef struct {
	uint32_t Mask;
} LCP_Bitfield32_t;

typedef struct {
	uint32_t Min;
	uint32_t Max;
	uint32_t Step;
	uint8_t Decimals;
} LCP_Uint32_t;

typedef struct {
	int32_t Min;
	int32_t Max;
	int32_t Step;
	uint8_t Decimals;
} LCP_Int32_t;

typedef struct {
	uint64_t Min;
	uint64_t Max;
	uint64_t Step;
	uint8_t Decimals;
} LCP_Uint64_t;

typedef struct {
	int64_t Min;
	int64_t Max;
	int64_t Step;
	uint8_t Decimals;
} LCP_Int64_t;

typedef struct {
	float Min;
	float Max;
	float Step;
} LCP_Float_t;

typedef struct {
	double Min;
	double Max;
	double Step;
} LCP_Double_t;

typedef struct {
	struct {
		int32_t Min;
		int32_t Max;
		int32_t Step;
	} X;
	struct {
		int32_t Min;
		int32_t Max;
		int32_t Step;
	} Y;
	uint8_t DataSize;
} LCP_Plot2Dsimple_int32_t;
