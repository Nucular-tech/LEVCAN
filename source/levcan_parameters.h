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

#include <stdint.h>
#include "levcan.h"

#pragma once

typedef struct {
	uint8_t EntryType; //LCP_Type_t
	uint8_t AccessLvl; //LCP_AccessLvl_t
	uint8_t Mode; //LCP_Mode_t
	uint8_t rsrvd;
	union {
		uint16_t VarSize; //in bytes
		uint16_t DirectoryIndex;
	};
	uint16_t DescSize; //in bytes
	const void *Variable; //address of variable or structure
	const void *Descriptor;
	const char *Name;
	const char *TextData;
} LCP_Entry_t;

typedef struct {
	const LCP_Entry_t *Entries;
	uint16_t Size;
	uint16_t ArrayIndex;
} LCP_Directory_t;

typedef enum {
	LCP_AccessLvl_Any = 0, //
	LCP_AccessLvl_User = 2, //
	LCP_AccessLvl_Service = 5, //
	LCP_AccessLvl_Dev = 7, //
} LCP_AccessLvl_t; //3 bit

typedef enum {
	LCP_Normal, LCP_ReadOnly, LCP_WriteOnly, LCP_Empty = 0xFF
} LCP_Mode_t;

typedef enum {
	LCP_Header, LCP_HeaderArray, LCP_Folder, LCP_Label, //
	LCP_Int32, //
	LCP_Uint32, //
	LCP_Int64, //
	LCP_Uint64, //
	LCP_Float, //
	LCP_Double, //
	LCP_Plot2Dsimple_int32, //
	LCP_End = 0xFF
} LCP_Type_t;

typedef struct {
	uint32_t Min;
	uint32_t Max;
	uint32_t Step;
	uint8_t Decimals;
	uint8_t Type;
} LCP_Uint32_t;

typedef struct {
	int32_t Min;
	int32_t Max;
	int32_t Step;
	uint8_t Decimals;
	uint8_t Type;
} LCP_Int32_t;

typedef struct {
	uint64_t Min;
	uint64_t Max;
	uint64_t Step;
	uint8_t Decimals;
	uint8_t Type;
} LCP_Uint64_t;

typedef struct {
	int64_t Min;
	int64_t Max;
	int64_t Step;
	uint8_t Decimals;
	uint8_t Type;
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

typedef enum {
	IT_value, 	//integer parameter, <name> and maybe <formatting> where '%s' defines position of converted <value>
	IT_bool, 	//ON/OFF parameter, <name> + ON / OFF default  (formatting not sent)
	IT_enum,	//string enumerator, <name> + <formatting> ("1\n2\n3\n"), where <value> = line index, or if out of index, write <value>
} LCP_IntType_t;

#define standardTypes(_val)  _Generic((_val),		\
		LCP_Uint32_t: 	LCP_Uint32,	\
		LCP_Int32_t: 	LCP_Int32,	\
		LCP_Uint64_t: 	LCP_Uint64,	\
		LCP_Int64_t: 	LCP_Int64,	\
		LCP_Float_t: 	LCP_Float,	\
		LCP_Double_t: 	LCP_Double)

#define param( _AccessLvl, _Mode, _Var, _Desc, _Name, _Text) \
	{standardTypes(_Desc), _AccessLvl, _Mode, 0, sizeof(_Var), sizeof(_Desc), (void*)&_Var, (void*)&_Desc, _Name, _Text}
