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
	LCP_Decimal32, //LCP_Decimal32_t
	LCP_String, //LCP_String_t
	LCP_End = 0xFF
} LCP_Type_t;

/*typedef struct {

 } LCP_Bool_t;*/

typedef struct {
	uint32_t Min;
	uint32_t Size;
} LCP_Enum_t;

typedef struct {
	uint32_t Mask;
} LCP_Bitfield32_t;

typedef struct {
	uint32_t Min;
	uint32_t Max;
	uint32_t Step;
} LCP_Uint32_t;

typedef struct {
	int32_t Min;
	int32_t Max;
	int32_t Step;
} LCP_Int32_t;

typedef struct {
	uint64_t Min;
	uint64_t Max;
	uint64_t Step;
} LCP_Uint64_t;

typedef struct {
	int64_t Min;
	int64_t Max;
	int64_t Step;
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
	int32_t Min;
	int32_t Max;
	int32_t Step;
	uint8_t Decimals;
} LCP_Decimal32_t;

typedef struct {
	uint16_t Flags; //LCP_StringFlags
} LCP_String_t;

typedef enum {
	StringFlags_None = 0, // just text
	StringFlags_Numeric = 1 << 0, // numbers only
	StringFlags_Letters = 1 << 1, // letters only
	StringFlags_SpecChar = 1 << 2, // special character, e.g., ! @ # ?
	StringFlags_Password = 1 << 3, // show only ***
	StringFlags_Email = 1 << 4, // one@two.com
	StringFlags_Phone = 1 << 5, // +1(234)567 89 89
	StringFlags_Website = 1 << 6, // www.dot.com
	StringFlags_Multiline = 1 << 7, // allow /r/n
	StringFlags_UTF8 = 1 << 8, // force UTF8 encoding (char)
	StringFlags_UTF16 = 1 << 9, // force UTF16 encoding (wchar)
} LCP_StringFlags;

LC_EXPORT LC_Return_t LCP_LimitValue(intptr_t *variable, uint16_t varSize, const intptr_t *descriptor, uint16_t descSize, uint8_t type);

int32_t lcp_getInt32(intptr_t *variable, uint16_t varSize);
void lcp_setInt32(intptr_t *variable, uint16_t varSize, int32_t value);
LC_Return_t lcp_i32inRange(intptr_t *variable, uint16_t varSize, int32_t min, int32_t max);
LC_Return_t lcp_i64inRange(intptr_t *variable, uint16_t varSize, int64_t min, int64_t max);

uint32_t lcp_getUint32(intptr_t *variable, uint16_t varSize);
void lcp_setUint32(intptr_t *variable, uint16_t varSize, uint32_t value);
LC_Return_t lcp_u32inRange(intptr_t *variable, uint16_t varSize, uint32_t min, uint32_t max);
LC_Return_t lcp_u64inRange(intptr_t *variable, uint16_t varSize, uint64_t min, uint64_t max);

LC_Return_t lcp_f32inRange(intptr_t *variable, uint16_t varSize, float min, float max);
LC_Return_t lcp_d64inRange(intptr_t *variable, uint16_t varSize, double min, double max);

int lcp_print_i32f(char *buffer, int32_t value, uint8_t decimals);
int lcp_print_u32b(char *buffer, uint32_t value) ;


uint32_t lcp_pow10i(uint8_t pow);
