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
#include "levcan_paramcommon.h"

#pragma once

//typedef LC_Return_t (*LCP_ParameterCallback_t)(LC_NodeDescriptor_t *node, LC_Header_t header, LCPS_Entry_t ** entry_out, uint16_t directory_index, uint16_t entry_index);

typedef struct {
	const void *Variable; //address of variable or structure
	const void *Descriptor; //depends on LCP_Type_t
	const char *Name; //null terminated
	const char *TextData; //null terminated
	uint16_t VarSize; //in bytes, OR directory index
	uint16_t DescSize; //in bytes
	uint8_t EntryType; //LCP_Type_t
	uint8_t AccessLvl; //LCP_AccessLvl_t
	uint8_t Mode; //LCP_Mode_t
	uint8_t reserved;
} LCPS_Entry_t;

typedef struct {
	const LCPS_Entry_t *Entries;
	const char *Name; //null terminated
	uint16_t Size;
	uint8_t ArrayIndex;
	uint8_t AccessLvl; //LCP_AccessLvl_t
} LCPS_Directory_t;

#define standardTypes(_val)  _Generic((_val),		\
		LCP_Uint32_t: 	LCP_Uint32,	\
		LCP_Int32_t: 	LCP_Int32,	\
		LCP_Uint64_t: 	LCP_Uint64,	\
		LCP_Int64_t: 	LCP_Int64,	\
		LCP_Float_t: 	LCP_Float,	\
		LCP_Double_t: 	LCP_Double, \
		LCP_Enum_t:  	LCP_Enum, 	\
		LCP_Bitfield32_t: LCP_Bitfield32)

#define pstd( _AccessLvl, _Mode, _Var, _Desc, _Name, _Text) \
	{ (void*)&_Var, (void*)&_Desc, _Name, _Text,  sizeof(_Var), sizeof(_Desc), standardTypes(_Desc), _AccessLvl, _Mode, 0}

#define pbool( _AccessLvl, _Mode, _Var, _Name, _Text) \
	{(void*)&_Var, 0, _Name, _Text,  sizeof(_Var), 0, LCP_Bool, _AccessLvl, _Mode, 0}

#define label( _AccessLvl, _Mode, _Name, _Text) \
	{ 0, 0, _Name, _Text,  0, 0, LCP_Label, _AccessLvl, _Mode, 0}

#define folder( _AccessLvl, _DirIndex, _Name, _Text) \
	{0, 0,  _Name, _Text, _DirIndex, 0, LCP_Folder, _AccessLvl, 0, 0}

#define ARRAYSIZ(array) (sizeof(array)/sizeof(array[0]))

#define directory(_directory, _arrayIndex, _AccessLvl, _Name  ) \
	{ _directory, _Name, ARRAYSIZ(_directory), _arrayIndex, _AccessLvl }

LC_Return_t LCP_ParameterServerInit(LC_NodeDescriptor_t *node);
