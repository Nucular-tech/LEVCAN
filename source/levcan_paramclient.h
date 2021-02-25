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
 * levcan_paramclient.h
 *
 *  Created on: 2 февр. 2021 г.
 *      Author: VasiliSk
 */

#include <stdint.h>
#include "levcan.h"
#include "levcan_paramcommon.h"

#pragma once

typedef struct {
	const void *Variable; //address of variable or structure
	const void *Descriptor; //depends on LCP_Type_t
	const char *Name; //null terminated
	const char *TextData; //null terminated
	uint16_t EntryIndex;
	union {
		uint16_t VarSize; //in bytes
		uint16_t DirectoryIndex;
	};
	uint16_t DescSize; //in bytes
	uint16_t TextSize; //for checking
	uint8_t EntryType; //LCP_Type_t
	uint8_t Mode; //LCP_Mode_t
} LCPC_Entry_t;

typedef struct {
	const char *Name; //null terminated
	uint16_t Size;
	uint16_t DirectoryIndex;
} LCPC_Directory_t;

LC_EXPORT LC_Return_t LCP_RequestEntry(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, LCPC_Entry_t *out_entry);
LC_EXPORT LC_Return_t LCP_RequestDirectory(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, LCPC_Directory_t *out_directory);
LC_EXPORT LC_Return_t LCP_SetValue(LC_NodeDescriptor_t *mynode, uint8_t remote_node, uint16_t directory_index, uint16_t entry_index, intptr_t *value, uint16_t valueSize);
LC_EXPORT LC_Return_t LCP_RequestValue(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, intptr_t *outVariable, uint16_t varSize);
LC_EXPORT void LCP_CleanEntry(LCPC_Entry_t *entry);
LC_EXPORT void LCP_CleanDirectory(LCPC_Directory_t *dir);