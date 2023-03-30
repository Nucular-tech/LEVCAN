//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

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

LC_EXPORT LC_Return_t LCP_ParameterClientInit(LC_NodeDescriptor_t *node);
LC_EXPORT LC_Return_t LCP_RequestEntry(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, LCPC_Entry_t *out_entry);
LC_EXPORT LC_Return_t LCP_RequestDirectory(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, LCPC_Directory_t *out_directory);
LC_EXPORT LC_Return_t LCP_SetValue(LC_NodeDescriptor_t *mynode, uint8_t remote_node, uint16_t directory_index, uint16_t entry_index, intptr_t *value, uint16_t valueSize);
LC_EXPORT LC_Return_t LCP_RequestValue(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, intptr_t *outVariable, uint16_t varSize);
LC_EXPORT void LCP_CleanEntry(LCPC_Entry_t *entry);
LC_EXPORT void LCP_CleanDirectory(LCPC_Directory_t *dir);
