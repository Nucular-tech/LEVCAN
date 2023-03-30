//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#pragma once

typedef struct {
	uint16_t Command;
	uint16_t Directory;
	uint16_t Entry;
} lc_request_entry_t;

typedef enum {
	lcp_reqNone = 0,
	lcp_reqData = 1 << 0,
	lcp_reqName = 1 << 1,
	lcp_reqText = 1 << 2,
	lcp_reqDescriptor = 1 << 3,
	lcp_reqVariable = 1 << 4,
	lcp_reqFullEntry = 0x1F,
	lcp_reqDirectoryInfo = 1 << 5,
	lcp_reqValueSet = 1 << 6,
} lcp_reqCommand_t;

typedef struct {
	uint16_t Command;
	uint16_t Directory;
} lc_request_directory_t;

typedef struct {
	uint8_t ErrorCode;
} lc_request_error_t;

typedef struct {
	uint16_t EntrySize;
	uint16_t NameSize;
	uint16_t DirectoryIndex;
} lc_directory_data_t;

typedef struct {
	uint8_t EntryType; //LCP_Type_t
	uint8_t Mode; //LCP_Mode_t
	uint16_t EntryIndex;
	uint16_t VarSize; //in bytes
	uint16_t DescSize; //in bytes
	uint16_t TextSize; //in bytes, no 0's, total name+text
} lc_entry_data_t;

typedef struct {
	uint16_t Command;
	uint16_t DirectoryIndex;
	uint16_t EntryIndex;
	uint8_t Data[];
} lc_value_set_t;

#ifndef LEVCAN_PARAM_MAX_NAMESIZE
#define LEVCAN_PARAM_MAX_NAMESIZE 128
#endif
#ifndef LEVCAN_PARAM_MAX_TEXTSIZE
#define LEVCAN_PARAM_MAX_TEXTSIZE 512
#endif
