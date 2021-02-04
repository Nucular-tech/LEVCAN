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
 * levcan_paraminternal.h
 *
 *  Created on: 2 февр. 2021 г.
 *      Author: VasiliSk
 */
#pragma once

typedef struct {
	uint16_t Directory;
	uint16_t Entry;
	uint8_t Command;
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
} lcp_reqCommand_t;

typedef struct {
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
	uint16_t EntryIndex; //in bytes
	uint16_t VarSize; //in bytes
	uint16_t DescSize; //in bytes
	uint16_t TextSize; //in bytes, no 0's, total name+text
} lc_entry_data_t;

#ifndef LEVCAN_PARAM_MAX_NAMESIZE
#define LEVCAN_PARAM_MAX_NAMESIZE 128
#endif
#ifndef LEVCAN_PARAM_MAX_TEXTSIZE
#define LEVCAN_PARAM_MAX_TEXTSIZE 512
#endif
