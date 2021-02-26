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

#include <stdint.h>
#include "levcan.h"
#include "levcan_config.h"
#include "levcan_filedef.h"

typedef struct {
	uint32_t Pointer;
	uint16_t Errors;
} LC_FileAck_t;

LC_Return_t LC_FileClientInit(LC_NodeDescriptor_t *node);
LC_FileResult_t LC_FileOpen(LC_NodeDescriptor_t *node, char *name, LC_FileAccess_t mode, uint8_t server_node);
LC_FileResult_t LC_FileRead(LC_NodeDescriptor_t *node, char *buffer, uint32_t btr, uint32_t *br);
LC_FileResult_t LC_FileWrite(LC_NodeDescriptor_t *node, const char *buffer, uint32_t btw, uint32_t *bw);
LC_FileResult_t LC_FileClose(LC_NodeDescriptor_t *node, uint8_t server_node);
LC_FileResult_t LC_FileLseek(LC_NodeDescriptor_t *node, uint32_t position);
uint32_t LC_FileTell(LC_NodeDescriptor_t *node);
uint32_t LC_FileSize(LC_NodeDescriptor_t *node);
LC_FileResult_t LC_FileTruncate(LC_NodeDescriptor_t *node);

LC_FileResult_t LC_FilePrintf(LC_NodeDescriptor_t *node, const char *format, ...);
#ifdef LEVCAN_BUFFER_FILEPRINTF
LC_FileResult_t LC_FilePrintFlush(LC_NodeDescriptor_t *node);
#else
#define LC_FilePrintFlush(arg)
#endif

LC_NodeShortName_t LC_FindFileServer(LC_NodeDescriptor_t *node, uint16_t *scnt);
LC_NodeShortName_t LC_FileGetServer(LC_NodeDescriptor_t *node);
