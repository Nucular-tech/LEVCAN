//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

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
