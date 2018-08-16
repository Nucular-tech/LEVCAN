/*
 * levcan_filedef.h
 *
 *  Created on: 16 Aug 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#pragma once

enum {
	fOpNoOp, fOpOpen, fOpRead, fOpWrite, fOpClose, fOpAck, fOpLseek, fOpData, fOpOpenDir, fOpReadDir
};

typedef struct {
	uint16_t Operation;
	LC_FileAccess_t Mode;
	char Name[];
} fOpOpen_t;

typedef struct {
	uint16_t Operation;
	uint16_t ToBeRead;
	int32_t Position;
} fOpRead_t;

typedef struct {
	uint16_t Operation;
	int32_t Position;
} fOpLseek_t;

typedef struct {
	uint16_t Operation;
} fOpClose_t;

typedef struct {
	uint16_t Operation;
	uint16_t Error;
	uint32_t Position;
} fOpAck_t;

typedef struct {
	uint16_t Operation;
	uint16_t Error;
	int32_t Position;
	uint16_t TotalRead;
	char Data[];
} fOpData_t;

typedef struct {
	char* Buffer;
	int32_t Position;
	uint16_t ReadBytes;
	uint16_t Error;
} fRead_t;

