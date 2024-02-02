//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#pragma once

#include "levcan_filedef.h"

typedef struct {
#ifdef LEVCAN_PARAMETERS_CLIENT
	void *paramClientQueue;
	LC_ObjectRecord_t paramClientRecord;
#endif
#ifdef LEVCAN_PARAMETERS_SERVER
	uint8_t paramServerLastAccessNodeId;
#endif
#ifdef LEVCAN_FILECLIENT
	uint8_t fnode;
	uint32_t fpos;
	volatile fRead_t rxtoread;
#ifdef LEVCAN_USE_RTOS_QUEUE
	void *frxQueue;
#else
	volatile fOpAck_t rxack;
#endif
#endif
} lc_Extensions_t;

#ifndef LEVCAN_MAX_OWN_NODES
#define LEVCAN_MAX_OWN_NODES 1
#endif

#ifdef LEVCAN_MEM_STATIC
extern lc_Extensions_t lc_ExtensionsStatic[LEVCAN_MAX_OWN_NODES];
#endif

extern LC_Object_t* lc_registerSystemObjects(LC_NodeDescriptor_t *node, uint8_t count);

#define LEVCAN_COMM_TIMEOUT 100
//message should be 3x time more than communication
#define LEVCAN_MESSAGE_TIMEOUT 350
#define LEVCAN_NODE_TIMEOUT 1500
