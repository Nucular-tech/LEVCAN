#pragma once

#include "levcan_filedef.h"

typedef struct {
#ifdef LEVCAN_PARAMETERS_CLIENT
	void *paramClientQueue;
	LC_ObjectRecord_t paramClientRecord;
#endif
#ifdef LEVCAN_FILECLIENT
	uint8_t fnode;
	uint32_t fpos;
	volatile fRead_t rxtoread;
#ifdef LEVCAN_USE_RTOS_QUEUE
	void *frxQueue;
#else
	volatile fOpAck_t rxack = { 0 };
#endif
#endif
} lc_Extensions_t;

extern LC_Object_t* lc_registerSystemObjects(LC_NodeDescriptor_t *node, uint8_t count);

#define LEVCAN_MESSAGE_TIMEOUT 500
#define LEVCAN_NODE_TIMEOUT 1500
