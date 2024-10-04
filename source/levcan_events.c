//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include "levcan_events.h"
#include "levcan_internal.h"
#ifndef LC_EVENT_SIZE
#define LC_EVENT_SIZE 256
#endif

typedef struct {
	uint16_t TextSize;
	uint8_t CaptionSize;
	uint8_t Buttons_Icons;
	char Text[];
} eventSend_t;

volatile uint8_t lc_eventButtonPressed = LC_EB_None;
char lc_event_buffer[LC_EVENT_SIZE];

LC_Return_t lc_sendEventMsg(LC_NodeDescriptor_t *node, uint16_t buffersize, uint8_t receiver);

LC_Return_t LC_EventInit(LC_NodeDescriptor_t *node) {
	if (node == 0 || node->Extensions == 0)
		return LC_InitError;
	//event response
	LC_Object_t *initObject = lc_registerSystemObjects(node, 1);
	if (initObject == 0) {
		return LC_MallocFail;
	}
	initObject->Address = (void*) &lc_eventButtonPressed;
	initObject->Attributes.Writable = 1;
	initObject->MsgID = LC_SYS_Events;
	initObject->Size = sizeof(lc_eventButtonPressed);

	return LC_Ok;
}

LC_EventResult_t LC_EventSend(LC_NodeDescriptor_t *node, const char *text, const char *caption, LC_EventButtons_t buttons, uint8_t receiver) {
	if (text == 0 || receiver > LC_Broadcast_Address)
		return LC_ER_None;

	eventSend_t *evnt = (eventSend_t*) lc_event_buffer;
	size_t maxtxtsize = sizeof(lc_event_buffer) - sizeof(eventSend_t);

	uint32_t texts = strnlen(text, maxtxtsize) + 1;
	uint32_t caps = 0;
	if (caption)
		caps = strnlen(caption, maxtxtsize) + 1;

	uint16_t buffersize = texts + caps + sizeof(eventSend_t);
	if (buffersize > LC_EVENT_SIZE)
		return LC_ER_None;

	evnt->Buttons_Icons = buttons;
	evnt->CaptionSize = caps; //including zero
	evnt->TextSize = texts;
	memcpy(evnt->Text, text, texts);
	memcpy(&evnt->Text[texts], caption, caps);

	lc_sendEventMsg(node, buffersize, receiver);

	return lc_eventButtonPressed;
}

LC_EventResult_t LC_EventSendF(LC_NodeDescriptor_t *node, LC_EventButtons_t buttons, uint8_t receiver, const char *caption, const char *text, ...) {
	if (text == 0 || receiver > LC_Broadcast_Address)
		return LC_ER_None;

	eventSend_t *evnt = (eventSend_t*) lc_event_buffer;
	size_t maxtxtsize = sizeof(lc_event_buffer) - sizeof(eventSend_t);

	va_list ap;
	va_start(ap, text);
	evnt->Text[0] = 0;
	int32_t texts = vsnprintf(evnt->Text, maxtxtsize - 1, text, ap) + 1;
	va_end(ap);

	if (texts < 0) {
		texts = 0;
	}
	uint32_t caps = 0;
	if (caption)
		caps = strnlen(caption, maxtxtsize) + 1;

	uint16_t buffersize = texts + caps + sizeof(eventSend_t);
	if (buffersize > LC_EVENT_SIZE)
		return LC_ER_None;

	evnt->Buttons_Icons = buttons;
	evnt->CaptionSize = caps; //including zero
	evnt->TextSize = texts;
	//copy only caption text
	memcpy(&evnt->Text[texts], caption, caps);

	lc_sendEventMsg(node, buffersize, receiver);
	return lc_eventButtonPressed;
}

LC_Return_t lc_sendEventMsg(LC_NodeDescriptor_t *node, uint16_t buffersize, uint8_t receiver) {
	//find any event receiver
	if (receiver >= LC_Null_Address)
		receiver = LC_FindEventServer(node, 0).NodeID;

	LC_ObjectRecord_t rec = { 0 };
	rec.Address = lc_event_buffer;
	rec.Size = buffersize;
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;
	rec.NodeID = receiver;

	return LC_SendMessage(node, &rec, LC_SYS_Events);
}

void LC_EventReset(LC_NodeDescriptor_t *node, uint8_t receiver) {
	if (receiver != LC_Null_Address) {
		if (receiver >= LC_Null_Address)
			receiver = LC_FindEventServer(node, 0).NodeID;

		LC_ObjectRecord_t rec = { 0 };
		rec.Address = 0;
		rec.Size = 0;
		rec.Attributes.TCP = 1;
		rec.Attributes.Priority = LC_Priority_Low;
		rec.NodeID = receiver;
		LC_SendMessage(node, &rec, LC_SYS_Events);
	}
	lc_eventButtonPressed = LC_EB_None;
}
/// Returns event server short name
/// @param scnt Pointer to stored position for search, can be null
/// @return LC_NodeShortName_t file server
LC_NodeShortName_t LC_FindEventServer(LC_NodeDescriptor_t *node, uint16_t *scnt) {
	uint16_t counter = 0;
	LC_NodeShortName_t nodeSN;
#ifdef LEVCAN_PARAMETERS_SERVER
	//send message to source of last parameters request (usually it have GUI and events manager too)
	if (((lc_Extensions_t*) node->Extensions)->paramServerLastAccessNodeId < LC_Null_Address)
		return LC_GetNode(node, ((lc_Extensions_t*) node->Extensions)->paramServerLastAccessNodeId);
#endif
	if (scnt)
		counter = *scnt;
	do {
		nodeSN = LC_GetActiveNodes(node, &counter);
		if (nodeSN.Events)
			break;
	} while (nodeSN.NodeID != LC_Broadcast_Address);

	if (scnt)
		*scnt = counter;
	return nodeSN;
}

#ifndef LEVCAN_MEM_STATIC
/// Parses input data and stores result in LC_Event_t* event
/// @param data Input data
/// @param dsize Input data size
/// @param sender Sender id
/// @param event Returned event
/// @return 0 - ok, 1 - error
int LC_EventReceive(const void *data, int32_t dsize, uint8_t sender, LC_Event_t *event) {
	const eventSend_t *evt = data;
	int ret = 0;
	//check valid
	if (event == 0 || data == 0 || (int32_t) (evt->TextSize + evt->CaptionSize + sizeof(eventSend_t)) != dsize)
		return 1;
	event->Text = (char*) lcmalloc(evt->TextSize);
	if (event->Text)
		memcpy(event->Text, evt->Text, evt->TextSize);
	else
		ret = 1;

	if (evt->CaptionSize > 0) {
		event->Caption = (char*) lcmalloc(evt->CaptionSize);
		if (event->Caption)
			memcpy(event->Caption, &evt->Text[evt->TextSize], evt->CaptionSize);
		else {
			if (event->Text)
				lcfree(event->Text);
			ret = 1;
		}
	} else
		event->Caption = 0;
	event->Buttons = evt->Buttons_Icons & 0xF;
	event->Icon = evt->Buttons_Icons & 0xF0;
	event->Sender = sender;
	return ret;
}
#endif
