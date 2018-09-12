/*
 * levcan_notifications.c
 *
 *  Created on: 12 sept 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "levcan_events.h"
#ifndef LC_EVENT_SIZE
#define LC_EVENT_SIZE 256
#endif

typedef struct {
	uint16_t TextSize;
	uint8_t CaptionSize;
	uint8_t Buttons;
	char Text[];
} eventSend_t;

#ifndef LEVCAN_MEM_STATIC
extern void *lcmalloc(uint32_t xWantedSize);
extern void lcfree(void *pv);
#endif

volatile uint8_t lc_eventButtonPressed = LC_EB_None;
char lc_event_buffer[LC_EVENT_SIZE];

LC_EventResult_t LC_EventSend(const char* text, const char* caption, LC_EventButtons_t buttons, uint8_t receiver) {
	if (text == 0 || receiver > LC_Broadcast_Address)
		return LC_ER_None;

	uint32_t texts = strlen(text) + 1;
	uint32_t caps = 0;
	if (caption)
		caps = strlen(caption) + 1;
	if (texts > INT16_MAX)
		texts = INT16_MAX;
	if (caps > UINT8_MAX)
		caps = UINT8_MAX;
	//find any event receiver
	if (receiver >= LC_Null_Address)
		receiver = LC_FindEventServer(0).NodeID;

	uint16_t buffersize = texts + caps + sizeof(eventSend_t);
	if (buffersize > LC_EVENT_SIZE)
		return LC_ER_None;

	eventSend_t* evnt = (eventSend_t*) lc_event_buffer;
	evnt->Buttons = buttons;
	evnt->CaptionSize = caps; //including zero
	evnt->TextSize = texts;
	memcpy(evnt->Text, text, texts);
	memcpy(&evnt->Text[texts], caption, caps);

	LC_ObjectRecord_t rec = { 0 };
	rec.Address = lc_event_buffer;
	rec.Size = buffersize;
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;
	rec.NodeID = receiver;
	LC_SendMessage(0, &rec, LC_SYS_Events);

	return lc_eventButtonPressed;
}

void LC_EventReset(uint8_t receiver) {
	if (receiver != LC_Null_Address) {
		if (receiver >= LC_Null_Address)
			receiver = LC_FindEventServer(0).NodeID;

		LC_ObjectRecord_t rec = { 0 };
		rec.Address = 0;
		rec.Size = 0;
		rec.Attributes.TCP = 1;
		rec.Attributes.Priority = LC_Priority_Low;
		rec.NodeID = receiver;
		LC_SendMessage(0, &rec, LC_SYS_Events);
	}
	lc_eventButtonPressed = LC_EB_None;
}
/// Returns event server short name
/// @param scnt Pointer to stored position for search, can be null
/// @return LC_NodeShortName_t file server
LC_NodeShortName_t LC_FindEventServer(uint16_t* scnt) {
	uint16_t counter = 0;
	LC_NodeShortName_t node;
	node.NodeID = LC_Broadcast_Address;
	if (scnt)
		counter = *scnt;
	while (node.NodeID != LC_Broadcast_Address) {
		node = LC_GetActiveNodes(&counter);
		if (node.Events)
			break;
	}
	if (scnt)
		*scnt = counter;
	return node;
}

#ifndef LEVCAN_MEM_STATIC
/// Parses input data and stores result in LC_Event_t* event
/// @param data Input data
/// @param dsize Input data size
/// @param sender Sender id
/// @param event Returned event
/// @return 0 - ok, 1 - error
int LC_EventReceive(const void* data, int32_t dsize, uint8_t sender, LC_Event_t* event) {
	const eventSend_t* evt = data;
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

	event->Buttons = evt->Buttons;
	event->Sender = sender;
	return ret;
}
#endif
