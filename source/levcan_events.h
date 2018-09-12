/*
 * levcan_notifications.h
 *
 *  Created on: 12 sept 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#include "levcan.h"
#include "levcan_config.h"

#pragma once

typedef enum {
	LC_EB_None, LC_EB_Ok, LC_EB_OkCancel, LC_EB_AbortRetryIgnore, LC_EB_YesNo, LC_EB_YesNoCancel, LC_EB_RetryCancel,
} LC_EventButtons_t;

typedef enum {
	//     Nothing is returned from the dialog box.
	LC_ER_None = 0,
	//     The dialog box return value is OK (usually sent from a button labeled OK).
	LC_ER_OK = 1,
	//     The dialog box return value is Cancel (usually sent from a button labeled Cancel).
	LC_ER_Cancel = 2,
	//     The dialog box return value is Abort (usually sent from a button labeled Abort).
	LC_ER_Abort = 3,
	//     The dialog box return value is Retry (usually sent from a button labeled Retry).
	LC_ER_Retry = 4,
	//     The dialog box return value is Ignore (usually sent from a button labeled Ignore).
	LC_ER_Ignore = 5,
	//     The dialog box return value is Yes (usually sent from a button labeled Yes).
	LC_ER_Yes = 6,
	//     The dialog box return value is No (usually sent from a button labeled No).
	LC_ER_No = 7
} LC_EventResult_t;

typedef struct {
	char* Text;
	char* Caption;
	LC_EventButtons_t Buttons;
	uint8_t Sender;
} LC_Event_t;

LC_EventResult_t LC_EventSend(const char* text, const char* caption, LC_EventButtons_t buttons, uint8_t receiver);
LC_NodeShortName_t LC_FindEventServer(uint16_t* scnt);
void LC_EventReset(uint8_t receiver);
int LC_EventReceive(const void* data, int32_t dsize, uint8_t sender, LC_Event_t* event);
