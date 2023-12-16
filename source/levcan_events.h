//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

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

typedef enum{
    //     The message box contain no symbols.
    None = 0,
    //     The message box contains a symbol consisting of white X in a circle with a red
    //     background.
    Error = 16,
    //     The message box contains a symbol consisting of a question mark in a circle.
    Question = 32,
    //     The message box contains a symbol consisting of an exclamation point in a triangle
    //     with a yellow background.
    Warning = 48,
    //     The message box contains a symbol consisting of a lowercase letter i in a circle.
    Information = 64
} LC_EventIcon_t;

typedef struct {
	char* Text;
	char* Caption;
	LC_EventButtons_t Buttons;
	LC_EventIcon_t Icon;
	uint8_t Sender;
} LC_Event_t;

LC_EXPORT LC_Return_t LC_EventInit(LC_NodeDescriptor_t *node);
LC_EXPORT LC_EventResult_t LC_EventSend(LC_NodeDescriptor_t* node, const char* text, const char* caption, LC_EventButtons_t buttons, uint8_t receiver);
LC_EXPORT LC_EventResult_t LC_EventSendF(LC_NodeDescriptor_t *node, LC_EventButtons_t buttons, uint8_t receiver, const char *caption, const char *text, ...);
LC_EXPORT LC_NodeShortName_t LC_FindEventServer(LC_NodeDescriptor_t* node, uint16_t *scnt) ;
LC_EXPORT void LC_EventReset(LC_NodeDescriptor_t* node, uint8_t receiver);
LC_EXPORT int LC_EventReceive(const void* data, int32_t dsize, uint8_t sender, LC_Event_t* event);
