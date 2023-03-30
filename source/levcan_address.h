//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#pragma once

LC_Return_t LC_AddressInit(LC_NodeDescriptor_t* node);
void LC_AddressManager(LC_NodeDescriptor_t* node, uint32_t time);
LC_EXPORT void LC_ConfigureFilters(LC_NodeDescriptor_t* node);

typedef enum {
	LC_AdressNothing,
	LC_AdressNew,
	LC_AdressChanged,
	LC_AdressDeleted
}LC_AddressState_t;

typedef void(*LC_RemoteNodeCallback_t)(LC_NodeShortName_t shortname, uint16_t index, uint16_t state);

extern LC_RemoteNodeCallback_t lc_addressCallback;
