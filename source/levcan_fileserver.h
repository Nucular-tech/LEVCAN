//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdint.h>

LC_EXPORT LC_Return_t LC_FileServerInit(LC_NodeDescriptor_t* node);
LC_EXPORT LC_Return_t LC_FileServer(LC_NodeDescriptor_t* node, uint32_t tick);
