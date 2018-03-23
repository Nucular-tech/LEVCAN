/*
 * levcan_config.h
 *
 *  Created on: 22 мар. 2018 г.
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#pragma once

#define LEVCAN_MAX_OWN_NODES 2
#define LEVCAN_MAX_TABLE_NODES 10
#define LEVCAN_TX_SIZE 20
#define LEVCAN_RX_SIZE 20
//enable parameters and setup receive buffer size
#define LEVCAN_PARAM
#define LEVCAN_PARAM_QUEUE_SIZE 5
//#define LEVCAN_MEM_STATIC

#ifdef LEVCAN_MEM_STATIC
#define LEVCAN_OBJECT_SIZE_TX 10
#define LEVCAN_OBJECT_SIZE_RX 10
#else
//external malloc functions
#define lcmalloc pvPortMalloc
#define lcfree vPortFree
#endif
