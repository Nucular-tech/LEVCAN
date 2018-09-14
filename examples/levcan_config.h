/*
 * levcan_config.h
 *
 *  Created on: 22 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#pragma once

//user functions for critical sections
static inline void lc_enable_irq(void)
{
  asm volatile ("cpsie i" : : : "memory");
}
static inline void lc_disable_irq(void)
{
	asm volatile ("cpsid i" : : : "memory");
}
//FEATURES
#define LEVCAN_FILECLIENT
//#define LEVCAN_FILESERVER
#define LEVCAN_PARAMETERS
#define LEVCAN_EVENTS

//Print debug messages using trace_printf
//#define LEVCAN_TRACE
//You can re-define trace_printf function
//#define trace_printf printf
//Float-point support
//#define LEVCAN_USE_FLOAT
//Memory packing, compiler specific
#define LEVCAN_PACKED __attribute__((packed))
//Max device created nodes
#define LEVCAN_MAX_OWN_NODES 2
//Network node table
#define LEVCAN_MAX_TABLE_NODES 10
//Above-driver buffer size. Used to store CAN messages before calling network manager
//Make shure that you cannot receive more messages before LC_NetworkManager update
#define LEVCAN_TX_SIZE 20
#define LEVCAN_RX_SIZE 30
//enable parameters and setup receive buffer size
#define LEVCAN_PARAM_QUEUE_SIZE 5
//Default size for malloc, maximum size for static mem. Minimum - 8byte
#define LEVCAN_OBJECT_DATASIZE 48
//Enable this to use only static memory
#define LEVCAN_MEM_STATIC

#ifdef LEVCAN_MEM_STATIC
//Maximum TX/RX objects. Excl. UDP data <=8byte, this receives in fast mode
#define LEVCAN_OBJECT_SIZE 10
#else
//external malloc functions
#define lcmalloc pvPortMalloc
#define lcfree vPortFree
#endif
