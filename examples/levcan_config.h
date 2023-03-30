//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#define FREERTOS
#ifdef FREERTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#endif

#pragma once

//user functions for critical sections
static inline void lc_enable_irq(void) {
	asm volatile ("cpsie i" : : : "memory");
}
static inline void lc_disable_irq(void) {
	asm volatile ("cpsid i" : : : "memory");
}
//#define LC_EXPORT __declspec(dllexport)
#define LC_EXPORT
//Memory packing, compiler specific
#define LEVCAN_PACKED __attribute__((packed))
//platform specific, define how many bytes in uint8_t
#define LEVCAN_MIN_BYTE_SIZE 1

#ifdef TRACE
//Print debug messages using trace_printf
//#define LEVCAN_TRACE
//You can re-define trace_printf function
//#define trace_printf printf
#endif

//define to use simple file io operations
#define LEVCAN_FILECLIENT
//#define LEVCAN_FILESERVER
//define to use buffered printf
#define LEVCAN_BUFFER_FILEPRINTF
//File operations timeout for client side (ms)
#define LEVCAN_FILE_TIMEOUT 500

//define to be able to configure your device over levcan
#define LEVCAN_PARAMETERS
//define to be able print and parse your parameters
#define LEVCAN_PARAMETERS_PARSING
//Float-point support for parameters
#define LEVCAN_USE_FLOAT
//parameters receive buffer size
#define LEVCAN_PARAM_QUEUE_SIZE 5

//defiene to use small messages pop-ups on display
#define LEVCAN_EVENTS

//Max own created nodes
#define LEVCAN_MAX_OWN_NODES 1

//max saved nodes short names (used for search)
#define LEVCAN_MAX_TABLE_NODES 10

//Above-driver buffer size. Used to store CAN messages before calling network manager
#define LEVCAN_TX_SIZE 20
#define LEVCAN_RX_SIZE 30

//Default size for malloc, maximum size for static mem, data size for file i/o
#define LEVCAN_OBJECT_DATASIZE 64
#define LEVCAN_FILE_DATASIZE 512

//Enable this to use only static memory
//#define LEVCAN_MEM_STATIC

#ifdef LEVCAN_MEM_STATIC
//Maximum TX/RX objects at one time. Excl. UDP data <=8byte, this receives in fast mode
#define LEVCAN_OBJECT_SIZE 20
#else
//external malloc functions
#define lcmalloc pvPortMalloc
#define lcfree vPortFree
#define lcdelay vTaskDelay

//enable to use RTOS managed queues
//#define LEVCAN_USE_RTOS_QUEUE

#ifdef LEVCAN_USE_RTOS_QUEUE
//setup your rtos functions here
#define LC_QueueCreate(length, itemSize) xQueueCreate(length, itemSize)
#define LC_QueueDelete(queue) vQueueDelete(queue)
#define LC_QueueReset(queue) xQueueReset(queue)
#define LC_QueueSendToBack(queue, buffer, ttwait) xQueueSendToBack(queue, buffer, ttwait)
#define LC_QueueSendToFront(queue, buffer, ttwait) xQueueSendToFront(queue, buffer, ttwait)
#define LC_QueueSendToBackISR(queue, item, yieldNeeded) xQueueSendToBackFromISR(queue, item, yieldNeeded)
#define LC_QueueSendToFrontISR xQueueSendToFrontFromISR
#define LC_QueueReceive(queue, buffer, ttwait) xQueueReceive(queue, buffer, ttwait)
#define LC_QueuePeek(queue, buffer, ttwait) xQueuePeek(queue, buffer, ttwait)
#define LC_QueueReceiveISR xQueueReceiveFromISR
#define LC_QueueStored(queue) uxQueueMessagesWaiting(queue)

#define LC_SemaphoreCreate xSemaphoreCreateBinary
#define LC_SemaphoreDelete(sem) vSemaphoreDelete(sem)
#define LC_SemaphoreGive(sem) xSemaphoreGive(sem)
#define LC_SemaphoreGiveISR(sem, yieldNeeded) xSemaphoreGiveFromISR(sem, yieldNeeded)
#define LC_SemaphoreTake(sem, ttwait) xSemaphoreTake(sem, ttwait)

#define LC_RTOSYieldISR(yield) portYIELD_FROM_ISR(yield)
#define YieldNeeded_t BaseType_t
#else

#endif
#endif
