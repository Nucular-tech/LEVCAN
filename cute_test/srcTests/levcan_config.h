#include <windows.h>
#include <string.h>

#pragma once

#ifdef TRACE
//Print debug messages using trace_printf
//#define LEVCAN_TRACE
//You can re-define trace_printf function
//#define trace_printf printf
#endif
//user functions for critical sections
#define lc_enable_irq()
#define lc_disable_irq()

#define LC_EXPORT
/*
 #define LC_HAL_Send
 */
#define LEVCAN_USE_INT64
#define LEVCAN_USE_DOUBLE
#define LEVCAN_FILECLIENT
//#define LEVCAN_PARAMETERS
//#define LEVCAN_PARAMETERS_PARSING
//#define LEVCAN_PARAMETERS_CLIENT
#define LEVCAN_PARAMETERS_SERVER
//Float-point support
#define LEVCAN_USE_FLOAT
#define LEVCAN_EVENTS
//Memory packing, compiler specific
#define LEVCAN_PACKED __attribute__((packed))
//Max device created nodes
#ifdef DEBUG
#define LEVCAN_MAX_OWN_NODES 1
#else
#define LEVCAN_MAX_OWN_NODES 1
#endif

//Network node table
#define LEVCAN_MAX_TABLE_NODES 16

//Above-driver buffer size. Used to store CAN messages before calling network manager
#define LEVCAN_TX_SIZE 20
#define LEVCAN_RX_SIZE 30
#define LEVCAN_NO_TX_QUEUE

//enable parameters and setup receive buffer size
#define LEVCAN_PARAM_QUEUE_SIZE 5

//Default size for malloc, maximum size for static mem, data size for file i/o
#define LEVCAN_OBJECT_DATASIZE 64
#define LEVCAN_FILE_DATASIZE 512
#define LEVCAN_BUFFER_FILEPRINTF

//File operations timeout for client side
#define LEVCAN_FILE_TIMEOUT 500

#define LC_EVENT_SIZE 368
//Enable this to use only static memory
//#define LEVCAN_MEM_STATIC

#ifdef LEVCAN_MEM_STATIC
//Maximum TX/RX objects. Excl. UDP data <=8byte, this receives in fast mode
#define LEVCAN_OBJECT_SIZE 20
#else
//external malloc functions
#define lcmalloc malloc
#define lcfree free
//extern delay function lcdelay(uint32_t time)
#define lcdelay sleep

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
