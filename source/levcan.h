//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#include "stdint.h"
#include "stddef.h"
 /* Application specific configuration options. */

#include "levcan_config.h"

#pragma once

typedef union {
	uint16_t Attributes;
	struct {
		uint16_t Readable : 1;	//Nodes can read from the variable
		uint16_t Writable : 1;	//Nodes may write to the variable
		uint16_t TCP : 1;		//Node should control packets sent (RTS/CTS) and create special tx/rx buffer
		uint16_t Priority : 2;
		uint16_t Record : 1;		//Object remapped to record array LC_ObjectRecord_t[Size],were LC_Object_t.Size will define array size
		uint16_t Function : 1;	//Functional call LC_FunctionCall_t, memory pointer will be cleared after call
		//received data will be saved as pointer to memory area, if there is already exists, it will be free
		uint16_t Pointer : 1;	//TX - data taken from pointer (where Address is pointer to pointer)
		uint16_t Cleanup : 1;	//after transmission pointer will call memfree
#ifdef LEVCAN_USE_RTOS_QUEUE
		uint16_t Queue : 1;		//will place data on specified queue (address should have queue pointer)
#endif
	} ;
} LC_ObjectAttributes_t;

typedef struct {
	uint16_t MsgID; //message id
	LC_ObjectAttributes_t Attributes;
	int32_t Size; //in bytes, can be negative (useful for strings), i.e. -1 = maximum length 1, -10 = maximum length 10. Request size 0 returns any first object
	void *Address; //pointer to variable or LC_FunctionCall_t or LC_ObjectRecord_t[]. if LC_ObjectAttributes_t.Pointer=1, this is pointer to pointer
} LC_Object_t;

typedef struct {
	uint8_t NodeID; //filters specified sender ID
	LC_ObjectAttributes_t Attributes;
	int32_t Size;
	void *Address; //pointer to memory data. if LC_ObjectAttributes_t.Pointer=1, this is pointer to pointer
} LC_ObjectRecord_t;

typedef struct {
	uint8_t Source; //7 bits
	uint8_t Target; //7 bits
	uint16_t MsgID; //10 bits
	union {
		uint16_t ControlBits;
		struct {
			uint16_t EoM : 1;
			uint16_t Parity : 1;
			uint16_t RTS_CTS : 1;
			uint16_t Priority : 2;
			uint16_t Request : 1;
		};
	};
} LC_Header_t;

typedef union {
	uint32_t ToUint32;
	struct {
		//index 29bit:
		uint32_t Source : 7; //From
		uint32_t Target : 7; //To
		uint32_t MsgID : 10; //Up to 1024 messages
		uint32_t EoM : 1; //End Of Message
		uint32_t Parity : 1; //Message parity
		uint32_t RTS_CTS : 1; //Ready to send / Clear to send
		uint32_t Priority : 2; //CAN bus priority bits
		//RTR bit:
		uint32_t Request : 1; //Request message
	} ;
} LC_HeaderPacked_t;

typedef struct {
	LC_Header_t Header;
	int32_t Size;
	intptr_t *Data;
} LC_ObjectData_t;

typedef struct {
	union {
		uint32_t ToUint32[2];
		struct {
			uint32_t Configurable : 1; 	//0 Have configurable parameters
			uint32_t Variables : 1; 	//1
			uint32_t SWUpdates : 1; 	//2 Have software update feature
			uint32_t Events : 1; 		//3 Have LEVCAN events
			uint32_t FileServer : 1;	//4 Have file server running
			uint32_t CodePage : 16;		//5-20 https://docs.microsoft.com/en-us/dotnet/api/system.text.encoding?view=netcore-3.1
			uint32_t reserved1 : (32 - 5 - 16 - 1);		//21-30
			uint32_t DynamicID : 1;		//31 1-Yes, 0-No, MSB bit, defines priority on CAN bus
			//32b align
			uint32_t DeviceType : 10;	//32-41 LC_Device_t
			uint32_t ManufacturerCode : 10;	//42-51 Custom field
			uint32_t SerialNumber : 12;	//52-63 Short SN
		};
	};
	uint16_t NodeID;
} LC_NodeShortName_t;

enum {
	LC_SYS_AddressClaimed = 0x380,
	LC_SYS_ComandedAddress,
	LC_SYS_NodeName = 0x388,
	LC_SYS_DeviceName,
	LC_SYS_VendorName,
	LC_SYS_VendorCode,
	LC_SYS_HWVersion,
	LC_SYS_SWVersion,
	LC_SYS_SerialNumber,
	LC_SYS_Parameters, //old parameters
	LC_SYS_Variables,
	LC_SYS_Events,
	LC_SYS_Trace,
	LC_SYS_DateTime,
	LC_SYS_SWUpdate,
	LC_SYS_Shutdown,
	LC_SYS_FileServer,
	LC_SYS_FileClient,
	LC_SYS_SaveData,
	LC_SYS_ParametersRequest,
	LC_SYS_ParametersData,
	LC_SYS_ParametersDescriptor,
	LC_SYS_ParametersName,
	LC_SYS_ParametersText,
	LC_SYS_ParametersValue,
	LC_SYS_End,
};

#ifndef LEVCAN_SYS_OBJ_SIZ
#define LEVCAN_SYS_OBJ_SIZ (LC_SYS_End - LC_SYS_NodeName)
#endif

typedef struct {
	uint8_t Hour; //24H
	uint8_t Minute;
	uint8_t Second;
	uint8_t WeekDay; //0=Monday ... 6=Sunday
	uint8_t Day;
	uint8_t Month;
	uint16_t Year;
} LC_SYS_DateTime_t;

enum {
	LCNodeState_Disabled, LCNodeState_NetworkDiscovery, LCNodeState_WaitingClaim, LCNodeState_Online
};


typedef enum {
	LC_Priority_Low, LC_Priority_Mid, LC_Priority_Control, LC_Priority_High,
} LC_Priority_t;

typedef enum {
	LC_Ok, LC_DataError, LC_ObjectError, LC_BufferFull, LC_BufferEmpty, LC_NodeOffline, LC_MallocFail, LC_Collision, LC_Timeout, LC_OutOfRange, LC_AccessError, LC_InitError
} LC_Return_t;

typedef enum {
	LC_SD_Shutdown, LC_SD_Reboot, LC_SD_Sleep
} LC_Shutdown_t;

enum {
	LC_Preffered_Address = 0, LC_Normal_Address = 64, LC_Null_Address = 126, LC_Broadcast_Address = 127, LC_Invalid_Address = 0xFF
};

enum {
	LC_RX, LC_TX, LC_NodeFreeIDmin = 64, LC_NodeFreeIDmax = 125
};


typedef LC_Return_t(*HAL_Send_t)(LC_HeaderPacked_t header, uint32_t* data, uint8_t length);
typedef LC_Return_t(*HAL_Filter_t)(LC_HeaderPacked_t* reg, LC_HeaderPacked_t* mask, uint16_t count);
typedef LC_Return_t(*HAL_TxHalfFull_t)(void);

typedef struct {
	HAL_Send_t Send;
	HAL_Filter_t Filter;
	HAL_TxHalfFull_t TxHalfFull;
} LC_DriverCalls_t;

typedef struct {
	LC_NodeShortName_t ShortName;
	uint32_t LastRXtime;
} LC_NodeTableEntry_t;

typedef struct {
	LC_NodeTableEntry_t* Table;
	uint16_t TableSize;
	//uint16_t FreeSlots;
} LC_NodeTable_t;

typedef struct {
	LC_HeaderPacked_t header;
	uint32_t data[2];
	uint8_t length;
} lc_msgBuffered;

typedef struct {
	union {
		char *Pointer;
#ifdef LEVCAN_MEM_STATIC
		char Data[LEVCAN_OBJECT_DATASIZE];
#else
		char Data[sizeof(char*)];
#endif
	};
	intptr_t *Next;
	intptr_t *Previous;
	int32_t Length;
	int32_t Position;    //get parity - divide by 8 and &1
	LC_HeaderPacked_t Header;
	uint16_t Time_since_comm;
	uint8_t Attempt;
	union {
		struct {
			uint8_t TCP :1;
			uint8_t TXcleanup :1;
			uint8_t ReadyToRX :1;
			uint8_t ToDelete :1;
		} Flags;
		uint8_t FlagsTotal;
	};
} lc_objBuffered;

typedef struct {
	const void* Driver;
	const char *NodeName;
	const char *DeviceName;
	const char *VendorName;
	LC_Object_t *Objects;
	void *Directories;
	LC_NodeShortName_t ShortName;
	uint32_t Serial[4];
	uint32_t LastTXtime;
	uint16_t ObjectsSize;
	uint16_t SystemSize;
	uint16_t DirectoriesSize;
	uint16_t LastID;
	uint8_t State;
	uint8_t AccessLevel;
	//not for public use
	struct {
#ifdef LEVCAN_MEM_STATIC
		lc_objBuffered objectBuffer[LEVCAN_OBJECT_SIZE];
		int16_t objectBuffer_freeID;
#endif
#ifdef LEVCAN_USE_RTOS_QUEUE
		void *rxQueue;
#else
		lc_msgBuffered rxFIFO[LEVCAN_RX_SIZE];
		volatile uint16_t rxFIFO_in, rxFIFO_out;
#endif
		volatile void *objTXbuf_start;
		volatile void *objTXbuf_end;
		volatile void *objRXbuf_start;
		volatile void *objRXbuf_end;
	} TxRxObjects;
	LC_NodeTable_t* NodeTable;
	void* Extensions;
	LC_Object_t SystemObjects[LEVCAN_SYS_OBJ_SIZ];
#ifdef LEVCAN_MEM_STATIC
	LC_NodeTable_t NodeTableStatic;
	LC_NodeTableEntry_t NodeTableEntryStatic[LEVCAN_MAX_TABLE_NODES];
#endif
} LC_NodeDescriptor_t;

typedef void(*LC_FunctionCall_t)(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);

LC_EXPORT LC_Return_t LC_InitNodeDescriptor(LC_NodeDescriptor_t *node);
LC_EXPORT LC_Return_t LC_CreateNode(LC_NodeDescriptor_t *node);
//Handlers should be called from CAN HAL ISR
LC_EXPORT void LC_ReceiveHandler(LC_NodeDescriptor_t* node, LC_HeaderPacked_t header, uint32_t *data, uint8_t length);

//Managers should be called from separate tasks, if LEVCAN_USE_RTOS_QUEUE set
LC_EXPORT void LC_NetworkManager(LC_NodeDescriptor_t* node, uint32_t time); //low priority
LC_EXPORT void LC_ReceiveManager(LC_NodeDescriptor_t* node); //high priority

LC_EXPORT LC_Return_t LC_SendMessage(LC_NodeDescriptor_t* node, LC_ObjectRecord_t *object, uint16_t index);
LC_EXPORT LC_Return_t LC_SendRequest(LC_NodeDescriptor_t* node, uint16_t target, uint16_t index);
LC_EXPORT LC_Return_t LC_SendRequestSpec(LC_NodeDescriptor_t* node, uint16_t target, uint16_t index, uint8_t size, uint8_t TCP);

LC_EXPORT LC_NodeShortName_t LC_GetActiveNodes(LC_NodeDescriptor_t* node, uint16_t *last_pos);
LC_EXPORT LC_NodeShortName_t LC_GetNode(LC_NodeDescriptor_t* node, uint16_t nodeID);
LC_EXPORT LC_NodeShortName_t LC_GetMyNodeName(LC_NodeDescriptor_t* node);

LC_EXPORT LC_HeaderPacked_t LC_HeaderPack(LC_Header_t header);
LC_EXPORT LC_Header_t LC_HeaderUnpack(LC_HeaderPacked_t header);
LC_EXPORT size_t LC_SizeOfDescriptor();
