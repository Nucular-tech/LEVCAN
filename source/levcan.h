/*
 * LEV-CAN: Light Electric Vehicle CAN protocol [LC]
 * levcan.h
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#include "stdint.h"
/* Application specific configuration options. */
#include "levcan_config.h"
#include "levcan_param.h"

#pragma once

typedef union {
	uint16_t Attributes;
	struct {
		unsigned Readable :1;	//Nodes can read from the variable
		unsigned Writable :1;	//Nodes may write to the variable
		unsigned TCP :1;		//Node should control packets sent (RTS/CTS) and create special tx/rx buffer
		unsigned Priority :2;
		unsigned Record :1;		//Object remapped to record array LC_ObjectRecord_t[Size],were LC_Object_t.Size will define array size
		unsigned Function :1;	//Functional call LC_FunctionCall_t, memory pointer will be cleared after call
		unsigned Pointer :1;	//received data will be saved as pointer to memory area, if there is already exists, it will be free. TX - data taken from pointerS
		unsigned Cleanup :1;	//after transmission pointer will call memfree
	};
} LC_ObjectAttributes_t;

typedef struct {
	uint16_t Index; //message id
	LC_ObjectAttributes_t Attributes;
	int32_t Size; //in bytes, can be negative (useful for strings), i.e. -1 = maximum length 1, -10 = maximum length 10. Request size 0 returns any first object
	void* Address; //pointer to variable or LC_FunctionCall_t or LC_ObjectRecord_t[]
} LC_Object_t;

typedef struct {
	int16_t Size;
	LC_ObjectAttributes_t Attributes;
	void* Address;
	uint8_t NodeID;
} LC_ObjectRecord_t;

typedef struct {
	uint8_t Source;
	uint8_t Target;
	uint16_t MsgID;
	struct {
		unsigned EoM :1;
		unsigned Request :1;
		unsigned Parity :1;
		unsigned RTS_CTS :1;
		unsigned Priority :2;
	};
} LC_Header;

typedef struct {
	union {
		uint32_t ToUint32[2];
		struct {
			unsigned Configurable :1;
			unsigned Variables :1;
			unsigned SWUpdates :1;
			unsigned Events :1;
			unsigned DeviceType :10;
			unsigned ManufacturerCode :10;
			unsigned SerialNumber :12;
		};
	};
	uint16_t NodeID;
} LC_NodeShortName;

typedef struct {
	char* NodeName;
	char* DeviceName;
	char* VendorName;
	uint16_t DeviceType;
	uint16_t VendorCode;
	struct {
		unsigned Configurable :1;
		unsigned SWUpdates :1;
		unsigned Notifications :1;
		unsigned Variables :1;
	};
	int16_t NodeID; //-1 will autodetect, 0-63 preffered address, 64-125 all
	uint32_t Serial;
	LC_Object_t* Objects;
	uint16_t ObjectsSize;
	LC_ParameterDirectory_t* Directories;
	uint16_t DirectoriesSize;
} LC_NodeInit_t;

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
	LC_SYS_Parameters,
	LC_SYS_Variables,
	LC_SYS_Events,
	LC_SYS_FWUpdate,
	LC_SYS_Trace,
	LC_SYS_DateTime,
	LC_SYS_End,
};

typedef struct {
	char* NodeName;
	char* DeviceName;
	char* VendorName;
	uint32_t Serial;
	LC_NodeShortName ShortName;
	uint32_t LastTXtime;
	uint16_t LastID;
	enum {
		LCNodeState_Disabled, LCNodeState_NetworkDiscovery, LCNodeState_WaitingClaim, LCNodeState_Online
	} State;
	LC_Object_t* Objects;
	uint16_t ObjectsSize;
	LC_Object_t SystemObjects[LC_SYS_End - LC_SYS_NodeName];
	LC_ParameterDirectory_t* Directories;
	uint16_t DirectoriesSize;
} LC_NodeDescription_t;

typedef struct {
	LC_NodeShortName ShortName;
	uint32_t LastRXtime;
} LC_NodeTable_t;

typedef LC_ObjectRecord_t (*LC_FunctionCall_t)(LC_NodeDescription_t* node, LC_Header header, void* data, int32_t size);

typedef enum {
	LC_Priority_Low, LC_Priority_Mid, LC_Priority_Control, LC_Priority_High,
} LC_Priority;

enum {
	LC_Preffered_Address = 0, LC_Normal_Address = 64, LC_Null_Address = 126, LC_Broadcast_Address = 127,
};

enum {
	LC_RX, LC_TX, LC_NodeFreeIDmin = 64, LC_NodeFreeIDmax = 125
};

uintptr_t* LC_CreateNode(LC_NodeInit_t node);
void LC_AddressClaimHandler(LC_NodeShortName node, uint16_t mode);
void LC_ReceiveHandler(uint32_t tick);
void LC_NetworkManager(uint32_t time);
void LC_SendMessage(void* sender, LC_ObjectRecord_t* object, uint16_t target, uint16_t index);
void LC_SendRequest(void* sender, uint16_t target, uint16_t index);
void LC_SendRequestSpec(void* sender, uint16_t target, uint16_t index, uint8_t size, uint8_t TCP);
void LC_SendDiscoveryRequest(uint16_t target);
void LC_TransmitHandler(void);
LC_NodeShortName LC_GetActiveNodes(void);
