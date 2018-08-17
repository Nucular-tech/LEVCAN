/*
 * LEV-CAN: Light Electric Vehicle CAN protocol [LC]
 * levcan.c
 *
 *  Created on: 17 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#include "levcan.h"
#include "levcan_param.h"

#include "string.h"
#include "stdlib.h"
#include "can_hal.h"

#if	defined(lcmalloc) && defined(lcfree)
extern void *lcmalloc(uint32_t size);
extern void lcfree(void *pointer);
#else
#ifndef LEVCAN_STATIC_MEM
#define LEVCAN_STATIC_MEM
//undefine
#define lcmalloc(...)
#define lcfree(...)
#endif
#endif

#if LEVCAN_OBJECT_DATASIZE < 8
#error "LEVCAN_OBJECT_DATASIZE should be more than one 8 byte for static memory"
#endif
typedef union {
	uint32_t ToUint32;
	struct {
		//can specific:
		unsigned reserved1 :1;
		unsigned Request :1;
		unsigned IDE :1; //29b=1
		//index 29bit:
		unsigned Source :7;
		unsigned Target :7;
		unsigned MsgID :10;
		unsigned EoM :1;
		unsigned Parity :1;
		unsigned RTS_CTS :1;
		unsigned Priority :2;
	}LEVCAN_PACKED;
} headerPacked_t;

typedef struct {
	headerPacked_t header;
	uint32_t data[2];
	uint8_t length;
} msgBuffered;

typedef struct {
	union {
		char* Pointer;
#ifdef LEVCAN_STATIC_MEM
		char Data[LEVCAN_OBJECT_DATASIZE];
#else
		char Data[sizeof(char*)];
#endif
	};
	int32_t Length;
	int32_t Position; //get parity - divide by 8 and &1
	headerPacked_t Header;
	uint16_t Time_since_comm;
	uint8_t Attempt;
	struct {
		unsigned TCP :1;
		unsigned TXcleanup :1;
	} Flags LEVCAN_PACKED;
	intptr_t* Next;
	intptr_t* Previous;
} objBuffered;

enum {
	Read, Write
};

#ifdef LEVCAN_TRACE
extern int trace_printf(const char* format, ...);
#endif

//#### PRIVATE VARIABLES ####
#ifdef LEVCAN_STATIC_MEM
objBuffered objectBuffer[LEVCAN_OBJECT_SIZE];
int16_t objectBuffer_freeID;
#endif
LC_NodeDescription_t own_nodes[LEVCAN_MAX_OWN_NODES];
LC_NodeTable_t node_table[LEVCAN_MAX_TABLE_NODES];
volatile objBuffered* objTXbuf_start = 0;
volatile objBuffered* objTXbuf_end = 0;
volatile objBuffered* objRXbuf_start = 0;
volatile objBuffered* objRXbuf_end = 0;
msgBuffered txFIFO[LEVCAN_TX_SIZE];
volatile uint16_t txFIFO_in, txFIFO_out;
msgBuffered rxFIFO[LEVCAN_RX_SIZE];
volatile uint16_t rxFIFO_in, rxFIFO_out;
volatile uint16_t own_node_count;

//#### PRIVATE FUNCTIONS ####
void initialize(void);
void configureFilters(void);
void addAddressFilter(uint16_t address);
void proceedAddressClaim(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size);
void claimFreeID(LC_NodeDescription_t* node);

int16_t compareNode(LC_NodeShortName_t a, LC_NodeShortName_t b);
LC_NodeDescription_t* findNode(uint16_t nodeID);
LC_ObjectRecord_t findObjectRecord(uint16_t index, int32_t size, LC_NodeDescription_t* node, uint8_t read_write, uint8_t nodeID);

headerPacked_t headerPack(LC_Header_t header);
LC_Header_t headerUnpack(headerPacked_t header);

LC_Return_t sendDataToQueue(headerPacked_t hdr, uint32_t data[], uint8_t length);
uint16_t objectRXproceed(objBuffered* object, msgBuffered* msg);
uint16_t objectTXproceed(objBuffered* object, headerPacked_t* request);
LC_Return_t objectRXfinish(headerPacked_t header, char* data, int32_t size, uint8_t memfree);
void deleteObject(objBuffered* obj, objBuffered** start, objBuffered** end);
#ifdef LEVCAN_STATIC_MEM
objBuffered* getFreeObject(void);
void releaseObject(objBuffered* obj);
#endif

int32_t getTXqueueSize(void);
uint16_t searchIndexCollision(uint16_t nodeID, LC_NodeDescription_t* ownNode);
objBuffered* findObject(objBuffered* array, uint16_t msgID, uint8_t target, uint8_t source);

void lc_default_handler(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size);
//#### EXTERNAL MODULES #### todo: other compiler support
extern void __attribute__ ((weak, alias ("lc_default_handler")))
proceedParam(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size);

extern void __attribute__ ((weak, alias ("lc_default_handler")))
proceedFileServer(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size);

extern void __attribute__ ((weak, alias ("lc_default_handler")))
proceedFileClient(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size);
//#### FUNCTIONS
uintptr_t* LC_CreateNode(LC_NodeInit_t node) {
	initialize();

	if (node.NodeID < 0 || node.NodeID > 125) {
		uint16_t id = node.Serial % 64;
		id |= 64;
		if (id > 125) {
			id &= ~3;
		}
		node.NodeID = id;
	}
	if (node.NodeName != 0 && strnlen(node.NodeName, 128) == 128)
		node.NodeName = 0; //too long name
	if (node.DeviceName != 0 && strnlen(node.DeviceName, 128) == 128)
		node.DeviceName = 0;
	if (node.VendorName != 0 && strnlen(node.VendorName, 128) == 128)
		node.VendorName = 0;

	LC_NodeDescription_t descr = { 0 };
	LC_NodeShortName_t sname = { 0 };
	sname.Configurable = node.Configurable;
	sname.DeviceType = node.DeviceType;
	sname.ManufacturerCode = node.ManufacturerCode;
	sname.Events = node.Events;
	sname.FileServer = node.FileServer;
	sname.SWUpdates = node.SWUpdates;
	sname.Variables = node.Variables;
	sname.SerialNumber = node.Serial;
	sname.NodeID = node.NodeID;

	descr.ShortName = sname;
	descr.Serial = node.Serial;
	descr.LastID = node.NodeID;
	descr.NodeName = node.NodeName;
	descr.DeviceName = node.DeviceName;
	descr.VendorName = node.VendorName;
	descr.State = LCNodeState_Disabled;
	descr.Objects = node.Objects;
	descr.ObjectsSize = node.ObjectsSize;
	descr.Directories = node.Directories;
	descr.DirectoriesSize = node.DirectoriesSize;

	//save node in table
	int i = 0;
	for (; i < LEVCAN_MAX_OWN_NODES; i++) {
		if (own_nodes[i].ShortName.NodeID == LC_Broadcast_Address) {
			own_nodes[i] = descr;
			break;
		}
	}
	if (i == LEVCAN_MAX_OWN_NODES)
		return 0; //out of range
	//clean up
	memset(own_nodes[i].SystemObjects, 0, sizeof(own_nodes[i].SystemObjects));
	//now setup system calls
	uint16_t sysinx = 0;
	LC_Object_t* objparam;
	//adress claim
	objparam = &own_nodes[i].SystemObjects[sysinx++];
	objparam->Address = proceedAddressClaim;
	objparam->Attributes.Readable = 1;
	objparam->Attributes.Writable = 1;
	objparam->Attributes.Function = 1;
	objparam->Index = LC_SYS_AddressClaimed;
	objparam->Size = 8;

	if (own_nodes[i].NodeName) {
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = own_nodes[i].NodeName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_NodeName;
		objparam->Size = strlen(own_nodes[i].NodeName) + 1; // plus zero byte
	}
	if (own_nodes[i].DeviceName) {
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = own_nodes[i].DeviceName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_DeviceName;
		objparam->Size = strlen(own_nodes[i].DeviceName) + 1;
	}
	if (own_nodes[i].VendorName) {
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = own_nodes[i].VendorName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_VendorName;
		objparam->Size = strlen(own_nodes[i].VendorName) + 1;
	}
	//SN
	objparam = &own_nodes[i].SystemObjects[sysinx++];
	objparam->Address = &own_nodes[i].Serial;
	objparam->Attributes.Readable = 1;
	objparam->Index = LC_SYS_SerialNumber;
	objparam->Size = sizeof(own_nodes[i].Serial);
	//todo add server also?!
	if (own_nodes[i].ShortName.Configurable && proceedParam != lc_default_handler) {
		//parameter editor
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = proceedParam;
		objparam->Attributes.Writable = 1;
		objparam->Attributes.Function = 1;
		objparam->Attributes.TCP = 1;
		objparam->Index = LC_SYS_Parameters;
		objparam->Size = -1;
	}
	if (node.FileServer && proceedFileServer != lc_default_handler) {
		//File server
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = proceedFileServer;
		objparam->Attributes.Writable = 1;
		objparam->Attributes.Function = 1;
		objparam->Attributes.TCP = 1;
		objparam->Index = LC_SYS_FileClient; //get client requests
		objparam->Size = -1; //anysize
	}
	if (proceedFileClient != lc_default_handler) {
		//File client
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = proceedFileServer;
		objparam->Attributes.Writable = 1;
		objparam->Attributes.Function = 1;
		objparam->Attributes.TCP = 1;
		objparam->Index = LC_SYS_FileServer; //get client requests
		objparam->Size = -1; //anysize
	}
	//begin network discovery for start
	own_nodes[i].LastTXtime = 0;
	LC_SendDiscoveryRequest(LC_Broadcast_Address);
	own_nodes[i].State = LCNodeState_NetworkDiscovery;
	return (uintptr_t*) &own_nodes[i];
}

void lc_default_handler(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size) {

}

void initialize(void) {
	static uint16_t startup = 0;
	if (startup)
		return;

	startup = 1;

	rxFIFO_in = 0;
	rxFIFO_out = 0;
	memset(rxFIFO, 0, sizeof(rxFIFO));

	txFIFO_in = 0;
	txFIFO_out = 0;
	memset(txFIFO, 0, sizeof(txFIFO));

	for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++)
		own_nodes[i].ShortName.NodeID = LC_Broadcast_Address;
	for (int i = 0; i < LEVCAN_MAX_TABLE_NODES; i++)
		node_table[i].ShortName.NodeID = LC_Broadcast_Address;

#ifdef LEVCAN_STATIC_MEM
	objectBuffer_freeID = 0;
	for (int i = 0; i < LEVCAN_OBJECT_SIZE; i++) {
		objectBuffer[i].Position = -1; //empty object
		objectBuffer[i].Pointer = 0;
		objectBuffer[i].Next = 0;
		objectBuffer[i].Previous = 0;
	}
#endif
	configureFilters();
}

int32_t getTXqueueSize(void) {
	if (txFIFO_in >= txFIFO_out)
		return txFIFO_in - txFIFO_out;
	else
		return txFIFO_in + (LEVCAN_TX_SIZE - txFIFO_out);
}

void proceedAddressClaim(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size) {
	if (header.Request) {
		//TODO what to do with null?
		if (header.Target == LC_Broadcast_Address) {
			//send every node id
			for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
				if (own_nodes[i].ShortName.NodeID < LC_Null_Address && own_nodes[i].State >= LCNodeState_WaitingClaim) {
					if (own_nodes[i].State == LCNodeState_Online)
						own_nodes[i].LastTXtime = 0; //reset online timer
					LC_AddressClaimHandler(own_nodes[i].ShortName, LC_TX);
				}
			}
		} else {
			//single node
			for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
				if (own_nodes[i].ShortName.NodeID == header.Target && own_nodes[i].State >= LCNodeState_WaitingClaim) {
					if (own_nodes[i].State == LCNodeState_Online)
						own_nodes[i].LastTXtime = 0; //reset online timer
					LC_AddressClaimHandler(own_nodes[i].ShortName, LC_TX);
					break;
				}
			}
		}
	} else {
		//got some other claim
		uint32_t* toui = data;
		if (toui != 0)
			LC_AddressClaimHandler((LC_NodeShortName_t ) { .ToUint32[0] = toui[0], .ToUint32[1] =toui[1], .NodeID = header.Source }, LC_RX);
	}
}

void LC_AddressClaimHandler(LC_NodeShortName_t node, uint16_t mode) {
	headerPacked_t header = { .Priority = ~LC_Priority_Control, .MsgID = LC_SYS_AddressClaimed, .Target = LC_Broadcast_Address, .Request = 0, .RTS_CTS = 1, .EoM = 1 };
	uint32_t data[2];

	int ownfound = 0;
	int idlost = 0;
	if (mode == LC_RX) {
		/* *
		 * received address claim message.
		 * we should compare it with our own and send claim if our name is not less by value
		 * or add it to table if there is none, or update existing if possible
		 * */
		if (node.NodeID < LC_Null_Address) {
			for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
				if (own_nodes[i].ShortName.NodeID == node.NodeID && own_nodes[i].State >= LCNodeState_WaitingClaim) {
					//same address
					ownfound = 1;
					if (compareNode(own_nodes[i].ShortName, node) != -1) {
						idlost = 1; //if we loose this id, we should try to add new node to table
						//less value - more priority. our not less, reset address
						header.Source = LC_Null_Address;
						//copy short name
						data[0] = own_nodes[i].ShortName.ToUint32[0];
						data[1] = own_nodes[i].ShortName.ToUint32[1];
						//later we will find new id
						own_nodes[i].ShortName.NodeID = LC_Null_Address;
						own_nodes[i].State = LCNodeState_WaitingClaim;
						configureFilters();
#ifdef LEVCAN_TRACE
						trace_printf("We lost ID:%d\n", node.NodeID);
#endif
					} else {
						//send own data to break other node id
						header.Source = own_nodes[i].ShortName.NodeID;
						data[0] = own_nodes[i].ShortName.ToUint32[0];
						data[1] = own_nodes[i].ShortName.ToUint32[1];
#ifdef LEVCAN_TRACE
						trace_printf("Collision found ID:%d\n", node.NodeID);
#endif
					}
					break;
				}
			}
		} else {
			//someone lost his id?
			for (int i = 0; i < LEVCAN_MAX_TABLE_NODES; i++)
				if (compareNode(node_table[i].ShortName, node) == 0) {
					//compare by short name, if found - delete this instance
#ifdef LEVCAN_TRACE
					trace_printf("Lost S/N:%08X ID:%d\n", node.SerialNumber, node_table[i].ShortName.NodeID);
#endif
					node_table[i].ShortName.NodeID = LC_Broadcast_Address;
					return;
				}
			return;
		}
		if ((ownfound == 0) || idlost) {
			//not found in own nodes table, look for external
			uint16_t empty = 255;
			for (int i = 0; i < LEVCAN_MAX_TABLE_NODES; i++)
				if (node_table[i].ShortName.NodeID >= LC_Null_Address) {
					//look for first new position
					if (empty == 255)
						empty = i;
				} else if (node_table[i].ShortName.NodeID == node.NodeID) {
					//same address
					int eql = compareNode(node_table[i].ShortName, node);
					if (eql == 1) {
						//less value - more priority. our table not less, setup new short name
						node_table[i].ShortName = node;
						node_table[i].LastRXtime = 0;
#ifdef LEVCAN_TRACE
						trace_printf("Replaced ID: %d from S/N: 0x%04X to S/N: 0x%04X\n", node_table[i].ShortName.NodeID, node_table[i].ShortName.SerialNumber,
								node.SerialNumber);
#endif
					} else if (eql == 0) {
						//	trace_printf("Claim Update ID: %d\n", node_table[i].ShortName.NodeID);
						node_table[i].LastRXtime = 0;
					}
					return; //replaced or not, return anyway. do not add
				}
			//we can add new node
			if (empty != 255) {
				node_table[empty].ShortName = node;
				node_table[empty].LastRXtime = 0;
#ifdef LEVCAN_TRACE
				trace_printf("New node detected ID:%d\n", node.NodeID);
#endif
			}
			if (!idlost)
				return;
		}

	}
	if (ownfound == 0) {
		//TX our own (new) node id
		header.Source = node.NodeID;
		//copy short name
		data[0] = node.ToUint32[0];
		data[1] = node.ToUint32[1];
	}
	sendDataToQueue(header, data, 8);
}

void configureFilters(void) {

	CAN_FiltersClear();
	CAN_FilterEditOn();
	//global filter
	headerPacked_t reg = { 0 }, mask = { 0 };
	reg.MsgID = LC_SYS_AddressClaimed;
	reg.RTS_CTS = 0; //no matter
	reg.Parity = 0; // no matter
	reg.Priority = 0; //no matter
	reg.Source = 0; //any source
	reg.Target = LC_Broadcast_Address; //we are target- Broadcast, this should match
	reg.Request = 0;
	//fill can mask match
	mask = reg;
	if (own_nodes[0].ShortName.NodeID < LC_Null_Address) {
		mask.MsgID = 0; //match any brdcast
	} else
		mask.MsgID = 0x3F0; //match for first 16 system messages
	mask.Request = 0; //any request or data
	//type cast
	CAN_CreateFilterMask((CAN_IR ) { .ToUint32 = reg.ToUint32 }, (CAN_IR ) { .ToUint32 = mask.ToUint32 }, 0);

	for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
		if (own_nodes[i].ShortName.NodeID < LC_Null_Address)
			addAddressFilter(own_nodes[i].ShortName.NodeID);
	}
	CAN_FilterEditOff();
}

void addAddressFilter(uint16_t address) {
	//global filter
	headerPacked_t reg = { 0 }, mask = { 0 };
	reg.MsgID = 0; //no matter
	reg.RTS_CTS = 0; //no matter
	reg.Parity = 0; // no matter
	reg.Priority = 0; //no matter
	reg.Source = 0; //any source
	reg.Target = address; //we are target, this should match
	reg.Request = 0; //no matter
	//fill can mask match
	mask = reg;
	mask.MsgID = 0; //match any
	mask.Target = LC_Broadcast_Address; // should match
	mask.Request = 0; //any request or data
	//type cast
	CAN_CreateFilterMask((CAN_IR ) { .ToUint32 = reg.ToUint32 }, (CAN_IR ) { .ToUint32 = mask.ToUint32 }, 0);
}

int16_t compareNode(LC_NodeShortName_t a, LC_NodeShortName_t b) {
	int16_t i = 0;
	for (; (i < 2) && (a.ToUint32[i] == b.ToUint32[i]); i++)
		;
	if (i == 2)
		return 0;
	else if (a.ToUint32[i] < b.ToUint32[i])
		return -1;
	else
		return 1;
}

objBuffered* findObject(objBuffered* array, uint16_t msgID, uint8_t target, uint8_t source) {
	objBuffered* obj = array;
	while (obj) {
		//same source and same ID ?
		//one ID&source can send only one message length a time
		if (obj->Header.MsgID == msgID && obj->Header.Target == target && obj->Header.Source == source) {
			return obj;
		}
		obj = (objBuffered*) obj->Next;
	}
	return 0;
}

void LC_ReceiveHandler(void) {
	static headerPacked_t header;
	static uint32_t data[2];
	static uint16_t length;
	//fast receive to clear input buffer, handle later in manager
	while (CAN_Receive(&header.ToUint32, data, &length) == CANH_Ok) {
		//buffer not full?
		if (rxFIFO_in == ((rxFIFO_out - 1 + LEVCAN_RX_SIZE) % LEVCAN_RX_SIZE))
			continue;
		//store in rx buffer
		rxFIFO[rxFIFO_in].data[0] = data[0];
		rxFIFO[rxFIFO_in].data[1] = data[1];
		rxFIFO[rxFIFO_in].length = length;
		rxFIFO[rxFIFO_in].header = header;
		rxFIFO_in = (rxFIFO_in + 1) % LEVCAN_RX_SIZE;
	}
}

void LC_NetworkManager(uint32_t time) {

	for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
		if (own_nodes[i].State == LCNodeState_Disabled)
			continue;
		//send every node id
		if (own_nodes[i].State == LCNodeState_NetworkDiscovery) {
			//check network discovery timeout
			own_nodes[i].LastTXtime += time;
			if (own_nodes[i].LastTXtime > 100) {
				//we ready to begin address claim
				//todo move to claimFreeID
				uint16_t freeid = own_nodes[i].LastID;
				while (searchIndexCollision(freeid, &own_nodes[i])) {
					freeid++;
					if (freeid > LC_NodeFreeIDmax || freeid < LC_NodeFreeIDmin)
						freeid = LC_NodeFreeIDmin;
				}
				own_nodes[i].State = LCNodeState_WaitingClaim;
				own_nodes[i].LastTXtime = 0;
				own_nodes[i].ShortName.NodeID = freeid;
				CAN_FilterEditOn();
				addAddressFilter(freeid);
				CAN_FilterEditOff();
				LC_AddressClaimHandler(own_nodes[i].ShortName, LC_TX);
#ifdef LEVCAN_TRACE
				trace_printf("Discovery finish id:%d\n", own_nodes[i].ShortName.NodeID);
#endif
			}
		} else if (own_nodes[i].ShortName.NodeID == LC_Null_Address) {
			//we've lost id, get new one
			//look for free id
			own_nodes[i].LastTXtime = 0;
			claimFreeID(&own_nodes[i]);
		} else if (own_nodes[i].ShortName.NodeID < LC_Broadcast_Address) {

			if (own_nodes[i].State == LCNodeState_WaitingClaim) {
				own_nodes[i].LastTXtime += time;
				if (own_nodes[i].LastTXtime > 250) {
					own_nodes[i].State = LCNodeState_Online;
					configureFilters(); //todo make it faster?
					own_nodes[i].LastTXtime = 0;
#ifdef LEVCAN_TRACE
					trace_printf("We are online ID:%d\n", own_nodes[i].ShortName.NodeID);
#endif
				}
			} else if (own_nodes[i].State == LCNodeState_Online) {
				static int alone = 0;
				//we are online! why nobody asking for it?
				own_nodes[i].LastTXtime += time;
				if (own_nodes[i].LastTXtime > 2500) {
					own_nodes[i].LastTXtime = 0;
					LC_AddressClaimHandler(own_nodes[i].ShortName, LC_TX);
#ifdef LEVCAN_TRACE
					if (alone == 0) {
						trace_printf("Are we alone?:%d\n", own_nodes[i].ShortName.NodeID);
					}
					alone = 2000;
#endif
				} else if (alone)
					alone -= time;
			}
		}
	}

	while (rxFIFO_in != rxFIFO_out) {
		//proceed RX FIFO
		headerPacked_t hdr = rxFIFO[rxFIFO_out].header;
		if (hdr.Request) {
			if (hdr.RTS_CTS == 0 && hdr.EoM == 0) {
				//Remote transfer request, try to create new TX object
				LC_NodeDescription_t* node = findNode(hdr.Target);
				LC_ObjectRecord_t obj = findObjectRecord(hdr.MsgID, rxFIFO[rxFIFO_out].length, node, Read, hdr.Source);
				obj.NodeID = hdr.Source; //receiver
				if (obj.Attributes.Function && obj.Address) {
					//function call before sending
					//unpack header
					LC_Header_t unpack = headerUnpack(hdr);
					//call object
					((LC_FunctionCall_t) obj.Address)(node, unpack, 0, 0);
				} else {
					//check for existing objects, dual request denied
					//ToDo is this best way? maybe reset tx?
					objBuffered* txProceed = findObject(objTXbuf_start, hdr.MsgID, hdr.Source, hdr.Target);
					if (txProceed == 0) {
						obj.Attributes.TCP |= hdr.Parity; //force TCP mode if requested
						LC_SendMessage((intptr_t*) node, &obj, hdr.MsgID);
					} else {
#ifdef LEVCAN_TRACE
						//trace_printf("RX dual request denied:%d, from node:%d \n", hdr.MsgID, hdr.Source);
#endif
					}
				}
			} else {
				//find existing TX object, tcp clear-to-send and end-of-msg-ack
				objBuffered* TXobj = findObject(objTXbuf_start, hdr.MsgID, hdr.Source, hdr.Target);
				if (TXobj)
					objectTXproceed(TXobj, &hdr);
			}
		} else {
			//we got data
			if (hdr.RTS_CTS) {
				//address valid?
				if (hdr.Source >= LC_Null_Address) {
					//get next buffer index
					rxFIFO_out = (rxFIFO_out + 1) % LEVCAN_RX_SIZE;
					continue;
				}
				if (hdr.EoM && hdr.Parity == 0) {
					//fast receive for udp
					if (objectRXfinish(hdr, rxFIFO[rxFIFO_out].data, rxFIFO[rxFIFO_out].length, 0)) {
#ifdef LEVCAN_TRACE
						trace_printf("RX fast failed:%d \n", hdr.MsgID);
#endif
					}
				} else {
					//find existing RX object, delete in case we get new RequestToSend
					objBuffered* RXobj = findObject(objRXbuf_start, hdr.MsgID, hdr.Target, hdr.Source);
					if (RXobj) {
						lcfree(RXobj->Pointer);
						deleteObject(RXobj, &objRXbuf_start, &objRXbuf_end);
					}
					//create new receive object
#ifndef LEVCAN_MEM_STATIC
					objBuffered* newRXobj = (objBuffered*) lcmalloc(sizeof(objBuffered));
#else
					objBuffered* newRXobj = getFreeObject();
#endif
					if (newRXobj == 0) {
						//get next buffer index
						rxFIFO_out = (rxFIFO_out + 1) % LEVCAN_RX_SIZE;
						continue;
					}
					//data alloc
#ifndef LEVCAN_MEM_STATIC
					newRXobj->Pointer = lcmalloc(LEVCAN_OBJECT_DATASIZE);
					if (newRXobj->Pointer == 0) {
						lcfree(newRXobj);
						rxFIFO_out = (rxFIFO_out + 1) % LEVCAN_RX_SIZE;
						continue;
					}
#endif
					newRXobj->Length = LEVCAN_OBJECT_DATASIZE;
					newRXobj->Header = hdr;
					newRXobj->Flags.TCP = hdr.Parity; //setup rx mode
					newRXobj->Position = 0;
					newRXobj->Attempt = 0;
					newRXobj->Time_since_comm = 0;
					newRXobj->Next = 0;
					newRXobj->Previous = 0;
					//not critical here
					if (objRXbuf_start == 0) {
						//no objects in rx array
						objRXbuf_start = newRXobj;
						objRXbuf_end = newRXobj;
					} else {
						//add to the end
						newRXobj->Previous = (intptr_t*) objRXbuf_end;
						objRXbuf_end->Next = (intptr_t*) newRXobj;
						objRXbuf_end = newRXobj;
					}
					//	trace_printf("New RX object created:%d\n", newRXobj->Header.MsgID);
					objectRXproceed(newRXobj, &rxFIFO[rxFIFO_out]);
				}
			} else {
				//find existing RX object
				objBuffered* RXobj = findObject(objRXbuf_start, hdr.MsgID, hdr.Target, hdr.Source);
				if (RXobj)
					objectRXproceed(RXobj, &rxFIFO[rxFIFO_out]);
			}
		}
		//get next buffer index
		rxFIFO_out = (rxFIFO_out + 1) % LEVCAN_RX_SIZE;
	}
	//count work time and clean up
	objBuffered* txProceed = (objBuffered*) objTXbuf_start;
	while (txProceed) {
		objBuffered* next = (objBuffered*) txProceed->Next;
		txProceed->Time_since_comm += time;
		if (txProceed->Flags.TCP == 0) {
			//UDP mode send data continuously
			objectTXproceed(txProceed, 0);
		} else {
			//TCP mode
			if (txProceed->Time_since_comm > 100) {
				if (txProceed->Attempt > 10) {
					//TX timeout, make it free!
#ifdef LEVCAN_TRACE
					trace_printf("TX object deleted by attempt:%d\n", txProceed->Header.MsgID);
#endif
					if (txProceed->Flags.TXcleanup) {
						lcfree(txProceed->Pointer);
					}
					deleteObject(txProceed, (objBuffered**) &objTXbuf_start, (objBuffered**) &objTXbuf_end);
				} else {
					// Try tx again
					objectTXproceed(txProceed, 0);
					//TOdo may cause buffer overflow if CAN is offline
					if (txProceed->Time_since_comm == 0)
						txProceed->Attempt++;
				}
			}
		}
		txProceed = next;
	}
	//count work time and recall
	objBuffered* rxProceed = (objBuffered*) objRXbuf_start;
	while (rxProceed) {
		objBuffered* next = (objBuffered*) rxProceed->Next;
		rxProceed->Time_since_comm += time;
		if (rxProceed->Time_since_comm > 1000) {
			//UDP mode rx timeout
#ifdef LEVCAN_TRACE
			trace_printf("RX object deleted by timeout:%d\n", rxProceed->Header.MsgID);
#endif
			lcfree(rxProceed->Pointer);
			deleteObject(rxProceed, (objBuffered**) &objRXbuf_start, (objBuffered**) &objRXbuf_end);
		}
		rxProceed = next;
	}

	LC_TransmitHandler(); //start tx

#if (LEVCAN_MAX_TABLE_NODES) > 0
	//now look for dead nodes...
	static uint32_t offline_tick = 0;
	const uint16_t off_period = 250; //0.25s
	offline_tick += time;
	if (offline_tick > off_period) {
		for (int i = 0; i < LEVCAN_MAX_TABLE_NODES; i++) {
			//send every node id
			if (node_table[i].ShortName.NodeID < LC_Null_Address) {
				node_table[i].LastRXtime += offline_tick;
				if (node_table[i].LastRXtime > 1500) {
					//timeout, delete node
#ifdef LEVCAN_TRACE
					trace_printf("Node lost, timeout:%d\n", node_table[i].ShortName.NodeID);
#endif
					node_table[i].ShortName.NodeID = LC_Broadcast_Address;
				} else if (node_table[i].LastRXtime > 1000) {
					//ask node, is it online?
					LC_SendDiscoveryRequest(node_table[i].ShortName.NodeID);
				}
			}
		}
		offline_tick = offline_tick % off_period;
	}
#endif
}

void deleteObject(objBuffered* obj, objBuffered** start, objBuffered** end) {
	lc_disable_irq();
	if (obj->Previous)
		((objBuffered*) obj->Previous)->Next = obj->Next; //junction
	else {
#ifdef LEVCAN_TRACE
		if ((*start) != obj) {
			trace_printf("Start object error\n");
		}
#endif
		(*start) = (objBuffered*) obj->Next; //Starting
		if ((*start) != 0)
			(*start)->Previous = 0;
	}
	if (obj->Next) {
		((objBuffered*) obj->Next)->Previous = obj->Previous;
	} else {
#ifdef LEVCAN_TRACE
		if ((*end) != obj) {
			trace_printf("End object error\n");
		}
#endif
		(*end) = (objBuffered*) obj->Previous; //ending
		if ((*end) != 0)
			(*end)->Next = 0;
	}
	lc_enable_irq();
//free this object
#ifdef LEVCAN_MEM_STATIC
	releaseObject(obj);
#else
	lcfree(obj);
#endif
}
#ifdef LEVCAN_MEM_STATIC
objBuffered* getFreeObject(void) {
	objBuffered* ret = 0;
//last free index
	lc_disable_irq();
	int freeid = objectBuffer_freeID;
	if (freeid >= 0 && freeid < LEVCAN_OBJECT_SIZE) {
//search free
		for (int i = freeid; i < LEVCAN_OBJECT_SIZE; i++) {
			if (objectBuffer[i].Position == -1 && objectBuffer[i].Next == 0 && objectBuffer[i].Previous == 0) {
				objectBuffer[i].Position = 0;
				ret = &objectBuffer[i];
				freeid = i + 1; //next is possible free
				break;
			}
		}
	}
	objectBuffer_freeID = freeid;
	lc_enable_irq();

	return ret;
}

void releaseObject(objBuffered* obj) {
	lc_disable_irq();
	int index = (obj - objectBuffer);
	if (index >= 0 && index < LEVCAN_OBJECT_SIZE) {
//mark as free
		objectBuffer[index].Position = -1;
		objectBuffer[index].Next = 0;
		objectBuffer[index].Previous = 0;
//save first free buffer in fast index
		if (index < objectBuffer_freeID)
		objectBuffer_freeID = index;
	} else {
#ifdef LEVCAN_TRACE
		trace_printf("Delete object error\n");
#endif
	}
	lc_enable_irq();
}
#endif

headerPacked_t headerPack(LC_Header_t header) {
	headerPacked_t hdr;
	hdr.RTS_CTS = header.RTS_CTS;
	hdr.MsgID = header.MsgID;
	hdr.Parity = header.Parity;
	hdr.Request = header.Request;
	hdr.Source = header.Source;
	hdr.Target = header.Target;
	hdr.EoM = header.EoM;
	return hdr;
}

LC_Header_t headerUnpack(headerPacked_t header) {
	LC_Header_t hdr;
	hdr.RTS_CTS = header.RTS_CTS;
	hdr.MsgID = header.MsgID;
	hdr.Parity = header.Parity;
	hdr.Request = header.Request;
	hdr.Source = header.Source;
	hdr.Target = header.Target;
	hdr.EoM = header.EoM;
	return hdr;
}

void claimFreeID(LC_NodeDescription_t* node) {
	uint16_t freeid = LC_NodeFreeIDmin;
	if (freeid < node->LastID) {
//this one will increment freeid
		freeid = node->LastID;
	}
	if (freeid > LC_NodeFreeIDmax)
		freeid = LC_NodeFreeIDmin;
//todo fix endless loop
	while (searchIndexCollision(freeid, node)) {
		freeid++;
		if (freeid > LC_NodeFreeIDmax)
			freeid = LC_NodeFreeIDmin;
	}
	node->LastID = freeid;
	node->ShortName.NodeID = freeid;
#ifdef LEVCAN_TRACE
	trace_printf("Trying claim ID:%d\n", freeid);
#endif
	LC_AddressClaimHandler(node->ShortName, LC_TX);
	node->LastTXtime = 0;
	node->State = LCNodeState_WaitingClaim;
//add new own address filter TODO add later after verification
	CAN_FilterEditOn();
	addAddressFilter(freeid);
	CAN_FilterEditOff();

}

uint16_t searchIndexCollision(uint16_t nodeID, LC_NodeDescription_t* ownNode) {
	for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
//todo add online check?
		if ((ownNode != &own_nodes[i]) && (own_nodes[i].ShortName.NodeID == nodeID) && (own_nodes[i].State != LCNodeState_Disabled))
			return 1;
	}
	for (int i = 0; i < LEVCAN_MAX_TABLE_NODES; i++) {
		if (node_table[i].ShortName.NodeID == nodeID)
			return 1;
	}
	return 0;
}

LC_Return_t sendDataToQueue(headerPacked_t hdr, uint32_t data[], uint8_t length) {
	lc_disable_irq();
	if (txFIFO_in == ((txFIFO_out - 1 + LEVCAN_TX_SIZE) % LEVCAN_TX_SIZE)) {
		lc_enable_irq();
		return LC_BufferFull;
	}

	int empty = 0;
	if (txFIFO_in == txFIFO_out)
		empty = 1;

	txFIFO[txFIFO_in].header = hdr;
	txFIFO[txFIFO_in].header.IDE = 1; //use EXID
	txFIFO[txFIFO_in].length = length;
	if (data) {
		txFIFO[txFIFO_in].data[0] = data[0];
		txFIFO[txFIFO_in].data[1] = data[1];
	} else {
		txFIFO[txFIFO_in].data[0] = 0;
		txFIFO[txFIFO_in].data[1] = 0;
	}

	txFIFO_in = (txFIFO_in + 1) % LEVCAN_TX_SIZE;
	lc_enable_irq();
//proceed queue if we can do;
	if (empty)
		LC_TransmitHandler();
	return LC_Ok;
}

uint16_t objectTXproceed(objBuffered* object, headerPacked_t* request) {
	int32_t length;
	uint32_t data[2];
	uint8_t parity = ~((object->Position + 7) / 8) & 1; //parity
	if (request) {
		if (request->EoM) {
			//TX finished? delete this buffer anyway
#ifdef LEVCAN_TRACE
			//trace_printf("TX TCP finished:%d\n", object->Header.MsgID);
			if (object->Position != object->Length)
				trace_printf("TX TCP length mismatch:%d, it is:%d, it should:%d\n", object->Header.MsgID, object->Position, object->Length);
#endif
#ifndef LEVCAN_MEM_STATIC
			//cleanup tx buffer also
			if (object->Flags.TXcleanup)
				lcfree(object->Pointer);
#endif
			//delete object from memory chain, find new endings
			deleteObject(object, (objBuffered**) &objTXbuf_start, (objBuffered**) &objTXbuf_end);
			return 0;
		}
		if (parity != request->Parity) {
			// trace_printf("Request got invalid parity -\n");
			if (object->Time_since_comm == 0)
				return 0; //avoid request spamming

			//requested previous data pack, latest was lost
			int reminder = object->Position % 8;
			reminder = (reminder == 0) ? 8 : reminder;
			//roll back position
			object->Position -= reminder;
			if (object->Position < 0)
				object->Position = 0; //just in case... WTF
			parity = ~((object->Position + 7) / 8) & 1; //parity
#ifdef LEVCAN_TRACE
			// trace_printf("TX object parity lost:%d position:%d\n", object->Header.MsgID, object->Position);
#endif
		} else {
#ifdef LEVCAN_TRACE
			// trace_printf("Request got valid parity\n");
#endif
		}
	}
	do {
		headerPacked_t newhdr = object->Header;
		length = 0;
		if (object->Length >= 0) {
			length = object->Length - object->Position;
			if (length > 8)
				length = 8;
			//set data end
			if (object->Length == object->Position + length)
				newhdr.EoM = 1;
			else
				newhdr.EoM = 0;
		} else {
			length = strnlen((char*) &object->Pointer[object->Position], 8);
			if (length < 8) {
				length++; //ending zero byte
				newhdr.EoM = 1;
			} else
				newhdr.EoM = 0;
		}
//Extract new portion of data in obj. null length cant be
		memcpy(data, &object->Pointer[object->Position], length);
		if (object->Position == 0) {
			//Request new buffer anyway. maybe there was wrong request while data wasn't sent at all?
			newhdr.RTS_CTS = 1;
		} else
			newhdr.RTS_CTS = 0;

		newhdr.Parity = object->Flags.TCP ? parity : 0; //parity
//try to send
		if (sendDataToQueue(newhdr, data, length))
			return 1;
//increment if sent succesful
		object->Position += length;
		object->Header = newhdr; //update to new only here
		object->Time_since_comm = 0; //data sent
//cycle if this is UDP till message end or buffer half fill
	} while ((object->Flags.TCP == 0) && (getTXqueueSize() * 3 < LEVCAN_TX_SIZE * 4) && (object->Header.EoM == 0));
//in UDP mode delete object when EoM is set
	if ((object->Flags.TCP == 0) && (object->Header.EoM == 1)) {
		if (object->Flags.TXcleanup) {
			lcfree(object->Pointer);
		}
		deleteObject(object, (objBuffered**) &objTXbuf_start, (objBuffered**) &objTXbuf_end);
	}
	return 0;
}

uint16_t objectRXproceed(objBuffered* object, msgBuffered* msg) {
	if ((msg != 0) && (msg->header.RTS_CTS && object->Position != 0))
		return 1; //position 0 can be started only with RTS (RTS will create new transfer object)

	uint8_t parity = ~((object->Position + 7) / 8) & 1; //parity
	int32_t position_new = object->Position;

//increment data if correct parity or if mode=0 (UDP)
	if (msg && ((msg->header.Parity == parity) || (object->Flags.TCP == 0))) {
//new correct data
		position_new += msg->length;
//check memory overload
		if (object->Length < position_new) {
#ifndef LEVCAN_MEM_STATIC
			char* newmem = lcmalloc(object->Length * 2);
			if (newmem) {
				memcpy(newmem, object->Pointer, object->Length);
				object->Length = object->Length * 2;
			}
			lcfree(object->Pointer);
			object->Pointer = newmem;
			//todo check possible pointer loose and close object
#else
			//out of stack, inform and delete
#ifdef LEVCAN_TRACE
			trace_printf("RX buffer overflow, object deleted:%d\n", object->Header.MsgID);
#endif
			deleteObject(object, (objBuffered**) &objRXbuf_start, (objBuffered**) &objRXbuf_end);
			return 0;
#endif
		}
#ifndef LEVCAN_MEM_STATIC
		if (object->Pointer)
			memcpy(&object->Pointer[object->Position], msg->data, msg->length);
#else
		memcpy(&object->Data[object->Position], msg->data, msg->length);
#endif
		object->Position = position_new;
		parity = ~((object->Position + 7) / 8) & 1; //update parity
		object->Header.EoM = msg->header.EoM;
//communication established
		object->Attempt = 0;
		object->Time_since_comm = 0;
	} /*else if (msg && (msg->header.Parity != parity))
	 trace_printf("RX parity error:%d position:%d\n", object->Header.MsgID, object->Position);
	 else if (msg == 0)
	 trace_printf("Timeout ");*/
//pack new header responce
	if (object->Flags.TCP) {
		headerPacked_t hdr = { 0 };
		if (object->Header.EoM) {
			hdr.EoM = 1; //end of message
			hdr.RTS_CTS = 0;
		} else {
			hdr.EoM = 0;
			hdr.RTS_CTS = 1; //clear to send
		}
		hdr.Priority = object->Header.Priority;
		hdr.Source = object->Header.Target; //we are target (receive)
		hdr.Target = object->Header.Source;
		hdr.Request = 1;
		hdr.MsgID = object->Header.MsgID;
		hdr.Parity = parity;
//finish? find right object in dictionary, copy data, close buffer
		/*	if (object->Header.EoM)
		 trace_printf("RX EOM sent:%d size:%d\n", object->Header.MsgID, object->Position);
		 else
		 trace_printf("RX request CTS sent:%d position:%d parity:%d\n", object->Header.MsgID, object->Position, hdr.Parity);
		 */
		sendDataToQueue(hdr, 0, 0);
	}
	if (object->Header.EoM) {
#ifndef LEVCAN_MEM_STATIC
		objectRXfinish(object->Header, object->Pointer, object->Position, 1);
#else
		objectRXfinish(object->Header, object->Data, object->Position, 0);
#endif
		//delete object from memory chain, find new endings
		deleteObject(object, (objBuffered**) &objRXbuf_start, (objBuffered**) &objRXbuf_end);
	}

	return 0;
}

LC_Return_t objectRXfinish(headerPacked_t header, char* data, int32_t size, uint8_t memfree) {
	LC_Return_t ret = LC_Ok;
	LC_NodeDescription_t* node = findNode(header.Target);
	//check check and check again
	LC_ObjectRecord_t obj = findObjectRecord(header.MsgID, size, node, Write, header.Source);
	if (obj.Address != 0 && (obj.Attributes.Writable) != 0) {
		if (obj.Attributes.Function) {
			//function call
			((LC_FunctionCall_t) obj.Address)(node, headerUnpack(header), data, size);
		} else if (obj.Attributes.Pointer) {
#ifndef LEVCAN_MEM_STATIC
			//store our memory pointer
			//TODO: call new malloc for smaller size?
			char* clean = *(char**) obj.Address;
			*(char**) obj.Address = data;
			//cleanup if there was pointer
			if (clean)
				lcfree(clean);
			memfree = 0;
#endif
		} else {
			//just copy data as usual to specific location
			int32_t sizeabs = abs(obj.Size);
			//limit size to received size
			if (sizeabs > size)
				sizeabs = size;

			memcpy(obj.Address, data, sizeabs);
			//string should be ended with zero
			if (obj.Size < 0)
				((uint8_t*) obj.Address)[sizeabs - 1] = 0;
		}
	} else {
		ret = LC_ObjectError;
#ifdef LEVCAN_TRACE
		trace_printf("RX finish failed %d no object found for size %d\n", header.MsgID, size);
#endif
	}
	//cleanup
	if (memfree)
		lcfree(data);
	return ret;
}

LC_NodeDescription_t* findNode(uint16_t nodeID) {
	LC_NodeDescription_t* node = 0;
	for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++)
		if (own_nodes[i].ShortName.NodeID == nodeID || (nodeID == LC_Broadcast_Address)) {
			node = &own_nodes[i];
			break;
		}
	return node;

}

LC_ObjectRecord_t findObjectRecord(uint16_t index, int32_t size, LC_NodeDescription_t* node, uint8_t read_write, uint8_t nodeID) {
	LC_ObjectRecord_t rec = { 0 };
	LC_ObjectRecord_t* record;
	if (node == 0)
		return rec;
	for (int source = 0; source < 2; source++) {
		int32_t objsize;
		LC_Object_t* objectArray;
//switch between system objects and external
		if (source) {
			objsize = node->ObjectsSize;
			objectArray = node->Objects;
		} else {
			objsize = sizeof(node->SystemObjects) / sizeof(node->SystemObjects[0]);
			objectArray = node->SystemObjects;
		}
//if system object not found, search in external
		for (int i = 0; i < objsize; i++) {
			//TODO optimize mimimi
			record = ((LC_ObjectRecord_t*) objectArray[i].Address);
			//extract pointer, size and attributes
			if (objectArray[i].Attributes.Record) {
				rec.Address = record->Address;
				rec.Size = record->Size;
				rec.Attributes = record->Attributes;
			} else {
				rec.Address = objectArray[i].Address;
				rec.Size = objectArray[i].Size;
				rec.Attributes = objectArray[i].Attributes;
				rec.NodeID = LC_Broadcast_Address;
			}
			//right index?
			if (objectArray[i].Index == index) {
				//record with multiple objects?
				if (objectArray[i].Attributes.Record && objectArray[i].Size > 0) {
					//scroll through all LC_ObjectRecord_t[]
					for (int irec = 0; irec < objectArray[i].Size; irec++) {
						//if size<0 - any length accepted up to specified abs(size), check r/w access and id
						if (((size == record[irec].Size) || (record[irec].Size < 0) || (read_write == Read && size == 0))
								&& ((record[irec].Attributes.Readable != read_write) || (record[irec].Attributes.Writable == read_write))
								&& (/*(nodeID == LC_Broadcast_Address) ||*/(record[irec].NodeID == LC_Broadcast_Address) || (record[irec].NodeID == nodeID)))
							return record[irec]; //yes
						else
							rec.Address = 0; //no
					}
				} else {
					//if size<0 - any length accepted up to specified abs(size), check r/w access and id
					//for request size 0 - any object
					if (((size == rec.Size) || (rec.Size < 0) || (read_write == Read && size == 0))
							&& ((rec.Attributes.Readable != read_write) || (rec.Attributes.Writable == read_write))
							&& (/*(nodeID == LC_Broadcast_Address) ||*/(rec.NodeID == LC_Broadcast_Address) || (rec.NodeID == nodeID)))
						return rec;
					else
						rec.Address = 0; //no
				}
			} else
				rec.Address = 0; //no
		}
	}
	return rec;
}

/// Sends LC_ObjectRecord_t to network
/// @param sender
/// @param object
/// @param target
/// @param index
/// @return
LC_Return_t LC_SendMessage(void* sender, LC_ObjectRecord_t* object, uint16_t index) {
	LC_NodeDescription_t* node = sender;
	if (node == 0)
		node = &own_nodes[0];
	if (node->State != LCNodeState_Online)
		return LC_NodeOffline;

	if (object == 0)
		return LC_ObjectError;
	char* dataAddr = object->Address;
	//check that data is real
	if (dataAddr == 0 && object->Size != 0)
		return LC_DataError;
	//extract pointer
	if (object->Attributes.Pointer)
		dataAddr = *(char**) dataAddr;
	//negative size means this is string - any length
	if ((object->Attributes.TCP) || (object->Size > 8) || ((object->Size < 0) && (strnlen(dataAddr, 8) == 8))) {
		//avoid dual same id
		objBuffered* txProceed = findObject(objTXbuf_start, index, object->NodeID, node->ShortName.NodeID);
		if (txProceed)
			return LC_Collision;

		if (object->NodeID == node->ShortName.NodeID)
			return LC_Collision;
		//form message header
		headerPacked_t hdr = { 0 };
		hdr.MsgID = index; //our node index
		hdr.Priority = ~object->Attributes.Priority;
		hdr.Request = 0; //data sending...
		hdr.Source = node->ShortName.NodeID;
		hdr.Target = object->NodeID;
		//create object sender instance
#ifndef LEVCAN_MEM_STATIC
		objBuffered* newTXobj = (objBuffered*) lcmalloc(sizeof(objBuffered));

#else
		//no cleanup for static mem!
		//todo make memcopy to data[] ?
		if (object->Attributes.Cleanup == 1)
		return LC_MallocFail;
		objBuffered* newTXobj = getFreeObject();
#endif
		if (newTXobj == 0)
			return LC_MallocFail;
		newTXobj->Attempt = 0;
		newTXobj->Header = hdr;
		newTXobj->Length = object->Size;
		newTXobj->Pointer = dataAddr;
		newTXobj->Position = 0;
		newTXobj->Time_since_comm = 0;
		newTXobj->Next = 0;
		newTXobj->Flags.TCP = object->Attributes.TCP;
		newTXobj->Flags.TXcleanup = object->Attributes.Cleanup;
		//add to queue, critical section
		lc_disable_irq();
		if (objTXbuf_start == 0) {
			//no objects in tx array
			newTXobj->Previous = 0;
			objTXbuf_start = newTXobj;
			objTXbuf_end = newTXobj;
		} else {
			//add to the end
			newTXobj->Previous = (intptr_t*) objTXbuf_end;
			objTXbuf_end->Next = (intptr_t*) newTXobj;
			objTXbuf_end = newTXobj;
		}
		lc_enable_irq();
#ifdef LEVCAN_TRACE
		//trace_printf("New TX object created:%d\n", newTXobj->Header.MsgID);
#endif
		objectTXproceed(newTXobj, 0);
	} else {
		//some short string? + ending
		int32_t size = object->Size;
		if (size < 0)
			size = strnlen(dataAddr, 7) + 1;

		//fast send
		uint32_t data[2];
		memcpy(data, dataAddr, size);

		headerPacked_t hdr = { 0 };
		hdr.MsgID = index;
		hdr.Priority = ~object->Attributes.Priority;
		hdr.Request = 0;
		hdr.Parity = 0;
		hdr.RTS_CTS = 1; //data start
		hdr.EoM = 1; //data end
		hdr.Source = node->ShortName.NodeID;
		hdr.Target = object->NodeID;

		return sendDataToQueue(hdr, data, size);
	}
	return LC_Ok;
}

LC_Return_t LC_SendRequest(void* sender, uint16_t target, uint16_t index) {
	return LC_SendRequestSpec(sender, target, index, 0, 0);
}

LC_Return_t LC_SendRequestSpec(void* sender, uint16_t target, uint16_t index, uint8_t size, uint8_t TCP) {

	LC_NodeDescription_t* node = sender;
	if (node == 0)
		node = &own_nodes[0];
	if (node->State != LCNodeState_Online)
		return LC_NodeOffline;

	headerPacked_t hdr = { 0 };
	hdr.MsgID = index;
	hdr.Priority = 0;
	hdr.Request = 1;
	hdr.Parity = TCP;
	hdr.RTS_CTS = 0;
	hdr.Source = node->ShortName.NodeID;
	hdr.Target = target;

	return sendDataToQueue(hdr, 0, size);
}

LC_Return_t LC_SendDiscoveryRequest(uint16_t target) {
	headerPacked_t hdr = { 0 };
	hdr.MsgID = LC_SYS_AddressClaimed;
	hdr.Priority = 0;
	hdr.Request = 1;
	hdr.Parity = 0;
	hdr.RTS_CTS = 0;
	hdr.Source = LC_Broadcast_Address;
	hdr.Target = target;

	return sendDataToQueue(hdr, 0, 0);
}

void LC_TransmitHandler(void) {
	//fill TX buffer till no empty slots
	while (1) {
		if (txFIFO_in == txFIFO_out)
			return; /* Queue Empty - nothing to send*/
		if (CAN_Send(txFIFO[txFIFO_out].header.ToUint32, txFIFO[txFIFO_out].data, txFIFO[txFIFO_out].length) != 0)
			return; //CAN full
		txFIFO_out = (txFIFO_out + 1) % LEVCAN_TX_SIZE;
	}
}

LC_NodeShortName_t LC_GetNode(uint16_t nodeID) {
	int i = 0;
	//search
	for (; i < LEVCAN_MAX_TABLE_NODES; i++) {
		if (node_table[i].ShortName.NodeID == nodeID) {
			return node_table[i].ShortName;
		}
	}
	LC_NodeShortName_t ret = (LC_NodeShortName_t ) { .NodeID = LC_Broadcast_Address };
	return ret;
}

int16_t LC_GetNodeIndex(uint16_t nodeID) {
	if (nodeID >= LC_Null_Address)
		return -1;
	//search
	for (int16_t i = 0; i < LEVCAN_MAX_TABLE_NODES; i++) {
		if (node_table[i].ShortName.NodeID == nodeID) {
			return i;
		}
	}
	return -1;
}
/// Call this function in loop get all active nodes. Ends when returns LC_Broadcast_Address
/// @param n Pointer to stored position for search
/// @return Returns active node short name
LC_NodeShortName_t LC_GetActiveNodes(uint16_t* last_pos) {
	int i = *last_pos;
	//new run
	if (*last_pos >= LEVCAN_MAX_TABLE_NODES)
		i = 0;
	//search
	for (; i < LEVCAN_MAX_TABLE_NODES; i++) {
		if (node_table[i].ShortName.NodeID != LC_Broadcast_Address) {
			*last_pos = i + 1;
			return node_table[i].ShortName;
		}
	}
	*last_pos = LEVCAN_MAX_TABLE_NODES;
	LC_NodeShortName_t ret = (LC_NodeShortName_t ) { .NodeID = LC_Broadcast_Address };
	return ret;
}

/// Returns own node short name, containing actual ID
/// @param mynode pointer to node, can be 0 for default node
/// @return LC_NodeShortName
LC_NodeShortName_t LC_GetMyNodeName(void* mynode) {
	LC_NodeDescription_t* node = mynode;
	if (node == 0)
		node = &own_nodes[0];
	return node->ShortName;
}

int16_t LC_GetMyNodeIndex(void* mynode) {
	if (mynode == 0)
		return 0;
	int16_t index = (own_nodes - (LC_NodeDescription_t*) mynode);
	if (index >= 0 && index < LEVCAN_MAX_OWN_NODES)
		return index;
	return -1;
}
