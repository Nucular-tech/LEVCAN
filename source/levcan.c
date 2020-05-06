/*
 * LEV-CAN: Light Electric Vehicle CAN protocol [LC]
 * levcan.c
 *
 *  Created on: 17 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#include "levcan.h"
#include "levcan_address.h"
#include "levcan_param.h"

#include "string.h"
#include "stdlib.h"

#if	defined(lcmalloc) && defined(lcfree)
//extern void *lcmalloc(uint32_t size);
//extern void lcfree(void *pointer);
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

#ifndef LEVCAN_MIN_BYTE_SIZE
#define LEVCAN_MIN_BYTE_SIZE 1
#endif

typedef struct {
	LC_HeaderPacked_t header;
	uint32_t data[2];
	uint8_t length;
} msgBuffered;

typedef struct {
	union {
		char *Pointer;
#ifdef LEVCAN_STATIC_MEM
		char Data[LEVCAN_OBJECT_DATASIZE];
#else
		char Data[sizeof(char*)];
#endif
	};
	int32_t Length;
	int32_t Position;    //get parity - divide by 8 and &1
	LC_HeaderPacked_t Header;
	uint16_t Time_since_comm;
	uint8_t Attempt;
	struct {
		unsigned TCP :1;
		unsigned TXcleanup :1;
	} Flags LEVCAN_PACKED;
	intptr_t *Next;
	intptr_t *Previous;
} objBuffered;

enum {
	Read, Write
};

#ifdef LEVCAN_TRACE
extern int trace_printf(const char *format, ...);
#endif

//#### PRIVATE VARIABLES ####
#ifdef LEVCAN_STATIC_MEM
objBuffered objectBuffer[LEVCAN_OBJECT_SIZE];
int16_t objectBuffer_freeID;
#endif
#ifdef LEVCAN_USE_RTOS_QUEUE
void *txQueue;
void *txSemph;
void *rxQueue;
#else
msgBuffered txFIFO[LEVCAN_TX_SIZE];
volatile uint16_t txFIFO_in, txFIFO_out;
msgBuffered rxFIFO[LEVCAN_RX_SIZE];
volatile uint16_t rxFIFO_in, rxFIFO_out;
#endif

LC_NodeDescriptor_t own_nodes[LEVCAN_MAX_OWN_NODES];
LC_NodeTable_t node_table[LEVCAN_MAX_TABLE_NODES];
volatile objBuffered *objTXbuf_start = 0;
volatile objBuffered *objTXbuf_end = 0;
volatile objBuffered *objRXbuf_start = 0;
volatile objBuffered *objRXbuf_end = 0;
volatile uint16_t own_node_count;
#ifdef DEBUG
volatile uint32_t lc_collision_cntr = 0;
volatile uint32_t lc_receive_ovfl_cntr = 0;
#endif
//#### PRIVATE FUNCTIONS ####
void initialize(void);

LC_NodeDescriptor_t* findNode(uint16_t nodeID);
LC_ObjectRecord_t findObjectRecord(uint16_t index, int32_t size, LC_NodeDescriptor_t *node, uint8_t read_write, uint8_t nodeID);

LC_Return_t lc_sendDataToQueue(LC_HeaderPacked_t hdr, uint32_t data[], uint8_t length);

uint16_t objectRXproceed(objBuffered *object, msgBuffered *msg);
uint16_t objectTXproceed(objBuffered *object, LC_HeaderPacked_t *request);
LC_Return_t objectRXfinish(LC_HeaderPacked_t header, char *data, int32_t size, uint8_t memfree);
void deleteObject(objBuffered *obj, objBuffered **start, objBuffered **end);
#ifdef LEVCAN_STATIC_MEM
objBuffered* getFreeObject(void);
void releaseObject(objBuffered *obj);
#endif

int32_t getTXqueueSize(void);
objBuffered* findObject(objBuffered *array, uint16_t msgID, uint8_t target, uint8_t source);

void lc_default_handler(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
//#### EXTERNAL MODULES ####
//HAL send/receive
extern LC_Return_t LC_HAL_Receive(LC_HeaderPacked_t *header, uint32_t *data, uint8_t *length);
extern LC_Return_t LC_HAL_Send(LC_HeaderPacked_t header, uint32_t *data, uint8_t length);

#ifdef LEVCAN_EVENTS
extern volatile uint8_t lc_eventButtonPressed;
#endif
#ifdef LEVCAN_PARAMETERS
extern void __attribute__((weak, alias("lc_default_handler")))
lc_proceedParam(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
#endif
#ifdef LEVCAN_FILESERVER
extern void __attribute__((weak, alias("lc_default_handler")))
proceedFileServer(LC_NodeDescriptor_t* node, LC_Header_t header, void* data, int32_t size);
#endif
#ifdef LEVCAN_FILECLIENT
#ifdef LEVCAN_USE_RTOS_QUEUE
extern void *frxQueue[];
#else
extern void __attribute__((weak, alias("lc_default_handler")))
proceedFileClient(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
#endif
#endif
extern void lc_processAddressClaim(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
extern LC_Return_t lc_sendDiscoveryRequest(uint16_t target);
//#### FUNCTIONS

LC_Return_t LC_InitNodeDescriptor(LC_NodeDescriptor_t **node) {
	initialize();
	//save node in table
	int i = 0;
	for (; i < LEVCAN_MAX_OWN_NODES; i++) {
		if (own_nodes[i].ShortName.NodeID == LC_Broadcast_Address) {
			//clean up
			own_nodes[i].State = LCNodeState_Disabled;
			memset(own_nodes[i].SystemObjects, 0, sizeof(own_nodes[i].SystemObjects));
			*node = &own_nodes[i];
			return LC_Ok;
		}
		break;
	}
	return LC_MallocFail; //out of range
}

LC_Return_t LC_CreateNode(LC_NodeDescriptor_t *node) {

	int16_t node_table_index = LC_GetMyNodeIndex(node);
	if (node_table_index)
		return LC_DataError;

	if (node->ShortName.NodeID < 0 || node->ShortName.NodeID > 125) {
		uint16_t id = node->Serial % 64;
		id |= 64;
		if (id > 125) {
			id &= ~3;
		}
		node->ShortName.NodeID = id;
	}
	if (node->NodeName != 0 && strnlen(node->NodeName, 128) == 128)
		node->NodeName = 0;    //too long name
	if (node->DeviceName != 0 && strnlen(node->DeviceName, 128) == 128)
		node->DeviceName = 0;
	if (node->VendorName != 0 && strnlen(node->VendorName, 128) == 128)
		node->VendorName = 0;

#ifdef LEVCAN_USE_RTOS_QUEUE
	txQueue = LC_QueueCreate(LEVCAN_TX_SIZE, sizeof(msgBuffered));
	rxQueue = LC_QueueCreate(LEVCAN_RX_SIZE, sizeof(msgBuffered));
	txSemph = LC_SemaphoreCreate();

	if (rxQueue == NULL || rxQueue == NULL || txSemph == NULL) {
		LC_SemaphoreDelete(txSemph);
		LC_QueueDelete(txQueue);
		LC_QueueDelete(rxQueue);

		return 0;
	}
#endif

	node->State = LCNodeState_Disabled;
	node->LastID = node->ShortName.NodeID;
	//now setup system calls
	uint16_t sysinx = 0;
	LC_Object_t *objparam;
	//adress claim, main network function
	//node->SystemObjects[sysinx++] = lc_obj_address_claim;
	objparam = &node->SystemObjects[sysinx++];
	objparam->Address = lc_processAddressClaim;
	objparam->Attributes.Readable = 1;
	objparam->Attributes.Writable = 1;
	objparam->Attributes.Function = 1;
	objparam->Index = LC_SYS_AddressClaimed;
	objparam->Size = (8 / LEVCAN_MIN_BYTE_SIZE);

	if (node->NodeName) {
		objparam = &node->SystemObjects[sysinx++];
		objparam->Address = node->NodeName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_NodeName;
		objparam->Size = strlen(node->NodeName) + 1;    // plus zero byte
	}
	if (node->DeviceName) {
		objparam = &node->SystemObjects[sysinx++];
		objparam->Address = node->DeviceName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_DeviceName;
		objparam->Size = strlen(node->DeviceName) + 1;
	}
	if (node->VendorName) {
		objparam = &node->SystemObjects[sysinx++];
		objparam->Address = node->VendorName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_VendorName;
		objparam->Size = strlen(node->VendorName) + 1;
	}
	//SN
	objparam = &node->SystemObjects[sysinx++];
	objparam->Address = &node->Serial;
	objparam->Attributes.Readable = 1;
	objparam->Index = LC_SYS_SerialNumber;
	objparam->Size = sizeof(node->Serial);
#ifdef LEVCAN_EVENTS
	//Event response
	objparam = &node->SystemObjects[sysinx++];
	objparam->Address = (void*) &lc_eventButtonPressed;
	objparam->Attributes.Writable = 1;
	objparam->Index = LC_SYS_Events;
	objparam->Size = sizeof(lc_eventButtonPressed);
#endif
#ifdef LEVCAN_PARAMETERS
	if (node->ShortName.Configurable && lc_proceedParam != lc_default_handler) {
		//parameter editor
		objparam = &node->SystemObjects[sysinx++];
		objparam->Address = lc_proceedParam;
		objparam->Attributes.Writable = 1;
		objparam->Attributes.Function = 1;
		objparam->Attributes.TCP = 1;
		objparam->Index = LC_SYS_Parameters;
		objparam->Size = -1;
	}
#endif
#ifdef LEVCAN_FILESERVER
	if (node.FileServer && proceedFileServer != lc_default_handler) {
		//File server
		objparam = &node->SystemObjects[sysinx++];
		objparam->Address = proceedFileServer;
		objparam->Attributes.Writable = 1;
		objparam->Attributes.Function = 1;
		objparam->Attributes.TCP = 1;
		objparam->Index = LC_SYS_FileClient;//get client requests
		objparam->Size = -1;//anysize
	}
#endif
#ifdef LEVCAN_FILECLIENT
#ifdef LEVCAN_USE_RTOS_QUEUE
	//File client
	objparam = &node->SystemObjects[sysinx++];
	frxQueue[node_table_index] = objparam->Address = LC_QueueCreate(LEVCAN_MAX_OWN_NODES, sizeof(LC_ObjectData_t));
	objparam->Attributes.Writable = 1;
	objparam->Attributes.Queue = 1;
	objparam->Attributes.TCP = 1;
	objparam->Index = LC_SYS_FileServer;      //get client requests
	objparam->Size = -1;      //anysize
#else
	if (proceedFileClient != lc_default_handler) {
		//File client
		objparam = &node->SystemObjects[sysinx++];
		objparam->Address = proceedFileClient;
		objparam->Attributes.Writable = 1;
		objparam->Attributes.Function = 1;
		objparam->Attributes.TCP = 1;
		objparam->Index = LC_SYS_FileServer;      //get client requests
		objparam->Size = -1;      //anysize
	}
#endif
#endif
	//begin network discovery for start
	node->LastTXtime = 0;
	lc_sendDiscoveryRequest(LC_Broadcast_Address);
	node->State = LCNodeState_NetworkDiscovery;
	return LC_Ok;
}

void lc_default_handler(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	(void) node; //no warnings
	(void) header;
	(void) data;
	(void) size;
}

void initialize(void) {
	static uint16_t startup = 0;
	if (startup)
		return;

	startup = 1;
#ifndef LEVCAN_USE_RTOS_QUEUE
	rxFIFO_in = 0;
	rxFIFO_out = 0;
	memset(rxFIFO, 0, sizeof(rxFIFO));

	txFIFO_in = 0;
	txFIFO_out = 0;
	memset(txFIFO, 0, sizeof(txFIFO));
#endif
	int i = 0;
#if (LEVCAN_MAX_OWN_NODES) > 1
	for (i = 0; i < LEVCAN_MAX_OWN_NODES; i++)
#endif
		own_nodes[i].ShortName.NodeID = LC_Broadcast_Address;
#if (LEVCAN_MAX_OWN_NODES) > 1
	for (i = 0; i < LEVCAN_MAX_TABLE_NODES; i++)
#endif
		node_table[i].ShortName.NodeID = LC_Broadcast_Address;

#ifdef LEVCAN_STATIC_MEM
	objectBuffer_freeID = 0;
	for (int i = 0; i < LEVCAN_OBJECT_SIZE; i++) {
		objectBuffer[i].Position = -1;    //empty object
		objectBuffer[i].Pointer = 0;
		objectBuffer[i].Next = 0;
		objectBuffer[i].Previous = 0;
	}
#endif
	LC_ConfigureFilters();
}

int32_t getTXqueueSize(void) {
#ifndef LEVCAN_USE_RTOS_QUEUE
	if (txFIFO_in >= txFIFO_out)
		return txFIFO_in - txFIFO_out;
	else
		return txFIFO_in + (LEVCAN_TX_SIZE - txFIFO_out);
#else
	return LC_QueueStored(txQueue);
#endif
}

objBuffered* findObject(objBuffered *array, uint16_t msgID, uint8_t target, uint8_t source) {
	objBuffered *obj = array;
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

#ifdef LEVCAN_USE_RTOS_QUEUE
	msgBuffered msgRX;
	uint8_t length = 0;
	YieldNeeded_t yield = 0;

	while (LC_HAL_Receive(&msgRX.header, &msgRX.data, &length) == LC_Ok) {
		//round up and divide to get target cpu message size
		msgRX.length = (length + LEVCAN_MIN_BYTE_SIZE - 1) / LEVCAN_MIN_BYTE_SIZE;
		LC_QueueSendToBackISR(rxQueue, &msgRX, &yield);
	}

	LC_RTOSYieldISR(yield);
#else
	static LC_HeaderPacked_t header;
	static uint32_t data[2];
	static uint16_t length;
	//fast receive to clear input buffer, handle later in manager
	while (LC_HAL_Receive(&header, &data, &length) == LC_Ok) {
		//buffer not full?

		if (rxFIFO_in == ((rxFIFO_out - 1 + LEVCAN_RX_SIZE) % LEVCAN_RX_SIZE)) {
#ifdef DEBUG
								lc_receive_ovfl_cntr++;
					#endif
			continue;
		}
		//store in rx buffer
		msgBuffered *msgRX = &rxFIFO[rxFIFO_in]; //less size in O2 with pointer?! maybe volatile problem
		msgRX->data[0] = data[0];
		msgRX->data[1] = data[1];
		msgRX->length = (length + LEVCAN_MIN_BYTE_SIZE - 1) / LEVCAN_MIN_BYTE_SIZE;
		msgRX->header = header;
		rxFIFO_in = (rxFIFO_in + 1) % LEVCAN_RX_SIZE;
	}
#endif
}

void LC_NetworkManager(uint32_t time) {

	LC_AddressManager(time);
	//proceed RX FIFO
	//LC_ReceiveHandler
	//count work time and clean up
	objBuffered *txProceed = (objBuffered*) objTXbuf_start;
	while (txProceed) {
		objBuffered *next = (objBuffered*) txProceed->Next;
		txProceed->Time_since_comm += time;
		if (txProceed->Flags.TCP == 0) {
			//UDP mode send data continuously
			objectTXproceed(txProceed, 0);
		} else {
			//TCP mode
			if (txProceed->Time_since_comm > 100) {
				if (txProceed->Attempt >= 3) {
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
	objBuffered *rxProceed = (objBuffered*) objRXbuf_start;
	while (rxProceed) {
		objBuffered *next = (objBuffered*) rxProceed->Next;
		rxProceed->Time_since_comm += time;
		if (rxProceed->Time_since_comm > 500) {
			//UDP mode rx timeout
#ifdef LEVCAN_TRACE
			trace_printf("RX object deleted by timeout:%d\n", rxProceed->Header.MsgID);
#endif
			lcfree(rxProceed->Pointer);
			deleteObject(rxProceed, (objBuffered**) &objRXbuf_start, (objBuffered**) &objRXbuf_end);
		}
		rxProceed = next;
	}

	//LC_TransmitManager();    //start tx, obsolete

#if (LEVCAN_MAX_TABLE_NODES) > 0
	//now look for dead nodes...
	static uint32_t offline_tick = 0;
	const uint16_t off_period = 250;    //0.25s
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
					lc_sendDiscoveryRequest(node_table[i].ShortName.NodeID);
				}
			}
		}
		offline_tick = offline_tick % off_period;
	}
#endif
}

void deleteObject(objBuffered *obj, objBuffered **start, objBuffered **end) {
	lc_disable_irq();
	if (obj->Previous)
		((objBuffered*) obj->Previous)->Next = obj->Next;    //junction
	else {
#ifdef LEVCAN_TRACE
		if ((*start) != obj) {
			trace_printf("Start object error\n");
		}
#endif
		(*start) = (objBuffered*) obj->Next;    //Starting
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
		(*end) = (objBuffered*) obj->Previous;    //ending
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
	objBuffered *ret = 0;
	//last free index
	lc_disable_irq();
	int freeid = objectBuffer_freeID;
	if (freeid >= 0 && freeid < LEVCAN_OBJECT_SIZE) {
		//search free
		for (int i = freeid; i < LEVCAN_OBJECT_SIZE; i++) {
			if (objectBuffer[i].Position == -1 && objectBuffer[i].Next == 0 && objectBuffer[i].Previous == 0) {
				objectBuffer[i].Position = 0;
				ret = &objectBuffer[i];
				freeid = i + 1;    //next is possible free
				break;
			}
		}
	}
	objectBuffer_freeID = freeid;
	lc_enable_irq();

	return ret;
}

void releaseObject(objBuffered *obj) {
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

LC_HeaderPacked_t LC_HeaderPack(LC_Header_t header) {
	LC_HeaderPacked_t hdr;
	hdr.RTS_CTS = header.RTS_CTS;
	hdr.MsgID = header.MsgID;
	hdr.Parity = header.Parity;
	hdr.Request = header.Request;
	hdr.Source = header.Source;
	hdr.Target = header.Target;
	hdr.EoM = header.EoM;
	return hdr;
}

LC_Header_t LC_HeaderUnpack(LC_HeaderPacked_t header) {
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

LC_Return_t lc_sendDataToQueue(LC_HeaderPacked_t hdr, uint32_t data[], uint8_t length) {

#ifndef LEVCAN_USE_RTOS_QUEUE
	lc_disable_irq();
	if (txFIFO_in == ((txFIFO_out - 1 + LEVCAN_TX_SIZE) % LEVCAN_TX_SIZE)) {
		lc_enable_irq();
		return LC_BufferFull;
	}
#endif
	msgBuffered msgTX;
	msgTX.header = hdr;
	//fix lengthfrom system size to byte-size
	msgTX.length = length;
	if (data) {
		//todo length ignored, potential unaligned access?
		msgTX.data[0] = data[0];
		msgTX.data[1] = data[1];
	} else {
		msgTX.data[0] = 0;
		msgTX.data[1] = 0;
	}
#ifdef LEVCAN_USE_RTOS_QUEUE
	//todo queue may return fault
	LC_QueueSendToBack(txQueue, &msgTX, 1);
	//LC_SemaphoreGive(txSemph);
#else
	txFIFO[txFIFO_in] = msgTX;
	txFIFO_in = (txFIFO_in + 1) % LEVCAN_TX_SIZE;
	lc_enable_irq();
#endif
	//proceed queue if we can do, NOT THREAD SAFE
	//	if (empty)
	//		LC_TransmitHandler();
	return LC_Ok;
}

uint16_t objectTXproceed(objBuffered *object, LC_HeaderPacked_t *request) {
	int32_t length;
	uint32_t data[2];
	uint32_t step_inc = (8 / LEVCAN_MIN_BYTE_SIZE);
	uint8_t parity = ~((object->Position + step_inc-1) / step_inc) & 1;    //parity
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
				return 0;    //avoid request spamming

			//requested previous data pack, latest was lost
			int reminder = object->Position % (8 / LEVCAN_MIN_BYTE_SIZE);
			reminder = (reminder == 0) ? (8 / LEVCAN_MIN_BYTE_SIZE) : reminder;
			//roll back position
			object->Position -= reminder;
			if (object->Position < 0)
				object->Position = 0;    //just in case... WTF
			parity = ~((object->Position + 7) / (8 / LEVCAN_MIN_BYTE_SIZE)) & 1;    //parity
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
		LC_HeaderPacked_t newhdr = object->Header;
		length = 0;
		if (object->Length >= 0) {
			length = object->Length - object->Position;
			if (length > (8 / LEVCAN_MIN_BYTE_SIZE))
				length = 8 / LEVCAN_MIN_BYTE_SIZE;
			//set data end
			if (object->Length == object->Position + length)
				newhdr.EoM = 1;
			else
				newhdr.EoM = 0;
		} else {
			length = strnlen((char*) &object->Pointer[object->Position], (8 / LEVCAN_MIN_BYTE_SIZE));
			if (length < (8 / LEVCAN_MIN_BYTE_SIZE)) {
				length++;    //ending zero byte
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

		newhdr.Parity = object->Flags.TCP ? parity : 0;    //parity
		//try to send
		if (lc_sendDataToQueue(newhdr, data, length))
			return 1;
		//increment if sent succesful
		object->Position += length;
		object->Header = newhdr;    //update to new only here
		object->Time_since_comm = 0;    //data sent
		//cycle if this is UDP till message end or buffer 3/4 fill
	} while ((object->Flags.TCP == 0) && (getTXqueueSize() * 4 < LEVCAN_TX_SIZE * 3) && (object->Header.EoM == 0));
	//in UDP mode delete object when EoM is set
	if ((object->Flags.TCP == 0) && (object->Header.EoM == 1)) {
		if (object->Flags.TXcleanup) {
			lcfree(object->Pointer);
		}
		deleteObject(object, (objBuffered**) &objTXbuf_start, (objBuffered**) &objTXbuf_end);
	}
	return 0;
}

uint16_t objectRXproceed(objBuffered *object, msgBuffered *msg) {
	if ((msg != 0) && (msg->header.RTS_CTS && object->Position != 0))
		return 1; //position 0 can be started only with RTS (RTS will create new transfer object)
	uint32_t step_inc = (8 / LEVCAN_MIN_BYTE_SIZE);
	uint8_t parity = ~((object->Position + step_inc - 1) / step_inc) & 1;    //parity
	int32_t position_new = object->Position;

	//increment data if correct parity or if mode=0 (UDP)
	if (msg && ((msg->header.Parity == parity) || (object->Flags.TCP == 0))) {
		//new correct data
		position_new += msg->length;
		//check memory overload
		if (object->Length < position_new) {
#ifndef LEVCAN_MEM_STATIC
			char *newmem = lcmalloc(object->Length * 2);
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
		parity = ~((object->Position + step_inc - 1) / step_inc) & 1;    //update parity
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
		LC_HeaderPacked_t hdr = { 0 };
		if (object->Header.EoM) {
			hdr.EoM = 1;    //end of message
			hdr.RTS_CTS = 0;
		} else {
			hdr.EoM = 0;
			hdr.RTS_CTS = 1;    //clear to send
		}
		hdr.Priority = object->Header.Priority;
		hdr.Source = object->Header.Target;    //we are target (receive)
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
		lc_sendDataToQueue(hdr, 0, 0);
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

LC_Return_t objectRXfinish(LC_HeaderPacked_t header, char *data, int32_t size, uint8_t memfree) {
	LC_Return_t ret = LC_Ok;
	LC_NodeDescriptor_t *node = findNode(header.Target);
	//check check and check again
	LC_ObjectRecord_t obj = findObjectRecord(header.MsgID, size, node, Write, header.Source);
	if (obj.Address != 0 && (obj.Attributes.Writable) != 0) {
		if (obj.Attributes.Function) {
			//function call
			((LC_FunctionCall_t) obj.Address)(node, LC_HeaderUnpack(header), data, size);
		} else if (obj.Attributes.Pointer) {
#ifndef LEVCAN_MEM_STATIC
			//store our memory pointer
			//TODO: call new malloc for smaller size?
			char *clean = *(char**) obj.Address;
			*(char**) obj.Address = data;
			//cleanup if there was pointer
			if (clean)
				lcfree(clean);
			memfree = 0;
#ifdef LEVCAN_USE_RTOS_QUEUE
		} else if (obj.Attributes.Queue) {

			if (memfree == 0) {
				//that means it uses static memory.
				char *allocated_mem = lcmalloc(size);
				if (allocated_mem) {
					memcpy(allocated_mem, data, size);
					data = allocated_mem;
				} else
					return LC_MallocFail;

			}
			LC_ObjectData_t qdata;
			qdata.Data = (intptr_t*) data; //user code should call mem free
			qdata.Header = LC_HeaderUnpack(header);
			qdata.Size = size;
			//queue stores LC_ObjectData_t that includes pointer to data
			if (LC_QueueSendToBack(obj.Address, &qdata, 0))
				memfree = 0; //queued successfully
#endif
#endif
		} else {
			//just copy data as usual to specific location
			int32_t sizeabs = abs(obj.Size);
			//limit size to received size
			if (sizeabs > size)
				sizeabs = size;

			memcpy(obj.Address, data, sizeabs);
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

LC_NodeDescriptor_t* findNode(uint16_t nodeID) {
	LC_NodeDescriptor_t *node = 0;
	int i = 0;
#if (LEVCAN_MAX_OWN_NODES) > 1
	for (; i < LEVCAN_MAX_OWN_NODES; i++)
#endif
		if (own_nodes[i].ShortName.NodeID == nodeID || (nodeID == LC_Broadcast_Address)) {
			node = &own_nodes[i];
#if (LEVCAN_MAX_OWN_NODES) > 1
			break;
#endif
		}
	return node;

}

LC_ObjectRecord_t findObjectRecord(uint16_t index, int32_t size, LC_NodeDescriptor_t *node, uint8_t read_write, uint8_t nodeID) {
	LC_ObjectRecord_t rec = { 0 };
	LC_ObjectRecord_t *record;
	if (node == 0)
		return rec;
	for (int source = 0; source < 2; source++) {
		int32_t objsize;
		LC_Object_t *objectArray;
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
							return record[irec];    //yes
						else
							rec.Address = 0;    //no
					}
				} else {
					//if size<0 - any length accepted up to specified abs(size), check r/w access and id
					//for request size 0 - any object
					if (((size == rec.Size) || (rec.Size < 0) || (read_write == Read && size == 0))
							&& ((rec.Attributes.Readable != read_write) || (rec.Attributes.Writable == read_write))
							&& (/*(nodeID == LC_Broadcast_Address) ||*/(rec.NodeID == LC_Broadcast_Address) || (rec.NodeID == nodeID)))
						return rec;
					else
						rec.Address = 0;    //no
				}
			} else
				rec.Address = 0;    //no
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
LC_Return_t LC_SendMessage(void *sender, LC_ObjectRecord_t *object, uint16_t index) {
	LC_NodeDescriptor_t *node = sender;
	if (node == 0)
		node = &own_nodes[0];
	if (node->State != LCNodeState_Online)
		return LC_NodeOffline;

	if (object == 0)
		return LC_ObjectError;
	char *dataAddr = object->Address;
//check that data is real
	if (dataAddr == 0 && object->Size != 0)
		return LC_DataError;
//extract pointer
	if (object->Attributes.Pointer)
		dataAddr = *(char**) dataAddr;
//negative size means this is string - any length
	if ((object->Attributes.TCP) || (object->Size > (8 / LEVCAN_MIN_BYTE_SIZE))
			|| ((object->Size < 0) && (strnlen(dataAddr, (8 / LEVCAN_MIN_BYTE_SIZE)) == (8 / LEVCAN_MIN_BYTE_SIZE)))) {
		//avoid dual same id
		objBuffered *txProceed = findObject((void*) objTXbuf_start, index, object->NodeID, node->ShortName.NodeID);
		if (txProceed) {
#ifdef DEBUG
			lc_collision_cntr++;
#endif
			return LC_Collision;
		}

		if (object->NodeID == node->ShortName.NodeID)
			return LC_Collision;
		//form message header
		LC_HeaderPacked_t hdr = { 0 };
		hdr.MsgID = index;    //our node index
		hdr.Priority = ~object->Attributes.Priority;
		hdr.Request = 0;    //data sending...
		hdr.Source = node->ShortName.NodeID;
		hdr.Target = object->NodeID;
		//create object sender instance
#ifndef LEVCAN_MEM_STATIC
		objBuffered *newTXobj = (objBuffered*) lcmalloc(sizeof(objBuffered));

#else
		//no cleanup for static mem!
		//todo make memcopy to data[] ?
		if (object->Attributes.Cleanup == 1)
			return LC_MallocFail;
		objBuffered *newTXobj = getFreeObject();
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

		LC_HeaderPacked_t hdr = { 0 };
		hdr.MsgID = index;
		hdr.Priority = ~object->Attributes.Priority;
		hdr.Request = 0;
		hdr.Parity = 0;
		hdr.RTS_CTS = 1;    //data start
		hdr.EoM = 1;    //data end
		hdr.Source = node->ShortName.NodeID;
		hdr.Target = object->NodeID;

		return lc_sendDataToQueue(hdr, data, size);
	}
	return LC_Ok;
}

LC_Return_t LC_SendRequest(void *sender, uint16_t target, uint16_t index) {
	return LC_SendRequestSpec(sender, target, index, 0, 0);
}

LC_Return_t LC_SendRequestSpec(void *sender, uint16_t target, uint16_t index, uint8_t size, uint8_t TCP) {

	LC_NodeDescriptor_t *node = sender;
	if (node == 0)
		node = &own_nodes[0];
	if (node->State != LCNodeState_Online)
		return LC_NodeOffline;

	LC_HeaderPacked_t hdr = { 0 };
	hdr.MsgID = index;
	hdr.Priority = 0;
	hdr.Request = 1;
	hdr.Parity = TCP;
	hdr.RTS_CTS = 0;
	hdr.Source = node->ShortName.NodeID;
	hdr.Target = target;

	return lc_sendDataToQueue(hdr, 0, size);
}

void LC_ReceiveManager(void) {
	msgBuffered rxBuffered;
#ifdef LEVCAN_USE_RTOS_QUEUE
	while (LC_QueueReceive(rxQueue, &rxBuffered, 100)) {
#else
	while (rxFIFO_in != rxFIFO_out) {
		rxBuffered = rxFIFO[rxFIFO_out];
		rxFIFO_out = (rxFIFO_out + 1) % LEVCAN_RX_SIZE;
#endif
		if (rxBuffered.header.Request) {
			if (rxBuffered.header.RTS_CTS == 0 && rxBuffered.header.EoM == 0) {
				//Remote transfer request, try to create new TX object
				LC_NodeDescriptor_t *node = findNode(rxBuffered.header.Target);
				LC_ObjectRecord_t obj = findObjectRecord(rxBuffered.header.MsgID, rxBuffered.length, node, Read, rxBuffered.header.Source);
				obj.NodeID = rxBuffered.header.Source;    //receiver
				if (obj.Attributes.Function && obj.Address) {
					//function call before sending
					//unpack header
					LC_Header_t unpack = LC_HeaderUnpack(rxBuffered.header);
					//call object
					((LC_FunctionCall_t) obj.Address)(node, unpack, 0, 0);
				} else {
					//check for existing objects, dual request denied
					//ToDo is this best way? maybe reset tx?
					objBuffered *txProceed = findObject((void*) objTXbuf_start, rxBuffered.header.MsgID, rxBuffered.header.Source, rxBuffered.header.Target);
					if (txProceed == 0) {
						obj.Attributes.TCP |= rxBuffered.header.Parity;    //force TCP mode if requested
						LC_SendMessage((intptr_t*) node, &obj, rxBuffered.header.MsgID);
					} else {
#ifdef LEVCAN_TRACE
						//trace_printf("RX dual request denied:%d, from node:%d \n", rxBuffered.header.MsgID, rxBuffered.header.Source);
#endif
					}
				}
			} else {
				//find existing TX object, tcp clear-to-send and end-of-msg-ack
				objBuffered *TXobj = findObject((void*) objTXbuf_start, rxBuffered.header.MsgID, rxBuffered.header.Source, rxBuffered.header.Target);
				if (TXobj)
					objectTXproceed(TXobj, &rxBuffered.header);
			}
		} else {
			//we got data
			if (rxBuffered.header.RTS_CTS) {
				//address valid?
				if (rxBuffered.header.Source >= LC_Null_Address) {
					//get next buffer index
					continue;
				}
				if (rxBuffered.header.EoM && rxBuffered.header.Parity == 0) {
					//fast receive for udp
					if (objectRXfinish(rxBuffered.header, (char*) &rxBuffered.data, rxBuffered.length, 0)) {
#ifdef LEVCAN_TRACE
						trace_printf("RX fast failed:%d \n", rxBuffered.header.MsgID);
#endif
					}
				} else {
					//find existing RX object, delete in case we get new RequestToSend
					objBuffered *RXobj = findObject((void*) objRXbuf_start, rxBuffered.header.MsgID, rxBuffered.header.Target, rxBuffered.header.Source);
					if (RXobj) {
						lcfree(RXobj->Pointer);
						deleteObject(RXobj, (void*) &objRXbuf_start, (void*) &objRXbuf_end);
					}
					//create new receive object
#ifndef LEVCAN_MEM_STATIC
					objBuffered *newRXobj = (objBuffered*) lcmalloc(sizeof(objBuffered));
#else
					objBuffered *newRXobj = getFreeObject();
#endif
					if (newRXobj == 0) {
						//get next buffer index
						continue;
					}
					//data alloc
#ifndef LEVCAN_MEM_STATIC
					newRXobj->Pointer = lcmalloc(LEVCAN_OBJECT_DATASIZE);
					if (newRXobj->Pointer == 0) {
						lcfree(newRXobj);
						continue;
					}
#endif
					newRXobj->Length = LEVCAN_OBJECT_DATASIZE;
					newRXobj->Header = rxBuffered.header;
					newRXobj->Flags.TCP = rxBuffered.header.Parity;    //setup rx mode
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
					objectRXproceed(newRXobj, &rxBuffered);
				}
			} else {
				//find existing RX object
				objBuffered *RXobj = findObject((void*) objRXbuf_start, rxBuffered.header.MsgID, rxBuffered.header.Target, rxBuffered.header.Source);
				if (RXobj)
					objectRXproceed(RXobj, &rxBuffered);
			}
		}
	}
}

void LC_TransmitManager(void) {
//fill TX buffer till no empty slots
//Some thread safeness
#ifdef LEVCAN_USE_RTOS_QUEUE
	msgBuffered msgTX;

//look for new messages
	while (LC_QueueReceive(txQueue, &msgTX, 100)) {
		//clear TX semaphore, maybe there is a lot of time passed alrdy
		LC_SemaphoreTake(txSemph, 0);
		//try to send data, hal uses true 8 bit - byte size
		if (LC_HAL_Send(msgTX.header, msgTX.data, msgTX.length * LEVCAN_MIN_BYTE_SIZE) != LC_Ok) {
			//send failed, TX full, need to wait
			LC_QueueSendToFront(txQueue, &msgTX, 1); //store item back
			//wait for CAN TX to be empty
			LC_SemaphoreTake(txSemph, 100);
		}
	}
#else
	static volatile uint32_t mutex = 0;
	if (mutex == 1)
		return;
	mutex = 1;
	while (1) {
		if (txFIFO_in == txFIFO_out)
			break; /* Queue Empty - nothing to send*/
		if (LC_HAL_Send(txFIFO[txFIFO_out].header, txFIFO[txFIFO_out].data, txFIFO[txFIFO_out].length * LEVCAN_MIN_BYTE_SIZE) != LC_Ok)
			break; //CAN full
		txFIFO_out = (txFIFO_out + 1) % LEVCAN_TX_SIZE;
	}
	mutex = 0;
#endif

}

void LC_TransmitHandler(void) {
#ifdef LEVCAN_USE_RTOS_QUEUE
	YieldNeeded_t yield = 0;
	LC_SemaphoreGiveISR(txSemph, &yield);
	LC_RTOSYieldISR(yield);
#else
	LC_TransmitManager();
#endif
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
LC_NodeShortName_t LC_GetActiveNodes(uint16_t *last_pos) {
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
LC_NodeShortName_t LC_GetMyNodeName(void *mynode) {
	LC_NodeDescriptor_t *node = mynode;
	if (node == 0)
		node = &own_nodes[0];
	return node->ShortName;
}

int16_t LC_GetMyNodeIndex(void *mynode) {
	if (mynode == 0)
		return 0;
	int16_t index = (own_nodes - (LC_NodeDescriptor_t*) mynode);
	if (index >= 0 && index < LEVCAN_MAX_OWN_NODES)
		return index;
	return -1;
}
