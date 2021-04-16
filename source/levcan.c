/*******************************************************************************
 * LEVCAN: Light Electric Vehicle CAN protocol [LC]
 * Copyright (C) 2020 Vasiliy Sukhoparov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include "levcan.h"
#include "levcan_internal.h"
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
			uint8_t ToDelete :1;
		} Flags;
		uint8_t FlagsTotal;
	};
} objBuffered;
#define toDeleteMark (1<<2)

enum {
	Read, Write
};

//#### PRIVATE VARIABLES ####
#ifdef DEBUG
volatile uint32_t lc_collision_cntr = 0;
volatile uint32_t lc_receive_ovfl_cntr = 0;
#endif
//#### PRIVATE FUNCTIONS ####
LC_ObjectRecord_t findObjectRecord(LC_NodeDescriptor_t *node, uint16_t messageID, int32_t size, uint8_t read_write, uint8_t nodeID);

LC_Return_t lc_sendDataToQueue(LC_NodeDescriptor_t *node, LC_HeaderPacked_t hdr, uint32_t data[], uint8_t length);

uint16_t objectRXproceed(LC_NodeDescriptor_t *node, objBuffered *object, msgBuffered *msg);
uint16_t objectTXproceed(LC_NodeDescriptor_t *node, objBuffered *object, LC_HeaderPacked_t *request);
LC_Return_t objectRXfinish(LC_NodeDescriptor_t *node, LC_HeaderPacked_t header, char *data, int32_t size, uint8_t memfree);
void deleteObject(objBuffered *obj, objBuffered **start, objBuffered **end);
uint8_t hashMultiplicative(const uint8_t *input, uint8_t len, uint8_t start);
#ifdef LEVCAN_STATIC_MEM
objBuffered* getFreeObject(void);
void releaseObject(objBuffered *obj);
#endif

#ifndef LEVCAN_NO_TX_QUEUE
int32_t getTXqueueSize(LC_NodeDescriptor_t *node);
#endif
objBuffered* findObject(objBuffered *array, uint16_t msgID, uint8_t target, uint8_t source);

//#### EXTERNAL MODULES ####
extern LC_Return_t lc_sendDiscoveryRequest(LC_NodeDescriptor_t *node, uint16_t target);
//#### FUNCTIONS

LC_Return_t LC_InitNodeDescriptor(LC_NodeDescriptor_t *node) {
	//clean up
	//memset(node->SystemObjects, 0, sizeof(node->SystemObjects));
	memset(node, 0, sizeof(LC_NodeDescriptor_t));
	node->State = LCNodeState_Disabled;
	node->ShortName.CodePage = 1250; //Central European
	node->ShortName.NodeID = LC_Broadcast_Address;
	node->LastID = LC_Broadcast_Address;

#ifndef LEVCAN_STATIC_MEM
	//private variables for specific usage
	node->Extensions = lcmalloc(sizeof(lc_Extensions_t));
	//can be sizeof 0
	if (sizeof(lc_Extensions_t) > 0 && node->Extensions == 0) {
		return LC_MallocFail;
	} else {
		memset(node->Extensions, 0, sizeof(lc_Extensions_t));
	}
	//node table can be pre-defined (global)
	node->NodeTable = lcmalloc(sizeof(LC_NodeTable_t));
	if (node->NodeTable == 0)
		return LC_MallocFail;

	node->NodeTable->Table = lcmalloc(sizeof(LC_NodeTableEntry_t[LEVCAN_MAX_TABLE_NODES]));
	//can be sizeof 0
	if (sizeof(LC_NodeTableEntry_t[LEVCAN_MAX_TABLE_NODES]) > 0 && node->NodeTable->Table == 0)
		return LC_MallocFail;
	int i = 0;
	for (i = 0; i < LEVCAN_MAX_TABLE_NODES; i++)
		node->NodeTable->Table[i].ShortName.NodeID = LC_Broadcast_Address;
	//node->NodeTable->FreeSlots = LEVCAN_MAX_TABLE_NODES;
	node->NodeTable->TableSize = LEVCAN_MAX_TABLE_NODES;

#endif // !LEVCAN_STATIC_MEM

	return LC_Ok;
}

LC_Return_t LC_CreateNode(LC_NodeDescriptor_t *node) {
	if (node == 0 || node->Driver == 0)
		return LC_DataError;
	if (node->ShortName.NodeID > 125) {
		uint8_t hashed = hashMultiplicative((void*) &node->ShortName.ToUint32[0], 8, 0);
		hashed = hashMultiplicative((void*) &node->Serial, sizeof(node->Serial), 0);
		node->ShortName.NodeID = hashed % 64;
	}
	if (node->NodeName != 0 && strnlen(node->NodeName, 128) == 128)
		node->NodeName = 0;    //too long name
	if (node->DeviceName != 0 && strnlen(node->DeviceName, 128) == 128)
		node->DeviceName = 0;
	if (node->VendorName != 0 && strnlen(node->VendorName, 128) == 128)
		node->VendorName = 0;

#ifdef LEVCAN_USE_RTOS_QUEUE
	node->TxRxObjects.rxQueue = LC_QueueCreate(LEVCAN_RX_SIZE, sizeof(msgBuffered));
#ifndef LEVCAN_NO_TX_QUEUE
	node->TxRxObjects.txQueue = LC_QueueCreate(LEVCAN_TX_SIZE, sizeof(msgBuffered));
	node->TxRxObjects.txSemph = LC_SemaphoreCreate();

	if (node->TxRxObjects.rxQueue == NULL || node->TxRxObjects.txQueue == NULL || node->TxRxObjects.txSemph == NULL) {
		LC_SemaphoreDelete(node->TxRxObjects.txSemph);
		LC_QueueDelete(node->TxRxObjects.txQueue);
		LC_QueueDelete(node->TxRxObjects.rxQueue);
		return LC_MallocFail;
	}
#else
	if (node->TxRxObjects.rxQueue == NULL) {
		//LC_QueueDelete(node->TxRxObjects.rxQueue);
		return LC_MallocFail;
	}
#endif
#endif

	node->State = LCNodeState_Disabled;
	node->LastID = node->ShortName.NodeID;
	//now setup system calls
	//adress claim, main network function
	LC_Object_t *initObject;	// = lc_registerSystemObjects(node, 1);
	if (LC_AddressInit(node))
		return LC_InitError;

	if (node->NodeName) {
		initObject = lc_registerSystemObjects(node, 1);
		if (initObject == 0)
			return LC_InitError;
		initObject->Address = (void*) node->NodeName;
		initObject->Attributes.Readable = 1;
		initObject->MsgID = LC_SYS_NodeName;
		initObject->Size = strlen(node->NodeName) + 1;    // plus zero byte
	}
	if (node->DeviceName) {
		initObject = lc_registerSystemObjects(node, 1);
		if (initObject == 0)
			return LC_InitError;
		initObject->Address = (void*) node->DeviceName;
		initObject->Attributes.Readable = 1;
		initObject->MsgID = LC_SYS_DeviceName;
		initObject->Size = strlen(node->DeviceName) + 1;
	}
	if (node->VendorName) {
		initObject = lc_registerSystemObjects(node, 1);
		if (initObject == 0)
			return LC_InitError;
		initObject->Address = (void*) node->VendorName;
		initObject->Attributes.Readable = 1;
		initObject->MsgID = LC_SYS_VendorName;
		initObject->Size = strlen(node->VendorName) + 1;
	}
	//SN
	initObject = lc_registerSystemObjects(node, 1);
	if (initObject == 0)
		return LC_InitError;
	initObject->Address = &node->Serial;
	initObject->Attributes.Readable = 1;
	initObject->MsgID = LC_SYS_SerialNumber;
	initObject->Size = sizeof(node->Serial);

	//begin network discovery for start
	node->LastTXtime = 0;
	LC_ConfigureFilters(node);
	lc_sendDiscoveryRequest(node, LC_Broadcast_Address);
	node->State = LCNodeState_NetworkDiscovery;
	return LC_Ok;
}

LC_Object_t* lc_registerSystemObjects(LC_NodeDescriptor_t *node, uint8_t count) {
	if (node == 0) {
		return 0;
	}
	if (node->SystemSize + count > LEVCAN_SYS_OBJ_SIZ) {
		//?? cant fit any more objects!
		return 0;
	}
	LC_Object_t *obj = &node->SystemObjects[node->SystemSize];
	node->SystemSize += count;

	return obj;
}
/*
 void initialize(void) {
 static uint16_t startup = 0;
 if (startup)
 return;

 startup = 1;
 #ifndef LEVCAN_USE_RTOS_QUEUE
 rxFIFO_in = 0;
 rxFIFO_out = 0;
 memset(rxFIFO, 0, sizeof(rxFIFO));
 #ifndef LEVCAN_NO_TX_QUEUE
 txFIFO_in = 0;
 txFIFO_out = 0;
 memset(txFIFO, 0, sizeof(txFIFO));
 #endif
 #endif
 int i = 0;
 #if (LEVCAN_MAX_TABLE_NODES) > 1
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
 }*/

#ifndef LEVCAN_NO_TX_QUEUE
int32_t getTXqueueSize(LC_NodeDescriptor_t *node) {
#ifndef LEVCAN_USE_RTOS_QUEUE
	if (txFIFO_in >= txFIFO_out)
		return txFIFO_in - txFIFO_out;
	else
		return txFIFO_in + (LEVCAN_TX_SIZE - txFIFO_out);
#else
	return LC_QueueStored(node->TxRxObjects.txQueue);
#endif
}
#endif

objBuffered* findObject(objBuffered *array, uint16_t msgID, uint8_t target, uint8_t source) {
	objBuffered *obj = array;
	while (obj) {
		//same source and same ID ?
		//one ID&source can send only one message length a time
		if (obj->Header.MsgID == msgID && obj->Header.Target == target && obj->Header.Source == source && obj->FlagsTotal < toDeleteMark) {
			return obj;
		}
		obj = (objBuffered*) obj->Next;
	}
	return 0;
}

uint8_t hashMultiplicative(const uint8_t *input, uint8_t len, uint8_t start) {
	uint8_t hash = start;
	for (uint8_t i = 0; i < len; ++i)
		hash = (uint8_t) 31 * hash + input[i];
	return hash;
}

void LC_ReceiveHandler(LC_NodeDescriptor_t *node, LC_HeaderPacked_t header, uint32_t *data, uint8_t length) {

#ifdef LEVCAN_USE_RTOS_QUEUE
	YieldNeeded_t yield = 0;
	//packing
	msgBuffered msgRX;
	msgRX.data[0] = data[0];
	msgRX.data[1] = data[1];
	msgRX.length = (length + LEVCAN_MIN_BYTE_SIZE - 1) / LEVCAN_MIN_BYTE_SIZE;
	msgRX.header = header;
	//add to queue
	LC_QueueSendToBackISR(node->TxRxObjects.rxQueue, &msgRX, &yield);

	LC_RTOSYieldISR(yield);
#else

	if (rxFIFO_in == ((rxFIFO_out - 1 + LEVCAN_RX_SIZE) % LEVCAN_RX_SIZE)) {
#ifdef DEBUG
		lc_receive_ovfl_cntr++;
#endif
		return;
	}
	//store in rx buffer
	msgBuffered *msgRX = &rxFIFO[rxFIFO_in]; //less size in O2 with pointer?! maybe volatile problem
	msgRX->data[0] = data[0];
	msgRX->data[1] = data[1];
	msgRX->length = (length + LEVCAN_MIN_BYTE_SIZE - 1) / LEVCAN_MIN_BYTE_SIZE;
	msgRX->header = header;
	rxFIFO_in = (rxFIFO_in + 1) % LEVCAN_RX_SIZE;

#endif
}

void LC_NetworkManager(LC_NodeDescriptor_t *node, uint32_t time) {
	LC_AddressManager(node, time);
	//count work time and clean up
	objBuffered *txProceed = (objBuffered*) node->TxRxObjects.objTXbuf_start;
	while (txProceed) {
		objBuffered *next = (objBuffered*) txProceed->Next;
		txProceed->Time_since_comm += time;

		//global timeout
		if (txProceed->Time_since_comm > LEVCAN_MESSAGE_TIMEOUT) {
#ifdef LEVCAN_TRACE
			trace_printf("TX object deleted by timeout:%d\n", txProceed->Header.MsgID);
#endif
			txProceed->Flags.ToDelete = 1;
		}
		if (txProceed->FlagsTotal >= toDeleteMark) {
			//garbage collector
			if (txProceed->Flags.TXcleanup) {
				lcfree(txProceed->Pointer);
			}
			txProceed->Pointer = 0;
			deleteObject(txProceed, (objBuffered**) &node->TxRxObjects.objTXbuf_start, (objBuffered**) &node->TxRxObjects.objTXbuf_end);
		} else if (txProceed->Flags.TCP == 0) {
			//UDP mode send data continuously
			objectTXproceed(node, txProceed, 0);
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
					deleteObject(txProceed, (objBuffered**) &node->TxRxObjects.objTXbuf_start, (objBuffered**) &node->TxRxObjects.objTXbuf_end);
				} else {
					// Try tx again
					objectTXproceed(node, txProceed, 0);
					//TOdo may cause buffer overflow if CAN is offline
					if (txProceed->Time_since_comm == 0)
						txProceed->Attempt++;
				}
			}
		}
		txProceed = next;
	}
	//count work time and recall
	objBuffered *rxProceed = (objBuffered*) node->TxRxObjects.objRXbuf_start;
	while (rxProceed) {
		objBuffered *next = (objBuffered*) rxProceed->Next;
		rxProceed->Time_since_comm += time;
		if ((rxProceed->Time_since_comm > LEVCAN_MESSAGE_TIMEOUT) || (rxProceed->FlagsTotal >= toDeleteMark)) {
			rxProceed->Flags.ToDelete = 1; //critical
			//UDP mode rx timeout or garbage collector
#ifdef LEVCAN_TRACE
			if (!(rxProceed->FlagsTotal >= toDeleteMark)) {
				trace_printf("RX object deleted by timeout:%d\n", rxProceed->Header.MsgID);
			}
#endif
			lcfree(rxProceed->Pointer);
			rxProceed->Pointer = 0;
			deleteObject(rxProceed, (objBuffered**) &node->TxRxObjects.objRXbuf_start, (objBuffered**) &node->TxRxObjects.objRXbuf_end);
		}
		rxProceed = next;
	}

}

void deleteObject(objBuffered *obj, objBuffered **start, objBuffered **end) {
	lc_disable_irq();
	//critical area
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
	}
	else {
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

LC_Return_t lc_sendDataToQueue(LC_NodeDescriptor_t *node, LC_HeaderPacked_t hdr, uint32_t data[], uint8_t length) {

#ifndef	LEVCAN_NO_TX_QUEUE
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
	LC_QueueSendToBack(node->TxRxObjects.txQueue, &msgTX, 0);
	//LC_SemaphoreGive(node->TxRxObjects.txSemph);
#else
	txFIFO[txFIFO_in] = msgTX;
	txFIFO_in = (txFIFO_in + 1) % LEVCAN_TX_SIZE;
	lc_enable_irq();
#endif
	return LC_Ok;
#else
	//direct data send
	return ((LC_DriverCalls_t*) node->Driver)->Send(hdr, data, length);
#endif
	//proceed queue if we can do, NOT THREAD SAFE
	//	if (empty)
	//		LC_TransmitHandler();
}

uint16_t objectTXproceed(LC_NodeDescriptor_t *node, objBuffered *object, LC_HeaderPacked_t *request) {
	int32_t length;
	uint32_t data[2];
	uint32_t step_inc = (8 / LEVCAN_MIN_BYTE_SIZE);
	uint8_t parity = ~((object->Position + step_inc - 1) / step_inc) & 1;    //parity
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
			object->Pointer = 0;
			object->FlagsTotal = toDeleteMark;
			//deleteObject(object, (objBuffered**) &objTXbuf_start, (objBuffered**) &objTXbuf_end);
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
		//if (length < 0)
		//	asm volatile ("bkpt");
		//Extract new portion of data in obj. null length cant be
		memcpy(data, &object->Pointer[object->Position], length);
		if (object->Position == 0) {
			//Request new buffer anyway. maybe there was wrong request while data wasn't sent at all?
			newhdr.RTS_CTS = 1;
		} else
			newhdr.RTS_CTS = 0;

		newhdr.Parity = object->Flags.TCP ? parity : 0;    //parity
		//try to send
		if (lc_sendDataToQueue(node, newhdr, data, length))
			return 1;
		//increment if sent succesful
		object->Position += length;
		object->Header = newhdr;    //update to new only here
		object->Time_since_comm = 0;    //data sent
		//cycle if this is UDP till message end or buffer 3/4 fill
	} while ((object->Flags.TCP == 0)
#ifndef LEVCAN_NO_TX_QUEUE
			&& (getTXqueueSize(node) * 4 < LEVCAN_TX_SIZE * 3)
#endif
			&& (object->Header.EoM == 0));
	//in UDP mode delete object when EoM is set
	if ((object->Flags.TCP == 0) && (object->Header.EoM == 1)) {
		if (object->Flags.TXcleanup) {
			lcfree(object->Pointer);
		}
		object->Pointer = 0;
		object->Flags.ToDelete = 1;
		//deleteObject(object, (objBuffered**) &objTXbuf_start, (objBuffered**) &objTXbuf_end);
	}
	return 0;
}

uint16_t objectRXproceed(LC_NodeDescriptor_t *node, objBuffered *object, msgBuffered *msg) {
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
			object->Flags.ToDelete = 1;			//deleteObject(object, (objBuffered**)&objRXbuf_start, (objBuffered**)&objRXbuf_end);
			return 0;
#endif
		}
#ifndef LEVCAN_MEM_STATIC
		if (object->Pointer) {
			memcpy(&object->Pointer[object->Position], msg->data, msg->length);
		}
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
		lc_sendDataToQueue(node, hdr, 0, 0);
	}
	if (object->Header.EoM) {
#ifndef LEVCAN_MEM_STATIC
		objectRXfinish(node, object->Header, object->Pointer, object->Position, 1);
#else
		objectRXfinish(node, object->Header, object->Data, object->Position, 0);
#endif
		//pointer by this time should be cleared or stored in a queue
		object->Pointer = 0;
		//delete object from memory chain, find new endings
		object->FlagsTotal = toDeleteMark;
		//deleteObject(object, (objBuffered**) &objRXbuf_start, (objBuffered**) &objRXbuf_end);
	}

	return 0;
}

LC_Return_t objectRXfinish(LC_NodeDescriptor_t *node, LC_HeaderPacked_t header, char *data, int32_t size, uint8_t memfree) {
	LC_Return_t ret = LC_Ok;
	//check check and check again
	LC_ObjectRecord_t obj = findObjectRecord(node, header.MsgID, size, Write, header.Source);
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
	if (memfree) {
		lcfree(data);
	}
	return ret;
}

LC_ObjectRecord_t findObjectRecord(LC_NodeDescriptor_t *node, uint16_t messageID, int32_t size, uint8_t read_write, uint8_t nodeID) {
	LC_ObjectRecord_t recBuffer = { 0 };
	LC_ObjectRecord_t *record;
	if (node == 0)
		return recBuffer;
	for (int source = 0; source < 2; source++) {
		int32_t objsize;
		LC_Object_t *objectArray;
		//switch between system objects and external
		if (source) {
			objsize = node->ObjectsSize;
			objectArray = node->Objects;
		} else {
			objsize = node->SystemSize;
			objectArray = node->SystemObjects;
		}
		//if system object not found, search in external
		for (int i = 0; i < objsize; i++) {
			//right index?
			if (objectArray[i].MsgID == messageID) {
				int recSize = 0;
				//extract pointer, size and attributes
				if (objectArray[i].Attributes.Record) {
					if (objectArray[i].Size > 0 && objectArray[i].Address != 0) {
						//get record pointer if its correct
						record = ((LC_ObjectRecord_t*) objectArray[i].Address);
						recBuffer.Address = record->Address;
						recBuffer.Size = record->Size;
						recBuffer.Attributes = record->Attributes;
						recSize = objectArray[i].Size; //record array size
					} else {
						continue; //bad object, skip
					}
				} else {
					//convert object to record type
					recBuffer.Address = objectArray[i].Address;
					recBuffer.Size = objectArray[i].Size;
					recBuffer.Attributes = objectArray[i].Attributes;
					recBuffer.NodeID = LC_Broadcast_Address;
					record = &recBuffer;
					recSize = 1; //1 element
				}
				//scroll through all LC_ObjectRecord_t[]
				for (int irec = 0; irec < recSize; irec++) {
					if (
					//@formatter:off
						//Group A
						((size == record[irec].Size) || 							//precise size match (positive only)
						((record[irec].Size < 0) && (-size >= record[irec].Size)) ||//variable size (up to -Size, where Size is negative) Note: INT_MIN == -INT_MIN, comparison will be broken
							(read_write == Read && size == 0)) &&  						//read (request) any first object if request size is 0
							//Group B
							((record[irec].Attributes.Readable != read_write) || 		//read access (should be .Readable=1)
						(record[irec].Attributes.Writable == read_write)) && 		//write access (should be .Writable=1)
						//Group C
								((record[irec].NodeID == LC_Broadcast_Address) || 			//any match == broadcast
						(record[irec].NodeID == nodeID))) 							//specific node ID match
											//@formatter:on
					{
						return record[irec];    //yes

					} else {
						recBuffer.Address = 0;    //no
					}
				}
			} else {
				recBuffer.Address = 0;    //no
			}
		}
	}
	return recBuffer;
}

/// Sends LC_ObjectRecord_t to network
/// @param sender
/// @param object
/// @param target
/// @param index
/// @return
LC_Return_t LC_SendMessage(LC_NodeDescriptor_t *node, LC_ObjectRecord_t *object, uint16_t index) {
	uint32_t strl = 0;

	if (node == 0)
		return LC_DataError;
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

	if (object->Size < 0)
		strl = strnlen(dataAddr, (8 / LEVCAN_MIN_BYTE_SIZE));
	//negative size means this is string - any length

	if ((object->Attributes.TCP) || (object->Size > (8 / LEVCAN_MIN_BYTE_SIZE)) || ((object->Size < 0) && (strl == (8 / LEVCAN_MIN_BYTE_SIZE)))) {
		//avoid dual same id
		objBuffered *txProceed = findObject((objBuffered*) node->TxRxObjects.objTXbuf_start, index, object->NodeID, node->ShortName.NodeID);
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
		newTXobj->FlagsTotal = 0;
		newTXobj->Flags.TCP = object->Attributes.TCP;
		newTXobj->Flags.TXcleanup = object->Attributes.Cleanup;
		//process it first to avoid collision in multithread
		objectTXproceed(node, newTXobj, 0);
		//add to queue, critical section
		lc_disable_irq();
		if ((volatile objBuffered*) node->TxRxObjects.objTXbuf_start == 0) {
			//no objects in tx array
			newTXobj->Previous = 0;
			node->TxRxObjects.objTXbuf_start = newTXobj;
			node->TxRxObjects.objTXbuf_end = newTXobj;
		} else {
			//add to the end
			newTXobj->Previous = (intptr_t*) node->TxRxObjects.objTXbuf_end;
			((objBuffered*) node->TxRxObjects.objTXbuf_end)->Next = (intptr_t*) newTXobj;
			node->TxRxObjects.objTXbuf_end = (void*) newTXobj;
		}
		lc_enable_irq();
#ifdef LEVCAN_TRACE
		//trace_printf("New TX object created:%d\n", newTXobj->Header.MsgID);
#endif
	} else {
		//some short string? + ending
		int32_t size = object->Size;
		if (size < 0)
			size = strl + 1;

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

		return lc_sendDataToQueue(node, hdr, data, size);
	}
	return LC_Ok;
}

LC_Return_t LC_SendRequest(LC_NodeDescriptor_t *node, uint16_t target, uint16_t index) {
	return LC_SendRequestSpec(node, target, index, 0, 0);
}

LC_Return_t LC_SendRequestSpec(LC_NodeDescriptor_t *node, uint16_t target, uint16_t index, uint8_t size, uint8_t TCP) {
	if (node == 0)
		return LC_DataError;
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

	return lc_sendDataToQueue(node, hdr, 0, size);
}

void LC_ReceiveManager(LC_NodeDescriptor_t *node) {

	msgBuffered rxBuffered;
#ifdef LEVCAN_USE_RTOS_QUEUE
	while (LC_QueueReceive(node->TxRxObjects.rxQueue, &rxBuffered, 100)) {
#else
	while (rxFIFO_in != rxFIFO_out) {
		rxBuffered = rxFIFO[rxFIFO_out];
		rxFIFO_out = (rxFIFO_out + 1) % LEVCAN_RX_SIZE;
#endif
		if (rxBuffered.header.Request) {
			if (rxBuffered.header.RTS_CTS == 0 && rxBuffered.header.EoM == 0) {
				//Remote transfer request, try to create new TX object
				LC_ObjectRecord_t obj = findObjectRecord(node, rxBuffered.header.MsgID, rxBuffered.length, Read, rxBuffered.header.Source);
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
					objBuffered *txProceed = findObject((void*) node->TxRxObjects.objTXbuf_start, rxBuffered.header.MsgID, rxBuffered.header.Source,
							rxBuffered.header.Target);
					if (txProceed == 0) {
						obj.Attributes.TCP |= rxBuffered.header.Parity;    //force TCP mode if requested
						LC_SendMessage(node, &obj, rxBuffered.header.MsgID);
					} else {
#ifdef LEVCAN_TRACE
						//trace_printf("RX dual request denied:%d, from node:%d \n", rxBuffered.header.MsgID, rxBuffered.header.Source);
#endif
#ifdef __GNUC__
						asm("nop");
#else
						__nop();
#endif
					}
				}
			} else {
				//find existing TX object, tcp clear-to-send and end-of-msg-ack
				objBuffered *TXobj = findObject((void*) node->TxRxObjects.objTXbuf_start, rxBuffered.header.MsgID, rxBuffered.header.Source,
						rxBuffered.header.Target);
				if (TXobj) {
					objectTXproceed(node, TXobj, &rxBuffered.header);
				}
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
					if (objectRXfinish(node, rxBuffered.header, (char*) &rxBuffered.data, rxBuffered.length, 0)) {
#ifdef LEVCAN_TRACE
						trace_printf("RX fast failed:%d \n", rxBuffered.header.MsgID);
#endif
					}
				} else {
					//find existing RX object, delete in case we get new RequestToSend
					objBuffered *RXobj = findObject((objBuffered*) node->TxRxObjects.objRXbuf_start, rxBuffered.header.MsgID, rxBuffered.header.Target,
							rxBuffered.header.Source);
					if (RXobj) {
						RXobj->FlagsTotal = toDeleteMark; //garbage collector mark
						//lcfree(RXobj->Pointer);
						//deleteObject(RXobj, (void*) &objRXbuf_start, (void*) &objRXbuf_end);
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
					newRXobj->FlagsTotal = 0;
					newRXobj->Flags.TCP = rxBuffered.header.Parity;    //setup rx mode
					newRXobj->Position = 0;
					newRXobj->Attempt = 0;
					newRXobj->Time_since_comm = 0;
					newRXobj->Next = 0;
					newRXobj->Previous = 0;
					//for future-proof anti-collision, processing first
					objectRXproceed(node, newRXobj, &rxBuffered);

					lc_disable_irq();
					//critical add to end
					if (node->TxRxObjects.objRXbuf_start == 0) {
						//no objects in rx array
						node->TxRxObjects.objRXbuf_start = newRXobj;
						node->TxRxObjects.objRXbuf_end = newRXobj;
					} else {
						//add to the end
						newRXobj->Previous = (intptr_t*) node->TxRxObjects.objRXbuf_end;
						((objBuffered*) node->TxRxObjects.objRXbuf_end)->Next = (intptr_t*) newRXobj;
						node->TxRxObjects.objRXbuf_end = newRXobj;
					}
					lc_enable_irq();
					//	trace_printf("New RX object created:%d\n", newRXobj->Header.MsgID);
				}
			} else {
				//find existing RX object
				objBuffered *RXobj = findObject((void*) node->TxRxObjects.objRXbuf_start, rxBuffered.header.MsgID, rxBuffered.header.Target,
						rxBuffered.header.Source);
				if (RXobj)
					objectRXproceed(node, RXobj, &rxBuffered);
			}
		}
	}
}
#ifndef LEVCAN_NO_TX_QUEUE
void LC_TransmitManager(LC_NodeDescriptor_t *node) {
	//fill TX buffer till no empty slots
	//Some thread safeness
#ifdef LEVCAN_USE_RTOS_QUEUE
	msgBuffered msgTX;

	//look for new messages
	while (LC_QueueReceive(node->TxRxObjects.txQueue, &msgTX, 100)) {
		//clear TX semaphore, maybe there is a lot of time passed alrdy
		LC_SemaphoreTake(node->TxRxObjects.txSemph, 0);
		//try to send data, hal uses true 8 bit - byte size
		if (((LC_DriverCalls_t*) node->Driver)->Send(msgTX.header, msgTX.data, msgTX.length * LEVCAN_MIN_BYTE_SIZE) != LC_Ok) {
			//send failed, TX full, need to wait
			LC_QueueSendToFront(node->TxRxObjects.txQueue, &msgTX, 1); //store item back
			//wait for CAN TX to be empty
			LC_SemaphoreTake(node->TxRxObjects.txSemph, 100);
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
		if (((LC_DriverCalls_t*) node->Driver)->Send(txFIFO[txFIFO_out].header, txFIFO[txFIFO_out].data, txFIFO[txFIFO_out].length * LEVCAN_MIN_BYTE_SIZE) != LC_Ok)
			break; //CAN full
		txFIFO_out = (txFIFO_out + 1) % LEVCAN_TX_SIZE;
	}
	mutex = 0;
#endif

}

void LC_TransmitHandler(LC_NodeDescriptor_t *node) {
#ifdef LEVCAN_USE_RTOS_QUEUE
	YieldNeeded_t yield = 0;
	LC_SemaphoreGiveISR(node->TxRxObjects.txSemph, &yield);
	LC_RTOSYieldISR(yield);
#else
	LC_TransmitManager();
#endif
}
#endif

LC_NodeShortName_t LC_GetNode(LC_NodeDescriptor_t *node, uint16_t nodeID) {
	int i = 0;

	if (node != 0 && nodeID < LC_Null_Address) {
		LC_NodeTableEntry_t *node_table = node->NodeTable->Table;
		//search
		for (; i < LEVCAN_MAX_TABLE_NODES; i++) {
			if (node_table[i].ShortName.NodeID == nodeID) {
				return node_table[i].ShortName;
			}
		}
	}
	LC_NodeShortName_t ret = (LC_NodeShortName_t ) { .NodeID = LC_Broadcast_Address };
	return ret;
}

int16_t LC_GetNodeIndex(LC_NodeDescriptor_t *node, uint16_t nodeID) {
	if (node == 0 || nodeID >= LC_Null_Address)
		return -1;
	LC_NodeTableEntry_t *node_table = node->NodeTable->Table;
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
LC_NodeShortName_t LC_GetActiveNodes(LC_NodeDescriptor_t *node, uint16_t *last_pos) {
	int i = *last_pos;
	//new run
	if (node == 0 || *last_pos >= LEVCAN_MAX_TABLE_NODES)
		i = 0;
	LC_NodeTableEntry_t *node_table = node->NodeTable->Table;
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
LC_NodeShortName_t LC_GetMyNodeName(LC_NodeDescriptor_t *node) {
	if (node == 0)
		return (LC_NodeShortName_t ) { 0 } ;
			return node->ShortName;
		}
