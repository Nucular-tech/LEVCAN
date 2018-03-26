/*
 * LEV-CAN: Light Electric Vehicle CAN protocol [LC]
 * levcan_param.c
 *
 *  Created on: 17 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#include "levcan.h"
#include "string.h"
#include "stdlib.h"
#include "can_hal.h"

typedef union {
	uint32_t ToUint32;
	struct {
		//can specific:
		unsigned reserved1 :1;
		unsigned Request :1;
		unsigned reserved2 :1;
		//index 29bit:
		unsigned Source :7;
		unsigned Target :7;
		unsigned MsgID :10;
		unsigned EoM :1;
		unsigned Parity :1;
		unsigned RTS_CTS :1;
		unsigned Priority :2;
	}__attribute__((packed));
} headerPacked_t;

typedef struct {
	headerPacked_t header;
	uint32_t data[2];
	uint8_t length;
} msgBuffered;

typedef struct {
	char* Pointer;
	int32_t Length;
	int32_t Position; //get parity - divde by 8 and &1
	headerPacked_t Header;
	uint16_t Time_since_comm;
	uint8_t Attempt;
	struct {
		unsigned TCP :1;
		unsigned TXcleanup :1;
	} Flags;
	intptr_t* Next;
	intptr_t* Previous;
} objBuffered;

enum {
	Read, Write
};

extern void *lcmalloc(uint32_t size);
extern void lcfree(void *pointer);
extern LC_ObjectRecord_t proceedParam(LC_NodeDescription_t* node, LC_Header header, void* data, int32_t size);
//#### PRIVATE VARIABLES ####
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
int16_t compareNode(LC_NodeShortName a, LC_NodeShortName b);
LC_NodeDescription_t* findNode(uint16_t nodeID);
LC_Object_t* findObject(uint16_t index, int32_t size, LC_NodeDescription_t* node);
LC_ObjectRecord_t findObjectRecord(uint16_t index, int32_t size, LC_NodeDescription_t* node, uint8_t read_write, uint8_t nodeID);
headerPacked_t headerPack(LC_Header header);
LC_Header headerUnpack(headerPacked_t header);
void claimFreeID(LC_NodeDescription_t* node);
uint16_t sendDataToQueue(headerPacked_t hdr, uint32_t data[], uint8_t length);
uint16_t objectRXproceed(objBuffered* object, msgBuffered* msg);
uint16_t objectTXproceed(objBuffered* object, headerPacked_t* request);
void deleteObject(objBuffered* obj, objBuffered** start, objBuffered** end);
int32_t getTXqueueSize(void);
uint16_t searchIndexCollision(uint16_t nodeID, LC_NodeDescription_t* ownNode);
//#### FUNCTIONS
uintptr_t* LC_CreateNode(LC_NodeInit_t node) {
	initialize();
	//TODO add id network check on creation
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

	LC_NodeDescription_t descr;
	LC_NodeShortName sname;
	sname.Configurable = node.Configurable;
	sname.DeviceType = node.DeviceType;
	sname.ManufacturerCode = node.VendorCode;
	sname.Events = node.Notifications;
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
	if (own_nodes[i].NodeName) {
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = own_nodes[i].NodeName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_NodeName;
		objparam->Size = strlen(own_nodes[i].NodeName);
	}
	if (own_nodes[i].DeviceName) {
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = own_nodes[i].DeviceName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_DeviceName;
		objparam->Size = strlen(own_nodes[i].DeviceName);
	}
	if (own_nodes[i].VendorName) {
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = own_nodes[i].VendorName;
		objparam->Attributes.Readable = 1;
		objparam->Index = LC_SYS_VendorName;
		objparam->Size = strlen(own_nodes[i].VendorName);
	}
	//SN
	objparam = &own_nodes[i].SystemObjects[sysinx++];
	objparam->Address = &own_nodes[i].Serial;
	objparam->Attributes.Readable = 1;
	objparam->Index = LC_SYS_SerialNumber;
	objparam->Size = sizeof(own_nodes[i].Serial);

	if (own_nodes[i].ShortName.Configurable) {
		//parameter editor
		objparam = &own_nodes[i].SystemObjects[sysinx++];
		objparam->Address = proceedParam;
		objparam->Attributes.Writable = 1;
		objparam->Attributes.Function = 1;
		objparam->Attributes.TCP = 1;
		objparam->Index = LC_SYS_Parameters;
		objparam->Size = -1;
	}

	//begin network discovery for start
	own_nodes[i].LastTXtime = 0;
	/*	CAN_FilterEditOn();
	 addAddressFilter(node.NodeID);
	 CAN_FilterEditOff();
	 own_nodes[i].State = LCNodeState_WaitingClaim;
	 LC_AddressClaimHandler(sname, LC_TX);*/
	LC_SendDiscoveryRequest(LC_Broadcast_Address);
	own_nodes[i].State = LCNodeState_NetworkDiscovery;
	return (uintptr_t*) &own_nodes[i];
}

void initialize(void) {
	static uint16_t startup = 0;
	if (startup)
		return;
	CAN_Init();
	startup = 1;

	for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++)
		own_nodes[i].ShortName.NodeID = LC_Broadcast_Address;
	for (int i = 0; i < LEVCAN_MAX_TABLE_NODES; i++)
		node_table[i].ShortName.NodeID = LC_Broadcast_Address;

	configureFilters();
}

int32_t getTXqueueSize(void) {
	if (txFIFO_in >= txFIFO_out)
		return txFIFO_in - txFIFO_out;
	else
		return txFIFO_in + (LEVCAN_TX_SIZE - txFIFO_out);
}

void LC_AddressClaimHandler(LC_NodeShortName node, uint16_t mode) {
	headerPacked_t header = { .Priority = ~LC_Priority_Control, .MsgID = LC_SYS_AddressClaimed, .Target = LC_Broadcast_Address, .Request = 0 };
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
	//TODO: it must be sent!?
	sendDataToQueue(header, data, 8);
}

void configureFilters(void) {

	CAN_FiltersClear();
	CAN_FilterEditOn();
//global filter
	headerPacked_t reg, mask;
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
	headerPacked_t reg, mask;
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

int16_t compareNode(LC_NodeShortName a, LC_NodeShortName b) {
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

void LC_ReceiveHandler(uint32_t time) {
	static headerPacked_t header;
	static uint32_t data[2];
	static uint16_t length;
//fast receive to clear input buffer, handle later in manager
	while (CAN_Receive(&header.ToUint32, data, &length) == CANH_Ok) {

		if (header.MsgID == LC_SYS_AddressClaimed) {
			LC_Header uheader = headerUnpack(header);
			if (uheader.Request) {
				//TODO what to do with null?
				if (uheader.Target == LC_Broadcast_Address) {
					//send every node id
					for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
						if (own_nodes[i].ShortName.NodeID < LC_Null_Address && own_nodes[i].State >= LCNodeState_WaitingClaim)
							LC_AddressClaimHandler(own_nodes[i].ShortName, LC_TX);
					}
				} else {
					//single node
					for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
						if (own_nodes[i].ShortName.NodeID == uheader.Target && own_nodes[i].State >= LCNodeState_WaitingClaim) {
							LC_AddressClaimHandler(own_nodes[i].ShortName, LC_TX);
							return;
						}
					}
				}
			} else
				LC_AddressClaimHandler((LC_NodeShortName ) { .ToUint32[0] = data[0], .ToUint32[1] = data[1], .NodeID = uheader.Source }, LC_RX);
		} else {
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

}

void LC_NetworkManager(uint32_t tick) {
	static uint32_t tick_last = 0;
	uint32_t time = tick - tick_last;
	tick_last = tick;
	for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
		//send every node id
		if (own_nodes[i].State == LCNodeState_NetworkDiscovery) {
			//check network discovery timeout
			own_nodes[i].LastTXtime += time;
			if (own_nodes[i].LastTXtime > 100) {
				//we ready to begin address claim
				//todo move to claimFreeID
				uint16_t freeid = own_nodes[i].LastID;
				while (searchIndexCollision(own_nodes[i].LastID, &own_nodes[i])) {
					freeid++;
					if (freeid > LC_NodeFreeIDmax || freeid < LC_NodeFreeIDmin)
						freeid = LC_NodeFreeIDmin;
				}
				own_nodes[i].State = LCNodeState_WaitingClaim;
				own_nodes[i].ShortName.NodeID = freeid;
				CAN_FilterEditOn();
				addAddressFilter(freeid);
				CAN_FilterEditOff();
				LC_AddressClaimHandler(own_nodes[i].ShortName, LC_TX);
				own_nodes[i].LastTXtime = 0;
#ifdef LEVCAN_TRACE
				trace_printf("Discovery finish id:%d\n", own_nodes[i].ShortName.NodeID);
#endif
			}
		} else if (own_nodes[i].ShortName.NodeID == LC_Null_Address) {
			//we've lost id, get new one
			//look for free id
			//todo add delay or detect network on startup
			claimFreeID(&own_nodes[i]);
		} else if (own_nodes[i].ShortName.NodeID < LC_Broadcast_Address) {
			if (own_nodes[i].State == LCNodeState_WaitingClaim) {
				own_nodes[i].LastTXtime += time;
				if (own_nodes[i].LastTXtime > 250) {
					own_nodes[i].State = LCNodeState_Online;
					configureFilters(); //todo make it faster?
#ifdef LEVCAN_TRACE
					trace_printf("We are online ID:%d\n", own_nodes[i].ShortName.NodeID);
#endif
				}
			}
		}
	}

	while (rxFIFO_in != rxFIFO_out) {
		//proceed RX FIFO
		//rxFIFO[rxBuffer_out].data[0];
		//rxFIFO[rxBuffer_out].data[1];
		//rxFIFO[rxBuffer_out].length;
		headerPacked_t hdr = rxFIFO[rxFIFO_out].header;
		if (hdr.Request) {
			if (hdr.RTS_CTS == 0 && hdr.EoM == 0) {
				//Remote transfer request, try to create new TX object
				LC_NodeDescription_t* node = findNode(hdr.Target);
				LC_ObjectRecord_t obj = findObjectRecord(hdr.MsgID, rxFIFO[rxFIFO_out].length, node, Read, hdr.Source);

				if (obj.Attributes.Function && obj.Address) {
					//function call before sending
					//unpack header
					LC_Header unpack = headerUnpack(hdr);
					//call object, return function should have processed record.
					obj = ((LC_FunctionCall_t) obj.Address)(node, unpack, 0, 0);
				}
				LC_SendMessage((intptr_t*) node, &obj, hdr.Source, hdr.MsgID);
			} else {
				//find existing TX object
				objBuffered* TXobj = (objBuffered*) objTXbuf_start;
				while (TXobj) {
					objBuffered* next = (objBuffered*) TXobj->Next; //this obj may be lost in tx function
					//same source and same ID ?
					if (TXobj && TXobj->Header.MsgID == hdr.MsgID && TXobj->Header.Target == hdr.Source) {
						//one source can send only one messageID a time
						objectTXproceed(TXobj, &hdr);
						break;
					}
					TXobj = next;
				}
			}
		} else {
			//we got data
			if (hdr.RTS_CTS) {
				//address valid?
				if (hdr.Source >= LC_Null_Address)
					continue;
				if (hdr.EoM) {
					//fast receive
					LC_NodeDescription_t* node = findNode(hdr.Target);
					LC_ObjectRecord_t obj = findObjectRecord(hdr.MsgID, rxFIFO[rxFIFO_out].length, node, Write, hdr.Source);
					if (obj.Address != 0 && (obj.Attributes.Writable) != 0) {
						if (obj.Attributes.Function) {
							//function call
							LC_Header unpack = headerUnpack(hdr);
							//call
							((LC_FunctionCall_t) obj.Address)(node, unpack, rxFIFO[rxFIFO_out].data, rxFIFO[rxFIFO_out].length);
						} else if (obj.Attributes.Pointer) {
							//store our memory pointer
							//TODO: call new malloc for smaller size?
							char* clean = *(char**) obj.Address;
							char* mem = lcmalloc(rxFIFO[rxFIFO_out].length);
							*(char**) obj.Address = mem;
							//cleanup if there was pointer
							if (clean)
								lcfree(clean);
						} else {
							//just copy data
							memcpy(obj.Address, rxFIFO[rxFIFO_out].data, obj.Size);
						}
#ifdef LEVCAN_TRACE
						trace_printf("RX fast finished:%d \n", hdr.MsgID);
#endif
						if (hdr.Parity) {
							//TCP End Of Message ACK fast
							headerPacked_t hdrtx;
							hdrtx.EoM = 1; //end of message
							hdrtx.RTS_CTS = 0;
							hdrtx.Priority = hdr.Priority;
							hdrtx.Source = hdr.Target; //we are target (receive)
							hdrtx.Target = hdr.Source;
							hdrtx.Request = 1;
							hdrtx.MsgID = hdr.MsgID;
							hdrtx.Parity = 1;
							sendDataToQueue(hdrtx, 0, 0);
						}
					} else {
#ifdef LEVCAN_TRACE
						trace_printf("RX fast failed:%d \n", hdr.MsgID);
#endif
					}
				} else {
					//create new receive object
					objBuffered* newRXobj = (objBuffered*) lcmalloc(sizeof(objBuffered));
					if (newRXobj == 0)
						continue;
					newRXobj->Pointer = lcmalloc(32);
					if (newRXobj->Pointer == 0) {
						lcfree(newRXobj);
						continue;
					}
					newRXobj->Length = 32;
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
				//	trace_printf("Data input:%d\n", hdr.MsgID);
				//find existing RX object
				objBuffered* RXobj = (objBuffered*) objRXbuf_start;
				while (RXobj) {
					objBuffered* next = (objBuffered*) RXobj->Next; //this obj may be lost in receive function
					//same source and same ID ?
					//one ID&source can send only one message length a time
					if (RXobj && RXobj->Header.MsgID == hdr.MsgID && RXobj->Header.Source == hdr.Source) {
						objectRXproceed(RXobj, &rxFIFO[rxFIFO_out]);
						break;
					}
					RXobj = next;
				}

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
		txProceed->Attempt = 0;
		if (txProceed->Flags.TCP == 0) {
			//UDP mode send data continuously
			objectTXproceed(txProceed, 0);
		} else if (txProceed->Time_since_comm > 1000) {
			//TX timeout, make it free!
#ifdef LEVCAN_TRACE
			trace_printf("TX object deleted by timeout:%d\n", txProceed->Header.MsgID);
#endif
			deleteObject(txProceed, (objBuffered**) &objTXbuf_start, (objBuffered**) &objTXbuf_end);
		}
		txProceed = next;
	}
//count work time and recall
	objBuffered* rxProceed = (objBuffered*) objRXbuf_start;
	while (rxProceed) {
		objBuffered* next = (objBuffered*) rxProceed->Next;
		rxProceed->Time_since_comm += time;
		if (rxProceed->Flags.TCP) {
			//TCP mode
			if (rxProceed->Time_since_comm > 100) {
				if (rxProceed->Attempt > 10) {
					//RX timeout, make it free!
#ifdef LEVCAN_TRACE
					trace_printf("RX object deleted by attempt:%d\n", rxProceed->Header.MsgID);
#endif
					lcfree(rxProceed->Pointer);
					deleteObject(rxProceed, (objBuffered**) &objRXbuf_start, (objBuffered**) &objRXbuf_end);
				} else {
					//	trace_printf("Try again request: %d\n", rxProceed->Header.MsgID);
					objectRXproceed(rxProceed, 0);
					rxProceed->Attempt++;
					rxProceed->Time_since_comm = 0;
				}
			}
		} else if (rxProceed->Time_since_comm > 1000) {
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
	offline_tick += time;
	if (offline_tick > 250) { //0.5s
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
		offline_tick = offline_tick % 500;
	}
#endif
//vTaskDelay(250);
}

void deleteObject(objBuffered* obj, objBuffered** start, objBuffered** end) {
	if (obj->Previous)
		((objBuffered*) obj->Previous)->Next = obj->Next; //junction
	else {
		(*start) = (objBuffered*) obj->Next; //Starting
		(*start)->Previous = 0;
	}
	if (obj->Next) {
		((objBuffered*) obj->Next)->Previous = obj->Previous;
	} else {
		(*end) = (objBuffered*) obj->Previous; //ending
		(*start)->Next = 0;
	}
//free this object
	lcfree(obj);
}

headerPacked_t headerPack(LC_Header header) {
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

LC_Header headerUnpack(headerPacked_t header) {
	LC_Header hdr;
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

uint16_t sendDataToQueue(headerPacked_t hdr, uint32_t data[], uint8_t length) {

	if (txFIFO_in == ((txFIFO_out - 1 + LEVCAN_TX_SIZE) % LEVCAN_TX_SIZE))
		return 1;

	int empty = 0;
	if (txFIFO_in == txFIFO_out)
		empty = 1;

	txFIFO[txFIFO_in].header = hdr;
	txFIFO[txFIFO_in].length = length;
	if (data) {
		txFIFO[txFIFO_in].data[0] = data[0];
		txFIFO[txFIFO_in].data[1] = data[1];
	} else {
		txFIFO[txFIFO_in].data[0] = 0;
		txFIFO[txFIFO_in].data[1] = 0;
	}

	txFIFO_in = (txFIFO_in + 1) % LEVCAN_TX_SIZE;

//proceed queue if we can do;
	if (empty)
		LC_TransmitHandler();
	return 0;
}

uint16_t objectTXproceed(objBuffered* object, headerPacked_t* request) {
	int32_t length;
	uint32_t data[2];
	uint8_t parity = ~((object->Position + 7) / 8) & 1; //parity
	if (request) {
		if (request->EoM) {
			//TX finished? delete this buffer anyway
#ifdef LEVCAN_TRACE
			trace_printf("TX TCP finished:%d\n", object->Header.MsgID);
#endif
			//cleanup tx buffer also
			if (object->Flags.TXcleanup)
				lcfree(object->Pointer);
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
			// trace_printf("TX object parity lost:%d position:%d\n", object->Header.MsgID, object->Position);
		} else {
			// trace_printf("Request got valid parity\n");
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
		//set data start only if not requested again, as this will create new buffer
		if (object->Position == 0 && request == 0)
			newhdr.RTS_CTS = 1;
		else
			newhdr.RTS_CTS = 0;

		newhdr.Parity = object->Flags.TCP ? parity : 0; //parity

		// trace_printf("TX object sent:%d position:%d parity:%d\n", object->Header.MsgID, object->Position, parity);
		if (sendDataToQueue(newhdr, data, length))
			return 1;
		object->Position += length;
		object->Header = newhdr; //update to new only here
		//object->Attempt++; //data sent
		object->Time_since_comm = 0; //data sent
		//cycle if this is UDP till message end or buffer half fill
	} while ((object->Flags.TCP == 0) && (getTXqueueSize() * 3 < LEVCAN_TX_SIZE * 4) && (object->Header.EoM == 0));
//in UDP mode delete object when EoM is set
	if ((object->Flags.TCP == 0) && (object->Header.EoM == 1)) {
		if (object->Flags.TXcleanup)
			lcfree(object->Pointer);
		deleteObject(object, (objBuffered**) &objTXbuf_start, (objBuffered**) &objTXbuf_end);
#ifdef LEVCAN_TRACE
		trace_printf("TX UDP finished:%d\n", object->Header.MsgID);
#endif
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
			char* newmem = lcmalloc(object->Length * 2);
			if (newmem) {
				memcpy(newmem, object->Pointer, object->Length);
				object->Length = object->Length * 2;
			}
			lcfree(object->Pointer);
			object->Pointer = newmem;
			//todo check possible pointer loose and close object
		}
		if (object->Pointer)
			memcpy(&object->Pointer[object->Position], msg->data, msg->length);
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
		headerPacked_t hdr;
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
		LC_NodeDescription_t* node = findNode(object->Header.Target);
		//check check and check again
		LC_ObjectRecord_t obj = findObjectRecord(object->Header.MsgID, object->Position, node, Write, object->Header.Source);
		if (obj.Address != 0 && (obj.Attributes.Writable) != 0) {
			if (obj.Attributes.Function) {
				//function call
				//unpack header
				LC_Header unpack = headerUnpack(object->Header);
				//call
				((LC_FunctionCall_t) obj.Address)(node, unpack, object->Pointer, object->Position);
				//cleanup
				lcfree(object->Pointer);
			} else if (obj.Attributes.Pointer) {
				//store our memory pointer
				//TODO: call new malloc for smaller size?
				char* clean = *(char**) obj.Address;
				*(char**) obj.Address = object->Pointer;
				//cleanup if there was pointer
				if (clean)
					lcfree(clean);
			} else {
				//just copy data as usual to specific location
				int32_t size = abs(obj.Size);
				memcpy(obj.Address, object->Pointer, size);
				//string should be ended with zero
				if (obj.Size < 0)
					((uint8_t*) obj.Address)[size - 1] = 0;
				//clean up memory buffer
				lcfree(object->Pointer);
			}
#ifdef LEVCAN_TRACE
			trace_printf("RX finished:%d success\n", object->Header.MsgID);
#endif
		} else {
			//clean up memory buffer
			lcfree(object->Pointer);
#ifdef LEVCAN_TRACE
			trace_printf("RX finished:%d failed\n", object->Header.MsgID);
#endif
		}
		//delete object from memory chain, find new endings
		deleteObject(object, (objBuffered**) &objRXbuf_start, (objBuffered**) &objRXbuf_end);
	}

	return 0;
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
			//extract pointer, size and attributes
			//TODO optimize mimimi
			record = ((LC_ObjectRecord_t*) objectArray[i].Address);
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
/*
 LC_Object_t* findObject(uint16_t index, int32_t size, nodeDescription* node) {
 LC_Object_t* rec = 0;
 int32_t findedsize = 0;
 if (node == 0)
 return rec;
 for (int i = 0; i < node->ObjectsSize; i++) {
 //extract
 if ((node->Objects[i].Attributes.Record) != 0) {
 LC_ObjectRecord_t* record = ((LC_ObjectRecord_t*) node->Objects[i].Address);
 findedsize = record->Size;
 rec = &node->Objects[i];
 } else
 findedsize = node->Objects[i].Size;

 //compare: right size and index?
 if (size == findedsize && rec->Index == index)
 break; //yes
 else
 rec = 0; //no
 }
 return rec;
 }*/

void LC_SendMessage(void* sender, LC_ObjectRecord_t* object, uint16_t target, uint16_t index) {
	LC_NodeDescription_t* node = sender;
	if (node == 0)
		node = &own_nodes[0];
	if (node->State != LCNodeState_Online)
		return;

	if (object == 0 || object->Address == 0)
		return;
	char* dataAddr = object->Address;
//check that data is real
	if (dataAddr == 0 && object->Size != 0)
		return;
//extract pointer
	if (object->Attributes.Pointer)
		dataAddr = *(char**) dataAddr;
//negative size means this is string - any length
	if ((object->Attributes.TCP) || (object->Size > 8) || ((object->Size < 0) && (strnlen(dataAddr, 8) == 8))) {
		//avoid dual same id
		objBuffered* txProceed = (objBuffered*) objTXbuf_start;
		while (txProceed) {
			if ((txProceed->Header.MsgID == index) && (txProceed->Header.Target == target))
				return;
			txProceed = ((objBuffered*) txProceed->Next);
		}
		if (target == node->ShortName.NodeID)
			return;
		//form message header
		headerPacked_t hdr;
		hdr.MsgID = index; //our node index
		hdr.Priority = ~object->Attributes.Priority;
		hdr.Request = 0; //data sending...
		hdr.Source = node->ShortName.NodeID;
		hdr.Target = target;
		//create object sender instance
		objBuffered* newTXobj = (objBuffered*) lcmalloc(sizeof(objBuffered));
		if (newTXobj == 0)
			return;
		newTXobj->Attempt = 0;
		newTXobj->Header = hdr;
		newTXobj->Length = object->Size;
		newTXobj->Pointer = dataAddr;
		newTXobj->Position = 0;
		newTXobj->Time_since_comm = 0;
		newTXobj->Next = 0;
		newTXobj->Flags.TCP = object->Attributes.TCP;
		newTXobj->Flags.TXcleanup = object->Attributes.Cleanup;
		//todo TASK CRITICAL
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
#ifdef LEVCAN_TRACE
		trace_printf("New TX object created:%d\n", newTXobj->Header.MsgID);
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

		headerPacked_t hdr;
		hdr.MsgID = index;
		hdr.Priority = ~object->Attributes.Priority;
		hdr.Request = 0;
		hdr.Parity = 0;
		hdr.RTS_CTS = 1; //data start
		hdr.EoM = 1; //data end
		hdr.Source = node->ShortName.NodeID;
		hdr.Target = target;

		sendDataToQueue(hdr, data, size);
	}
}

void LC_SendRequest(void* sender, uint16_t target, uint16_t index) {
	LC_SendRequestSpec(sender, target, index, 0, 0);
}

void LC_SendRequestSpec(void* sender, uint16_t target, uint16_t index, uint8_t size, uint8_t TCP) {

	LC_NodeDescription_t* node = sender;
	if (node == 0)
		node = &own_nodes[0];
	if (node->State != LCNodeState_Online)
		return;

	headerPacked_t hdr;
	hdr.MsgID = index;
	hdr.Priority = 0;
	hdr.Request = 1;
	hdr.Parity = TCP;
	hdr.RTS_CTS = 0;
	hdr.Source = node->ShortName.NodeID;
	hdr.Target = target;

	sendDataToQueue(hdr, 0, size);
}

void LC_SendDiscoveryRequest(uint16_t target) {
	headerPacked_t hdr;
	hdr.MsgID = LC_SYS_AddressClaimed;
	hdr.Priority = 0;
	hdr.Request = 1;
	hdr.Parity = 0;
	hdr.RTS_CTS = 0;
	hdr.Source = LC_Broadcast_Address;
	hdr.Target = target;

	sendDataToQueue(hdr, 0, 0);
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

