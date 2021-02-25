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
#include "levcan_address.h"

#ifndef LEVCAN_MIN_BYTE_SIZE
#define LEVCAN_MIN_BYTE_SIZE 1
#endif
//### Private functions ###
void lc_processAddressClaim(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
void lc_addressClaimHandler(LC_NodeShortName_t node, uint16_t mode);
int16_t lc_compareNodes(LC_NodeShortName_t a, LC_NodeShortName_t b);
uint16_t lc_searchIndexCollision(uint16_t nodeID, LC_NodeDescriptor_t *ownNode);
void lc_claimFreeID(LC_NodeDescriptor_t *node);
LC_Return_t lc_sendDiscoveryRequest(uint16_t target);

extern LC_Return_t lc_sendDataToQueue(LC_HeaderPacked_t hdr, uint32_t data[], uint8_t length);
//### Private variables
extern LC_NodeDescriptor_t own_nodes[];
extern LC_NodeTable_t node_table[];
// extern HAL functions
extern LC_Return_t LC_HAL_CreateFilterMasks(LC_HeaderPacked_t *reg, LC_HeaderPacked_t *mask, uint16_t count);

LC_RemoteNodeCallback_t lc_addressCallback = 0;

void LC_AddressManager(uint32_t time) {
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
				while (lc_searchIndexCollision(freeid, &own_nodes[i])) {
					freeid++;
					if (freeid > LC_NodeFreeIDmax || freeid < LC_NodeFreeIDmin)
						freeid = LC_NodeFreeIDmin;
				}
				own_nodes[i].State = LCNodeState_WaitingClaim;
				own_nodes[i].LastTXtime = 0;
				own_nodes[i].ShortName.NodeID = freeid;
				LC_ConfigureFilters();
				lc_addressClaimHandler(own_nodes[i].ShortName, LC_TX);
#ifdef LEVCAN_TRACE
				trace_printf("Discovery finish id:%d\n", own_nodes[i].ShortName.NodeID);
#endif
			}
		} else if (own_nodes[i].ShortName.NodeID == LC_Null_Address) {
			//we've lost id, get new one
			//look for free id
			own_nodes[i].LastTXtime = 0;
			lc_claimFreeID(&own_nodes[i]);
		} else if (own_nodes[i].ShortName.NodeID < LC_Broadcast_Address) {

			if (own_nodes[i].State == LCNodeState_WaitingClaim) {
				own_nodes[i].LastTXtime += time;
				if (own_nodes[i].LastTXtime > 250) {
					own_nodes[i].State = LCNodeState_Online;
					LC_ConfigureFilters();
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
					lc_addressClaimHandler(own_nodes[i].ShortName, LC_TX);
#ifdef LEVCAN_TRACE
					if (alone == 0) {
						trace_printf("Are we alone?:%d\n", own_nodes[i].ShortName.NodeID);
					}
					alone = 3000;
#endif
				} else if (alone)
					alone -= time;
			}
		}
	}
}

void lc_processAddressClaim(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	(void) size; //no warnings
	(void) node;

	if (header.Request) {
		int i = 0;
		//TODO what to do with null?
		if (header.Target == LC_Broadcast_Address) {
			//send every node id
#if (LEVCAN_MAX_OWN_NODES) > 1
			for ( i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
#else
			{
#endif
				if (own_nodes[i].ShortName.NodeID < LC_Null_Address && own_nodes[i].State >= LCNodeState_WaitingClaim) {
					if (own_nodes[i].State == LCNodeState_Online)
						own_nodes[i].LastTXtime = 0;    //reset online timer
					lc_addressClaimHandler(own_nodes[i].ShortName, LC_TX);
				}
			}
		} else {
			//single node
#if (LEVCAN_MAX_OWN_NODES) > 1
			for ( i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
#else
			{
#endif
				if (own_nodes[i].ShortName.NodeID == header.Target && own_nodes[i].State >= LCNodeState_WaitingClaim) {
					if (own_nodes[i].State == LCNodeState_Online)
						own_nodes[i].LastTXtime = 0;    //reset online timer
					lc_addressClaimHandler(own_nodes[i].ShortName, LC_TX);
#if (LEVCAN_MAX_OWN_NODES) > 1
					break;
#endif
				}
			}
		}
	} else {
		//got some other claim
		uint32_t *toui = data;
		if (toui != 0)
			lc_addressClaimHandler((LC_NodeShortName_t ) { .ToUint32[0] = toui[0], .ToUint32[1] = toui[1], .NodeID = header.Source }, LC_RX);
	}
}

void lc_addressClaimHandler(LC_NodeShortName_t node, uint16_t mode) {
	LC_HeaderPacked_t header = { //
			.Priority = (~LC_Priority_Control) & 0x3,    //
					.MsgID = LC_SYS_AddressClaimed,    //
					.Target = LC_Broadcast_Address,    //
					.Request = 0, .RTS_CTS = 1, .EoM = 1 };
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
					if (lc_compareNodes(own_nodes[i].ShortName, node) != -1) {
						idlost = 1;    //if we loose this id, we should try to add new node to table
						//less value - more priority. our not less, reset address
						header.Source = LC_Null_Address;
						//copy short name
						data[0] = own_nodes[i].ShortName.ToUint32[0];
						data[1] = own_nodes[i].ShortName.ToUint32[1];
						//later we will find new id
						own_nodes[i].ShortName.NodeID = LC_Null_Address;
						own_nodes[i].State = LCNodeState_WaitingClaim;
						LC_ConfigureFilters();
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
				if (lc_compareNodes(node_table[i].ShortName, node) == 0) {
					//compare by short name, if found - delete this instance
#ifdef LEVCAN_TRACE
					trace_printf("Lost S/N:%08X ID:%d\n", node.SerialNumber, node_table[i].ShortName.NodeID);
#endif
					if (lc_addressCallback) {
						lc_addressCallback(node_table[i].ShortName, i, LC_AdressDeleted);
					}

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
					int eql = lc_compareNodes(node_table[i].ShortName, node);
					if (eql == 1) {
						//less value - more priority. our table not less, setup new short name
						node_table[i].ShortName = node;
						node_table[i].LastRXtime = 0;

						if (lc_addressCallback) {
							lc_addressCallback(node, i, LC_AdressChanged);
						}
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

				if (lc_addressCallback) {
					lc_addressCallback(node, empty, LC_AdressNew);
				}
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
	lc_sendDataToQueue(header, data, 8 / LEVCAN_MIN_BYTE_SIZE);
}

void lc_claimFreeID(LC_NodeDescriptor_t *node) {
	uint16_t freeid = LC_NodeFreeIDmin;
	if (freeid < node->LastID) {
		//this one will increment freeid
		freeid = node->LastID;
	}
	if (freeid > LC_NodeFreeIDmax)
		freeid = LC_NodeFreeIDmin;
	//todo fix endless loop
	while (lc_searchIndexCollision(freeid, node)) {
		freeid++;
		if (freeid > LC_NodeFreeIDmax)
			freeid = LC_NodeFreeIDmin;
	}
	node->LastID = freeid;
	node->ShortName.NodeID = freeid;
#ifdef LEVCAN_TRACE
	trace_printf("Trying claim ID:%d\n", freeid);
#endif
	lc_addressClaimHandler(node->ShortName, LC_TX);
	node->LastTXtime = 0;
	node->State = LCNodeState_WaitingClaim;
	//add new own address filter TODO add later after verification
	LC_ConfigureFilters();
}

uint16_t lc_searchIndexCollision(uint16_t nodeID, LC_NodeDescriptor_t *ownNode) {
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

int16_t lc_compareNodes(LC_NodeShortName_t a, LC_NodeShortName_t b) {
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

void LC_ConfigureFilters(void) {

	LC_HeaderPacked_t reg[LEVCAN_MAX_OWN_NODES + 1] = { 0 };
	LC_HeaderPacked_t mask[LEVCAN_MAX_OWN_NODES + 1] = { 0 };
	int8_t active_filters = 0;
	//global filter
	reg[active_filters].MsgID = LC_SYS_AddressClaimed;
	//reg.RTS_CTS = 0;    //no matter
	//reg.Parity = 0;    // no matter
	//reg.Priority = 0;    //no matter
	//reg.Source = 0;    //any source
	reg[active_filters].Target = LC_Broadcast_Address;    //we are target- Broadcast, this should match
	//reg.Request = 0;
	//fill can mask match
	mask[active_filters] = reg[active_filters];
	active_filters++;
	//mask.Request = 0;    //any request or data
	//type cast
	uint8_t active_node_exist = 0;
	for (int i = 0; i < LEVCAN_MAX_OWN_NODES; i++) {
		if (own_nodes[i].ShortName.NodeID < LC_Null_Address) {
			//reg.MsgID = 0;    //no matter
			//reg.RTS_CTS = 0;    //no matter
			//reg.Parity = 0;    // no matter
			//reg.Priority = 0;    //no matter
			//reg.Source = 0;    //any source
			reg[active_filters].Target = own_nodes[i].ShortName.NodeID;    //we are target, this should match
			//reg.Request = 0;    //no matter
			//fill can mask match
			//mask = reg;
			//mask.MsgID = 0;    //match any
			mask[active_filters].Target = LC_Broadcast_Address;    // should match
			//mask.Request = 0;    //any request or data
			//type cast
			active_node_exist = 1;
			active_filters++;
		}
	}

	if (active_node_exist) {
		mask[0].MsgID = 0;    //match any brdcast
	} else
		mask[0].MsgID = 0x3F0;    //match for first 16 system messages

	LC_HAL_CreateFilterMasks(reg, mask, active_filters);
}

LC_Return_t lc_sendDiscoveryRequest(uint16_t target) {
	LC_HeaderPacked_t hdr = { 0 };
	hdr.MsgID = LC_SYS_AddressClaimed;
	hdr.Priority = 0;
	hdr.Request = 1;
	hdr.Parity = 0;
	hdr.RTS_CTS = 0;
	hdr.Source = LC_Broadcast_Address;
	hdr.Target = target;

	return lc_sendDataToQueue(hdr, 0, 0);
}
