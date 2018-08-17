/*
 * Simple file i/o operations over LEVCAN protocol, client side
 * levcan_fileclient.c
 *
 *  Created on: 10 July 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "levcan_fileclient.h"
#include "levcan_filedef.h"

#if LEVCAN_OBJECT_DATASIZE < 16
#error "Too small LEVCAN_OBJECT_DATASIZE size for file io!"
#endif
//extern functions
extern void *lcdelay(uint32_t time);
// Private variables
volatile fRead_t rxtoread[LEVCAN_MAX_OWN_NODES] = { 0 };
volatile uint32_t fpos[LEVCAN_MAX_OWN_NODES] = { 0 };
volatile fOpAck_t rxack[LEVCAN_MAX_OWN_NODES] = { 0 };

void proceedFileClient(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size) {
	if (size < 2)
		return;
	uint16_t* op = data;
	switch (*op) {
	case fOpAck: {
		if (sizeof(fOpAck_t) == size) {
			int id = LC_GetMyNodeIndex(node);
			if (id >= 0) {
				rxack[id].Error = ((fOpAck_t*) data)->Error;
				rxack[id].Operation = ((fOpAck_t*) data)->Operation; //finish
			}
		}
	}
		break;
	case fOpData: {
		fOpData_t* fop = data;
		int id = LC_GetMyNodeIndex(node);
		//check if it is okay
		if (id >= 0) {
			if (rxtoread[id].Buffer != 0 && fop->TotalRead <= rxtoread[id].ReadBytes && rxtoread[id].Position == -1
					&& size == (int32_t) (rxtoread[id].ReadBytes + sizeof(fOpData_t))) {
				memcpy(rxtoread[id].Buffer, fop->Data, fop->TotalRead);
				rxtoread[id].ReadBytes = fop->TotalRead;
				rxtoread[id].Error = fop->Error;
				rxtoread[id].Position = fop->Position; //trigger
			} else {
				rxtoread[id].Error = LC_FR_NetworkError; //data error
				rxtoread[id].ReadBytes = 0;
				rxtoread[id].Position = 0;
			}
		}
	}
		break;
	}
}

LC_FileResult_t LC_FileOpen(char* name, LC_FileAccess_t mode, void* sender_node, uint16_t server_node) {
	uint16_t attempt;
	LC_NodeShortName_t server;
	//look for any server node
	if (server_node == LC_Broadcast_Address)
		server = LC_FindFileServer(0);
	else
		server = LC_GetNode(server_node);
	//checks
	if (server.FileServer == 0 || server.NodeID == LC_Broadcast_Address)
		return LC_NodeOffline;
	if (name == 0)
		return LC_DataError;
	//get index for array
	int id = LC_GetMyNodeIndex(sender_node);
	if (id < 0)
		return LC_NodeOffline;

	int datasize = sizeof(fOpOpen_t) + strlen(name) + 1;
	//create buffer
	char datasend[datasize];
	memset(datasend, 0, datasize);
	//buffer tx file operation
	fOpOpen_t* openf = (fOpOpen_t*) datasend;
	openf->Operation = fOpOpen;
	openf->Name[0] = 0;
	strcpy(&openf->Name[0], name);
	openf->Mode = mode;
	//prepare message
	LC_ObjectRecord_t rec = { 0 };
	rec.Address = openf;
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;
	rec.NodeID = server.NodeID;
	rec.Size = datasize;
	//reset ACK
	rxack[id] = (fOpAck_t ) { 0 };
	fpos[id] = 0;
	attempt = 0;
	for (attempt = 0; attempt < 3; attempt++) {
		LC_Return_t sr = LC_SendMessage(sender_node, &rec, LC_SYS_FileClient);
		//send error?
		if (sr) {
			if (sr == LC_BufferFull)
				return LC_FR_NetworkBusy;
			if (sr == LC_MallocFail)
				return LC_FR_MemoryFull;
			return LC_FR_NetworkError;
		}
		//wait 100ms
		for (int time = 0; time < 100; time++) {
			lcdelay(100);
			if (rxack[id].Operation == fOpAck)
				return rxack[id].Error; //Finish!!
		}
	}
	return LC_FR_NetworkTimeout;
}

LC_FileResult_t LC_FileRead(char* buffer, uint32_t btr, uint32_t* br, void* sender_node, uint16_t server_node) {
	uint16_t attempt;
	LC_NodeShortName_t server;

	if (br == 0 || buffer == 0)
		return LC_DataError;
	//look for any server node
	if (server_node == LC_Broadcast_Address)
		server = LC_FindFileServer(0);
	else
		server = LC_GetNode(server_node);
	//checks
	if (server.FileServer == 0 || server.NodeID == LC_Broadcast_Address)
		return LC_NodeOffline;

	int id = LC_GetMyNodeIndex(sender_node);
	if (id < 0)
		return LC_NodeOffline;

	fOpRead_t readf;
	readf.Operation = fOpRead;
	LC_ObjectRecord_t rec = { 0 };
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;
	rec.NodeID = server.NodeID;
	rec.Address = &readf;
	rec.Size = sizeof(fOpRead_t);
	//reset
	*br = 0;
	attempt = 0;
	LC_FileResult_t ret = LC_FR_Ok;

	for (uint32_t position = 0; position < btr;) {
		uint32_t toreadnow = btr - position;
		int32_t globalpos = position + fpos[id];
		//finish?
		if (toreadnow == 0)
			return LC_Ok;
		if (toreadnow > LEVCAN_OBJECT_DATASIZE - sizeof(fOpData_t))
			toreadnow = LEVCAN_OBJECT_DATASIZE - sizeof(fOpData_t);
		if (toreadnow > INT16_MAX)
			toreadnow = INT16_MAX;
		//Prepare receiver
		rxtoread[id].Buffer = &buffer[position];
		rxtoread[id].Error = 0;
		rxtoread[id].Position = -1;
		rxtoread[id].ReadBytes = toreadnow;
		readf.ToBeRead = toreadnow;
		readf.Position = globalpos; //add global position

		LC_Return_t sr = LC_SendMessage(sender_node, &rec, LC_SYS_FileClient);
		//send error?
		if (sr) {
			if (sr == LC_BufferFull)
				ret = LC_FR_NetworkBusy;
			else if (sr == LC_MallocFail)
				ret = LC_FR_MemoryFull;
			else
				ret = LC_FR_NetworkError;
			break;
		}
		//wait 500ms
		for (int time = 0; time < 50000; time++) {
			lcdelay(10);
			if (rxtoread[id].Position != -1)
				break;
		}
		//just make some checks
		if (rxtoread[id].Position >= 0 && rxtoread[id].Position == globalpos && rxtoread[id].ReadBytes <= toreadnow) {
			//received something, data already filled, move pointers
			position += rxtoread[id].ReadBytes;
			*br += rxtoread[id].ReadBytes;
			attempt = 0;

			if (rxtoread[id].ReadBytes < toreadnow)
				break; //receive finished
		} else {
			attempt++;
			if (attempt > 3) {
				ret = LC_FR_NetworkTimeout;
				break;
			}
		}
		//got some errs?
		if (rxtoread[id].Error && rxtoread[id].Error != LC_FR_NetworkError) {
			ret = rxtoread[id].Error;
			break;
		}
	}
	//reset buffer
	fpos[id] += *br;
	rxtoread[id].Buffer = 0;
	return ret;
}

LC_FileResult_t LC_FileClose(void* sender_node, uint16_t server_node) {
	uint16_t attempt;
	LC_NodeShortName_t server;
	//look for any server node
	if (server_node == LC_Broadcast_Address)
		server = LC_FindFileServer(0);
	else
		server = LC_GetNode(server_node);
	//checks
	if (server.FileServer == 0 || server.NodeID == LC_Broadcast_Address)
		return LC_NodeOffline;
	//get index for array
	int id = LC_GetMyNodeIndex(sender_node);
	if (id < 0)
		return LC_NodeOffline;

	//create buffer
	fOpClose_t closef;
	closef.Operation = fOpClose;
	//prepare message
	LC_ObjectRecord_t rec = { 0 };
	rec.Address = &closef;
	rec.Size = sizeof(fOpClose_t);
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;
	rec.NodeID = server.NodeID;
	//reset ACK
	rxack[id] = (fOpAck_t ) { 0 };
	fpos[id] = 0;
	attempt = 0;
	for (attempt = 0; attempt < 3; attempt++) {
		LC_Return_t sr = LC_SendMessage(sender_node, &rec, LC_SYS_FileClient);
		//send error?
		if (sr) {
			if (sr == LC_BufferFull)
				return LC_FR_NetworkBusy;
			if (sr == LC_MallocFail)
				return LC_FR_MemoryFull;
			return LC_FR_NetworkError;
		}
		//wait 100ms
		for (int time = 0; time < 10; time++) {
			lcdelay(10);
			if (rxack[id].Operation == fOpAck)
				return rxack[id].Error; //Finish!!
		}
	}
	return LC_FR_NetworkTimeout;
}

/// Returns file server short name
/// @param scnt Pointer to stored position for search, can be null
/// @return LC_NodeShortName_t file server
LC_NodeShortName_t LC_FindFileServer(uint16_t* scnt) {
	uint16_t counter = 0;
	LC_NodeShortName_t node;
	if (scnt)
		counter = *scnt;
	while (node.NodeID != LC_Broadcast_Address) {
		node = LC_GetActiveNodes(&counter);
		if (node.FileServer)
			break;
	}
	if (scnt)
		*scnt = counter;
	return node;
}
