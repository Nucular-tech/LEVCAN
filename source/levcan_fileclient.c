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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include "levcan_fileclient.h"
#include "levcan_filedef.h"
#include "levcan_internal.h"

#ifndef LEVCAN_FILE_DATASIZE
				#define LEVCAN_FILE_DATASIZE LEVCAN_OBJECT_DATASIZE
				#endif

#if LEVCAN_OBJECT_DATASIZE < 16
				#error "Too small LEVCAN_OBJECT_DATASIZE size for file io!"
				#endif
//extern functions
//private functions
LC_FileResult_t lc_client_sendwait(LC_NodeDescriptor_t *node, void *data, uint16_t size, fOpAck_t *askOut);
void processReceivedData(volatile fRead_t *rxtoread, fOpData_t *opdata, int32_t rsize);
#ifndef LEVCAN_USE_RTOS_QUEUE
void proceedFileClient(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
#endif
//private variables
#ifdef LEVCAN_BUFFER_FILEPRINTF
char lc_printf_buffer[LEVCAN_FILE_DATASIZE - sizeof(fOpData_t)];
uint32_t lc_printf_size = 0;
#endif

LC_Return_t LC_FileClientInit(LC_NodeDescriptor_t *node) {
#ifdef LEVCAN_FILECLIENT
	//File client
	LC_Object_t *initObject = lc_registerSystemObjects(node, 1);
	if (node == 0 || node->Extensions == 0 || initObject == 0)
		return LC_InitError;
#ifdef LEVCAN_USE_RTOS_QUEUE

	initObject->Address = LC_QueueCreate(LEVCAN_MAX_OWN_NODES, sizeof(LC_ObjectData_t));
	((lc_Extensions_t*) node->Extensions)->frxQueue = initObject->Address;

	initObject->Attributes.Writable = 1;
	initObject->Attributes.Queue = 1;
	initObject->Attributes.TCP = 1;
	initObject->MsgID = LC_SYS_FileServer;      //get client requests
	initObject->Size = -LEVCAN_FILE_DATASIZE;      //anysize
#else
	initObject->Address = proceedFileClient;
	initObject->Attributes.Writable = 1;
	initObject->Attributes.Function = 1;
	initObject->Attributes.TCP = 1;
	initObject->MsgID = LC_SYS_FileServer;      //get client requests
	initObject->Size = -LEVCAN_FILE_DATASIZE;      //anysize

#endif
#endif
	((lc_Extensions_t*) node->Extensions)->fnode = LC_Broadcast_Address;
	((lc_Extensions_t*) node->Extensions)->fpos = 0;
	return LC_Ok;
}
#ifndef LEVCAN_USE_RTOS_QUEUE
void proceedFileClient(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	(void) header; //no warnings

	if (size < 2)
		return;
	uint16_t *op = data;
	switch (*op) {
	case fOpAck: {
		if (sizeof(fOpAck_t) == size) {
			fOpAck_t *fac = data;
			((lc_Extensions_t*)node->Extensions)->rxack.Error = fac->Error;
			((lc_Extensions_t*)node->Extensions)->rxack.Operation = fac->Operation; //finish
			((lc_Extensions_t*)node->Extensions)->rxack.Position = fac->Position;
		}
	}
		break;
	case fOpData: {
		fOpData_t *fop = data;
		//check if it is okay
		volatile fRead_t *rxtoread = &((lc_Extensions_t*)node->Extensions)->rxtoread;
		processReceivedData(rxtoread, fop, size);

	}
		break;
	}
}
#endif
/// Open/Create a file
/// @param name File name
/// @param mode Mode flags LC_FileAccess_t
/// @param sender_node Own network node
/// @param server_node Server id, can be LC_Broadcast_Address to find first one
/// @return LC_FileResult_t
LC_FileResult_t LC_FileOpen(LC_NodeDescriptor_t *node, char *name, LC_FileAccess_t mode, uint8_t server_node) {
	//look for any server node
	if (server_node == LC_Broadcast_Address)
		server_node = LC_FindFileServer(node, 0).NodeID;

	//save server
	//((lc_Extensions_t*)node->Extensions)->frxQueue
	((lc_Extensions_t*) node->Extensions)->fnode = server_node;
	//create buffer
	int datasize = sizeof(fOpOpen_t) + strlen(name) + 1;
	char datasend[datasize];
	memset(datasend, 0, datasize);
	//buffer tx file operation
	fOpOpen_t *openf = (fOpOpen_t*) datasend;
	openf->Operation = fOpOpen;
	strcpy(&openf->Name[0], name);
	openf->Mode = mode;
	LC_FileResult_t ret = lc_client_sendwait(node, openf, datasize, 0);
	if (ret != LC_FR_Ok)
		((lc_Extensions_t*) node->Extensions)->fnode = LC_Broadcast_Address; //reset server
	else
		((lc_Extensions_t*) node->Extensions)->fpos = 0;
	return ret;
}

/// Read data from the file
/// @param buffer Buffer to store read data
/// @param btr Number of bytes to read
/// @param br Number of bytes read
/// @param sender_node Own network node
/// @return LC_FileResult_t
LC_FileResult_t LC_FileRead(LC_NodeDescriptor_t *node, char *buffer, uint32_t btr, uint32_t *br) {
	uint16_t attempt;
	LC_NodeShortName_t server;

	if (br == 0 || buffer == 0)
		return LC_FR_InvalidParameter;
	//reset
	*br = 0;
	attempt = 0;
	//look for any server node
	if (((lc_Extensions_t*) node->Extensions)->fnode == LC_Broadcast_Address)
		return LC_FR_FileNotOpened;
	else
		server = LC_GetNode(node, ((lc_Extensions_t*) node->Extensions)->fnode);
	//checks
	if (server.FileServer == 0 || server.NodeID == LC_Broadcast_Address)
		return LC_FR_NodeOffline;

	fOpRead_t readf;
	readf.Operation = fOpRead;
	LC_ObjectRecord_t rec = { 0 };
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;
	rec.NodeID = server.NodeID;
	rec.Address = &readf;
	rec.Size = sizeof(fOpRead_t);
	LC_FileResult_t ret = LC_FR_Ok;
	volatile fRead_t *rxtoread = &((lc_Extensions_t*) node->Extensions)->rxtoread;

	for (uint32_t position = 0; position < btr;) {
		uint32_t toreadnow = btr - position;
		uint32_t globalpos = position + ((lc_Extensions_t*) node->Extensions)->fpos;
		//finish?
		if (toreadnow == 0)
			return LC_FR_Ok;
		if (toreadnow > LEVCAN_FILE_DATASIZE - sizeof(fOpData_t))
			toreadnow = LEVCAN_FILE_DATASIZE - sizeof(fOpData_t);
		if (toreadnow > INT16_MAX)
			toreadnow = INT16_MAX;
		//Prepare receiver
		rxtoread->Buffer = &buffer[position];
		rxtoread->Error = 0;
		rxtoread->Position = UINT32_MAX;
		rxtoread->ReadBytes = toreadnow;
		readf.ToBeRead = toreadnow;
		readf.Position = globalpos; //add global position

#ifdef LEVCAN_USE_RTOS_QUEUE
		LC_QueueReset(((lc_Extensions_t* ) node->Extensions)->frxQueue);
#endif
		LC_Return_t sr = LC_SendMessage(node, &rec, LC_SYS_FileClient);
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
#ifdef LEVCAN_USE_RTOS_QUEUE
		LC_ObjectData_t obj_data = { 0 };
		//wait for receive
		if (LC_QueueReceive(((lc_Extensions_t* ) node->Extensions)->frxQueue, &obj_data, LEVCAN_FILE_TIMEOUT)) {
			fOpData_t *op_data = (fOpData_t*) obj_data.Data;
			if (op_data->Operation == fOpData) {
				processReceivedData(rxtoread, op_data, obj_data.Size);
				lcfree(obj_data.Data);
			}
		}
#else
		//wait 500ms
		for (int time = 0; time < LEVCAN_FILE_TIMEOUT; time++) {
			lcdelay(1);
			if (rxtoread->Position != UINT32_MAX)
				break;
		}
#endif
		//just make some checks
		if (rxtoread->Position != UINT32_MAX && rxtoread->Position == globalpos && rxtoread->ReadBytes <= toreadnow) {
			//received something, data already filled, move pointers
			position += rxtoread->ReadBytes;
			*br += rxtoread->ReadBytes;
			attempt = 0;
			if (rxtoread->ReadBytes < toreadnow)
				break; //receive finished

		} else {
			attempt++;
			if (attempt > 3) {
				ret = LC_FR_NetworkTimeout;
				break;
			}
		}
		//got some errs?
		if (rxtoread->Error && rxtoread->Error != LC_FR_NetworkError) {
			ret = rxtoread->Error;
			break;
		}
	}
	//reset buffer
	((lc_Extensions_t*) node->Extensions)->fpos += *br;
	rxtoread->Buffer = 0;
	return ret;
}

void processReceivedData(volatile fRead_t *rxtoread, fOpData_t *opdata, int32_t rsize) {
	if (rxtoread->Buffer != 0 && opdata->TotalBytes <= rxtoread->ReadBytes && rxtoread->Position == UINT32_MAX
			&& rsize == (int32_t) (opdata->TotalBytes + sizeof(fOpData_t))) {
		memcpy(rxtoread->Buffer, opdata->Data, opdata->TotalBytes);
		rxtoread->ReadBytes = opdata->TotalBytes;
		rxtoread->Error = opdata->Error;
		rxtoread->Position = opdata->Position; //trigger
	} else {
		rxtoread->Error = LC_FR_NetworkError; //data error
		rxtoread->ReadBytes = 0;
		rxtoread->Position = 0;
	}
}

/// Writes data to a file.
/// @param buffer Pointer to the data to be written
/// @param btw Number of bytes to write
/// @param bw Pointer to the variable to return number of bytes written
/// @param sender_node Own network node
/// @return LC_FileResult_t
LC_FileResult_t LC_FileWrite(LC_NodeDescriptor_t *node, const char *buffer, uint32_t btw, uint32_t *bw) {
	LC_FileResult_t ret = LC_FR_Ok;
	uint16_t attempt;
	char writedata[btw + sizeof(fOpData_t)];

	if (bw == 0 || buffer == 0)
		return LC_FR_InvalidParameter;
	//reset
	*bw = 0;
	attempt = 0;

	fOpData_t *writef = (fOpData_t*) &writedata;
	writef->Operation = fOpData;

	for (uint32_t position = 0; position < btw;) {
		uint32_t towritenow = btw - position;
		uint32_t globalpos = position + ((lc_Extensions_t*) node->Extensions)->fpos;

		if (towritenow > LEVCAN_FILE_DATASIZE - sizeof(fOpData_t))
			towritenow = LEVCAN_FILE_DATASIZE - sizeof(fOpData_t);

		//copy data to fOpData_t
		writef->Position = globalpos;
		writef->TotalBytes = towritenow;
		memcpy(&writef->Data[0], &buffer[position], towritenow);
		//setup packet size
		fOpAck_t ask_result = { 0 };
		ret = lc_client_sendwait(node, writef, sizeof(fOpData_t) + towritenow, &ask_result);

		if (ask_result.Operation == fOpAck) {
			//rxtoread->Position is bytes written
			position += ask_result.Position;
			*bw += ask_result.Position;
			attempt = 0;
			if (ask_result.Position < towritenow)
				break; //not all written
		} else {
			attempt++;
			if (attempt > 3) {
				ret = LC_FR_NetworkTimeout;
				break;
			}
		}
		//got some errs?
		if (ret != LC_FR_NetworkError) {
			break;
		}
	}
	((lc_Extensions_t*) node->Extensions)->fpos += *bw;
	return ret;
}

/// Writes line to a file.
/// @param buffer Pointer to the data to be written
/// @param btw Number of bytes to write
/// @param bw Pointer to the variable to return number of bytes written
/// @param sender_node Own network node
/// @return LC_FileResult_t
LC_FileResult_t LC_FilePrintf(LC_NodeDescriptor_t *node, const char *format, ...) {
	va_list ap;
	va_start(ap, format);
	static char buf[(LEVCAN_FILE_DATASIZE > 256) ? 256 : LEVCAN_FILE_DATASIZE];

	LC_FileResult_t result = 0;
	uint32_t size = 0;
	// Print to the local buffer
	size = vsnprintf(buf, sizeof(buf), format, ap);
#ifdef LEVCAN_BUFFER_FILEPRINTF
	char *bufref = buf;
	uint32_t copysize = 0;
	uint32_t maxsize = 0;
	do {
		//send only full buffer
		if (lc_printf_size == sizeof(lc_printf_buffer)) {
			result = LC_FileWrite(node, lc_printf_buffer, lc_printf_size, &lc_printf_size);
			lc_printf_size = 0;
		}
		//fill buffer
		if (size > 0) {
			maxsize = sizeof(lc_printf_buffer) - lc_printf_size;
			if (size > maxsize)
				copysize = maxsize;
			else
				copysize = size;
			memcpy(&lc_printf_buffer[lc_printf_size], bufref, copysize);
			lc_printf_size += copysize;
			size -= copysize;
			bufref += copysize;
		}
	} while (lc_printf_size == sizeof(lc_printf_buffer));
#else
	if (size > 0) {
		// Transfer the buffer to the server
		result = LC_FileWrite(node, buf, size, &size);
	}
#endif
	va_end(ap);
	return result;
}

#ifdef LEVCAN_BUFFER_FILEPRINTF
LC_FileResult_t LC_FilePrintFlush(LC_NodeDescriptor_t *node) {
	LC_FileResult_t res = 0;
	uint32_t size = 0;

	if (lc_printf_size > 0)
		res = LC_FileWrite(node, lc_printf_buffer, lc_printf_size, &size);
	lc_printf_size = 0;
	return res;
}
#endif

///  Move read/write pointer, Expand size
/// @param sender_node Own network node
/// @param server_node Server id, can be LC_Broadcast_Address to find first one
/// @return LC_FileResult_t
LC_FileResult_t LC_FileLseek(LC_NodeDescriptor_t *node, uint32_t position) {
	//create buffer
	fOpLseek_t lseekf;
	lseekf.Operation = fOpLseek;
	lseekf.Position = position;

	fOpAck_t ask_result = { 0 };
	LC_FileResult_t result = lc_client_sendwait(node, &lseekf, sizeof(fOpLseek_t), &ask_result);
	if (ask_result.Operation == fOpAck)
		((lc_Extensions_t*) node->Extensions)->fpos = ask_result.Position; //update position
	return result;
}

/// Get current read/write pointer
/// @param sender_node Own network node
/// @return Pointer to the open file.
uint32_t LC_FileTell(LC_NodeDescriptor_t *node) {
	return ((lc_Extensions_t*) node->Extensions)->fpos;
}

/// Get file size
/// @param sender_node Own network node
/// @return Size of the open file. Can be null if file not opened.
uint32_t LC_FileSize(LC_NodeDescriptor_t *node) {
	//create buffer
	fOpOperation_t closef;
	closef.Operation = fOpAckSize;

	fOpAck_t ask_result = { 0 };
	lc_client_sendwait(node, &closef, sizeof(fOpOperation_t), &ask_result);

	if (ask_result.Operation == fOpAck)
		return ask_result.Position; //file size
	return 0;
}

/// Truncates the file size.
/// @param sender_node Own network node
/// @return LC_FileResult_t
LC_FileResult_t LC_FileTruncate(LC_NodeDescriptor_t *node) {
	//create buffer
	fOpOperation_t closef;
	closef.Operation = fOpTruncate;
	return lc_client_sendwait(node, &closef, sizeof(fOpOperation_t), 0);
}

/// Close an open file
/// @param sender_node Own network node
/// @param server_node Server id, can be LC_Broadcast_Address to find first one
/// @return LC_FileResult_t
LC_FileResult_t LC_FileClose(LC_NodeDescriptor_t *node, uint8_t server_node) {
	LC_NodeShortName_t server;

	//this node may be already closed here, but let it try again
	if (((lc_Extensions_t*) node->Extensions)->fnode == LC_Broadcast_Address) {
		//incoming server known?
		if (server_node == LC_Broadcast_Address)
			server = LC_FindFileServer(node, 0); //search
		else
			server = LC_GetNode(node, server_node); //get it
	} else
		server = LC_GetNode(node, ((lc_Extensions_t*) node->Extensions)->fnode);
	//checks
	if (server.FileServer == 0 || server.NodeID == LC_Broadcast_Address) {
		((lc_Extensions_t*) node->Extensions)->fnode = LC_Broadcast_Address; //reset server anyway
		return LC_FR_NodeOffline;
	}
	((lc_Extensions_t*) node->Extensions)->fnode = server.NodeID;
	//create buffer
	fOpOperation_t closef;
	closef.Operation = fOpClose;
	LC_FileResult_t result = lc_client_sendwait(node, &closef, sizeof(fOpOperation_t), 0);
	//reset server
	((lc_Extensions_t*) node->Extensions)->fnode = LC_Broadcast_Address;
	return result;
}

LC_NodeShortName_t LC_FileGetServer(LC_NodeDescriptor_t *node) {
	const LC_NodeShortName_t nullname = { .NodeID = LC_Broadcast_Address };
	LC_NodeShortName_t server;

	//look for any server node
	if (((lc_Extensions_t*) node->Extensions)->fnode == LC_Broadcast_Address) {
		return nullname;
	} else
		server = LC_GetNode(node, ((lc_Extensions_t*) node->Extensions)->fnode);
	//checks
	if (server.FileServer == 0 || server.NodeID == LC_Broadcast_Address)
		return nullname;

	return server;
}

/// Returns file server short name
/// @param scnt Pointer to stored position for search, can be null
/// @return LC_NodeShortName_t file server
LC_NodeShortName_t LC_FindFileServer(LC_NodeDescriptor_t *node, uint16_t *scnt) {
	uint16_t counter = 0;
	LC_NodeShortName_t nodeSN = { 0 };
	if (scnt)
		counter = *scnt;
	uint16_t resetcount = counter;
	do {
		nodeSN = LC_GetActiveNodes(node, &counter);
		if (nodeSN.FileServer)
			break;
	} while (nodeSN.NodeID != LC_Broadcast_Address && resetcount != counter);

	if (scnt)
		*scnt = counter;
	return nodeSN;
}

LC_FileResult_t lc_client_sendwait(LC_NodeDescriptor_t *node, void *data, uint16_t size, fOpAck_t *askOut) {
	LC_NodeShortName_t server;

	if (node == 0 || node->Extensions == 0
#ifdef LEVCAN_USE_RTOS_QUEUE
						|| ((lc_Extensions_t*)node->Extensions)->frxQueue == 0
				#endif
			) {
		return LC_FR_IntErr;
	}

	//reset ACK
#ifndef LEVCAN_USE_RTOS_QUEUE
	((lc_Extensions_t*)node->Extensions)->rxack = (fOpAck_t ) { 0 };
#endif
	//look for any server node
	if (((lc_Extensions_t*) node->Extensions)->fnode == LC_Broadcast_Address)
		return LC_FR_FileNotOpened;
	else
		server = LC_GetNode(node, ((lc_Extensions_t*) node->Extensions)->fnode);
	//checks
	if (server.FileServer == 0 || server.NodeID == LC_Broadcast_Address)
		return LC_FR_NodeOffline;
	//prepare message
	LC_ObjectRecord_t rec = { 0 };
	rec.Address = data;
	rec.Size = size;
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;
	rec.NodeID = server.NodeID;
	uint16_t attempt = 0;
	for (attempt = 0; attempt < 3; attempt++) {
#ifdef LEVCAN_USE_RTOS_QUEUE
		LC_QueueReset(((lc_Extensions_t* ) node->Extensions)->frxQueue);
#endif
		LC_Return_t sr = LC_SendMessage(node, &rec, LC_SYS_FileClient);
		//send error?
		if (sr) {
			if (sr == LC_BufferFull)
				return LC_FR_NetworkBusy;
			if (sr == LC_MallocFail)
				return LC_FR_MemoryFull;
			return LC_FR_NetworkError;
		}
		//wait
#ifdef LEVCAN_USE_RTOS_QUEUE
		LC_ObjectData_t ack_data = { 0 };
		//wait for receive
		if (LC_QueueReceive(((lc_Extensions_t* ) node->Extensions)->frxQueue, &ack_data, LEVCAN_FILE_TIMEOUT)) {
			if (sizeof(fOpAck_t) == ack_data.Size) {
				//convert data to ack type
				fOpAck_t ack = *((fOpAck_t*) ack_data.Data);
				if (ack.Operation == fOpAck) {
					if (askOut)
						*askOut = ack;
					lcfree(ack_data.Data);
					return ack.Error; //Finish!
				}
			}
			lcfree(ack_data.Data);
		}
#else
		for (int time = 0; time < LEVCAN_FILE_TIMEOUT; time++) {
			lcdelay(1);
			if (((lc_Extensions_t*)node->Extensions)->rxack.Operation == fOpAck) {
				if (askOut)
					*askOut = ((lc_Extensions_t*)node->Extensions)->rxack;
				return ((lc_Extensions_t*)node->Extensions)->rxack.Error; //Finish!
			}
		}
#endif
	}

	return LC_FR_NetworkTimeout;
}

