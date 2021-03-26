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
#include <math.h>

#include "levcan.h"
#include "levcan_internal.h"
#include "levcan_fileclient.h"
#include "levcan_fileserver.h"
#include "levcan_filedef.h"

#if	defined(lcmalloc) && defined(lcfree)
#else
#error "You should define lcmalloc, lcfree for levcan_fileserver.c!"
#endif

typedef struct {
	uint16_t Operation;
	uint16_t Size;
	union {
		uint32_t Position;
		LC_FileAccess_t Mode;
	};
	uint8_t NodeID;
	char* Data;
} fOpDataAdress_t;

typedef struct {
	void* FileObject;
	uint16_t Timeout;
	uint16_t LastError;
	uint8_t NodeID;
	void* Next;
	void* Previous;
} fSrvObj;

//private functions
fSrvObj* findFile(uint8_t source);
LC_FileResult_t sendAck(LC_NodeDescriptor_t* node, uint32_t position, uint16_t error, uint8_t receiver);
LC_FileResult_t deleteFSObject(fSrvObj* obj);

//server request fifo
fOpDataAdress_t fsFIFO[LEVCAN_MAX_TABLE_NODES];
volatile uint16_t fsFIFO_in, fsFIFO_out;

//server stored open files
volatile fSrvObj* file_start;
volatile fSrvObj* file_end;
volatile int initFS = 0;

void proceedFileServer(LC_NodeDescriptor_t* node, LC_Header_t header, void* data, int32_t size) {
	if (size < 2 || initFS == 0)
		return;
	uint16_t* op = data;
	uint16_t gotfifo = 0;
	if (fsFIFO_in == ((fsFIFO_out - 1 + LEVCAN_MAX_TABLE_NODES) % LEVCAN_MAX_TABLE_NODES)) {
		sendAck(node, 0, LC_FR_MemoryFull, header.Source);
		return; //buffer full
	}
	fOpDataAdress_t* fsinput = &fsFIFO[fsFIFO_in];

	//fill in data
	switch (*op) {
	case fOpOpen: {
		fOpOpen_t* fop = data;
		//fill data
		fsinput->Mode = fop->Mode;
		fsinput->Size = 0;
		//copy name to new buffer
		size_t length = strlen(fop->Name) + 1;
		//todo make error if name too long
		if (length > 512)
			length = 512;
		char* name = lcmalloc(length);
		if (name) {
			strncpy(name, fop->Name, length);
			name[length - 1] = 0;
		}
		fsinput->Data = name;

		//add to fifo
		gotfifo = 1;
	}
				  break;
	case fOpRead: {
		fOpRead_t* fop = data;
		fsinput->Position = fop->Position;
		fsinput->Size = fop->ToBeRead;
		fsinput->Data = 0;
		//add to fifo
		gotfifo = 1;
	}
				  break;
	case fOpTruncate:
	case fOpAckSize:
	case fOpClose: {
		fsinput->Position = 0;
		fsinput->Size = 0;
		fsinput->Data = 0;
		//add to fifo
		gotfifo = 1;
	}
				   break;
	case fOpLseek: {
		if (size == sizeof(fOpLseek_t)) {
			fOpLseek_t* fop = data;
			fsinput->Position = fop->Position;
			fsinput->Size = 0;
			fsinput->Data = 0;
			//add to fifo
			gotfifo = 1;
		}
	}
				   break;
	case fOpData: {
		fOpData_t* fop = data;
		//fill data
		fsinput->Size = fop->TotalBytes;
		fsinput->Position = fop->Position;
		//check data size
		if (size == (int32_t)(fop->TotalBytes + sizeof(fOpData_t))) {
			//copy data to new buffer
			char* data_write = lcmalloc(fop->TotalBytes);
			if (data_write)
				memcpy(data_write, &fop->Data[0], fop->TotalBytes);
			fsinput->Data = data_write;
		}
		else {
			fsinput->Data = 0;
			fsinput->Size = 0;
		}
		//add to fifo
		gotfifo = 1;
	}
				  break;
	}
	if (gotfifo) {
		fsinput->Operation = *op;
		fsinput->NodeID = header.Source;
		fsFIFO_in = (fsFIFO_in + 1) % LEVCAN_MAX_TABLE_NODES;
		//send request to process messages.
		//make your own implementation of LC_FileServerOnReceive to use semaphore for main file process
		//this should speed-up communication
		LC_FileServerOnReceive();
	}
}

//default ack
const fOpAck_t fask_open_many = { .Operation = fOpAck,.Position = 0,.Error = LC_FR_TooManyOpenFiles };
const fOpAck_t fask_mem_out = { .Operation = fOpAck,.Position = 0,.Error = LC_FR_MemoryFull };
const fOpAck_t fask_deni = { .Operation = fOpAck,.Position = 0,.Error = LC_FR_Denied };

LC_Return_t LC_FileServerInit(LC_NodeDescriptor_t* node) {
	//File server
	LC_Object_t* initObject = lc_registerSystemObjects(node, 1);
	if (initObject == 0)
		return LC_InitError;
	initObject->Address = proceedFileServer;
	initObject->Attributes.Writable = 1;
	initObject->Attributes.Function = 1;
	initObject->Attributes.TCP = 1;
	initObject->MsgID = LC_SYS_FileClient;//get client requests
	initObject->Size = INT32_MIN;//anysize

	node->ShortName.FileServer = 1;
	return LC_Ok;
}

LC_Return_t LC_FileServer(LC_NodeDescriptor_t* node, uint32_t tick) {
	if (initFS == 0) {
		fsFIFO_in = 0;
		fsFIFO_out = 0;
		memset(fsFIFO, 0, sizeof(fsFIFO));
		file_start = 0;
		file_end = 0;
		initFS = 1;
	}
	if (node == 0 || node->Driver == 0)
		return LC_DataError;
	if (node->State != LCNodeState_Online)
		return LC_NodeOffline;

	for (; fsFIFO_in != fsFIFO_out; fsFIFO_out = (fsFIFO_out + 1) % LEVCAN_MAX_TABLE_NODES) {
		//proceed FS FIFO
		fOpDataAdress_t* fsinput = &fsFIFO[fsFIFO_out];
		LC_ObjectRecord_t rec = { 0 };
		rec.NodeID = fsinput->NodeID;
		rec.Attributes.TCP = 1;
		rec.Attributes.Priority = LC_Priority_Low;

		switch (fsinput->Operation) {
		case fOpOpen: {
			if (findFile(fsinput->NodeID)) {
				//free name
				lcfree(fsinput->Data);
				fsinput->Data = 0;
				//already opened file
				sendAck(node, 0, LC_FR_TooManyOpenFiles, fsinput->NodeID);
			}
			else {
				void* file;
				LC_FileResult_t res = lcfopen(&file, fsinput->Data, fsinput->Mode);
				//free name
				lcfree(fsinput->Data);
				fsinput->Data = 0;
				//Prepare answer
				if (res == 0 && file != 0) {
					//looks fine!
					fSrvObj* fileNode = lcmalloc(sizeof(fSrvObj));
					if (fileNode == 0) {
						//can't do anything, memory fail
						sendAck(node, 0, LC_FR_MemoryFull, fsinput->NodeID); //file error
						lcfclose(file);
						continue;
					}
					//prepare node file
					fileNode->FileObject = file;
					fileNode->LastError = res;
					fileNode->NodeID = fsinput->NodeID;
					fileNode->Timeout = 0;
					//put in array
					if (file_start == 0) {
						//no objects in tx array
						fileNode->Previous = 0;
						fileNode->Next = 0;
						file_start = fileNode;
						file_end = fileNode;
					}
					else {
						//add to the end
						fileNode->Previous = (intptr_t*)file_end;
						fileNode->Next = 0;
						file_end->Next = (intptr_t*)fileNode;
						file_end = fileNode;
					}
					//done!
				}
				if (file == 0 && res == 0)
					res = LC_FR_MemoryFull;
				sendAck(node, 0, res, fsinput->NodeID);
			}
		}
					  break;
		case fOpRead: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened/created file for this node?
			if (fileNode) {
				fileNode->Timeout = 0;
				//get current position
				uint32_t filepos = lcftell(fileNode->FileObject);
				LC_FileResult_t result = 0;
				//try to move
				if (fsinput->Position != filepos) {
					result = lcflseek(fileNode->FileObject, fsinput->Position);
					filepos = lcftell(fileNode->FileObject);
				}
				if (result) {
					//error happened
					sendAck(node, 0, result, fsinput->NodeID);
					continue;
				}
				uint32_t btr = fsinput->Size;
				if (fsinput->Position != filepos)
					btr = 0; //pointer not moved

				fOpData_t* buffer = lcmalloc(sizeof(fOpData_t) + btr);
				if (buffer == 0) {
					sendAck(node, 0, LC_FR_MemoryFull, fsinput->NodeID); //file error
					continue;
				}
				buffer->Operation = fOpData;
				result = lcfread(fileNode->FileObject, &buffer->Data[0], btr, &btr);
				buffer->Error = result;
				buffer->Position = filepos;
				buffer->TotalBytes = btr;
				//send
				rec.Address = buffer;
				rec.Size = sizeof(fOpData_t) + btr;
				rec.Attributes.Cleanup = 1;

				if (LC_SendMessage(node, &rec, LC_SYS_FileServer))
					lcfree(buffer);
			}
			else {
				sendAck(node, 0, LC_FR_FileNotOpened, fsinput->NodeID);
			}
		}
					  break;
		case fOpData: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened/created file for this node?
			if (fileNode) {
				fileNode->Timeout = 0;

				if (fsinput->Size == 0) {
					sendAck(node, 0, LC_FR_NetworkError, fsinput->NodeID);
				}
				else if (fsinput->Data == 0) {
					sendAck(node, 0, LC_FR_MemoryFull, fsinput->NodeID);
				}
				else {
					//get current position
					uint32_t filepos = lcftell(fileNode->FileObject);
					LC_FileResult_t result = 0;
					//try to move
					if (fsinput->Position != filepos) {
						result = lcflseek(fileNode->FileObject, fsinput->Position);
						filepos = lcftell(fileNode->FileObject);
					}
					if (result) {
						//error happened
						sendAck(node, 0, result, fsinput->NodeID);
					}
					else {
						uint32_t btw = fsinput->Size;
						if (fsinput->Position != filepos)
							btw = 0; //pointer not moved
						//write file
						result = lcfwrite(fileNode->FileObject, fsinput->Data, btw, &btw);
						sendAck(node, btw, result, fsinput->NodeID);
					}
				}
			}
			else {
				sendAck(node, 0, LC_FR_FileNotOpened, fsinput->NodeID);
			}
			//free data
			if (fsinput->Data)
				lcfree(fsinput->Data);
		}
					  break;
		case fOpClose: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened file for this node?
			LC_FileResult_t rslt = LC_FR_Ok;
			if (fileNode) {
				rslt = deleteFSObject(fileNode);
			}
			else
				rslt = LC_FR_FileNotOpened;
			sendAck(node, 0, rslt, fsinput->NodeID);
		}
					   break;
		case fOpLseek: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened file for this node?
			LC_FileResult_t rslt = LC_FR_Denied;
			uint32_t filepos = 0;
			if (fileNode) {
				rslt = lcflseek(fileNode->FileObject, fsinput->Position);
				filepos = lcftell(fileNode->FileObject);
			}
			else
				rslt = LC_FR_FileNotOpened;
			sendAck(node, filepos, rslt, fsinput->NodeID);
		}
					   break;
		case fOpAckSize: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened file for this node?
			LC_FileResult_t rslt = LC_FR_Ok;
			uint32_t filesize = 0;
			if (fileNode) {
				filesize = lcfsize(fileNode->FileObject);
			}
			else
				rslt = LC_FR_FileNotOpened;
			sendAck(node, filesize, rslt, fsinput->NodeID);
		}
						 break;
		case fOpTruncate: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened file for this node?
			LC_FileResult_t rslt = LC_FR_Ok;
			if (fileNode) {
				rslt = lcftruncate(fileNode->FileObject);
			}
			else
				rslt = LC_FR_FileNotOpened;
			sendAck(node, 0, rslt, fsinput->NodeID);
		}
						  break;
		}
	}
	static uint16_t timesec = 0;
	timesec += tick;
	if (timesec >= 1000) {
		timesec -= 1000;
		fSrvObj* next;
		for (fSrvObj* obj = (fSrvObj*)file_start; obj != 0; obj = next) {
			next = (fSrvObj*)obj->Next;
			obj->Timeout++;
			//5 minute delete
			if (obj->Timeout > 60 * 5)
				deleteFSObject(obj);
		}
	}

	return LC_Ok;
}

LC_FileResult_t sendAck(LC_NodeDescriptor_t* node, uint32_t position, uint16_t error, uint8_t receiver) {
	LC_ObjectRecord_t rec = { 0 };
	rec.NodeID = receiver;
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;

	switch (error) {
	case LC_FR_MemoryFull:
		rec.Address = (void*)&fask_mem_out;
		rec.Size = sizeof(fask_mem_out);
		LC_SendMessage(node, &rec, LC_SYS_FileServer);
		return LC_FR_Ok;

	case LC_FR_TooManyOpenFiles:
		rec.Address = (void*)&fask_open_many;
		rec.Size = sizeof(fask_open_many);
		LC_SendMessage(node, &rec, LC_SYS_FileServer);
		return LC_FR_Ok;

	case LC_FR_Denied:
		rec.Address = (void*)&fask_deni;
		rec.Size = sizeof(fask_deni);
		LC_SendMessage(node, &rec, LC_SYS_FileServer);
		return LC_FR_Ok;
	}
	fOpAck_t* ack = lcmalloc(sizeof(fOpAck_t));
	if (ack == 0) {
		//can't do anything, memory fail
		rec.Address = (void*)&fask_mem_out;
		rec.Size = sizeof(fask_mem_out);
		LC_SendMessage(node, &rec, LC_SYS_FileServer);
		return LC_FR_MemoryFull;
	}
	//reply
	ack->Operation = fOpAck;
	ack->Position = position;
	ack->Error = error;

	rec.Address = ack;
	rec.Size = sizeof(fOpAck_t);
	rec.Attributes.Cleanup = 1;
	if (LC_SendMessage(node, &rec, LC_SYS_FileServer))
		lcfree(ack); //can't send, clean now
	return LC_FR_Ok;
}

fSrvObj* findFile(uint8_t source) {
	fSrvObj* obj = (fSrvObj*)file_start;
	while (obj) {
		//search file for specified nodeID
		if (obj->NodeID == source) {
			return obj;
		}
		obj = (fSrvObj*)obj->Next;
	}
	return 0;
}

LC_FileResult_t deleteFSObject(fSrvObj* obj) {
	if (obj->Previous)
		((fSrvObj*)obj->Previous)->Next = obj->Next; //junction
	else {
#ifdef LEVCAN_TRACE
		if (file_start != obj) {
			trace_printf("Start object error\n");
		}
#endif
		file_start = (fSrvObj*)obj->Next; //Starting
		if (file_start != 0)
			file_start->Previous = 0;
	}
	if (obj->Next) {
		((fSrvObj*)obj->Next)->Previous = obj->Previous;
	}
	else {
#ifdef LEVCAN_TRACE
		if (file_end != obj) {
			trace_printf("End object error\n");
		}
#endif
		file_end = (fSrvObj*)obj->Previous; //ending
		if (file_end != 0)
			file_end->Next = 0;
	}

	LC_FileResult_t resul = lcfclose(obj->FileObject);
	//free this object
	lcfree(obj);
	return resul;
}
