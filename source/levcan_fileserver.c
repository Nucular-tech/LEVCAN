/*
 * Simple file i/o operations over LEVCAN protocol, server side
 * levcan_fileserver.c
 *
 *  Created on: 15 Aug 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "levcan.h"
#include "levcan_fileclient.h"
#include "levcan_fileserver.h"
#include "levcan_filedef.h"

#if	defined(lcmalloc) && defined(lcfree)
extern void *lcmalloc(uint32_t size);
extern void lcfree(void *pointer);
#else
#error "You should define lcmalloc, lcfree for levcan_fileserver.c!"
#endif

typedef struct {
	uint16_t Operation;
	uint16_t Size;
	union {
		int32_t Position;
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

//extern functions
extern LC_FileResult_t lcfopen(void** fileObject, char* name, LC_FileAccess_t mode);
extern int32_t lcftell(void* fileObject);
extern LC_FileResult_t lcflseek(void* fileObject, int32_t pointer);
extern LC_FileResult_t lcfread(void* fileObject, char* buffer, int32_t bytesToRead, int32_t* bytesReaded);
extern LC_FileResult_t lcfclose(void* fileObject);
//private functions
fSrvObj* findFile(uint8_t source);
LC_FileResult_t sendAck(uint32_t position, uint16_t error, void* sender, uint8_t node);
LC_FileResult_t deleteFSObject(fSrvObj* obj);
//server request fifo
fOpDataAdress_t fsFIFO[LEVCAN_MAX_TABLE_NODES];
volatile uint16_t fsFIFO_in, fsFIFO_out;
//server stored open files
volatile fSrvObj* file_start;
volatile fSrvObj* file_end;
volatile int initFS = 0;

void proceedFileServer(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size) {
	if (size < 2 || initFS == 0)
		return;
	uint16_t* op = data;
	uint16_t gotfifo = 0;
	if (fsFIFO_in == ((fsFIFO_out - 1 + LEVCAN_MAX_TABLE_NODES) % LEVCAN_MAX_TABLE_NODES)) {
		//todo send ACK full
		return;//buffer full
	}
	fOpDataAdress_t* fsinput = &fsFIFO[fsFIFO_in];
	//int empty = 0;
	//if (fsFIFO_in == fsFIFO_out)
	//	empty = 1;
	//fill in data
	switch (*op) {
	case fOpOpen: {
		fOpOpen_t* fop = data;
		//fill data
		fsinput->Mode = fop->Mode;
		fsinput->Size = 0;
		//copy name to new buffer
		size_t length = strlen(fop->Name) + 1;
		char* name = lcmalloc(length);
		name[0] = 0;
		if (name && length < 512)
			strcpy(name, fop->Name);
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
	}
	if (gotfifo) {
		fsinput->Operation = *op;
		fsinput->NodeID = header.Source;
		fsFIFO_in = (fsFIFO_in + 1) % LEVCAN_MAX_TABLE_NODES;
	}
}

//default ack
const fOpAck_t fask_open_many = { .Operation = fOpAck, .Position = 0, .Error = LC_FR_TooManyOpenFiles };
const fOpAck_t fask_mem_out = { .Operation = fOpAck, .Position = 0, .Error = LC_FR_MemoryFull };
const fOpAck_t fask_deni = { .Operation = fOpAck, .Position = 0, .Error = LC_FR_Denied };

void LC_FileServer(uint32_t tick, void* server) {
	if (initFS == 0) {
		fsFIFO_in = 0;
		fsFIFO_out = 0;
		memset(fsFIFO, 0, sizeof(fsFIFO));
		file_start = 0;
		file_end = 0;
		initFS = 1;
	}

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
				sendAck(0, fsinput->NodeID, server, LC_FR_TooManyOpenFiles);
			} else {
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
						sendAck(0, fsinput->NodeID, server, LC_FR_MemoryFull); //file error
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
					} else {
						//add to the end
						fileNode->Previous = (intptr_t*) file_end;
						fileNode->Next = 0;
						file_end->Next = (intptr_t*) fileNode;
						file_end = fileNode;
					}
					//done!
				}
				if (file == 0 && res == 0)
					res = LC_FR_MemoryFull;
				sendAck(0, fsinput->NodeID, server, res);
			}
		}
			break;
		case fOpRead: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened/created file for this node?
			if (fileNode) {
				fileNode->Timeout = 0;
				//get current position
				int32_t filepos = lcftell(fileNode->FileObject);
				LC_FileResult_t result = 0;
				//try to move
				if (fsinput->Position != filepos) {
					result = lcflseek(fileNode->FileObject, fsinput->Position);
					filepos = lcftell(fileNode->FileObject);
				}
				if (result) {
					//error happened
					sendAck(0, fsinput->NodeID, server, result);
					continue;
				}
				int32_t btr = fsinput->Size;
				if (fsinput->Position != filepos)
					btr = 0; //pointer not moved

				fOpData_t* buffer = lcmalloc(sizeof(fOpData_t) + btr);
				if (buffer == 0) {
					sendAck(0, fsinput->NodeID, server, LC_FR_MemoryFull); //file error
					continue;
				}
				buffer->Operation = fOpData;
				result = lcfread(fileNode->FileObject, &buffer->Data[0], btr, &btr);
				buffer->Error = result;
				buffer->Position = filepos;
				buffer->TotalRead = btr;
				//send
				rec.Address = buffer;
				rec.Size = sizeof(fOpData_t) + btr;
				rec.Attributes.Cleanup = 1;

				if (LC_SendMessage(server, &rec, LC_SYS_FileServer))
					lcfree(buffer);
			} else {
				sendAck(0, fsinput->NodeID, server, LC_FR_Denied);
			}
		}
			break;
		case fOpClose: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened file for this node?
			LC_FileResult_t rslt = LC_FR_Ok;
			if (fileNode)
				rslt = deleteFSObject(fileNode);
			sendAck(0, fsinput->NodeID, server, rslt);
		}
			break;
		case fOpLseek: {
			fSrvObj* fileNode = findFile(fsinput->NodeID);
			//do we have opened file for this node?
			LC_FileResult_t rslt = LC_FR_Denied;
			int32_t filepos = 0;
			if (fileNode) {
				rslt = lcflseek(fileNode->FileObject, fsinput->Position);
				filepos = lcftell(fileNode->FileObject);
			}
			sendAck(filepos, fsinput->NodeID, server, rslt);
		}
			break;
		}
	}
	static uint16_t timesec = 0;
	timesec += tick;
	if (timesec >= 1000) {
		timesec -= 1000;
		fSrvObj* next;
		for (fSrvObj* obj = file_start; obj != 0; obj = next) {
			next = (fSrvObj*) obj->Next;
			obj->Timeout++;
			//5 minute delete
			if (obj->Timeout > 60 * 5)
				deleteFSObject(obj);
		}
	}
}

LC_FileResult_t sendAck(uint32_t position, uint16_t error, void* sender, uint8_t node) {
	LC_ObjectRecord_t rec = { 0 };
	rec.NodeID = node;
	rec.Attributes.TCP = 1;
	rec.Attributes.Priority = LC_Priority_Low;

	switch (error) {
	case LC_FR_MemoryFull:
		rec.Address = &fask_mem_out;
		rec.Size = sizeof(fask_mem_out);
		LC_SendMessage(sender, &rec, LC_SYS_FileServer);
		return LC_FR_Ok;

	case LC_FR_TooManyOpenFiles:
		rec.Address = &fask_open_many;
		rec.Size = sizeof(fask_open_many);
		LC_SendMessage(sender, &rec, LC_SYS_FileServer);
		return LC_FR_Ok;

	case LC_FR_Denied:
		rec.Address = &fask_deni;
		rec.Size = sizeof(fask_deni);
		LC_SendMessage(sender, &rec, LC_SYS_FileServer);
		return LC_FR_Ok;
	}
	fOpAck_t* ack = lcmalloc(sizeof(fOpAck_t));
	if (ack == 0) {
		//can't do anything, memory fail
		rec.Address = &fask_mem_out;
		rec.Size = sizeof(fask_mem_out);
		LC_SendMessage(sender, &rec, LC_SYS_FileServer);
		return LC_FR_MemoryFull;
	}
	//reply
	ack->Operation = fOpAck;
	ack->Position = position;
	ack->Error = error;

	rec.Address = ack;
	rec.Size = sizeof(fOpAck_t);
	rec.Attributes.Cleanup = 1;
	if (LC_SendMessage(sender, &rec, LC_SYS_FileServer))
		lcfree(ack);
	return LC_FR_Ok;
}

fSrvObj* findFile(uint8_t source) {
	fSrvObj* obj = file_start;
	while (obj) {
		//search file for specified nodeID
		if (obj->NodeID == source) {
			return obj;
		}
		obj = (fSrvObj*) obj->Next;
	}
	return 0;
}

LC_FileResult_t deleteFSObject(fSrvObj* obj) {
	if (obj->Previous)
		((fSrvObj*) obj->Previous)->Next = obj->Next; //junction
	else {
#ifdef LEVCAN_TRACE
		if (file_start != obj) {
			trace_printf("Start object error\n");
		}
#endif
		file_start = (fSrvObj*) obj->Next; //Starting
		if (file_start != 0)
			file_start->Previous = 0;
	}
	if (obj->Next) {
		((fSrvObj*) obj->Next)->Previous = obj->Previous;
	} else {
#ifdef LEVCAN_TRACE
		if (file_end != obj) {
			trace_printf("End object error\n");
		}
#endif
		file_end = (fSrvObj*) obj->Previous; //ending
		if (file_end != 0)
			file_end->Next = 0;
	}

	LC_FileResult_t resul = lcfclose(obj->FileObject);
	//free this object
	lcfree(obj);
	return resul;
}
