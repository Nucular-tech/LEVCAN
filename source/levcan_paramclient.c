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
#include "levcan_paramclient.h"
#include "levcan_paraminternal.h"
#include <string.h>

#ifndef LEVCAN_USE_RTOS_QUEUE
 //#error "Queues needed for parameter client exchange. Define LEVCAN_USE_RTOS_QUEUE"
#endif
#ifdef LEVCAN_MEM_STATIC
#error "Can't use static memory for parameters client. Undefine LEVCAN_MEM_STATIC"
#endif

void *paramQueue[LEVCAN_MAX_OWN_NODES] = { 0 };
//LC_Object_t *obj[LEVCAN_MAX_OWN_NODES] = { 0 };
LC_ObjectRecord_t objRec[LEVCAN_MAX_OWN_NODES] = { 0 };

#define OBJ_PARAM_SIZE (LC_SYS_ParametersValue - LC_SYS_ParametersData + 1)

extern LC_Object_t* lc_registerSystemObjects(LC_NodeDescriptor_t *node, uint8_t count);
static void clearQueueAndInit(uint8_t id, uint8_t from_node, void* queue);
static LC_Return_t requestData(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, void *outData, uint16_t dataSize, uint16_t command);

LC_Return_t lcp_clientInit(LC_NodeDescriptor_t *node, uint8_t nodeIndex) {
	paramQueue[nodeIndex] = LC_QueueCreate(5, sizeof(LC_ObjectData_t));
	LC_Object_t *initObject = lc_registerSystemObjects(node, OBJ_PARAM_SIZE);
	if (initObject == 0 || paramQueue[nodeIndex] == 0) {
		return LC_MallocFail;
	}
	//obj[nodeIndex] = initObject;
	//prepare specific record, we need to strictly sort out other messages form senders
	//objRec[nodeIndex].Address = 0;
	objRec[nodeIndex].Attributes.Queue = 1;
	objRec[nodeIndex].Attributes.Writable = 1;
	objRec[nodeIndex].Size = -(LEVCAN_PARAM_MAX_TEXTSIZE + LEVCAN_PARAM_MAX_NAMESIZE);
	//dont place address here atm, we dont need junk
	objRec[nodeIndex].NodeID = LC_Invalid_Address;
	//objects needed to define message ID and store objRecord
	int objID = 0;
	for (; objID < OBJ_PARAM_SIZE; objID++) {
		initObject[objID].Attributes.Record = 1;
		initObject[objID].Address = &objRec[nodeIndex];
		initObject[objID].MsgID = LC_SYS_ParametersData + objID;
		initObject[objID].Size = 1;
	}
	return LC_Ok;
}

LC_Return_t LCP_RequestEntry(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, LCPC_Entry_t *out_entry) {

	memset(out_entry, 0, sizeof(out_entry));
	return requestData(mynode, from_node, directory_index, entry_index, out_entry, sizeof(out_entry), lcp_reqFullEntry);
}


LC_Return_t LCP_RequestDirectory(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, LCPC_Directory_t *out_directory) {

	memset(out_directory, 0, sizeof(out_directory));
	return requestData(mynode, from_node, directory_index, 0, out_directory, sizeof(out_directory), lcp_reqDirectoryInfo);

	/*
	LC_Return_t result = LC_Ok;
	if (error.ErrorCode) {
		result = error.ErrorCode;
		//some errors happened, clean up everything
		if (bufferDir.Name) {
			lcfree((void*)bufferDir.Name);
		}

	}
	else {
		//check received data
		if (directdata.NameSize > 0) {
			directdata.NameSize--; //skip 0s
		}
		if (nameDataSize != directdata.NameSize) {
			//text error
		}
		if (bufferDir.DirectoryIndex != directory_index) {
			//index error
		}
	}
	//thread safe
	*out_directory = bufferDir;
	return result;*/
}

LC_Return_t LCP_RequestValue(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, intptr_t *outVariable, uint16_t varSize) {
	return requestData(mynode, from_node, directory_index, entry_index, outVariable, varSize, lcp_reqVariable);
}

static LC_Return_t requestData(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, void *outData, uint16_t dataSize, uint16_t command) {
	LC_ObjectRecord_t sendReq = { .NodeID = from_node,.Attributes.Priority = LC_Priority_Low,.Attributes.TCP = 1 };
	lc_request_entry_t request = { 0 };
	lc_entry_data_t entrydata = { 0 }; //toBeCleaned
	lc_directory_data_t directdata = { 0 }; //toBeCleaned
	uint32_t nameDataSize = 0, textDataSize = 0;
	lc_request_error_t error = { 0 };
	LCPC_Entry_t bufferEntry = { 0 };
	LCPC_Directory_t bufferDir = { 0 };

	int id = LC_GetMyNodeIndex(mynode);
	if (id < 0) {
		return LC_NodeOffline;
	}
	if (from_node >= LC_Null_Address || outData == 0 || dataSize == 0) {
		return LC_DataError;
	}

	intptr_t *queue = paramQueue[id];
	LC_ObjectData_t objData;
	//clean
	bufferEntry.Mode = LCP_Invalid;
	//request full parameter
	request.Command = command;
	request.Directory = directory_index;
	request.Entry = entry_index;

	sendReq.Address = &request;
	if (command == lcp_reqDirectoryInfo) {
		//same but size smaller
		sendReq.Size = sizeof(lc_request_directory_t);
	}
	else {
		sendReq.Size = sizeof(request);
	}
	//prepare receive
	clearQueueAndInit(id, from_node, queue);
	//send request!
	LC_SendMessage(mynode, &sendReq, LC_SYS_ParametersRequest);
	lcp_reqCommand_t sequence = command;
	if (command == lcp_reqDirectoryInfo) {
		//directory doesnt needs so much
		sequence = lcp_reqName | lcp_reqData;
	}
	//wait for all data
	for (int attempt = 0; sequence != 0 && attempt < 3;) {
		int result = LC_QueueReceive(queue, &objData, 500);
		//todo wrong sender filter
		if (result) {
			//data input
			switch (objData.Header.MsgID) {
			case LC_SYS_ParametersData: {
				if (command == lcp_reqDirectoryInfo) { //directory request
					if (objData.Size == sizeof(lc_directory_data_t)) {
						//description of entry
						directdata = *((lc_directory_data_t*)objData.Data);
						bufferDir.DirectoryIndex = directdata.DirectoryIndex;
						bufferDir.Size = directdata.EntrySize;
					}
				}
				else { //entry request
					if (objData.Size == sizeof(lc_entry_data_t)) {
						//description of entry, later used for checking
						entrydata = *((lc_entry_data_t*)objData.Data);
						bufferEntry.DescSize = entrydata.DescSize;
						bufferEntry.EntryType = entrydata.EntryType;
						bufferEntry.Mode = entrydata.Mode;
						bufferEntry.VarSize = entrydata.VarSize;
						bufferEntry.EntryIndex = entrydata.EntryIndex;
						bufferEntry.TextSize = entrydata.TextSize;

						if (bufferEntry.Mode & LCP_WriteOnly || bufferEntry.EntryType == LCP_Folder || bufferEntry.VarSize == 0) {
							sequence &= ~lcp_reqVariable; //skip value reception
						}
						if (bufferEntry.DescSize == 0 || bufferEntry.EntryType == LCP_Folder) {
							sequence &= ~lcp_reqDescriptor;
						}
					}
				}

				if (objData.Size == sizeof(lc_request_error_t)) {
					//only for errors, like access or end of list
					error = *((lc_request_error_t*)objData.Data);
					sequence = 0;
				}

				lcfree(objData.Data);
				sequence &= ~lcp_reqData;
			}
										break;

			case LC_SYS_ParametersName: {
				if (command == lcp_reqDirectoryInfo) { //directory request
					bufferDir.Name = (char*)objData.Data;
				}
				else { //entry request
					bufferEntry.Name = (char*)objData.Data;
				}
				nameDataSize = objData.Size;

				sequence &= ~lcp_reqName;
			}
										break;

			case LC_SYS_ParametersText: {
				bufferEntry.TextData = (char*)objData.Data;
				textDataSize = objData.Size;
				sequence &= ~lcp_reqText;
			}
										break;

			case LC_SYS_ParametersDescriptor: {
				bufferEntry.Descriptor = objData.Data;
				bufferEntry.DescSize = objData.Size;
				sequence &= ~lcp_reqDescriptor;
			}
											  break;

			case LC_SYS_ParametersValue: {
				bufferEntry.Variable = objData.Data;
				bufferEntry.VarSize = objData.Size;
				sequence &= ~lcp_reqVariable;
			}
										 break;

			default:
				if (objData.Size > 0) {
					lcfree(objData.Data);
				}
				break;
			}
			//may happen after name or Data
			if (((sequence & (lcp_reqName | lcp_reqData)) == 0) && (nameDataSize - 1 == bufferEntry.TextSize)) {
				sequence &= ~lcp_reqText; //no text data will be
			}
		}
		else {
			attempt++;
			//request leftovers
			request.Command = sequence;
			if (command == lcp_reqDirectoryInfo) {
				//directory doesnt needs so much
				sequence = lcp_reqName | lcp_reqData;
			}
			LC_SendMessage(mynode, &sendReq, LC_SYS_ParametersRequest);
			if (attempt > 2) {
				error.ErrorCode = LC_Timeout;
			}
			//timeout, don't need to wait
			break;
		}

	}

	//stop receive
	objRec[id].NodeID = LC_Invalid_Address;
	objRec[id].Address = 0;
	LC_Return_t result = LC_Ok;

	if (error.ErrorCode) {
		result = error.ErrorCode;
		//some errors happened, clean up everything
		LCP_CleanEntry(&bufferEntry);
		LCP_CleanDirectory(&bufferDir);
	}
	else {
		//check received data
		uint32_t totaltext = 0; //nameDataSize + textDataSize;
		if (nameDataSize > 0)
			totaltext += nameDataSize - 1; //should be 0's at end
		if (textDataSize > 0)
			totaltext += textDataSize - 1; //should be 0's at end

		if (totaltext != entrydata.TextSize) {
			//text error
		}
		if (entrydata.DescSize != bufferEntry.DescSize) {
		}
		if (entrydata.VarSize != bufferEntry.VarSize) {
		}
	}
	//thread safe
	if (command == lcp_reqVariable) {
		if (bufferEntry.VarSize == dataSize) {
			memcpy(outData, bufferEntry.Variable, dataSize);
			LCP_CleanEntry(&bufferEntry); //clean, since data copied
		}
		else {
			result = LC_DataError;
		}
	}
	else if (command == lcp_reqDirectoryInfo) {
		*(LCPC_Directory_t*)outData = bufferDir;
		LCP_CleanEntry(&bufferEntry); //if something was received
	}
	else {
		*(LCPC_Entry_t*)outData = bufferEntry;
	}

	return result;
}

LC_Return_t LCP_SetValue(LC_NodeDescriptor_t *mynode, uint8_t remote_node, uint16_t directory_index, uint16_t entry_index, intptr_t *value, uint16_t valueSize) {
	LC_ObjectRecord_t sendReq = { .NodeID = remote_node,.Attributes.Priority = LC_Priority_Low,.Attributes.TCP = 1 };
	uint32_t sizeValSet = sizeof(lc_value_set_t) + valueSize;
	lc_value_set_t* valSet = lcmalloc(sizeValSet);
	if (valSet == 0)
		return LC_MallocFail;
	if (value == 0 || valueSize > LEVCAN_PARAM_MAX_TEXTSIZE + LEVCAN_PARAM_MAX_NAMESIZE)
		return LC_DataError;

	int id = LC_GetMyNodeIndex(mynode);
	if (id < 0) {
		return LC_NodeOffline;
	}
	if (remote_node >= LC_Null_Address) {
		return LC_DataError;
	}

	LC_Return_t state = LC_Ok;
	intptr_t *queue = paramQueue[id];
	//copy
	valSet->Command = lcp_reqValueSet;
	valSet->DirectoryIndex = directory_index;
	valSet->EntryIndex = entry_index;
	memcpy(&valSet->Data[0], value, valueSize);
	//init record
	sendReq.Address = valSet;
	sendReq.Size = sizeValSet;
	sendReq.Attributes.Cleanup = 1; //free call
	//prepare receive
	clearQueueAndInit(id, remote_node, queue);
	if (LC_SendMessage(mynode, &sendReq, LC_SYS_ParametersRequest) == LC_Ok) {
		LC_ObjectData_t objData;
		int result = LC_QueueReceive(queue, &objData, 500);
		if (result) {
			if (objData.Size == sizeof(lc_request_error_t)) {
				//only for errors, like access or end of list
				state = ((lc_request_error_t*)objData.Data)->ErrorCode;
			}
		}
		else {
			state = LC_Timeout;
		}
	}
	//stop receive
	objRec[id].NodeID = LC_Invalid_Address;
	objRec[id].Address = 0;
	return state;
}

void LCP_CleanEntry(LCPC_Entry_t *entry) {
	if (entry->Variable) {
		lcfree((void*)entry->Variable);
		entry->Variable = 0;
	}
	if (entry->Descriptor) {
		lcfree((void*)entry->Descriptor);
		entry->Descriptor = 0;
	}
	if (entry->TextData) {
		lcfree((void*)entry->TextData);
		entry->TextData = 0;
	}
	if (entry->Name) {
		lcfree((void*)entry->Name);
		entry->Name = 0;
	}
	entry->Mode = LCP_Invalid;
}

void LCP_CleanDirectory(LCPC_Directory_t *dir) {
	if (dir->Name) {
		lcfree((void*)dir->Name);
		dir->Name = 0;
	}
}

static void clearQueueAndInit(uint8_t id, uint8_t from_node, void* queue) {
	LC_ObjectData_t temp;
	//prepare receive
	objRec[id].NodeID = from_node;
	objRec[id].Address = queue;
	//request!	
	for (int result = 0; result = LC_QueueReceive(queue, &temp, 0);) {
		lcfree(temp.Data);
	}
}