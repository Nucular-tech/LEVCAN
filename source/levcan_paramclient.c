//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#include "levcan.h"
#include "levcan_internal.h"
#include "levcan_paramclient.h"
#include "levcan_paraminternal.h"
#include <string.h>

#ifndef LEVCAN_USE_RTOS_QUEUE
 //#error "Queues needed for parameter client exchange. Define LEVCAN_USE_RTOS_QUEUE"
#endif
#ifdef LEVCAN_MEM_STATIC
#error "Can't use static memory for parameters client. Undefine LEVCAN_MEM_STATIC"
#endif

#define OBJ_PARAM_SIZE (LC_SYS_ParametersValue - LC_SYS_ParametersData + 1)

static void clearQueueAndInit(LC_NodeDescriptor_t *node, uint8_t from_node);
static LC_Return_t requestData(LC_NodeDescriptor_t *node, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, void *outData, uint16_t dataSize,
		uint16_t command);

LC_Return_t LCP_ParameterClientInit(LC_NodeDescriptor_t *node) {
	LC_Object_t *initObject = lc_registerSystemObjects(node, OBJ_PARAM_SIZE);
	LC_ObjectRecord_t objRec = { 0 };
	if (initObject == 0) {
		return LC_MallocFail;
	}
	intptr_t *clientQueue = LC_QueueCreate(5, sizeof(LC_ObjectData_t));
	if (clientQueue == 0) {
		return LC_MallocFail;
	}
	//prepare specific record, we need to strictly sort out other messages form senders
	objRec.Attributes.Queue = 1;
	objRec.Attributes.Writable = 1;
	objRec.Size = -(LEVCAN_PARAM_MAX_TEXTSIZE + LEVCAN_PARAM_MAX_NAMESIZE);
	//dont place address here atm, we dont need junk
	objRec.NodeID = LC_Invalid_Address;
	//assign objRec to the node
	((lc_Extensions_t*) node->Extensions)->paramClientRecord = objRec;
	//objects needed to define message ID and store objRecord
	int objID = 0;
	for (; objID < OBJ_PARAM_SIZE; objID++) {
		initObject[objID].Attributes.Record = 1;
		initObject[objID].Address = &(((lc_Extensions_t*) node->Extensions)->paramClientRecord);
		initObject[objID].MsgID = LC_SYS_ParametersData + objID;
		initObject[objID].Size = 1;
	}
	((lc_Extensions_t*) node->Extensions)->paramClientQueue = clientQueue;
	return LC_Ok;
}

LC_Return_t LCP_RequestEntry(LC_NodeDescriptor_t *node, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, LCPC_Entry_t *out_entry) {

	memset(out_entry, 0, sizeof(*out_entry));
	return requestData(node, from_node, directory_index, entry_index, out_entry, sizeof(*out_entry), lcp_reqFullEntry);
}

LC_Return_t LCP_RequestDirectory(LC_NodeDescriptor_t *node, uint8_t from_node, uint16_t directory_index, LCPC_Directory_t *out_directory) {

	memset(out_directory, 0, sizeof(*out_directory));
	return requestData(node, from_node, directory_index, 0, out_directory, sizeof(*out_directory), lcp_reqDirectoryInfo);
}

LC_Return_t LCP_RequestValue(LC_NodeDescriptor_t *node, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, intptr_t *outVariable, uint16_t varSize) {
	return requestData(node, from_node, directory_index, entry_index, outVariable, varSize, lcp_reqVariable);
}

static LC_Return_t requestData(LC_NodeDescriptor_t *node, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, void *outData, uint16_t dataSize,
		uint16_t command) {
	LC_ObjectRecord_t sendReq = { .NodeID = from_node, .Attributes.Priority = LC_Priority_Low, .Attributes.TCP = 1 };
	lc_request_entry_t request = { 0 };
	lc_entry_data_t entrydata = { 0 }; //toBeCleaned
	lc_directory_data_t directdata = { 0 }; //toBeCleaned
	uint32_t nameDataSize = 0, textDataSize = 0;
	lc_request_error_t error = { 0 };
	LCPC_Entry_t bufferEntry = { 0 };
	LCPC_Directory_t bufferDir = { 0 };

	if (node == 0 || node->Extensions == 0 || ((lc_Extensions_t*) node->Extensions)->paramClientQueue == 0) {
		return LC_InitError;
	}
	if (from_node >= LC_Null_Address || outData == 0 || dataSize == 0) {
		return LC_DataError;
	}

	intptr_t *queue = ((lc_Extensions_t*) node->Extensions)->paramClientQueue;
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
	} else {
		sendReq.Size = sizeof(request);
	}
	//prepare receive
	clearQueueAndInit(node, from_node);
	//send request!
	error.ErrorCode = LC_SendMessage(node, &sendReq, LC_SYS_ParametersRequest);
	if (error.ErrorCode != LC_Ok)
		return error.ErrorCode;

	lcp_reqCommand_t sequence = command;
	if (command == lcp_reqDirectoryInfo) {
		//directory doesnt needs so much
		sequence = lcp_reqName | lcp_reqData;
	}
	//wait for all data
	for (int attempt = 0; sequence != 0 && attempt < 3;) {
		int result = LC_QueueReceive(queue, &objData, LEVCAN_MESSAGE_TIMEOUT);
		//todo wrong sender filter
		if (result) {
			//data input
			switch (objData.Header.MsgID) {
			case LC_SYS_ParametersData: {
				if (command == lcp_reqDirectoryInfo) { //directory request
					if (objData.Size == sizeof(lc_directory_data_t)) {
						//description of entry
						directdata = *((lc_directory_data_t*) objData.Data);
						bufferDir.DirectoryIndex = directdata.DirectoryIndex;
						bufferDir.Size = directdata.EntrySize;
					}
				} else { //entry request
					if (objData.Size == sizeof(lc_entry_data_t)) {
						//description of entry, later used for checking
						entrydata = *((lc_entry_data_t*) objData.Data);
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
					error = *((lc_request_error_t*) objData.Data);
					sequence = 0;
				}

				lcfree(objData.Data);
				sequence &= ~lcp_reqData;
			}
				break;

			case LC_SYS_ParametersName: {
				if (command == lcp_reqDirectoryInfo) { //directory request
					bufferDir.Name = (char*) objData.Data;
				} else { //entry request
					bufferEntry.Name = (char*) objData.Data;
				}
				nameDataSize = objData.Size;

				sequence &= ~lcp_reqName;
			}
				break;

			case LC_SYS_ParametersText: {
				bufferEntry.TextData = (char*) objData.Data;
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
		} else {
			attempt++;
			//request leftovers
			request.Command = sequence;
			if (command == lcp_reqDirectoryInfo) {
				//directory doesnt needs so much
				sequence = lcp_reqName | lcp_reqData;
			}
			LC_SendMessage(node, &sendReq, LC_SYS_ParametersRequest);
			if (attempt > 2) {
				error.ErrorCode = LC_Timeout;
				//timeout, don't need to wait
				break;
			}
		}

	}

	//stop receive
	((lc_Extensions_t*) node->Extensions)->paramClientRecord.NodeID = LC_Invalid_Address;
	((lc_Extensions_t*) node->Extensions)->paramClientRecord.Address = 0;
	LC_Return_t result = LC_Ok;

	if (error.ErrorCode) {
		result = error.ErrorCode;
		//some errors happened, clean up everything
		LCP_CleanEntry(&bufferEntry);
		LCP_CleanDirectory(&bufferDir);
	} else {
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
			if (bufferEntry.Variable)
				memcpy(outData, bufferEntry.Variable, dataSize);
			LCP_CleanEntry(&bufferEntry); //clean, since data copied
		} else {
			result = LC_DataError;
		}
	} else if (command == lcp_reqDirectoryInfo) {
		*(LCPC_Directory_t*) outData = bufferDir;
		LCP_CleanEntry(&bufferEntry); //if something was received
	} else {
		*(LCPC_Entry_t*) outData = bufferEntry;
	}

	return result;
}

LC_Return_t LCP_SetValue(LC_NodeDescriptor_t *node, uint8_t remote_node, uint16_t directory_index, uint16_t entry_index, intptr_t *value, uint16_t valueSize) {
	LC_ObjectRecord_t sendReq = { .NodeID = remote_node, .Attributes.Priority = LC_Priority_Low, .Attributes.TCP = 1 };
	uint32_t sizeValSet = sizeof(lc_value_set_t) + valueSize;
	lc_value_set_t *valSet = lcmalloc(sizeValSet);
	if (valSet == 0)
		return LC_MallocFail;
	if (value == 0 || valueSize > LEVCAN_PARAM_MAX_TEXTSIZE + LEVCAN_PARAM_MAX_NAMESIZE)
		return LC_DataError;

	if (node == 0 || node->Extensions == 0 || ((lc_Extensions_t*) node->Extensions)->paramClientQueue == 0) {
		return LC_InitError;
	}
	if (remote_node >= LC_Null_Address || value == 0 || valueSize == 0) {
		return LC_DataError;
	}

	LC_Return_t state = LC_Ok;
	intptr_t *queue = ((lc_Extensions_t*) node->Extensions)->paramClientQueue;
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
	clearQueueAndInit(node, remote_node);
	if (LC_SendMessage(node, &sendReq, LC_SYS_ParametersRequest) == LC_Ok) {
		LC_ObjectData_t objData;
		int result = LC_QueueReceive(queue, &objData, 500);
		if (result) {
			if (objData.Size == sizeof(lc_request_error_t)) {
				//only for errors, like access or end of list
				state = ((lc_request_error_t*) objData.Data)->ErrorCode;
			}
		} else {
			state = LC_Timeout;
		}
	}
	//stop receive
	((lc_Extensions_t*) node->Extensions)->paramClientRecord.NodeID = LC_Invalid_Address;
	((lc_Extensions_t*) node->Extensions)->paramClientRecord.Address = 0;
	return state;
}

void LCP_CleanEntry(LCPC_Entry_t *entry) {
	if (entry->Variable) {
		lcfree((void*) entry->Variable);
		entry->Variable = 0;
	}
	if (entry->Descriptor) {
		lcfree((void*) entry->Descriptor);
		entry->Descriptor = 0;
	}
	if (entry->TextData) {
		lcfree((void*) entry->TextData);
		entry->TextData = 0;
	}
	if (entry->Name) {
		lcfree((void*) entry->Name);
		entry->Name = 0;
	}
	entry->Mode = LCP_Invalid;
}

void LCP_CleanDirectory(LCPC_Directory_t *dir) {
	if (dir->Name) {
		lcfree((void*) dir->Name);
		dir->Name = 0;
	}
}

static void clearQueueAndInit(LC_NodeDescriptor_t *node, uint8_t from_node) {
	LC_ObjectData_t temp = { 0 };
	void *queue = ((lc_Extensions_t*) node->Extensions)->paramClientQueue;
	//prepare receive
	((lc_Extensions_t*) node->Extensions)->paramClientRecord.NodeID = from_node;
	((lc_Extensions_t*) node->Extensions)->paramClientRecord.Address = queue;
	//request!	
	for (int result = 0; result != 0;) {
		result = LC_QueueReceive(queue, &temp, 0);
		if (result && temp.Data != 0) {
			lcfree(temp.Data);
		}
	}
}
