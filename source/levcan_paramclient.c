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
#error "Queues needed for parameter client exchange. Define LEVCAN_USE_RTOS_QUEUE"
#endif
#ifdef LEVCAN_MEM_STATIC
#error "Can't use static memory for parameters client. Undefine LEVCAN_MEM_STATIC"
#endif

void *paramQueue[LEVCAN_MAX_OWN_NODES] = { 0 };

LC_Return_t LCP_RequestEntry(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, uint16_t entry_index, LCPC_Entry_t *out_entry) {
	LC_ObjectRecord_t sendReq = { .NodeID = from_node, .Attributes.Priority = LC_Priority_Low, .Attributes.TCP = 1 };
	lc_request_entry_t request = { 0 };
	lc_entry_data_t *entrydata = 0; //toBeCleaned
	uint32_t nameDataSize = 0, textDataSize = 0;
	lc_request_error_t error = { 0 };
	LCPC_Entry_t bufferEntry = { 0 };

	int id = LC_GetMyNodeIndex(mynode);
	if (id < 0) {
		return LC_NodeOffline;
	}
	if (from_node >= LC_Null_Address || out_entry == 0) {
		return LC_DataError;
	}

	intptr_t *queue = paramQueue[id];
	LC_ObjectData_t objData;
	//clean
	memset(out_entry, 0, sizeof(LCPC_Entry_t));
	bufferEntry.Mode = LCP_Invalid;
	//request full parameter
	request.Command = lcp_reqFullEntry;
	request.Directory = directory_index;
	request.Entry = entry_index;

	sendReq.Address = &request;
	sendReq.Size = sizeof(request);
	LC_SendMessage(mynode, &sendReq, LC_SYS_ParametersRequest);

	//wait for all data
	for (int i = 0; i < 5; i++) {
		int result = LC_QueueReceive(queue, &objData, 500);
		//todo wrong sender filter
		if (result) {
			//data input
			switch (objData.Header.MsgID) {
			case LC_SYS_ParametersRequest: {
				//only for errors, like access or end of list
				error = *((lc_request_error_t*) objData.Data);
				i = 10; //end of for cycle
				lcfree(objData.Data);
			}
				break;
			case LC_SYS_ParametersData: {
				//description of entry
				entrydata = (lc_entry_data_t*) objData.Data;
				bufferEntry.DescSize = entrydata->DescSize;
				bufferEntry.EntryType = entrydata->EntryType;
				bufferEntry.Mode = entrydata->Mode;
				bufferEntry.VarSize = entrydata->VarSize;
				bufferEntry.EntryIndex = entrydata->EntryIndex;

				if (bufferEntry.Mode & LCP_WriteOnly) {
					i++; //skip value reception
				}
			}
				break;
			case LC_SYS_ParametersName: {
				bufferEntry.Name = (char*) objData.Data;
				nameDataSize = objData.Size;
			}
				break;
			case LC_SYS_ParametersText: {
				bufferEntry.TextData = (char*) objData.Data;
				textDataSize = objData.Size;
			}
				break;
			case LC_SYS_ParametersDescriptor: {
				bufferEntry.Descriptor = objData.Data;
				bufferEntry.DescSize = objData.Size;
			}
				break;
			case LC_SYS_ParametersValue: {
				bufferEntry.Variable = objData.Data;
				bufferEntry.VarSize = objData.Size;
			}
				break;
			default:
				if (objData.Size > 0) {
					lcfree(objData.Data);
				}
				break;
			}
		} else {
			error.ErrorCode = LC_Timeout;
			//timeout, don't need to wait
			break;
		}

	}
	LC_Return_t result = LC_Ok;

	if (error.ErrorCode) {
		result = error.ErrorCode;
		//some errors happened, clean up everything
		LCP_CleanEntry(&bufferEntry);
	} else {
		//check received data
		uint32_t totaltext = 0; //nameDataSize + textDataSize;
		if (nameDataSize > 0)
			totaltext += nameDataSize - 1; //should be 0's at end
		if (textDataSize > 0)
			totaltext += textDataSize - 1; //should be 0's at end

		if (totaltext != entrydata->TextSize) {
			//text error
		}
		if (entrydata->DescSize != bufferEntry.DescSize) {
		}
		if (entrydata->VarSize != bufferEntry.VarSize) {
		}
	}
	//thread safe
	*out_entry = bufferEntry;
	if (entrydata) {
		lcfree(entrydata);
	}
	return result;
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

LC_Return_t LCP_RequestDirectory(LC_NodeDescriptor_t *mynode, uint8_t from_node, uint16_t directory_index, LCPC_Directory_t *out_directory) {
	LC_ObjectRecord_t sendReq = { .NodeID = from_node, .Attributes.Priority = LC_Priority_Low, .Attributes.TCP = 1 };
	lc_request_directory_t request = { 0 };
	lc_directory_data_t *directdata = 0; //toBeCleaned
	uint32_t nameDataSize = 0;
	lc_request_error_t error = { 0 };
	LCPC_Directory_t bufferDir = { 0 };

	int id = LC_GetMyNodeIndex(mynode);
	if (id < 0) {
		return LC_NodeOffline;
	}
	if (from_node >= LC_Null_Address || out_directory == 0) {
		return LC_DataError;
	}

	intptr_t *queue = paramQueue[id];
	LC_ObjectData_t objData;
	//clean
	memset(out_directory, 0, sizeof(LCPC_Entry_t));
	//request full parameter
	request.Directory = directory_index;
	sendReq.Address = &request;
	sendReq.Size = sizeof(request);
	LC_SendMessage(mynode, &sendReq, LC_SYS_ParametersRequest);

	//wait for all data
	for (int i = 0; i < 2; i++) {
		int result = LC_QueueReceive(queue, &objData, 500);
		//todo wrong sender filter
		if (result) {
			//data input
			switch (objData.Header.MsgID) {
			case LC_SYS_ParametersRequest: {
				//only for errors, like access or end of list
				error = *((lc_request_error_t*) objData.Data);
				i = 10; //end of for cycle
				lcfree(objData.Data);
			}
				break;
			case LC_SYS_ParametersData: {
				//description of entry
				directdata = (lc_directory_data_t*) objData.Data;
				bufferDir.DirectoryIndex = directdata->DirectoryIndex;
				bufferDir.Size = directdata->EntrySize;
			}
				break;
			case LC_SYS_ParametersName: {
				bufferDir.Name = (char*) objData.Data;
				nameDataSize = objData.Size;
			}
				break;
			default:
				if (objData.Size > 0) {
					lcfree(objData.Data);
				}
				break;
			}
		} else {
			error.ErrorCode = LC_Timeout;
			//timeout, don't need to wait
			break;
		}

	}

	LC_Return_t result = LC_Ok;
	if (error.ErrorCode) {
		result = error.ErrorCode;
		//some errors happened, clean up everything
		if (bufferDir.Name) {
			lcfree((void*)bufferDir.Name);
		}

	} else {
		//check received data
		if (directdata->NameSize > 0) {
			directdata->NameSize--; //skip 0s
		}
		if (nameDataSize != directdata->NameSize) {
			//text error
		}
		if (bufferDir.DirectoryIndex != directory_index) {
			//index error
		}
	}
	//thread safe
	*out_directory = bufferDir;
	if (directdata) {
		lcfree(directdata);
	}
	return result;
}
