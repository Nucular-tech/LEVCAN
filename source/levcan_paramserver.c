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
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "levcan_paramserver.h"
#include "levcan_paraminternal.h"
#include "levcan_paramcommon.h"

static int checkExists(const LCPS_Directory_t directories[], uint16_t dirsize, uint16_t directory, int32_t entry_index);
static const char* extractEntryName(const LCPS_Directory_t directories[], uint16_t dirsize, const LCPS_Entry_t *entry);
void lc_proceedParameterRequest(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
extern LC_Object_t* lc_registerSystemObjects(LC_NodeDescriptor_t *node, uint8_t count);
void* getValueByIndex(const void *variable0, uint16_t size, uint8_t arrayIndex);

LC_Return_t LCP_ParameterServerInit(LC_NodeDescriptor_t *node) {
	//register function call to this object
	LC_Object_t *initObject = lc_registerSystemObjects(node, 1);
	if (initObject == 0) {
		return LC_MallocFail;
	}
	initObject->Address = lc_proceedParameterRequest;
	initObject->Attributes.Writable = 1;
	initObject->Attributes.Function = 1;
	initObject->Attributes.TCP = 1;
	initObject->MsgID = LC_SYS_ParametersRequest;
	initObject->Size = -LEVCAN_FILE_DATASIZE; //up to size
	node->ShortName.Configurable = 1;
	return LC_Ok;
}

void lc_proceedParameterRequest(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	LC_ObjectRecord_t sendRec = { .NodeID = header.Source, .Attributes.Priority = LC_Priority_Low, .Attributes.TCP = 1 };
	(void) header;
	uint8_t status = LC_Ok;
	if (header.MsgID != LC_SYS_ParametersRequest)
		return;

	int sendResponce = 1;
	LCPS_Directory_t *directories = (LCPS_Directory_t*) node->Directories;
	uint16_t dirsize = node->DirectoriesSize;

	uint16_t reqCommand = *((uint16_t*) data);
	if (size == sizeof(lc_request_directory_t) && reqCommand == lcp_reqDirectoryInfo) {
		//request directory
		lc_request_directory_t request = *((lc_request_directory_t*) data);
		//check directory
		if (checkExists(directories, dirsize, request.Directory, 0)) {
			//get root
			LCPS_Directory_t *directory = &(directories[request.Directory]);
			//current access should be same or higher
			//header "not exists" if there is no access to it
			if (node->AccessLevel >= directory->AccessLvl) {
				//this fits in one can message, so gonna be placed in a buffer
				lc_directory_data_t dirdata;
				dirdata.EntrySize = directory->Size;
				dirdata.NameSize = strnlen(directory->Name, 128);
				dirdata.DirectoryIndex = request.Directory;

				sendRec.Address = &dirdata;
				sendRec.Size = sizeof(lc_directory_data_t);
				status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersData);

				sendRec.Address = (void*) directory->Name;
				sendRec.Size = dirdata.NameSize + 1; //+0
				status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersName);

				sendResponce = 0;
			} else {
				status = LC_AccessError;
			}
		} else {
			status = LC_OutOfRange;
		}
	} else if (size == sizeof(lc_request_entry_t) && reqCommand <= lcp_reqFullEntry) {
		//request entry
		lc_request_entry_t request = *((lc_request_entry_t*) data);
		//check directory
		status = LC_OutOfRange;

		while (checkExists(directories, dirsize, request.Directory, request.Entry)) {
			//get root
			LCPS_Directory_t *directory = &(directories[request.Directory]);
			LCPS_Entry_t *entry = (void*) &directory->Entries[request.Entry];
			//current access should be same or higher
			//header "not exists" if there is no access to it
			if (node->AccessLevel >= directory->AccessLvl) {
				if (node->AccessLevel >= entry->AccessLvl) {
					/*if (entry->Function) {
					 if (entry->Variable != 0) {
					 //get parameter from function
					 status = ((LCP_ParameterCallback_t) entry->Variable)(node, header, &entry_funct_buffer, request.Directory, request.Entry);
					 entry = &entry_funct_buffer;
					 } else {
					 status = LC_ObjectError;
					 break; //end while
					 }
					 } else {*/
					status = LC_Ok; //plain data
					//}
					if (status == LC_Ok) {
						//this fits in one can message, so gonna be placed in a buffer
						char *name = (void*) extractEntryName(directories, dirsize, entry);

						if (request.Command & lcp_reqData) {
							//todo fix static
#ifdef LEVCAN_MEM_STATIC
							static lc_entry_data_t entrydataStatic;
							lc_entry_data_t* entrydata = &entrydataStatic
#else //dynamic mem
							lc_entry_data_t *entrydata = 0;
							entrydata = lcmalloc(sizeof(lc_entry_data_t));
							if (entrydata == 0) {
								status = LC_MallocFail;
								break;
							}
#endif //LEVCAN_MEM_STATIC
							entrydata->DescSize = entry->DescSize;
							entrydata->EntryType = entry->EntryType;
							entrydata->Mode = entry->Mode;
							entrydata->VarSize = entry->VarSize;
							entrydata->EntryIndex = request.Entry;

							int textlen = strnlen(name, LEVCAN_PARAM_MAX_NAMESIZE);
							if (entry->TextData) {
								textlen += strnlen(entry->TextData, LEVCAN_PARAM_MAX_TEXTSIZE);
							}
							entrydata->TextSize = textlen;

							sendRec.Address = entrydata;
#ifdef LEVCAN_MEM_STATIC
							sendRec.Attributes.Cleanup = 0;
#else //dynamic mem
							sendRec.Attributes.Cleanup = 1;
#endif //LEVCAN_MEM_STATIC
							sendRec.Size = sizeof(lc_entry_data_t);
							status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersData);
							sendResponce = 0;
						}

						if (request.Command & lcp_reqName) {
							sendRec.Address = (void*) name;
							sendRec.Attributes.Cleanup = 0;
							sendRec.Size = strnlen(name, LEVCAN_PARAM_MAX_NAMESIZE) + 1;
							status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersName);
							sendResponce = 0;
						}

						if ((entry->TextData != 0) && (request.Command & lcp_reqText)) {
							sendRec.Address = (void*) entry->TextData;
							sendRec.Attributes.Cleanup = 0;
							sendRec.Size = strnlen(entry->TextData, LEVCAN_PARAM_MAX_TEXTSIZE) + 1;
							status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersText);
							sendResponce = 0;
						}

						if ((entry->DescSize != 0) && (request.Command & lcp_reqDescriptor)) {
							sendRec.Address = (void*) entry->Descriptor;
							sendRec.Attributes.Cleanup = 0;
							sendRec.Size = entry->DescSize;
							status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersDescriptor);
							sendResponce = 0;
						}

						if (request.Command & lcp_reqVariable) {
							if ((entry->Variable != 0) && ((entry->Mode & LCP_WriteOnly) == 0)) {
								sendRec.Address = getValueByIndex(entry->Variable, entry->VarSize, directory->ArrayIndex);
								sendRec.Attributes.Cleanup = 0;
								sendRec.Size = entry->VarSize;
								status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersValue);

								sendResponce = 0;
							} else if (request.Command == lcp_reqVariable) {
								//value request only. this needs a access error response
								status = LC_AccessError;
								sendResponce = 1;
							}
						}
						break; //end while

					}
				} else { //entry access lvl
						//partial request is not a search
						status = LC_AccessError;
						break; //end while
				}
			} else { //directory access lvl
				status = LC_AccessError;
				break; //end while
			}
		}
	} else if (size >= (int32_t) sizeof(lc_value_set_t) && reqCommand == lcp_reqValueSet) {
		lc_value_set_t *request = (lc_value_set_t*) data;
		//check directory
		status = LC_OutOfRange;

		if (checkExists(directories, dirsize, request->DirectoryIndex, request->EntryIndex)) {
			//get root
			LCPS_Directory_t *directory = &(directories[request->DirectoryIndex]);
			LCPS_Entry_t *entry = (void*) &directory->Entries[request->EntryIndex];
			int varsize = size - sizeof(lc_value_set_t);
			if ((node->AccessLevel >= directory->AccessLvl) && (node->AccessLevel >= entry->AccessLvl)) {
				//size should match
				if (varsize == entry->VarSize && entry->Variable != 0) {
					//align data
					memmove(data, &request->Data[0], varsize);
					//'request' invalid now here
					//out of range will be limited to min/max
					status = LCP_LimitValue(data, varsize, entry->Descriptor, entry->DescSize, entry->EntryType);
					//however we can ignore it or accept only valid values
					if (status == LC_Ok
#ifndef LEVCAN_PVALUE_DROP_OUTOFRANGE
							|| status == LC_OutOfRange
#endif
									) {
						status = LC_Ok;
						memcpy(getValueByIndex(entry->Variable, entry->VarSize, directory->ArrayIndex), data, varsize);
					}
				} else {
					status = LC_DataError;
				}
			} else {
				status = LC_AccessError;
			}
		}
	} else {
		status = LC_DataError;
	}

	//todo "not found message"
	if (sendResponce) {
		sendRec.Address = &status;
		sendRec.Attributes.Cleanup = 0;
		sendRec.Size = sizeof(lc_request_error_t);
		LC_SendMessage(node, &sendRec, LC_SYS_ParametersData);
	}
}

void* getValueByIndex(const void *variable0, uint16_t size, uint8_t arrayIndex) {
	if (variable0 == 0)
		return 0;
	//get value from array[]
	char *varPtr = (char*) variable0;
	varPtr += size * arrayIndex;
	return varPtr;
}

static int checkExists(const LCPS_Directory_t directories[], uint16_t dirsize, uint16_t directory, int32_t entry_index) {
	if (directories == 0 || directory >= dirsize || directories[directory].Entries == 0 || entry_index >= directories[directory].Size)
		return 0;
	return 1;
}

const char *unknown = "Unknown";
static const char* extractEntryName(const LCPS_Directory_t directories[], uint16_t dirsize, const LCPS_Entry_t *entry) {
	const char *source = unknown;

	if (entry->EntryType == LCP_Folder) {
		//extracting name from directory index
		int32_t dindex = entry->VarSize; //directory index
		if (entry->Name) {
			//custon entry name
			source = entry->Name;
		} else if (checkExists(directories, dirsize, dindex, -1)) {
			//use directory name
			const LCPS_Directory_t *dir = &directories[dindex];
			if (dir->Name) {
				source = dir->Name;
			}
		}

	} else if (entry->Name) {
		source = entry->Name;
	}
	return source;
}

const char *equality = " = ";
const char *newline = "\n";

void LCP_PrintParam(char *buffer, const LCPS_Directory_t *dir, uint16_t index) {
	if (buffer == 0)
		return;

	if (index >= dir->Size)
		return;
	const LCPS_Entry_t *entry = &dir->Entries[index];
	if (entry == NULL)
		return;
	//for string this enough
	buffer[0] = 0;
	strcpy(buffer, entry->Name);
	if (entry->EntryType != LCP_Label)
		strcat(buffer, equality); //" = "

	switch (entry->EntryType) {
	case LCP_Label: {
		strcat(buffer, " ");
		strcat(buffer, entry->TextData);
	}
		break;
	case LCP_String: {
		if (entry->Variable) {
			//todo upgrade logic here
			snprintf(buffer + strlen(buffer), 128, "\"%s\"", (char*) entry->Variable);
		}
	}
		break;
	case LCP_Bool:
	case LCP_Bitfield32:
	case LCP_Uint32: {
		uint32_t val_u32 = lcp_getUint32(getValueByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, val_u32);
		} else {
			sprintf(buffer + strlen(buffer), "%" PRIu32, val_u32);
		}
	}
		break;
	case LCP_Uint64: {
		uint64_t val_u64 = *(uint64_t*) getValueByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, val_u64);
		} else {
			sprintf(buffer + strlen(buffer), "%" PRIu64, val_u64);
		}
	}
		break;
	case LCP_Int32: {
		int32_t val_i32 = lcp_getInt32(getValueByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, val_i32);
		} else {
			sprintf(buffer + strlen(buffer), "%" PRId32, val_i32);
		}
	}
		break;
	case LCP_Int64: {
		int64_t val_i64 = *(int64_t*) getValueByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, val_i64);
		} else {
			sprintf(buffer + strlen(buffer), "%" PRId64, val_i64);
		}
	}
		break;
	case LCP_Decimal32: {
		strcat(buffer, equality);
		int32_t val_u32 = lcp_getInt32(getValueByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);
		int dec = 0;
		if (entry->Descriptor && entry->EntryType == LCP_Uint32) {
			dec = ((LCP_Decimal32_t*) entry->Descriptor)->Decimals;
		}
		char *tDataEnd = 0;
		if (entry->TextData) {
			char *ptr = strstr(entry->TextData, "%s");
			if (ptr) {
				tDataEnd = ptr + 2; //skip %s
				int size = entry->TextData - ptr;
				if (size > 0) {
					//print beginnings
					strncat(buffer, entry->TextData, size);
				}
			}
		}
		lcp_print_i32f(buffer + strlen(buffer), val_u32, dec);
		if (tDataEnd) {
			//print end of custom text
			strcat(buffer, tDataEnd);
		}
	}
		break;
	case LCP_Enum: {
		uint32_t val_u32 = lcp_getUint32(getValueByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);
		LCP_Enum_t *enumDesc = (LCP_Enum_t*) entry->Descriptor;

		const char *position = 0;
		if (enumDesc->Min <= val_u32) {
			//value should be above minimum
			position = entry->TextData;
			uint32_t minval = val_u32 - enumDesc->Min;
			for (uint32_t i = 0; (i < minval); i++) {
				position = strchr(position, '\n'); //look for buffer specified by val index
				if (position == 0)
					break;
				position++; //skip '\n'
			}
		}
		if (position == 0) {
			sprintf(buffer + strlen(buffer), "%" PRIu32, val_u32);
		} else {
			char *end = strchr(position, '\n'); //search for buffer end
			if (end) {
				strncat(buffer, position, end - position);
			} else
				strcat(buffer, position);	//end is possible zero
		}
	}
		break;

#ifdef LEVCAN_USE_FLOAT
	case LCP_Float: {
		float fval = *(float*) getValueByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, fval);
		} else {
			sprintf(buffer + strlen(buffer), "%.9f", fval);
		}
	}
		break;
#endif
#ifdef LEVCAN_USE_DOUBLE
	case LCP_Double: {
		double dval = *(double*) getValueByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, dval);
		} else {
			sprintf(buffer + strlen(buffer), "%.17g", dval);
		}
	}
		break;
#endif
	default:
		break;
	}
	strcat(buffer, newline);

	return;
}
