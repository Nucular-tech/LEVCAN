//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <stdlib.h>

#include "levcan_paramserver.h"
#include "levcan_paraminternal.h"
#include "levcan_paramcommon.h"

//Private functions
void lc_proceedParameterRequest(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
extern LC_Object_t* lc_registerSystemObjects(LC_NodeDescriptor_t *node, uint8_t count);
static int checkExists(const LCPS_Directory_t directories[], uint16_t dirsize, uint16_t directory, int32_t entry_index);
static const char* extractEntryName(const LCPS_Directory_t directories[], uint16_t dirsize, const LCPS_Entry_t *entry);
void* getVAddressByIndex(const void *variable0, uint16_t size, uint8_t arrayIndex);
const char* skipspaces(const char *s);

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
							lc_entry_data_t* entrydata = &entrydataStatic;
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
								sendRec.Address = getVAddressByIndex(entry->Variable, entry->VarSize, directory->ArrayIndex);
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
						memcpy(getVAddressByIndex(entry->Variable, entry->VarSize, directory->ArrayIndex), data, varsize);
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

/// Extracts value pointer from array (arrayIndex > 0)
void* getVAddressByIndex(const void *variable0, uint16_t size, uint8_t arrayIndex) {
	if (variable0 == 0)
		return 0;
	//get value from array[]
	char *varPtr = (char*) variable0;
	varPtr += size * arrayIndex;
	return varPtr;
}

static int checkExists(const LCPS_Directory_t directories[], uint16_t dirsize, uint16_t directory, int32_t entry_index) {
	if (directories == 0 || directory >= dirsize || entry_index >= directories[directory].Size)
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
const char *endline = "#=\n\r";
const char *endlinedir = "]#=\n\r";
const char *off_on = "OFF\nON";

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
	case LCP_Bitfield32: {
		uint32_t val_u32 = lcp_getUint32(getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);
		lcp_print_u32b(buffer + strlen(buffer), val_u32);
	}
		break;
	case LCP_Uint32: {
		uint32_t val_u32 = lcp_getUint32(getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, val_u32);
		} else {

			sprintf(buffer + strlen(buffer), "%" PRIu32, val_u32);

		}
	}
		break;
	case LCP_Int32: {
		int32_t val_i32 = lcp_getInt32(getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, val_i32);
		} else {
			sprintf(buffer + strlen(buffer), "%" PRId32, val_i32);
		}
	}
		break;
	case LCP_Uint64: {
		uint64_t val_u64 = *(uint64_t*) getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, val_u64);
		} else {
			sprintf(buffer + strlen(buffer), "%" PRIu64, val_u64);
		}
	}
		break;
	case LCP_Int64: {
		int64_t val_i64 = *(int64_t*) getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex);
		if (entry->TextData) {
			sprintf(buffer + strlen(buffer), entry->TextData, val_i64);
		} else {
			sprintf(buffer + strlen(buffer), "%" PRId64, val_i64);
		}
	}
		break;
	case LCP_Decimal32: {
		int32_t val_u32 = lcp_getInt32(getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);
		int dec = 0;
		if (entry->Descriptor) {
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
	case LCP_Bool:
	case LCP_Enum: {
		uint32_t val_u32 = lcp_getUint32(getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex), entry->VarSize);

		const char *position = 0;
		if (entry->EntryType == LCP_Bool) {
			position = off_on;
			if (entry->TextData) {
				position = entry->TextData;
			}
			if (val_u32) {
				position = strchr(position, '\n'); //look for buffer specified by val index
				if (position != 0) {
					position++; //skip '\n'
				}
			}
		} else {
			LCP_Enum_t *enumDesc = (LCP_Enum_t*) entry->Descriptor;
			if (enumDesc && (enumDesc->Min <= val_u32)) {
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
		float fval = *(float*) getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex);
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
		double dval = *(double*) getVAddressByIndex(entry->Variable, entry->VarSize, dir->ArrayIndex);
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

/// Parses string line and returns it's directory or index
const char* LCP_ParseParameterName(LC_NodeDescriptor_t *node, const char *input, int16_t *directory, int16_t *index) {
	if (index == 0 || directory == 0 || input == 0 || node == 0)
		return 0;
	int16_t dir = *directory;
	int16_t indx = -1;

	//skip first spaces
	const char *line = skipspaces(input);
	int linelength = strcspn(line, "\n\r");
	if (linelength > 0) {
		if (*line == '[') {
			//[directory name]
			line++; //skip '['
			line = skipspaces(line); //skip possible spaces at begin
			//directory detected, calculate distance to end
			dir = LCP_IsDirectory((LCPS_Directory_t*) node->Directories, node->DirectoriesSize, line);
			//too short or large, or new line? wrong
			if (dir >= 0) {
				indx = -1; //reset parameter
			} else {
				//Directory not found
			}
			line += linelength;
		} else if ((dir >= 0) && (*line != '#') && (*line != 0) && (*line != '\n')) {
			//parameter = value
			//Vasya = Pupkin #comment
			int endofname = strcspn(line, endline);
			indx = LCP_IsParameter(&((LCPS_Directory_t*) node->Directories)[dir], line);
			//too short or large, or new line? wrong
			if (indx >= 0 && line[endofname] == '=') {
				//endofname is here '='
				line += endofname + 1;
				//Pupkin #comment \n
			} else {
				line += linelength;
			}
		} else {
			//comment or random text
			line += linelength;
		}
	} else
		return 0;
	*index = indx;
	*directory = dir;
	//go to new line
	return line;
}

/// Returns directory index if valid, -1 if not found
/// @param directories
/// @param dirsize
/// @param s
/// @return
int16_t LCP_IsDirectory(const LCPS_Directory_t directories[], uint16_t dirsize, const char *s) {
	//index [0] is directory entry, dont scan
	int searchlen = strcspn(s, endlinedir);
	//remove space ending
	for (; searchlen > 0 && isblank(s[searchlen - 1]); searchlen--)
		;
	for (uint16_t i = 0; i < dirsize; i++) {
		const char *name = directories[i].Name;
		if (name && strncmp(name, s, searchlen) == 0) {
			return i;
		}
	}
	return -1;
}

/// Returns parameter index if valid, -1 if not found
/// @param directory
/// @param s
/// @return
int16_t LCP_IsParameter(const LCPS_Directory_t *directory, const char *s) {
	//index [0] is directory entry, dont scan
	int searchlen = strcspn(s, endline);
	//remove space ending
	for (; searchlen > 0 && isblank(s[searchlen - 1]); searchlen--)
		;
	for (uint16_t i = 0; i < directory->Size; i++) {
		//todo carefully check types
		const char *name = directory->Entries[i].Name;
		if (name && (strncmp(name, s, searchlen) == 0)) {
			return i;
		}
	}
	return -1;
}

/// Tries to get value for specified parameter with string value
/// @param parameter
/// @param value output
/// @return
LC_Return_t LCP_ParseParameterValue(const LCPS_Entry_t *parameter, const uint8_t arrayIndex, const char *s, char **out) {
	int32_t integer = 0;
	if (parameter == 0 || s == 0 || out == 0) {
		return LC_DataError;
	}
	int type = parameter->EntryType;
	//skip what is not need parsing
	if (parameter->VarSize == 0 || type == LCP_Label) {
		type = LCP_End; //scroll to next line
	}

	s = skipspaces(s);
	*out = (char*)s;

	if (parameter->Variable == 0)
		return LC_DataError;

	intptr_t *vaddress = getVAddressByIndex(parameter->Variable, parameter->VarSize, arrayIndex);
	int length = strcspn(s, endline);
	int varsize = parameter->VarSize;
	LC_Return_t result = LC_Ok;

	switch (type) {
	case LCP_String: {

	}
		break;
	case LCP_Bitfield32: {
		//check for 0b1010101 match
		if (s[0] != '0' && s[1] != 'b') {
			result = LC_DataError;
			break;
		}
		uint32_t u32 = 0;
		u32 = strtoul(s + 2, out, 2);
		if (s == *out) {
			result = LC_DataError;
			break;
		}
		if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Bitfield32_t)) {
			result = LC_ObjectError;
			break;
		}

		LCP_Bitfield32_t *desc = (LCP_Bitfield32_t*) parameter->Descriptor;
		lcp_setUint32(vaddress, varsize, u32);
		result = lcp_bitinRange(vaddress, varsize, desc->Mask);
	}
		break;
	case LCP_Uint32: {
		//negative check!
		uint32_t u32 = 0;
		if (*s != '-') {
			u32 = strtoul(s, out, 0);
		}

		if (s == *out) {
			result = LC_DataError;
			break;
		}
		if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Uint32_t)) {
			result = LC_ObjectError;
			break;
		}

		LCP_Uint32_t *desc = (LCP_Uint32_t*) parameter->Descriptor;
		lcp_setUint32(vaddress, varsize, u32);
		result = lcp_u32inRange(vaddress, varsize, desc->Min, desc->Max);
	}
		break;
	case LCP_Int32: {
		int32_t i32 = strtol(s, out, 0);
		if (s == *out) {
			result = LC_DataError;
			break;
		}
		if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Int32_t)) {
			result = LC_ObjectError;
			break;
		}

		LCP_Int32_t *desc = (LCP_Int32_t*) parameter->Descriptor;
		lcp_setInt32(vaddress, varsize, i32);
		result = lcp_i32inRange(vaddress, varsize, desc->Min, desc->Max);
	}
		break;
#ifdef LEVCAN_USE_INT64
	case LCP_Uint64: {
		uint64_t u64 = strtoull(s, out, 0);
		if (s == *out) {
			result = LC_DataError;
			break;
		}
		if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Uint64_t)) {
			result = LC_ObjectError;
			break;
		}

		LCP_Uint64_t *desc = (LCP_Uint64_t*) parameter->Descriptor;
		*((uint64_t*) vaddress) = u64;
		result = lcp_u64inRange(vaddress, varsize, desc->Min, desc->Max);
	}
		break;
	case LCP_Int64: {
		int64_t i64 = strtoll(s, out, 0);
		if (s == *out) {
			result = LC_DataError;
			break;
		}
		if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Int64_t)) {
			result = LC_ObjectError;
			break;
		}

		LCP_Int64_t *desc = (LCP_Int64_t*) parameter->Descriptor;
		*((int64_t*) vaddress) = i64;
		result = lcp_i64inRange(vaddress, varsize, desc->Min, desc->Max);

	}
		break;
#endif
	case LCP_Decimal32: {
#ifdef LEVCAN_USE_FLOAT
		float temp = strtof(s, out);
		if (s == *out) {
			result = LC_DataError;
			break;
		}
		if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Decimal32_t)) {
			result = LC_ObjectError;
			break;
		}

		LCP_Decimal32_t *ddesc = (LCP_Decimal32_t*) parameter->Descriptor;
		integer = temp * lcp_pow10i(ddesc->Decimals);
		lcp_setInt32(vaddress, varsize, integer);
		result = lcp_i32inRange(vaddress, varsize, ddesc->Min, ddesc->Max);

#endif
	}
		break;
	case LCP_Bool:
	case LCP_Enum: {
		const char *haystack = parameter->TextData;
		if (type == LCP_Bool) {
			haystack = off_on;
		} else {
			if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Enum_t)) {
				result = LC_ObjectError;
				break;
			}
		}
		//end is comment or newline
		int haylen = strlen(haystack);
		//remove space ending
		for (; length > 0 && isblank(s[length - 1]); length--)
			;
		int found = 0;
		//look in haystack
		int h = 0;
		while (h < haylen && length > 0) {
			//compare new line
			if (strncmp(&haystack[h], s, length) == 0) {
				found = 1;
				break;
			}
			//count till new line
			for (; h < haylen; h++) {
				//compare needle
				if (haystack[h] == '\n') {
					integer++;
					h++;
					break;
				}
			}
		}
		if (found) {
			lcp_setUint32(vaddress, varsize, integer);
			if (type == LCP_Bool) {
				result = lcp_u32inRange(vaddress, varsize, 0, 1);
			} else {
				LCP_Enum_t *desc = (LCP_Enum_t*) parameter->Descriptor;
				//todo min-size logic?
				uint32_t max = desc->Size > 0 ? desc->Min + desc->Size - 1 : desc->Min;
				result = lcp_u32inRange(vaddress, varsize, desc->Min, max);
			}
		}
	}
		break;

#ifdef LEVCAN_USE_FLOAT
	case LCP_Float: {
		float f32 = strtof(s, out);
		if (s == *out) {
			result = LC_DataError;
			break;
		}
		if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Float_t)) {
			result = LC_ObjectError;
			break;
		}

		LCP_Float_t *desc = (LCP_Float_t*) parameter->Descriptor;
		*((float*) vaddress) = f32;
		result = lcp_f32inRange(vaddress, varsize, desc->Min, desc->Max);
	}
		break;
#endif
#ifdef LEVCAN_USE_DOUBLE
	case LCP_Double: {
		double d64 = strtod(s, out);
		if (s == *out) {
			result = LC_DataError;
			break;
		}
		if (parameter->Descriptor == 0 || parameter->DescSize != sizeof(LCP_Double_t)) {
			result = LC_ObjectError;
			break;
		}

		LCP_Double_t *desc = (LCP_Double_t*) parameter->Descriptor;
		*((double*) vaddress) = d64;
		result = lcp_d64inRange(vaddress, varsize, desc->Min, desc->Max);
	}
		break;
#endif
	default:
		break;
	}

	*out = (char*) s + strcspn(s, newline); //scroll to new line
	*out = (char*) skipspaces(*out);
	return result;
}

const char* skipspaces(const char *s) {
	for (; isspace((uint8_t)*s); s++)
		;
	return s;
}
