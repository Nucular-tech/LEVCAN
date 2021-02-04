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
#include "levcan_paramserver.h"
#include "levcan_paraminternal.h"
#include <string.h>

static int checkExists(const LCPS_Directory_t directories[], uint16_t dirsize, uint16_t directory, int32_t entry_index);
static const char* extractEntryName(const LCPS_Directory_t directories[], uint16_t dirsize, const LCPS_Entry_t *entry);

void lc_proceedParameterRequest(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	LC_ObjectRecord_t sendRec = { .NodeID = header.Source, .Attributes.Priority = LC_Priority_Low, .Attributes.TCP = 1 };
	LCPS_Entry_t entry_funct_buffer = { 0 };
	(void) header;
	uint8_t status = LC_Ok;
	if (header.MsgID != LC_SYS_ParametersRequest)
		return;
	if (size == sizeof(lc_request_directory_t)) {
		//request directory
		lc_request_directory_t request = *((lc_request_directory_t*) data);
		//check directory
		LCPS_Directory_t *directories = (LCPS_Directory_t*) node->Directories;
		uint16_t dirsize = node->DirectoriesSize;
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

				sendRec.Address = (void*)directory->Name;
				sendRec.Size = dirdata.NameSize + 1; //+0
				status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersName);
			} else {
				status = LC_AccessError;
			}
		} else {
			status = LC_OutOfRange;
		}
	} else if (size == sizeof(lc_request_entry_t)) {
		//request entry
		lc_request_entry_t request = *((lc_request_entry_t*) data);
		//check directory
		LCPS_Directory_t *directories = (LCPS_Directory_t*) node->Directories;
		uint16_t dirsize = node->DirectoriesSize;
		status = LC_OutOfRange;

		while (checkExists(directories, dirsize, request.Directory, request.Entry)) {
			//get root
			LCPS_Directory_t *directory = &(directories[request.Directory]);
			LCPS_Entry_t *entry = (void*)&directory->Entries[request.Entry];
			//current access should be same or higher
			//header "not exists" if there is no access to it
			if (node->AccessLevel >= directory->AccessLvl) {
				if (node->AccessLevel >= entry->AccessLvl) {
					if (entry->Function) {
						if (entry->Callback != 0) {
							//get parameter from function
							status = entry->Callback(node, header, &entry_funct_buffer, request.Directory, request.Entry);
							entry = &entry_funct_buffer;
						} else {
							status = LC_ObjectError;
							break; //end while
						}
					} else {
						status = LC_Ok; //plain data
					}
					if (status == LC_Ok) {
						//this fits in one can message, so gonna be placed in a buffer
						char *name = (void*)extractEntryName(directories, dirsize, entry);

						if (request.Command & lcp_reqData) {
							lc_entry_data_t entrydata;
							entrydata.DescSize = entry->DescSize;
							entrydata.EntryType = entry->EntryType;
							entrydata.Mode = entry->Mode;
							entrydata.VarSize = entry->VarSize;
							entrydata.EntryIndex = request.Entry;

							int textlen = strnlen(name, LEVCAN_PARAM_MAX_NAMESIZE);
							if (entry->TextData) {
								textlen += strnlen(entry->TextData, LEVCAN_PARAM_MAX_TEXTSIZE);
							}
							entrydata.TextSize = textlen;

							sendRec.Address = &entrydata;
							sendRec.Size = sizeof(lc_entry_data_t);
							status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersData);
						}

						if (request.Command & lcp_reqName) {
							sendRec.Address = (void*)entry->Name;
							sendRec.Size = strnlen(name, 128) + 1;
							status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersName);
						}

						if (request.Command & lcp_reqText) {
							sendRec.Address = (void*)entry->TextData;
							sendRec.Size = strnlen(entry->TextData, 512) + 1;
							status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersText);
						}

						if (request.Command & lcp_reqDescriptor) {
							sendRec.Address = (void*)entry->Descriptor;
							sendRec.Size = entry->DescSize;
							status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersDescriptor);
						}

						if (request.Command & lcp_reqVariable) {
							if ((entry->Mode & LCP_WriteOnly) == 0) {
								sendRec.Address = (void*)entry->Variable;
								sendRec.Size = entry->VarSize;
								status = LC_SendMessage(node, &sendRec, LC_SYS_ParametersValue);
							}
						}
						break; //end while
					}
				} else {
					if (request.Command < lcp_reqFullEntry) {
						//partial request is not a search
						status = LC_AccessError;
						break; //end while
					} else {
						//skip locked entries
						if (request.Entry < directory->Size) {
							request.Entry++;
						} else {
							status = LC_OutOfRange;
							break; //end while
						}
					}
				}
			} else {
				status = LC_AccessError;
				break; //end while
			}
		}
	} else {
		status = LC_DataError;
	}

	//todo "not found message"
	if (status != LC_Ok) {
		sendRec.Address = &status;
		sendRec.Size = sizeof(lc_request_error_t);
		LC_SendMessage(node, &sendRec, header.MsgID);
	}
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
		int32_t dindex = entry->DirectoryIndex;
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
