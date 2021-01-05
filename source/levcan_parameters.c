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

#include "levcan_parameters.h"

typedef struct {
	uint16_t Directory;
	uint16_t Entry;
} lc_param_request_t;

typedef struct {
	uint8_t EntryType; //LCP_Type_t
	uint8_t Mode;
	uint16_t VarSize;
	uint16_t DescSize;
	uint16_t NameSize;
	uint16_t TextSize;
} lc_entry_data_t;

static int checkExists(const LCP_Directory_t directories[], uint16_t dirsize, uint16_t directory, uint16_t eindex);
static uint8_t checkAccess(const LCP_Directory_t directories[], uint16_t dirsize, uint16_t directory, uint16_t eindex);
static const char* extractName(const LCP_Directory_t directories[], uint16_t dirsize, uint16_t directory, uint16_t eindex);

void lc_proceedParameterRequest(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	const LCP_Entry_t emptyEntry = { .EntryType = LCP_End };
	(void) header;

	if (size != sizeof(lc_param_request_t))
		return;
	lc_param_request_t request = *((lc_param_request_t*) data);

	const LCP_Entry_t *entry = 0;
	LCP_Directory_t *directories = (LCP_Directory_t*) node->Directories;
	uint16_t dirsize = node->DirectoriesSize;

	if (checkExists(directories, dirsize, request.Directory, 0)) {
		//get root and header
		LCP_Directory_t *directory = &(directories[request.Directory]);
		const LCP_Entry_t *header = &directory->Entries[0];

		if (node->AccessLevel < header->AccessLvl) {
			//header "not exists" if there is no access to it
			while (checkExists(directories, dirsize, request.Directory, request.Entry)) {
				if (node->AccessLevel < checkAccess(directories, dirsize, request.Directory, request.Entry)) {
					//access locked
					if (request.Entry != 0)
						request.Entry++; //get next one
					continue;
				} else {
					//access allowed
					entry = &directory->Entries[request.Entry];
				}
			} //while end
		}
	}

	if (entry == 0) {
		//send "end" entry
		entry = &emptyEntry;
	}
	lc_entry_data_t data;
}

static int checkExists(const LCP_Directory_t directories[], uint16_t dirsize, uint16_t directory, uint16_t eindex) {
	if (directories == 0 || directory >= dirsize || directories[directory].Entries == 0 || eindex >= directories[directory].Size)
		return 0;
	return 1;
}
//returns highest access in a tree (directory, entry or reference to an entry)
static uint8_t checkAccess(const LCP_Directory_t directories[], uint16_t dirsize, uint16_t directory, uint16_t eindex) {
	uint8_t accesslvl = 0;
	if (checkExists(directories, dirsize, directory, eindex)) {
		const LCP_Directory_t *dir = &directories[directory];
		const LCP_Entry_t *param = &dir->Entries[0];
		//check directory header
		if (param->AccessLvl > accesslvl)
			accesslvl = param->AccessLvl;

		if (eindex > 0) {
			//check specified item if not header
			param = &dir->Entries[eindex];
			if (param->AccessLvl > accesslvl)
				accesslvl = param->AccessLvl;
			//nested
			if (param->EntryType == LCP_Header) {
				uint8_t diraccess = checkAccess(directories, dirsize, param->DirectoryIndex, 0);
				if (diraccess > accesslvl)
					accesslvl = diraccess;
			}
		}
	}

	return accesslvl;
}

static const char* extractName(const LCP_Directory_t directories[], uint16_t dirsize, uint16_t directory, uint16_t eindex) {
	const char *unknown = "Unknown";
	const char *source = 0;

	//NOTE directory and eindex here have unsafe access, boundary check should be done outside of this function
	const LCP_Directory_t *dir = &directories[directory];

	if (checkExists(directories, dirsize, directory, eindex)) {
		const LCP_Entry_t *param = &dir->Entries[eindex];

		if (param->EntryType == LCP_HeaderArray) {
			//extracting name from array type directory
			int32_t aindex = dir->ArrayIndex;
			char **index_name = (char**) param->Name;
			if (index_name) {
				source = index_name[aindex];
			} else
				source = unknown; //directory should have a name anyway
		} else if (param->Name)
			source = param->Name;
		else if (param->EntryType == LCP_Header) {
			//it is directory and no name, try get name from child
			source = extractName(directories, dirsize, param->DirectoryIndex, 0);
		}
	}
	if (source == 0)
		source = unknown; //directory should have a name anyway

	return source;
}
