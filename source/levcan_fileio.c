/*
 * levcan_fileio.c
 *
 *  Created on: 10 July 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#include "levcan.h"
#include "levcan_fileio.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

enum {
	LC_FA_Read, // Specifies read access to the object. Data can be read from the file.
	LC_FA_Write, // Specifies write access to the object. Data can be written to the file. Combine with LC_FA_Read for read-write access.
	LC_FA_OpenExisting, // Opens the file. The function fails if the file is not existing. (Default)
	LC_FA_CreateNew, // Creates a new file. The function fails with FR_EXIST if the file is existing.
	LC_FA_CreateAlways, // Creates a new file. If the file is existing, it will be truncated and overwritten.
	LC_FA_OpenAlways, //	Opens the file if it is existing. If not, a new file will be created.
	LC_FA_OpenAppend, // Same as LC_FA_OpenAlways except the read/write pointer is set end of the file.
};

typedef struct {

} LC_File_t;

int LC_FileOpen(LC_File_t* fp, char* name, uint16_t server_node, uint16_t mode) {
	if (server_node == LC_Broadcast_Address) {
		//look for any server node
		while (1) {
			int counter = 0;
			LC_NodeShortName node = LC_GetActiveNodes(&counter);
			if (node.NodeID == LC_Broadcast_Address)
				return 1; //nothing found
			if (node.FileServer) {
				server_node = node.NodeID;
				break; //found!
			}
		}
	}

}
