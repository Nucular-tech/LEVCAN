/*
 * levcan_fileclient.h
 *
 *  Created on: 10 July 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */
#pragma once

#include <stdint.h>
#include "levcan.h"
#include "levcan_config.h"

typedef enum {
	LC_FA_Read = 0x01, 			// Specifies read access to the object. Data can be read from the file.
	LC_FA_Write = 0x02, 		// Specifies write access to the object. Data can be written to the file. Combine with LC_FA_Read for read-write access.
	LC_FA_OpenExisting = 0x00, 	// Opens the file. The function fails if the file is not existing. (Default)
	LC_FA_CreateNew = 0x04, 	// Creates a new file. The function fails with FR_EXIST if the file is existing.
	LC_FA_CreateAlways = 0x08, 	// Creates a new file. If the file is existing, it will be truncated and overwritten.
	LC_FA_OpenAlways = 0x10, 	// Opens the file if it is existing. If not, a new file will be created.
	LC_FA_OpenAppend = 0x30, 	// Same as LC_FA_OpenAlways except the read/write pointer is set end of the file.
} LC_FileAccess_t;

typedef enum {
	LC_FR_Ok = 0,				/* (0) Succeeded */
	LC_FR_DiskErr,				/* (1) A hard error occurred in the low level disk I/O layer */
	LC_FR_IntErr,				/* (2) Assertion failed */
	LC_FR_NotReady,				/* (3) The physical drive cannot work */
	LC_FR_NoFile,				/* (4) Could not find the file */
	LC_FR_NoPath,				/* (5) Could not find the path */
	LC_FR_InvalidName,			/* (6) The path name format is invalid */
	LC_FR_Denied,				/* (7) Access denied due to prohibited access or directory full */
	LC_FR_Exist,				/* (8) Access denied due to prohibited access */
	LC_FR_InvalidObject,		/* (9) The file/directory object is invalid */
	LC_FR_WriteProtected,		/* (10) The physical drive is write protected */
	LC_FR_Reserved1,			/* (11 NOT USED) The logical drive number is invalid */
	LC_FR_Reserved2,			/* (12 NOT USED) The volume has no work area */
	LC_FR_Reserved3,			/* (13 NOT USED) There is no valid FAT volume */
	LC_FR_Reserved4,			/* (14 NOT USED) The f_mkfs() aborted due to any problem */
	LC_FR_Timeout,				/* (15) Could not get a grant to access the volume within defined period */
	LC_FR_Locked,				/* (16) The operation is rejected according to the file sharing policy */
	LC_FR_Reserved5,			/* (17 NOT USED) LFN working buffer could not be allocated */
	LC_FR_TooManyOpenFiles,		/* (18) Number of open files > FF_FS_LOCK */
	LC_FR_InvalidParameter,		/* (19) Given parameter is invalid */
	LC_FR_NetworkTimeout,		/* (20) Could not get access the node */
	LC_FR_NetworkError,			/* (21) Data corrupted during transmission */
	LC_FR_NetworkBusy,			/* (22) Buffer full */
	LC_FR_MemoryFull,			/* (23) Could not allocate data */
	LC_FR_NodeOffline,			/* (24) Node disabled */
	LC_FR_FileNotOpened,		/* (25) File was closed by timeout or it wasn't opened at all  */
} LC_FileResult_t;

typedef struct {
	uint32_t Pointer;
	uint16_t Errors;
} LC_FileAck_t;

LC_FileResult_t LC_FileOpen(char* name, LC_FileAccess_t mode, void* sender_node, uint8_t server_node);
LC_FileResult_t LC_FileRead(char* buffer, uint32_t btr, uint32_t* br, void* sender_node);
LC_FileResult_t LC_FileWrite(const char* buffer, uint32_t btw, uint32_t* bw, void* sender_node);
LC_FileResult_t LC_FileClose(void* sender_node, uint8_t server_node);
LC_FileResult_t LC_FileLseek(uint32_t position, void* sender_node);
uint32_t LC_FileTell(void* sender_node);
uint32_t LC_FileSize(void* sender_node);
LC_FileResult_t LC_FileTruncate(void* sender_node);

LC_NodeShortName_t LC_FindFileServer(uint16_t* scnt);
LC_NodeShortName_t LC_FileGetServer(void* sender_node);
