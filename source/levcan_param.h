/*
 * LEV-CAN: Light Electric Vehicle CAN protocol, parameters manager
 * levcan_param.h
 *
 *  Created on: 17 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */
#include "stdint.h"
#include "levcan.h"

#pragma once

//parameter size type, used in slaves
typedef enum {
	VT_uint8, VT_int8, VT_uint16, VT_int16, VT_int32, VT_float, VT_unknown
} LC_ValueType_t;

//eclipse parser workaround
#ifdef __CDT_PARSER__
#define _Generic(...) 0
#endif
//macro for auto-type specifier
#define pti(param, min, max, step, decimal) (void*)&(param), min, max, step, decimal, _Generic((param),		\
        uint8_t: 	VT_uint8,	\
		int8_t: 	VT_int8,	\
		uint16_t: 	VT_uint16,	\
		int16_t: 	VT_int16,	\
		uint32_t: 	VT_int32,	\
		int32_t: 	VT_int32,	\
		float: 		VT_float,	\
		LC_ParameterAdress_t: 0,\
		default: 	VT_unknown)

//WIP - work in progress
typedef enum {
	PT_value, 	//integer parameter, <name> and maybe <formatting> where '%s' defines position of converted <value>
	PT_mask, 	//mask parameter, (not implemented)
	PT_bool, 	//ON/OFF parameter, <name> + ON / OFF default  (formatting not sent)
	PT_enum,	//string enumerator, <name> + <formatting> ("1\n2\n3\n"), where <value> = line index, or if out of index, write <value>
	PT_string,	//string optional <name> + editable (or not if RO) <formatting> (not implemented)
	PT_bitfield, //bit field,  <name> + <formatting> ("Flag1\nFlag2\nFlag3\n"), value is bitfield, where new line is a one bit. (not implemented)
	PT_func,	//function entry (not implemented)
	PT_remap,	//entry remapped to address (not implemented)
	PT_dir,		//directory entry, <name> only, readonly directory means runtime variables and will not be exported
	PT_readonly = 0x20,	//read only bit, applicable to value, mask, bool, enum, string, directory
	PT_reqval = 0x40,	//value requested
	PT_noinit = 0x80,	//parameter to be received
	PT_invalid = 0xFF,	//ending
} LC_ParamType_t;

typedef struct {
	void* Address;
	int32_t Min;
	int32_t Max;
	int32_t Step;
	uint8_t Decimal;
	LC_ValueType_t ValueType; //addressed parameter size and type
	LC_ParamType_t ParamType; //this entry mode/type
	const char* Name;
	const char* Formatting;
} LC_ParameterAdress_t;

typedef struct {
	int32_t Value;
	union {
		int32_t Min;
		int32_t DirectoryIndex;
	};
	union {
		int32_t Max;
		int32_t EntryIndex;
	};
	int32_t Step;
	uint8_t Index;
	uint8_t Decimal;
	LC_ParamType_t ParamType;
	char* Name;
	char* Formatting;
} LC_ParameterValue_t;

typedef struct {
	const LC_ParameterAdress_t* Address;
	uint16_t Size;
} LC_ParameterDirectory_t;

typedef struct {
	void* Address;
	LC_ParamType_t ParamType;
	char* Formatting;
} LC_VariableAdress_t;

typedef struct {
	int32_t Value;
	LC_ParamType_t ParamType;
	char* Formatting;
} LC_VariableValue_t;

typedef struct {
	const LC_VariableAdress_t* Address;
	uint16_t Size;
} LC_VariableDirectory_t;

typedef struct {
	int32_t Size;
	int32_t Textsize;
	int32_t Parameters;
	int32_t ParametersWritable;
} LC_ParameterTableSize_t;

#define DIRDEF(NAME)		{NAME,(sizeof(NAME)/sizeof(NAME[0]))}

LC_ParameterTableSize_t LC_ParamInfo_Size(void* vnode);
void LC_ParametersPrintAll(void* vnode);
LC_Return_t LC_ParameterSet(LC_ParameterValue_t* paramv, uint16_t dir, void* sender_node, uint16_t receiver_node) ;
LC_Return_t LC_ParameterUpdateAsync(LC_ParameterValue_t* paramv, uint16_t dir, void* sender_node, uint16_t receiver_node, uint8_t full);
void LC_ParametersStopUpdating(void);
