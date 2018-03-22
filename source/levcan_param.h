/*
 * LEV-CAN: Light Electric Vehicle CAN protocol, parameters manager
 * levcan_param.h
 *
 *  Created on: 17 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */
#include "stdint.h"

#pragma once

//parameter size type, used in slaves
typedef enum {
	PT_uint8, PT_int8, PT_uint16, PT_int16, PT_int32, PT_float, PT_unknown
} LC_ParamType_t;

//eclipse parser workaround
#ifdef __CDT_PARSER__
#define _Generic(...) 0
#endif
//macro for auto-type specifier
#define pti(param, min, max, step, decimal) &(param), min, max, step, decimal, _Generic((param),		\
        uint8_t: 	PT_uint8,	\
		int8_t: 	PT_int8,	\
		uint16_t: 	PT_uint16,	\
		int16_t: 	PT_int16,	\
		uint32_t: 	PT_int32,	\
		int32_t: 	PT_int32,	\
		float: 		PT_float,	\
		default: 	PT_unknown)

//WIP - work in progress
typedef enum {
	RT_value, 	//integer parameter, <name> and maybe <formatting> where '%s' defines position of converted <value>
	RT_mask, 	//mask parameter, WIP
	RT_bool, 	//ON/OFF parameter, <name> + ON / OFF default  (formatting not sent)
	RT_enum,	//string enumerator, <name> + <formatting> ("1\n2\n3\n"), where <value> = line index, or if out of index, write <value>
	RT_string,	//string optional <name> + editable (or not if RO) <formatting>
	RT_bitfield, //bit field,  <name> + <formatting> ("Flag1\nFlag2\nFlag3\n"), value is bitfield, where new line is a one bit.
	RT_func,	//function entry, WIP
	RT_remap,	//entry remapped to address
	RT_dir = 0x3F,		//directory entry, <name> only
	RT_readonly = 0x40,	//read only bit, applicable to value, mask, bool, enum, string
	RT_invalid = 0xFF,	//ending
} LC_ReadType_t; //todo rename to EntryType

typedef struct {
	void* Address;
	int32_t Min;
	int32_t Max;
	int32_t Step;
	uint8_t Decimal;
	LC_ParamType_t Type;
	LC_ReadType_t ReadType;
	char* Name;
	char* Formatting;
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
	LC_ReadType_t ReadType;
	char* Name;
	char* Formatting;
} LC_ParameterValue_t;

typedef struct {
	const LC_ParameterAdress_t* Address;
	uint16_t Size;
} LC_ParameterDirectory_t;

typedef struct {
	void* Address;
	LC_ReadType_t ReadType;
	char* Formatting;
} LC_VariableAdress_t;

typedef struct {
	int32_t Value;
	LC_ReadType_t ReadType;
	char* Formatting;
} LC_VariableValue_t;

typedef struct {
	const LC_VariableAdress_t* Address;
	uint16_t Size;
} LC_VariableDirectory_t;

#define DIRDEF(NAME)		{NAME,(sizeof(NAME)/sizeof(NAME[0]))}

void ParamInfo_Size(void* vnode);
void LC_ParametersPrintAll(void* vnode);
void LC_ParameterUpdateAsync(LC_ParameterValue_t* paramv, uint16_t dir, void* sender_node, uint16_t receiver_node, int full);
void LC_ParametersStopUpdating(void);
