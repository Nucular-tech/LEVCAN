/*
 * LEV-CAN: Light Electric Vehicle CAN protocol, parameters manager
 * levcan_param.c
 *
 *  Created on: 17 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */
#include "levcan.h"
#include "levcan_param.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern void *lcmalloc(uint32_t xWantedSize);
extern void lcfree(void *pv);

typedef struct {
	int32_t Value;
	int32_t Min;
	int32_t Max;
	int32_t Step;
	uint8_t Decimal;
	uint8_t Directory;
	uint8_t Index;
	LC_ParamType_t ParamType;
	char Literals[];
} parameterValuePacked_t;

typedef struct {
	int32_t Value;
	uint8_t Directory;
	uint8_t Index;
} storeValuePacked_t;

typedef struct {
	LC_ParameterValue_t* Param;
	void* Node;
	uint16_t Directory;
	uint16_t Source;
	uint8_t Full;
} bufferedParam_t;

//### Local functions ###
void proceedParam(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size);
uint16_t check_align(const LC_ParameterAdress_t* parameter);
char* extractName(const LC_ParameterAdress_t* param);
bufferedParam_t* findReceiver(int16_t dir, int16_t index, int16_t source);
int isDirectory(LC_NodeDescription_t* node, const char* s);
int isParameter(LC_NodeDescription_t* node, const char* s, uint8_t directory);
int32_t getParameterValue(const LC_ParameterAdress_t* parameter);
int setParameterValue(const LC_ParameterAdress_t* parameter, int32_t value);
char* printParam(const LC_ParameterAdress_t* parameter);
void proceed_RX(void);

//### Local variables ###
bufferedParam_t receive_buffer[LEVCAN_PARAM_QUEUE_SIZE];
volatile uint16_t receiveFIFO_in = 0, receiveFIFO_out = 0;
volatile uint16_t receive_busy = 0;

char* extractName(const LC_ParameterAdress_t* param) {
	char* source = 0;
	//try to get name
	if (param->Name)
		source = param->Name;
	else if ((param->ParamType & ~PT_readonly) == PT_dir) {
		//it is directory and no name, try get name from child
		if (param->Address && ((LC_ParameterAdress_t*) param->Address)[0].Name) {
			//this is directory, it's name can stored in child [0] directory index
			source = ((LC_ParameterAdress_t*) param->Address)[0].Name;
		} else
			source = "Unknown directory"; //directory should have a name anyway
	}
	return source;
}

parameterValuePacked_t param_invalid = { .Index = 0, .Directory = 0, .ParamType = PT_invalid, .Literals = { 0, 0 } };

bufferedParam_t* findReceiver(int16_t dir, int16_t index, int16_t source) {
	if (receiveFIFO_in != receiveFIFO_out) {
		//receive_buffer[receiveFIFO_out]
		int out = receiveFIFO_out;
		receiveFIFO_out = (receiveFIFO_out + 1) % LEVCAN_PARAM_QUEUE_SIZE;

		if (receive_buffer[out].Param != 0 && receive_buffer[out].Directory == dir && receive_buffer[out].Param->Index == index && receive_buffer[out].Source == source)
			return &receive_buffer[out];
	}
	return 0;
}

uint16_t check_align(const LC_ParameterAdress_t* parameter) {
//parameter should be aligned to avoid memory fault, this may happen when int16 accessed as int32
	uint16_t align = 0;
	switch (parameter->ValueType) {
	case VT_int16:
	case VT_uint16:
		align = (int32_t) parameter->Address % 2;
		break;
	case VT_int32:
		align = (int32_t) parameter->Address % 4;
		break;
	default:
		break;
	}
#ifdef LEVCAN_TRACE
	if (align) {
		trace_printf("Invalid address align 0x%08X in parameter '%s'\n", parameter->Address, parameter->Name);
	}
#endif
	return align;
}

const char* equality = " = ";

char* printParam(const LC_ParameterAdress_t* parameter) {
	static char line[50];

	if (!parameter)
		return 0;
//not thread safe
//for string this enough
//for (int s = 0; s < 50; s++) {
	line[0] = 0;
//}
	switch (parameter->ParamType) {
	case PT_invalid:
		return 0;

	case PT_dir: {
		strcat(line, "\n[");
		strcat(line, parameter->Name);
		strcat(line, "]\n");
	}
		break;
	case PT_enum: {
		strcpy(line, parameter->Name);
		strcat(line, equality);

		int32_t val = getParameterValue(parameter);

		char *position = parameter->Formatting;
		for (int i = 0; (i < val); i++) {
			position = strchr(position, '\n'); //look for line specified by val index
			if (position == 0)
				break;
			position++; //skip '\n'
		}
		if (position == 0)
			strcat(line, sf32toa(val, 0, 0));
		else {
			char* end = strchr(position, '\n'); //search for line end
			if (end) {
				strncat(line, position, end - position);
			} else
				strcat(line, position);	//end is possible zero
		}
		strcat(line, "\n");
	}
		break;
	case PT_value | PT_readonly:
	case PT_value: {
		int32_t val = getParameterValue(parameter);
		strcpy(line, parameter->Name);
		strcat(line, equality);
		strcat(line, sf32toa(val, parameter->Decimal, 0));
		strcat(line, "\n");
	}
		break;

	case PT_bool | PT_readonly:
	case PT_bool: {
		int32_t val = getParameterValue(parameter);
		strcpy(line, parameter->Name);
		strcat(line, equality);
		if (val)
			strcat(line, "ON\n");
		else
			strcat(line, "OFF\n");
	}
		break;
	default:
		break;
	}

	return line;
}

int32_t getParameterValue(const LC_ParameterAdress_t* parameter) {
	int32_t value = 0;
	if (((uint32_t) parameter->Address > UINT8_MAX) && (check_align(parameter) == 0)) {
		switch (parameter->ValueType) {
		default:
			value = (*(uint8_t*) (parameter->Address));
			break;
		case VT_int8:
			value = (*(int8_t*) parameter->Address);
			break;
		case VT_int16:
			value = (*(int16_t*) parameter->Address);
			break;
		case VT_uint16:
			value = (*(uint16_t*) parameter->Address);
			break;
		case VT_int32:
			value = (*(int32_t*) parameter->Address);
			break;
#ifdef LEVCAN_USE_FLOAT
		case VT_float:
			value = (*(float*) parameter->Address) * powf(10, parameter->Decimal);
			break;
#endif
		}
	}
	return value;
}

int setParameterValue(const LC_ParameterAdress_t* parameter, int32_t value) {
	if (value > parameter->Max || value < parameter->Min)
		return 1;
	if (((uint32_t) parameter->Address > UINT8_MAX) && (check_align(parameter) == 0))
		switch (parameter->ValueType) {
		case VT_int8:
		case VT_uint8:
			(*(uint8_t*) parameter->Address) = (uint8_t) value;
			break;
		case VT_int16:
		case VT_uint16:
			(*(uint16_t*) parameter->Address) = (uint16_t) value;
			break;
		case VT_int32:
			(*(uint32_t*) parameter->Address) = (uint32_t) value;
			break;
#ifdef LEVCAN_USE_FLOAT
		case VT_float:
			(*(float*) parameter->Address) = (float) value / powf(10, parameter->Decimal);
			break;
#endif
		default:
			break;
		}
	return 0;
}

int setParameterStr(const LC_ParameterAdress_t* parameter, char* value) {
	int ret = -1;
	if (parameter->ParamType == PT_enum || parameter->ParamType == PT_bool) {
		value = skipspaces(value);
		char* haystack = parameter->Formatting;
		if (parameter->ParamType == PT_bool)
			haystack = "OFF\nON";
		char* finded = strstr(parameter->Formatting, value);
		//match found!
		if (finded && finded[0] != '#') {
			int i;
			for (i = 0; haystack >= finded; finded--) {
				//increase value index in enum each founded newline
				if (*finded == '\n')
					i++;
			}
			ret = setParameterValue(parameter, i);
		}
	}
	if (ret == -1) {
		//other types failed? try integer type, anywhere supported
		int32_t vali = 0;
		//float valf = 0;
		int type = strtoif(value, &vali, 0, parameter->Decimal);
		if (type)
			ret = setParameterValue(parameter, vali);
	}

	return ret;
}

int isDirectory(LC_NodeDescription_t* node, const char* s) {

	for (uint16_t i = 0; i < node->DirectoriesSize; i++) {
		if (((LC_ParameterDirectory_t*) node->Directories)[i].Address[0].Name && strcmp(((LC_ParameterDirectory_t*) node->Directories)[i].Address[0].Name, s) == 0) {
			return i;
		}
	}
	return -1;
}

int isParameter(LC_NodeDescription_t* node, const char* s, uint8_t directory) {
//index [0] is directory entry, dont scan
	for (uint16_t i = 1; i < ((LC_ParameterDirectory_t*) node->Directories)[directory].Size; i++) {
		//todo carefully check types
		if ((((LC_ParameterDirectory_t*) node->Directories)[directory].Address[i].ParamType < PT_dir)
				&& ((LC_ParameterDirectory_t*) node->Directories)[directory].Address[i].Name
				&& (strcmp(((LC_ParameterDirectory_t*) node->Directories)[directory].Address[i].Name, s) == 0)) {
			return i;
		}
	}
	return -1;
}

void proceedParam(LC_NodeDescription_t* node, LC_Header_t header, void* data, int32_t size) {
#ifdef LEVCAN_MEM_STATIC
	static char static_buffer[sizeof(parameterValuePacked_t) + 128] = {0};
#endif
	LC_ObjectRecord_t txrec = { 0 };
	txrec.Attributes.Priority = LC_Priority_Low;
	txrec.Attributes.TCP = 1;
	if (data == 0)
		return; // nothing to do so here
	//TODO add node filter
	switch (size) {
	case 2: {
		//2 byte - request (index and directory)
		uint16_t pdindex = ((uint8_t*) data)[0];
		uint16_t pddir = ((uint8_t*) data)[1];
		//trace_printf("Request: id=%d, dir=%d\n", pdindex,pddir);
		int32_t value = 0;
		//array correctly filled?
		if (pddir < node->DirectoriesSize && pdindex < ((LC_ParameterDirectory_t*) node->Directories)[pddir].Size) {
			int32_t namelength = 0, formatlength = 0;
			//check strings
			char* s_name = extractName(&((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex]);
			if (s_name)
				namelength = strlen(s_name);
			if (((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].Formatting)
				formatlength = strlen(((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].Formatting);
			//now allocate full size
#ifdef LEVCAN_MEM_STATIC
			parameterValuePacked_t* param_to_send = &static_buffer;
			if (namelength + formatlength + 2 > 128) {
				formatlength = 0;
				if (namelength + 2 > 128)
				namelength = 128 - 2;
			}
#endif
			int32_t totalsize = sizeof(parameterValuePacked_t) + namelength + formatlength + 2;
#ifndef LEVCAN_MEM_STATIC
			parameterValuePacked_t* param_to_send = lcmalloc(totalsize);
#endif
			if (param_to_send == 0)
				return; // nothing to do so here

			//just copy-paste
			value = getParameterValue(&((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex]);
			param_to_send->Value = value;
			param_to_send->Min = ((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].Min;
			param_to_send->Max = ((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].Max;
			param_to_send->Decimal = ((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].Decimal;
			param_to_send->Step = ((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].Step;
			param_to_send->ParamType = ((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].ParamType;
			param_to_send->Index = pdindex;
			param_to_send->Directory = pddir;

			param_to_send->Literals[0] = 0;
			if (namelength)
				strncat(param_to_send->Literals, extractName(&((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex]), namelength);

			param_to_send->Literals[namelength + 1] = 0; //next string start
			if (formatlength)
				strncat(&param_to_send->Literals[namelength + 1], ((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].Formatting, formatlength);

			txrec.Address = param_to_send;
			txrec.Size = totalsize;
			txrec.NodeID = header.Source;
#ifndef LEVCAN_MEM_STATIC
			txrec.Attributes.Cleanup = 1;
#endif
			if (LC_SendMessage(node, &txrec, LC_SYS_Parameters)) {
#ifndef LEVCAN_MEM_STATIC
				lcfree(param_to_send);
#endif
			}
		} else {
			//non existing item
			param_invalid.Index = pdindex;
			param_invalid.Directory = pddir;
			txrec.Address = &param_invalid;
			txrec.Size = sizeof(parameterValuePacked_t) + 2;
			txrec.NodeID = header.Source;
			LC_SendMessage(node, &txrec, LC_SYS_Parameters);
			//trace_printf("Info ERR sent i:%d, d:%d\n", pdindex, pddir);
		}
	}
		break;
	case 3: {
		//request data value
		uint16_t pdindex = ((uint8_t*) data)[0];
		uint16_t pddir = ((uint8_t*) data)[1];
		//array correctly filled?
		if ((pddir < node->DirectoriesSize && pdindex < ((LC_ParameterDirectory_t*) node->Directories)[pddir].Size)
				&& (((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].ParamType & ~PT_readonly) != PT_dir
				&& (((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex].ParamType & ~PT_readonly) != PT_func) {
			storeValuePacked_t sendvalue;
			sendvalue.Index = pdindex;
			sendvalue.Directory = pddir;
			sendvalue.Value = getParameterValue(&((LC_ParameterDirectory_t*) node->Directories)[pddir].Address[pdindex]);

			txrec.Address = &sendvalue;
			txrec.Size = sizeof(storeValuePacked_t) + 1;
			txrec.NodeID = header.Source;
			//send back data value
			LC_SendMessage(node, &txrec, LC_SYS_Parameters);
		}
	}
		break;
	case sizeof(storeValuePacked_t): {
		//store value
		storeValuePacked_t* store = data;
		if (store->Directory < node->DirectoriesSize && store->Index < ((LC_ParameterDirectory_t*) node->Directories)[store->Directory].Size)
			setParameterValue(&((LC_ParameterDirectory_t*) node->Directories)[store->Directory].Address[store->Index], store->Value);
	}
		break;
	case sizeof(storeValuePacked_t) + 1: {
		//update requested value
		storeValuePacked_t* update = data;
		bufferedParam_t* receiver = findReceiver(update->Directory, update->Index, header.Source);
		//somebody receiving
		if (receiver) {
			receiver->Param->Value = update->Value;
			receiver->Param->ParamType &= ~PT_reqval;
			receiver->Param = 0;
		}
	}
		break;
#ifndef LEVCAN_MEM_STATIC
		//no receive for static memory
	default: {
		if (size > sizeof(parameterValuePacked_t)) {
			//parameter full receive
			parameterValuePacked_t* param_received = data;
			bufferedParam_t* receiver = findReceiver(param_received->Directory, param_received->Index, header.Source);
			//somebody receiving
			if (receiver) {
				receiver->Param->Decimal = param_received->Decimal;
				receiver->Param->Min = param_received->Min;
				receiver->Param->Max = param_received->Max;
				receiver->Param->Step = param_received->Step;
				receiver->Param->Value = param_received->Value;
				receiver->Param->ParamType = param_received->ParamType;
				//receiver->Param->Index=param_received->Index; //should be equal
				int32_t maxstr = size - sizeof(parameterValuePacked_t);
				int sizefail = 0;
				//extract name
				char* clean = receiver->Param->Name;
				int strpos = 0;
				if (param_received->Literals[0] != 0) {
					int length = strnlen(param_received->Literals, 128);
					if (length > maxstr) {
						//broken size!
						sizefail = 1;
						length = maxstr;
					}
					receiver->Param->Name = lcmalloc(length + 1);
					if (receiver->Param->Name) {
						strncpy(receiver->Param->Name, &param_received->Literals[strpos], length);
						receiver->Param->Name[length] = 0; //terminate string
					}
					strpos = length;
				} else
					receiver->Param->Name = 0;
				//cleanup if there was pointer
				if (clean)
					lcfree(clean);
				strpos++; //skip one terminating character
				//extract formatting
				clean = receiver->Param->Formatting;
				if (param_received->Literals[strpos] != 0 && strpos < maxstr) {
					int length = strnlen(&param_received->Literals[strpos], 128);
					if (strpos + length > maxstr) {
						//broken size!
						sizefail = 1;
						length = maxstr - strpos;
					}
					receiver->Param->Formatting = lcmalloc(length + 1);
					if (receiver->Param->Formatting) {
						strcpy(receiver->Param->Formatting, &param_received->Literals[strpos]);
						receiver->Param->Formatting[length] = 0; //terminate string
					}
					strpos = length + 1;
				} else
					receiver->Param->Formatting = 0;
				//cleanup if there was pointer
				if (clean)
					lcfree(clean);
				//delete receiver
				receiver->Param = 0;

#ifdef LEVCAN_TRACE
				if (sizefail)
				trace_printf("Parameter RX size fail\n");
#endif
			}
		}
	}
#endif
		break;
	}
	//get new now
	receive_busy = 0;
	proceed_RX();
	return; // nothing to do so here
}

/// Returns and prints local parameters information
LC_ParameterTableSize_t LC_ParamInfo_Size(void* vnode) {
	LC_NodeDescription_t* node = vnode;
	int32_t size = 0;
	int32_t textsize = 0;
	int32_t parameters = 0;
	int32_t parameters_writable = 0;
	for (int i = 0; i < node->DirectoriesSize; i++) {
		size += ((LC_ParameterDirectory_t*) node->Directories)[i].Size * sizeof(LC_ParameterAdress_t);
		for (int b = 0; b < ((LC_ParameterDirectory_t*) node->Directories)[i].Size; b++) {
			parameters++;
			if ((((LC_ParameterDirectory_t*) node->Directories)[i].Address[b].ParamType & PT_readonly) == 0)
				parameters_writable++;
			if (((LC_ParameterDirectory_t*) node->Directories)[i].Address[b].Name)
				textsize += strlen(((LC_ParameterDirectory_t*) node->Directories)[i].Address[b].Name) + 1;
			if (((LC_ParameterDirectory_t*) node->Directories)[i].Address[b].Formatting)
				textsize += strlen(((LC_ParameterDirectory_t*) node->Directories)[i].Address[b].Formatting) + 1;
		}
	}
#ifdef LEVCAN_TRACE
	trace_printf("Parameters: %d, writable: %d, size bytes: %d, text: %d, total: %d\n", parameters, parameters_writable, size, textsize, size + textsize);
#endif
	LC_ParameterTableSize_t ptsize;
	ptsize.Size = size;
	ptsize.ParametersWritable = parameters_writable;
	ptsize.Parameters = parameters;
	ptsize.Textsize = textsize;
	return ptsize;
}

void LC_ParametersPrintAll(void* vnode) {
	LC_NodeDescription_t* node = vnode;
#ifdef LEVCAN_TRACE
	trace_printf("# %s\n", node->DeviceName);
	trace_printf("# Network ID: %d\n", node->ShortName.NodeID);
	trace_printf("# %s system config:\n", node->NodeName);
#endif
	for (int dir = 0; dir < node->DirectoriesSize; dir++) {
		for (int i = 1; i < ((LC_ParameterDirectory_t*) node->Directories)[dir].Size; i++) {
#ifdef LEVCAN_TRACE
			trace_printf(printParam(&((LC_ParameterDirectory_t*) node->Directories)[dir].Address[i]));
#endif
		}
	}
}

/// Sends new parameter value to receiver
/// @param paramv Pointer to parameter
/// @param dir Directory index
/// @param sender_node Sender node
/// @param receiver_node Receiver ID node
LC_Return_t LC_ParameterSet(LC_ParameterValue_t* paramv, uint16_t dir, void* sender_node, uint16_t receiver_node) {
	//send function will use fast send, so it is  safe
	storeValuePacked_t store = { 0 };
	store.Directory = dir;
	store.Index = paramv->Index;
	store.Value = paramv->Value;

	LC_ObjectRecord_t record = { 0 };
	record.Address = &store;
	record.Attributes.TCP = 1;
	record.Attributes.Priority = LC_Priority_Low;
	record.Size = sizeof(storeValuePacked_t);
	record.NodeID = receiver_node; //receiver
	return LC_SendMessage(sender_node, &record, LC_SYS_Parameters);
}

/// Asynchroniously updates parameter. Setup index and directory to get one.
/// @param paramv Pointer to parameter, where it will be stored. Setup LC_ParameterValue_t.Index here
/// @param dir Directory index
/// @param sender_node Sender node, who is asking for
/// @param receiver_node Receiver ID node
/// @param full	0 - request just value, 1 - request full parameter information.
LC_Return_t LC_ParameterUpdateAsync(LC_ParameterValue_t* paramv, uint16_t dir, void* sender_node, uint16_t receiver_node, uint8_t full) {
	LC_NodeDescription_t* node = sender_node;
	//todo reentrancy
	if (receiveFIFO_in == ((receiveFIFO_out - 1 + LEVCAN_PARAM_QUEUE_SIZE) % LEVCAN_PARAM_QUEUE_SIZE))
		return LC_BufferFull;

	bufferedParam_t* receive = &receive_buffer[receiveFIFO_in];
	receive->Param = paramv;
	receive->Directory = dir;
	receive->Source = receiver_node;
	receive->Full = full;
	receive->Node = sender_node;

	if (paramv->ParamType == PT_invalid)
		paramv->ParamType = 0;

	if (full) {
		paramv->ParamType |= PT_noinit;
	} else {
		paramv->ParamType |= PT_reqval;
	}
	receiveFIFO_in = (receiveFIFO_in + 1) % LEVCAN_PARAM_QUEUE_SIZE;
	if (receive_busy == 0)
		proceed_RX();

	return LC_Ok;
}

/// Stops all async updates of parameters
void LC_ParametersStopUpdating(void) {
	//clean up all tx buffers,
	//todo thread safe
	for (int i = 0; i < LEVCAN_PARAM_QUEUE_SIZE; i++) {
		receive_buffer[i] = (bufferedParam_t ) { 0 };
	}
	receiveFIFO_in = 0;
	receiveFIFO_out = 0;
	receive_busy = 0;
}

void proceed_RX(void) {
	if (receiveFIFO_in != receiveFIFO_out) {
		//receive_buffer[receiveFIFO_out]

		uint8_t data[3] = { 0 };
		data[0] = receive_buffer[receiveFIFO_out].Param->Index;
		data[1] = receive_buffer[receiveFIFO_out].Directory;
		data[2] = 0;

		LC_ObjectRecord_t record = { 0 };
		record.Address = &data;
		record.Attributes.TCP = 1;
		record.Attributes.Priority = LC_Priority_Low;
		record.NodeID = receive_buffer[receiveFIFO_out].Source;

		if (receive_buffer[receiveFIFO_out].Full) {
			record.Size = 2;
		} else {
			record.Size = 3;
		}
		//get next buffer index. sent in short mode
		if (LC_SendMessage(receive_buffer[receiveFIFO_out].Node, &record, LC_SYS_Parameters) == 0)
			receive_busy = 1;
	}
}

/*
 void ParseAllParameters(void) {

 static char sbuffer[50];
 //static char right[50];
 trace_printf("# Parameter parsing...\n");
 int dir = 0;
 const char* curLine = dataparse;
 ParamDef paramParsed;
 while (curLine) {
 //skip first spaces
 curLine = skipspaces(curLine);
 if (*curLine == '[') {
 //[directory name]
 curLine++; //skip '['
 curLine = skipspaces(curLine); //skip possible spaces at begin
 //directory detected, calculate distance to end
 int endofname = strcspn(curLine, "]\n");
 //too short or large, or new line? wrong
 if (endofname < 1 || endofname > 49 || curLine[endofname] == '\n') {
 dir = -1; //if end character is endofline, file corrupted
 //dump
 endofname = endofname > 49 ? 49 : endofname;
 trace_puts("Parse error directory: \"");
 strncpy(sbuffer, curLine - 1, endofname);
 sbuffer[endofname + 1] = 0;
 trace_puts(sbuffer);
 trace_puts("\"\n");
 } else {
 //valid name parameter? Remove ending spaces and ']'
 endofname = indexspacesbk(curLine, endofname - 1);
 strncpy(sbuffer, curLine, endofname + 1); //copy name
 sbuffer[endofname + 1] = 0; //line end
 dir = isDirectory(sbuffer);
 if (dir >= 0)
 trace_printf("Parsed dir: \"%s\"\n", sbuffer);
 else {
 dir = -1;
 trace_printf("Directory not found: \"%s\"\n", sbuffer);
 }

 }
 } else if ((dir >= 0) && (*curLine != '#') && (*curLine != 0) && (*curLine != '\n')) {
 //parameter = value
 //Vasya = Pupkin #comment
 int endofname = strcspn(curLine, "#=\n");
 //too short or large, or new line? wrong
 if (endofname < 1 || endofname > 49 || curLine[endofname] == '\n') {
 //dump
 trace_puts("Parse error name: \"");
 endofname = endofname > 49 ? 49 : endofname;
 strncpy(sbuffer, curLine, endofname);
 sbuffer[endofname + 1] = 0;			//line end
 trace_puts(sbuffer);
 trace_puts("\"\n");
 } else {
 //valid name parameter? Remove ending spaces and ']'
 int endnm = indexspacesbk(curLine, endofname - 1);
 strncpy(sbuffer, curLine, endnm + 1); //copy name
 sbuffer[endnm + 1] = 0; //line end
 int parindex = isParameter(sbuffer, dir);
 if (parindex >= 0) {
 //trace
 trace_puts("Parsed parameter: ");
 trace_puts(sbuffer);
 trace_puts("=");
 //endofname is '=' here
 curLine += endofname + 1; //curLine = skipspaces(curLine + endofname + 1); //skip possible spaces at value start
 int endofval = strcspn(curLine, "#\r\n");
 endofval = indexspacesbk(curLine, endofval - 1); //remove spaces at end
 endofval = endofval > 49 ? 49 : endofval;
 strncpy(sbuffer, curLine, endofval + 1);
 sbuffer[endofval + 1] = 0; //line end
 //clean value send to function
 int r = SetParameterStr(&PD_Root[parindex], sbuffer);
 trace_puts(sbuffer);

 switch (r) {
 case 1:
 trace_puts(" - out of range \n");
 break;
 case 0:
 trace_puts(" - OK \n");
 break;
 case -1:
 trace_puts(" - unknown \n");
 break;
 }
 } else {
 trace_puts("Unknown parameter: ");
 trace_puts(sbuffer);
 }

 }
 }
 curLine = strchr(curLine, '\n');
 if (curLine)
 curLine++; //skip '\n'
 }

 vTaskDelete(0);
 while (1)
 ;
 }*/

