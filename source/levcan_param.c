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
	LC_ReadType_t ReadType;
	char Literals[];
} parameterValuePacked_t;

typedef struct {
	int32_t Value;
	uint8_t Directory;
	uint8_t Index;
} storeValuePacked_t;

typedef struct {
	LC_ParameterValue_t* Param;
	uint16_t Directory;
	uint16_t Source;
} bufferedParam_t;

LC_ObjectRecord_t proceedParam(LC_NodeDescription_t* node, LC_Header header, void* data, int32_t size);
uint16_t check_align(const LC_ParameterAdress_t* parameter);
char* extractName(const LC_ParameterAdress_t* param);
bufferedParam_t* findReceiver(int16_t dir, int16_t index, int16_t source);
int isDirectory(LC_NodeDescription_t* node, const char* s);
int isParameter(LC_NodeDescription_t* node, const char* s, uint8_t directory);
int32_t getParameterValue(const LC_ParameterAdress_t* parameter);
int setParameterValue(const LC_ParameterAdress_t* parameter, int32_t value);
char* printParam(const LC_ParameterAdress_t* parameter);
bufferedParam_t* findFreeRx(void);

bufferedParam_t receive_buffer[LEVCAN_PARAM_QUEUE_SIZE];
volatile int16_t receive_free;

char* extractName(const LC_ParameterAdress_t* param) {
	char* source = 0;
	//try to get name
	if (param->Name)
		source = param->Name;
	else if ((param->ReadType & ~RT_readonly) == RT_dir) {
		//it is directory and no name, try get name from child
		if (param->Address && ((LC_ParameterAdress_t*) param->Address)[0].Name) {
			//this is directory, it's name can stored in child [0] directory index
			source = ((LC_ParameterAdress_t*) param->Address)[0].Name;
		} else
			source = "Unknown directory"; //directory should have a name anyway
	}
	return source;
}

parameterValuePacked_t param_invalid = { .Index = 0, .Directory = 0, .ReadType = RT_invalid, .Literals = { 0, 0 } };

bufferedParam_t* findReceiver(int16_t dir, int16_t index, int16_t source) {
	for (int i = 0; i < LEVCAN_PARAM_QUEUE_SIZE; i++) {
		if (receive_buffer[i].Param != 0 && receive_buffer[i].Directory == dir && receive_buffer[i].Param->Index == index && receive_buffer[i].Source == source)
			return &receive_buffer[i];
	}
	return 0;
}

uint16_t check_align(const LC_ParameterAdress_t* parameter) {
//parameter should be aligned to avoid memory fault, this may happen when int16 accessed as int32
	uint16_t align = 0;
	switch (parameter->Type) {
	case PT_int16:
	case PT_uint16:
		align = (int32_t) parameter->Address % 2;
		break;
	case PT_int32:
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
	switch (parameter->ReadType) {
	case RT_invalid:
		return 0;

	case RT_dir: {
		strcat(line, "\n[");
		strcat(line, parameter->Name);
		strcat(line, "]\n");
	}
		break;
	case RT_enum: {
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
	case RT_value | RT_readonly:
	case RT_value: {
		int32_t val = getParameterValue(parameter);
		strcpy(line, parameter->Name);
		strcat(line, equality);
		strcat(line, sf32toa(val, parameter->Decimal, 0));
		strcat(line, "\n");
	}
		break;

	case RT_bool | RT_readonly:
	case RT_bool: {
		int32_t val = getParameterValue(parameter);
		strcpy(line, parameter->Name);
		strcat(line, equality);
		if (val)
			strcat(line, "ON\n");
		else
			strcat(line, "OFF\n");
	}
		break;
	}

	return line;
}

int32_t getParameterValue(const LC_ParameterAdress_t* parameter) {
	int32_t value = 0;
	if (((uint32_t) parameter->Address > UINT8_MAX) && (check_align(parameter) == 0)) {
		switch (parameter->Type) {
		default:
			value = (*(uint8_t*) (parameter->Address));
			break;
		case PT_int8:
			value = (*(int8_t*) parameter->Address);
			break;
		case PT_int16:
			value = (*(int16_t*) parameter->Address);
			break;
		case PT_uint16:
			value = (*(uint16_t*) parameter->Address);
			break;
		case PT_int32:
			value = (*(int32_t*) parameter->Address);
			break;
		case PT_float:
			value = (*(float*) parameter->Address) * powf(10, parameter->Decimal);
			break;
		}
	}
	return value;
}

int setParameterValue(const LC_ParameterAdress_t* parameter, int32_t value) {
	if (value > parameter->Max || value < parameter->Min)
		return 1;
	if (((uint32_t) parameter->Address > UINT8_MAX) && (check_align(parameter) == 0))
		switch (parameter->Type) {
		case PT_int8:
		case PT_uint8:
			(*(uint8_t*) parameter->Address) = (uint8_t) value;
			break;
		case PT_int16:
		case PT_uint16:
			(*(uint16_t*) parameter->Address) = (uint16_t) value;
			break;
		case PT_int32:
			(*(uint32_t*) parameter->Address) = (uint32_t) value;
			break;
		case PT_float:
			(*(float*) parameter->Address) = (float) value / powf(10, parameter->Decimal);
			break;
		default:
			break;
		}
	return 0;
}

int setParameterStr(const LC_ParameterAdress_t* parameter, char* value) {
	int ret = -1;
	if (parameter->ReadType == RT_enum || parameter->ReadType == RT_bool) {
		value = skipspaces(value);
		char* haystack = parameter->Formatting;
		if (parameter->ReadType == RT_bool)
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
		if (node->Directories[i].Address[0].Name && strcmp(node->Directories[i].Address[0].Name, s) == 0) {
			return i;
		}
	}
	return -1;
}

int isParameter(LC_NodeDescription_t* node, const char* s, uint8_t directory) {
//index [0] is directory entry, dont scan
	for (uint16_t i = 1; i < node->Directories[directory].Size; i++) {
		//todo carefully check types
		if ((node->Directories[directory].Address[i].ReadType < RT_dir) && node->Directories[directory].Address[i].Name
				&& (strcmp(node->Directories[directory].Address[i].Name, s) == 0)) {
			return i;
		}
	}
	return -1;
}

bufferedParam_t* findFreeRx(void) {
	int position = receive_free;
	for (int i = 0; i < LEVCAN_PARAM_QUEUE_SIZE; i++) {
		if (receive_buffer[(i + position) % LEVCAN_PARAM_QUEUE_SIZE].Param == 0) {
			receive_free = (i + position + 1) % LEVCAN_PARAM_QUEUE_SIZE;
			return &receive_buffer[(i + position) % LEVCAN_PARAM_QUEUE_SIZE];
		}
	}
	return 0;
}
const LC_ObjectRecord_t nullrec;
//LC_ObjectRecord_t (*LC_FunctionCall_t)(LC_NodeDescription_t* node, LC_Header header, void* data, int32_t size);
LC_ObjectRecord_t proceedParam(LC_NodeDescription_t* node, LC_Header header, void* data, int32_t size) {

	LC_ObjectRecord_t txrec;
	txrec.Attributes.Priority = LC_Priority_Low;
	txrec.Attributes.TCP = 1;
	if (data == 0)
		return nullrec; // nothing to do so here
	//TODO add node filter
	switch (size) {
	case 2: {
		//2 byte - request (index and directory)
		uint16_t pdindex = ((uint8_t*) data)[0];
		uint16_t pddir = ((uint8_t*) data)[1];
		//trace_printf("Request: id=%d, dir=%d\n", pdindex,pddir);
		int32_t value = 0;
		//array correctly filled?
		if (pddir < node->DirectoriesSize && pdindex < node->Directories[pddir].Size) {
			int32_t namelength = 0, formatlength = 0;
			//check strings
			if (extractName(&node->Directories[pddir].Address[pdindex]))
				namelength = strlen(extractName(&node->Directories[pddir].Address[pdindex]));
			if (node->Directories[pddir].Address[pdindex].Formatting)
				formatlength = strlen(node->Directories[pddir].Address[pdindex].Formatting);
			//now allocate full size
			int32_t totalsize = sizeof(parameterValuePacked_t) + namelength + formatlength + 2;
			parameterValuePacked_t* param_to_send = lcmalloc(totalsize);

			if (param_to_send == 0)
				return nullrec; // nothing to do so here

			//just copy-paste
			value = getParameterValue(&node->Directories[pddir].Address[pdindex]);
			param_to_send->Value = value;
			param_to_send->Min = node->Directories[pddir].Address[pdindex].Min;
			param_to_send->Max = node->Directories[pddir].Address[pdindex].Max;
			param_to_send->Decimal = node->Directories[pddir].Address[pdindex].Decimal;
			param_to_send->Step = node->Directories[pddir].Address[pdindex].Step;
			param_to_send->ReadType = node->Directories[pddir].Address[pdindex].ReadType;
			param_to_send->Index = pdindex;
			param_to_send->Directory = pddir;

			param_to_send->Literals[0] = 0;
			if (namelength)
				strcat(param_to_send->Literals, extractName(&node->Directories[pddir].Address[pdindex]));

			param_to_send->Literals[namelength + 1] = 0; //next string start
			if (formatlength)
				strcat(&param_to_send->Literals[namelength + 1], node->Directories[pddir].Address[pdindex].Formatting);

			txrec.Address = param_to_send;
			txrec.Size = totalsize;
			txrec.Attributes.Cleanup = 1;

			LC_SendMessage(node, &txrec, header.Source, LC_SYS_Parameters);
		} else {
			//non existing item
			param_invalid.Index = pdindex;
			param_invalid.Directory = pddir;
			txrec.Address = &param_invalid;
			txrec.Size = sizeof(parameterValuePacked_t) + 2;

			LC_SendMessage(node, &txrec, header.Source, LC_SYS_Parameters);
			//trace_printf("Info ERR sent i:%d, d:%d\n", pdindex, pddir);
		}
	}
		break;
	case 3: {
		//request data value
		uint16_t pdindex = ((uint8_t*) data)[0];
		uint16_t pddir = ((uint8_t*) data)[1];
		//array correctly filled?
		if ((pddir < node->DirectoriesSize && pdindex < node->Directories[pddir].Size) && (node->Directories[pddir].Address[pdindex].ReadType & ~RT_readonly) != RT_dir
				&& (node->Directories[pddir].Address[pdindex].ReadType & ~RT_readonly) != RT_func) {
			storeValuePacked_t sendvalue;
			sendvalue.Index = pdindex;
			sendvalue.Directory = pddir;
			sendvalue.Value = getParameterValue(&node->Directories[pddir].Address[pdindex]);

			txrec.Address = &sendvalue;
			txrec.Size = sizeof(storeValuePacked_t) + 1;
			//send back data value
			LC_SendMessage(node, &txrec, header.Source, LC_SYS_Parameters);
		}
	}
		break;
	case sizeof(storeValuePacked_t): {
		//store value
		storeValuePacked_t* store = data;
		if (store->Directory < node->DirectoriesSize && store->Index < node->Directories[store->Directory].Size)
			setParameterValue(&node->Directories[store->Directory].Address[store->Index], store->Value);
	}
		break;
	case sizeof(storeValuePacked_t) + 1: {
		//update requested value
		storeValuePacked_t* update = data;
		bufferedParam_t* receiver = findReceiver(update->Directory, update->Index, header.Source);
		//somebody receiving
		if (receiver) {
			receiver->Param->Value = update->Value;
			receiver->Param = 0;
		}
	}
		break;
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
				//receiver->Param->Index=param_received->Index; //should be equal
				//extract name
				int strpos = 0;
				if (param_received->Literals[0] != 0) {
					int length = strlen(param_received->Literals);
					receiver->Param->Name = lcmalloc(length + 1);
					if (receiver->Param->Name)
						strcpy(receiver->Param->Name, &param_received->Literals[strpos]);
					strpos = length;
				} else
					receiver->Param->Name = 0;
				strpos++;				//skip one terminating character
				//extract formatting
				if (param_received->Literals[strpos] != 0) {
					int length = strlen(&param_received->Literals[strpos]);
					receiver->Param->Formatting = lcmalloc(length + 1);
					if (receiver->Param->Formatting)
						strcpy(receiver->Param->Formatting, &param_received->Literals[strpos]);
					strpos = length + 1;
				} else
					receiver->Param->Formatting = 0;
				//delete receiver
				receiver->Param = 0;
			}
		}
	}
		break;
	}
	return nullrec; // nothing to do so here
}

void LC_ParamInfo_Size(void* vnode) {
	LC_NodeDescription_t* node = vnode;
	int32_t size = 0;
	int32_t textsize = 0;
	int32_t parameters = 0;
	for (int i = 0; i < node->DirectoriesSize; i++) {
		size += node->Directories[i].Size * sizeof(LC_ParameterAdress_t);
		for (int b = 0; b < node->Directories[i].Size; b++) {
			parameters++;
			if (node->Directories[i].Address[b].Name)
				textsize += strlen(node->Directories[i].Address[b].Name) + 1;
			if (node->Directories[i].Address[b].Formatting)
				textsize += strlen(node->Directories[i].Address[b].Formatting) + 1;
		}
	}
#ifdef LEVCAN_TRACE
	trace_printf("Parameters: %d, size bytes: %d, text: %d, total: %d\n", parameters, size, textsize, size + textsize);
#endif
}

void LC_ParametersPrintAll(void* vnode) {
	LC_NodeDescription_t* node = vnode;
#ifdef LEVCAN_TRACE
	trace_printf("# %s\n", node->DeviceName);
	trace_printf("# Network ID: %d\n", node->ShortName.NodeID);
	trace_printf("# %s system config:\n", node->NodeName);
#endif
	for (int dir = 0; dir < node->DirectoriesSize; dir++) {
		for (int i = 1; i < node->Directories[dir].Size; i++) {
#ifdef LEVCAN_TRACE
			trace_printf(printParam(&node->Directories[dir].Address[i]));
#endif
		}
	}
}

void LC_ParameterSet(LC_ParameterValue_t* paramv, uint16_t dir, void* sender_node, uint16_t receiver_node) {
//send function will use fast send, so it is  safe
	storeValuePacked_t store;
	store.Directory = dir;
	store.Index = paramv->Index;
	store.Value = paramv->Value;

	LC_ObjectRecord_t record;
	record.Address = &store;
	record.Attributes.TCP = 1;
	record.Attributes.Priority = LC_Priority_Low;
	record.Size = sizeof(storeValuePacked_t);
	LC_SendMessage(sender_node, &record, receiver_node, LC_SYS_Parameters);
}

void LC_ParameterUpdateAsync(LC_ParameterValue_t* paramv, uint16_t dir, void* sender_node, uint16_t receiver_node, int full) {
	LC_NodeDescription_t* node = sender_node;

	if (node == 0 || node->State != LCNodeState_Online)
		return;
	bufferedParam_t* receive = findFreeRx();
	if (receive == 0)
		return;
	receive->Param = paramv;
	receive->Directory = dir;
	receive->Source = receiver_node;

	uint8_t data[3];
	data[0] = paramv->Index;
	data[1] = dir;
	data[2] = 0;

	LC_ObjectRecord_t record;
	record.Address = &data;
	record.Attributes.TCP = 1;
	record.Attributes.Priority = LC_Priority_Low;
	if (full)
		record.Size = 2;
	else
		record.Size = 3;
	LC_SendMessage(sender_node, &record, receiver_node, LC_SYS_Parameters);
}

void LC_ParametersStopUpdating(void) {
	//clean up all tx buffers,
	for (int i = 0; i < LEVCAN_PARAM_QUEUE_SIZE; i++) {
		receive_buffer[i].Param = 0;
		receive_buffer[i].Directory = 0;
		receive_buffer[i].Source = 0;
	}
	receive_free = 0;
}

/*
 void ParseAllParameters(void) {

 static char sbuffer[50];
 //static char right[50];
 trace_printf("# ABController parameter parsing...\n");
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

