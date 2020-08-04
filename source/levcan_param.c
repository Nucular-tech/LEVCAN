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

#include "levcan.h"
#include "levcan_param.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

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

typedef union {
	void *Void;
	uint8_t *Uint8;
	int8_t *Int8;
	uint16_t *Uint16;
	int16_t *Int16;
	int32_t *Int32;
	float *Float;
} padress_t;

typedef struct {
	int32_t Value;
	uint8_t Directory;
	uint8_t Index;
} storeValuePacked_t;

typedef struct {
	LC_ParameterValue_t *Param;
	void *Node;
	uint16_t Directory;
	uint16_t Source;
	uint8_t Full;
} bufferedParam_t;

//### Local functions ###
void lc_proceedParam(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);
const char* extractName(const LC_ParameterDirectory_t directories[], uint16_t directory, uint16_t index);
uint16_t check_align(const LC_ParameterAdress_t *parameter);
bufferedParam_t* findReceiver(int16_t dir, int16_t index, int16_t source);
void proceed_RX(void);
const char* skipspaces(const char *s);
int32_t pow10i(int32_t dec);
int printIntegerWithDecimal(char *buffer, int32_t value, uint8_t decimals);
//### Local variables ###
bufferedParam_t receive_buffer[LEVCAN_PARAM_QUEUE_SIZE];
volatile uint16_t receiveFIFO_in = 0, receiveFIFO_out = 0;
volatile uint16_t receive_busy = 0;

const char* extractName(const LC_ParameterDirectory_t directories[], uint16_t directory, uint16_t index) {
	const char *unknown = "Unknown directory";
	const char *source = 0;

	//todo check directory index is in range
	const LC_ParameterDirectory_t *dir = &directories[directory];

	if (index < dir->Size) {
		const LC_ParameterAdress_t *param = &dir->Address[index];

		if ((param->ParamType & PT_typeMask) == PT_dirArray) {
			//extracting name from array type directory
			int32_t index = dir->Index;
			char **index_name = (char**) param->Name;
			if (index_name) {
				source = index_name[index];
			} else
				source = unknown; //directory should have a name anyway
		} else if (param->Name)
			source = param->Name;
		else if ((param->ParamType & PT_typeMask) == PT_dir) {
			//it is directory and no name, try get name from child
			if (param->Address) {
				source = extractName(directories, param->Min, 0);
			}
		}

	}
	if (source == 0)
		source = unknown; //directory should have a name anyway

	return source;
}

parameterValuePacked_t param_invalid = { .Index = 0, .Directory = 0, .ParamType = PT_invalid, .Literals = { 0, 0 } };

bufferedParam_t* findReceiver(int16_t dir, int16_t index, int16_t source) {
	if (receiveFIFO_in != receiveFIFO_out) {
		//receive_buffer[receiveFIFO_out]
		int out = receiveFIFO_out;
		receiveFIFO_out = (receiveFIFO_out + 1) % LEVCAN_PARAM_QUEUE_SIZE;

		if (receive_buffer[out].Param != 0 && receive_buffer[out].Directory == dir && receive_buffer[out].Param->Index == index
				&& receive_buffer[out].Source == source)
			return &receive_buffer[out];
	}
	return 0;
}

uint16_t check_align(const LC_ParameterAdress_t *parameter) {
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

int32_t LC_GetParameterValue(const LC_ParameterDirectory_t directories[], uint16_t directory, uint16_t index) {
	//todo check directory index is in range
	const LC_ParameterDirectory_t *dir = &directories[directory];
	int32_t value = 0;
	if (index < dir->Size) {
		const LC_ParameterAdress_t *parameter = &dir->Address[index];

		if (((uint32_t) parameter->Address > UINT8_MAX) && (check_align(parameter) == 0)) {
			padress_t adress;
			//cast type
			adress.Void = parameter->Address;
			uint16_t val_index = 0;
			//get array index if it is array
			if ((dir->Address[0].ParamType & PT_typeMask) == PT_dirArray)
				val_index = dir->Index;

			switch (parameter->ValueType) {
			default:
				value = adress.Uint8[val_index];
				break;
			case VT_int8:
				value = adress.Int8[val_index];
				break;
			case VT_uint16:
				value = adress.Uint16[val_index];
				break;
			case VT_int16:
				value = adress.Int16[val_index];
				break;
			case VT_int32:
				value = adress.Int32[val_index];
				break;
#ifdef LEVCAN_USE_FLOAT
			case VT_float:
				value = adress.Float[val_index] * pow10i(parameter->Decimal);
				break;
#endif
			}
		}
	}
	return value;
}

int LC_SetParameterValue(const LC_ParameterDirectory_t directories[], uint16_t directory, uint16_t index, int32_t value) {
	const LC_ParameterDirectory_t *dir = &directories[directory];
	if (index < dir->Size) {
		const LC_ParameterAdress_t *parameter = &dir->Address[index];

		if (value > parameter->Max || value < parameter->Min || (parameter->ParamType & PT_readonly) || parameter->Address == 0)
			return 1;
		if (((uint32_t) parameter->Address > UINT8_MAX) && (check_align(parameter) == 0)) {
			padress_t adress;
			//cast type
			adress.Void = parameter->Address;
			uint16_t val_index = 0;
			//get array index if it is array
			if ((dir->Address[0].ParamType & PT_typeMask) == PT_dirArray)
				val_index = dir->Index;

			switch (parameter->ValueType) {
			case VT_int8:
			case VT_uint8:
				adress.Uint8[val_index] = (uint8_t) value;
				break;
			case VT_int16:
			case VT_uint16:
				adress.Uint16[val_index] = (uint16_t) value;
				break;
			case VT_int32:
				adress.Int32[val_index] = value;
				break;
#ifdef LEVCAN_USE_FLOAT
			case VT_float:
				adress.Float[val_index] = (float) value / pow10i(parameter->Decimal);
				break;
#endif
			default:
				break;
			}
		}
	}
	return 0;
}

void lc_proceedParam(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
#ifdef LEVCAN_MEM_STATIC
	static char static_buffer[sizeof(parameterValuePacked_t) + 128] = { 0 };
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
		LC_ParameterDirectory_t *directories = (LC_ParameterDirectory_t*) node->Directories;
		LC_ParameterDirectory_t *directory = &(directories[pddir]);
		//array correctly filled?
		if (pddir < node->DirectoriesSize && pdindex < directory->Size) {
			const LC_ParameterAdress_t *parameter = &directory->Address[pdindex];
			int32_t namelength = 0, formatlength = 0;
			//check strings
			const char *s_name = extractName(directories, pddir, pdindex);
			if (s_name)
				namelength = strlen(s_name);
			if (parameter->Formatting)
				formatlength = strlen(parameter->Formatting);
			//now allocate full size
#ifdef LEVCAN_MEM_STATIC
			parameterValuePacked_t *param_to_send = (void*) &static_buffer;
			if (namelength + formatlength + 2 > 128) {
				formatlength = 0;
				if (namelength + 2 > 128)
					namelength = 128 - 2;
			}
#endif
			int32_t totalsize = sizeof(parameterValuePacked_t) + namelength + formatlength + 2;
#ifndef LEVCAN_MEM_STATIC
			parameterValuePacked_t *param_to_send = lcmalloc(totalsize);
#endif
			if (param_to_send == 0)
				return; // nothing to do so here

			//just copy-paste
			value = LC_GetParameterValue(directories, pddir, pdindex);
			param_to_send->Value = value;
			param_to_send->Min = parameter->Min;
			if (pdindex == 0)
				param_to_send->Max = directory->Size; //directory size
			else
				param_to_send->Max = parameter->Max;
			param_to_send->Decimal = parameter->Decimal;
			param_to_send->Step = parameter->Step;
			if (parameter->ParamType == PT_dirArray) //backward compatibility
				param_to_send->ParamType = PT_dir;
			else
				param_to_send->ParamType = parameter->ParamType;
			param_to_send->Index = pdindex;
			param_to_send->Directory = pddir;

			param_to_send->Literals[0] = 0;
			if (namelength)
				strncat(param_to_send->Literals, s_name, namelength);

			param_to_send->Literals[namelength + 1] = 0; //next string start
			if (formatlength)
				strncat(&param_to_send->Literals[namelength + 1], parameter->Formatting, formatlength);

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
			sendvalue.Value = LC_GetParameterValue(((LC_ParameterDirectory_t*) node->Directories), pddir, pdindex);

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
		storeValuePacked_t *store = data;
		if (store->Directory < node->DirectoriesSize && store->Index < ((LC_ParameterDirectory_t*) node->Directories)[store->Directory].Size)
			LC_SetParameterValue(((LC_ParameterDirectory_t*) node->Directories), store->Directory, store->Index, store->Value);
	}
		break;
	case sizeof(storeValuePacked_t) + 1: {
		//update requested value
		storeValuePacked_t *update = data;
		bufferedParam_t *receiver = findReceiver(update->Directory, update->Index, header.Source);
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
		if (size > (int32_t) sizeof(parameterValuePacked_t)) {
			//parameter full receive
			parameterValuePacked_t *param_received = data;
			bufferedParam_t *receiver = findReceiver(param_received->Directory, param_received->Index, header.Source);
			int sizefail = 0;
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
				//extract name
				char *clean = receiver->Param->Name;
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
					int length = strnlen(&param_received->Literals[strpos], 512);
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
#else
				(void) sizefail; //no warnings
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
LC_ParameterTableSize_t LC_ParamInfo_Size(void *vnode) {
	LC_NodeDescriptor_t *node = vnode;
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

/// Sends new parameter value to receiver
/// @param paramv Pointer to parameter
/// @param dir Directory index
/// @param sender_node Sender node
/// @param receiver_node Receiver ID node
LC_Return_t LC_ParameterSet(LC_ParameterValue_t *paramv, uint16_t dir, void *sender_node, uint16_t receiver_node) {
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
LC_Return_t LC_ParameterUpdateAsync(LC_ParameterValue_t *paramv, uint16_t dir, void *sender_node, uint16_t receiver_node, uint8_t full) {
	//todo reentrancy
	if (receiveFIFO_in == ((receiveFIFO_out - 1 + LEVCAN_PARAM_QUEUE_SIZE) % LEVCAN_PARAM_QUEUE_SIZE))
		return LC_BufferFull;

	bufferedParam_t *receive = &receive_buffer[receiveFIFO_in];
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

const LC_ParameterAdress_t* LC_GetParameterAdress(const LC_NodeDescriptor_t *node, int16_t dir, int16_t index) {
	//array correctly filled?
	if (index >= 0 && dir >= 0 && dir < node->DirectoriesSize) {
		LC_ParameterDirectory_t *directory = &(((LC_ParameterDirectory_t*) node->Directories)[dir]);
		if (index < directory->Size) {
			return &directory->Address[index];
		}
	}
	return 0;
}

#ifdef LEVCAN_PARAMETERS_PARSING

const char *equality = " = ";

void LC_PrintParam(char *buffer, const LC_ParameterDirectory_t directories[], uint16_t directory, uint16_t index) {
	if (buffer == 0)
		return;
	const LC_ParameterDirectory_t *dir = &directories[directory];
	if (index >= dir->Size)
		return;
	const LC_ParameterAdress_t *parameter = &dir->Address[index];
	if (parameter == NULL)
		return;
	//for string this enough
	buffer[0] = 0;
	switch (parameter->ParamType & PT_typeMask) {
	case PT_dirArray:
	case PT_dir: {
		strcat(buffer, "\n[");
		strcat(buffer, extractName(directories, directory, index));
		strcat(buffer, "]\n");
	}
		break;
	case PT_enum: {
		strcpy(buffer, parameter->Name);
		strcat(buffer, equality);

		int32_t val = LC_GetParameterValue(directories, directory, index);

		const char *position = parameter->Formatting;
		for (int i = 0; (i < val); i++) {
			position = strchr(position, '\n'); //look for buffer specified by val index
			if (position == 0)
				break;
			position++; //skip '\n'
		}
		if (position == 0) {
			sprintf(buffer + strlen(buffer), "%ld", val);
		} else {
			char *end = strchr(position, '\n'); //search for buffer end
			if (end) {
				strncat(buffer, position, end - position);
			} else
				strcat(buffer, position);	//end is possible zero
		}
		strcat(buffer, "\n");
	}
		break;

	case PT_value: {
		int32_t val = LC_GetParameterValue(directories, directory, index);
		strcpy(buffer, parameter->Name);
		strcat(buffer, equality);
		printIntegerWithDecimal(buffer + strlen(buffer), val, parameter->Decimal);
		strcat(buffer, "\n");
	}
		break;

	case PT_bool: {
		int32_t val = LC_GetParameterValue(directories, directory, index);
		strcpy(buffer, parameter->Name);
		strcat(buffer, equality);
		if (val)
			strcat(buffer, "ON\n");
		else
			strcat(buffer, "OFF\n");
	}
		break;
	default:
		break;
	}

	return;
}

int printIntegerWithDecimal(char *buffer, int32_t value, uint8_t decimals) {
	char buf[16];
	int32_t powww = pow10i(decimals);
	int32_t integer = value / powww;
	int32_t decimal = value % powww;
	int isize = 0;

	if (decimals == 0) {
		isize = sprintf(buf, "%ld", integer);
	} else {
		isize = sprintf(buf, "%ld.", integer);
		int dsize = sprintf(buf + isize, "%ld", decimal);
		//add extra 0's
		if (dsize < decimals) {
			for (int i = 0; i <= decimals; i++) {
				if (i > dsize) {
					//add 0's after point
					buf[isize + decimals - i] = '0';
				} else {
					//move decimal part in buffer including zero
					buf[isize + decimals - i] = buf[isize + dsize - i];
				}
			}
			//buf[isize + decimals] = 0;
		}
	}
	//total size
	isize = isize + decimals;
	memcpy(buffer, buf, isize + 1); //+null
	return isize;
}

/// Tryes to get value for specified parameter with string value
/// @param parameter
/// @param value output
/// @return 0 if ok, 1 if error
int LC_GetParameterValueFromStr(const LC_ParameterAdress_t *parameter, const char *s, int32_t *value) {
	int32_t integer = 0;
	int type = parameter->ParamType & PT_typeMask;
	s = skipspaces(s);
	int length = strcspn(s, "#\n\r");
	if (type == PT_enum || type == PT_bool) {
		const char *haystack = parameter->Formatting;
		if (parameter->ParamType == PT_bool)
			haystack = "OFF\nON";
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
			*value = integer;
			return 0;
		}
	}
	if (type != PT_string && type != PT_func && type != PT_dir) {
		//other types failed? try integer type, anywhere supported
		integer = strtol(s, 0, 0);
		if (integer == INT32_MAX)
			return 1;
		char *decStr = memchr(s, '.', length);
		if (decStr != NULL) {
#ifdef LEVCAN_USE_FLOAT
			float temp = atoff(s);
			integer = temp * pow10i(parameter->Decimal);
#endif
		} else
			integer *= pow10i(parameter->Decimal);
		*value = integer;
		return 0;

	}
	return 1;
}

int16_t LC_IsDirectory(LC_NodeDescriptor_t *node, const char *s) {
	//index [0] is directory entry, dont scan
	int searchlen = strcspn(s, "]#=\n\r");
	//remove space ending
	for (; searchlen > 0 && isblank(s[searchlen - 1]); searchlen--)
		;
	for (uint16_t i = 0; i < node->DirectoriesSize; i++) {
		const char *name = extractName((LC_ParameterDirectory_t*) node->Directories, i, 0);
		if (name && strncmp(name, s, searchlen) == 0) {
			return i;
		}
	}
	return -1;
}

int16_t LC_IsParameter(LC_NodeDescriptor_t *node, const char *s, uint8_t directory) {
	//index [0] is directory entry, dont scan
	int searchlen = strcspn(s, "#=\n\r");
	//remove space ending
	for (; searchlen > 0 && isblank(s[searchlen - 1]); searchlen--)
		;
	for (uint16_t i = 1; i < ((LC_ParameterDirectory_t*) node->Directories)[directory].Size; i++) {
		//todo carefully check types
		const char *name = extractName((LC_ParameterDirectory_t*) node->Directories, directory, i);
		//other directories entry don't have name in the pointer, so they will be skipped
		if (name && (strncmp(name, s, searchlen) == 0)) {
			return i;
		}
	}
	return -1;
}

#ifdef LEVCAN_TRACE
void LC_ParametersPrintAll(void* vnode) {
	LC_NodeDescriptor_t* node = vnode;
	char buffer[128];
	trace_printf("# %s\n", node->DeviceName);
	trace_printf("# Network ID: %d\n", node->ShortName.NodeID);

	for (int dir = 0; dir < node->DirectoriesSize; dir++) {
		for (int i = 0; i < ((LC_ParameterDirectory_t*) node->Directories)[dir].Size; i++) {
			buffer[0] = 0;
			const LC_ParameterAdress_t* param = &((LC_ParameterDirectory_t*) node->Directories)[dir].Address[i];
			if ((param->ParamType & PT_typeMask) != PT_dir || i == 0) {
				LC_PrintParam(buffer, param);
				trace_printf(buffer);
			}
		}
	}
}
#endif

/// Parses string line and returns it's directory or index and value
/// @param node
/// @param line
/// @param directory
/// @param index
/// @return
const char* LC_ParseParameterLine(LC_NodeDescriptor_t *node, const char *input, int16_t *directory, int16_t *index, int32_t *value) {
	if (index == 0 || directory == 0 || value == 0 || input == 0 || node == 0)
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
			dir = LC_IsDirectory(node, line);
			//too short or large, or new line? wrong
			if (dir >= 0) {
				indx = -1;
				//trace_printf("Parsed dir: \"%s\"\n", sbuffer);
			} else {
				//trace_printf("Directory not found: \"%s\"\n", sbuffer);
			}

		} else if ((dir >= 0) && (*line != '#') && (*line != 0) && (*line != '\n')) {
			//parameter = value
			//Vasya = Pupkin #comment
			int endofname = strcspn(line, "#=\n\r");
			indx = LC_IsParameter(node, line, dir);
			//too short or large, or new line? wrong
			if (indx > 0 && line[endofname] == '=') {
				//endofname is '=' here
				line += endofname + 1;
				const LC_ParameterAdress_t *addr = LC_GetParameterAdress(node, dir, indx);
				if (addr && LC_GetParameterValueFromStr(addr, line, value)) {
					indx = -1;
				}
			}
		}
	} else
		return 0;
	*index = indx;
	*directory = dir;
//go to new line
	return line + strcspn(line, "\n\r");
}

#endif

const char* skipspaces(const char *s) {
	for (; isspace(*s); s++)
		;
	return s;
}

int32_t pow10i(int32_t dec) {
	int32_t ret = 1;
	for (; dec > 0; dec--)
		ret *= 10;
	return ret;
}

