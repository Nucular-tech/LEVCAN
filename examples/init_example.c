/*
 * init_example.c
 *
 *  Created on: 22 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "levcan.h"
#include "can_hal.h"

struct {
	uint32_t Data1;
	uint16_t Data2;
	uint8_t Data3;
	uint32_t Data4;
	char String[128];
} UserVariables;

struct {
	uint32_t Parameter1;
	float Parameter2;
	struct {
		enum {
			ThrottleOff, ThrottleActive, ThrottlePassive
		} EnumParam;
		uint16_t Min;
		uint16_t Max;
	} Throttle;
} UserConfig;

//#### object dictionary. used for transfer between nodes ####
const LC_Object_t node_obj[] = { //
		{ 0, { .TCP = 0, .Readable = 1, .Writable = 1 }, sizeof(UserVariables.Data1), (intptr_t*) &UserVariables.Data1 }, //
		{ 1, { .TCP = 0, .Readable = 1, .Writable = 1 }, sizeof(UserVariables.Data1), (intptr_t*) &UserVariables.Data1 }, //
		{ 2, { .TCP = 0, .Readable = 1, .Writable = 1 }, sizeof(UserVariables.Data1), (intptr_t*) &UserVariables.Data1 }, //
		{ 3, { .TCP = 0, .Readable = 1, .Writable = 1 }, sizeof(UserVariables.Data1), (intptr_t*) &UserVariables.Data1 }, //
															//any size up to
		{ LC_SYS_DeviceName, { .TCP = 1, .Writable = 1 }, -sizeof(UserVariables.String), (intptr_t*) &UserVariables.String }, //
		};
const uint16_t node_obj_size = sizeof(node_obj) / sizeof(node_obj[0]);

//#### parametric configuration, for GUI ####
extern const LC_ParameterAdress_t PD_root[], PD_Dir1[];
const LC_ParameterAdress_t PD_root[]
//		DIRECTORY_ARRAY					DIR_INDEX	ENTRY_INDEX	NU		NU		NU			RdTYPE=dir	DIR_NAME	NU
= {{ (void*) &PD_root, 							0, 		0, 		0,		0,		0,			RT_dir,		"Some awesome device", 0 }, //Directory entry
   { (void*) &PD_Dir1, 							1, 		0, 		0,		0,		0,			RT_dir,		0, 0 }, //entrance to other directory
//		ADRESS									MIN		MAX		STEP	DECIMAL	TYPE		RdTYPE		NAME		FORMATTING
   { pti(UserConfig.Parameter1, 				0, 		0, 		0,		0),		RT_value | RT_readonly,	"Shutdown time", "%s sec"  }, //
   { pti(UserConfig.Parameter2 , 				6000, 	100000,	100,	3),					RT_value,	"Maximum voltage", "%sV" }, //
};

const LC_ParameterAdress_t PD_Dir1[]
//[0]=PARENT_ADDRESS				PARENT_INDEX	ENTRY_INDEX	NU		NU		NU			RdTYPE=dir	DIR_NAME	NU
= {	{ (void*) &PD_root, 						0, 		0, 		0,		0, 		0,			RT_dir,		"Directory One",		0 }, //Directory entry
//		ADRESS									MIN		MAX		STEP	DECIMAL	TYPE		RdTYPE		NAME		FORMATTING
	{ pti(UserConfig.Throttle.EnumParam, 		0, 		1, 		1,		0),					RT_enum,	"Mode",	"Off\nActive\nPassive" }, //
	{ pti(UserConfig.Throttle.Min, 				0, 		5000, 	10,		3),					RT_value,	"Min mV",	0}, //
	{ pti(UserConfig.Throttle.Max, 				0, 		5000, 	10,		3),					RT_value,	"Max mV",	0 }, //
};
//all directories stored here
const LC_ParameterDirectory_t paramDirectories[] = { DIRDEF(PD_root), DIRDEF(PD_Dir1) };
//and it's size
const uint32_t paramDirectoriesSize = sizeof(paramDirectories) / sizeof(paramDirectories[0]);

LC_NodeDescription_t* mynode;

void nwrk_manager(void);
void Init_LEVCAN(void) {
	
	CAN_InitFromClock(RCC_APB1_CLK, 1000, 2, 87);
	
	LC_NodeInit_t node_init;

	node_init.DeviceName = "Some Awesome Device";
	node_init.NodeName = "NodeName";
	node_init.VendorName = "CompanyName LLC.";

	node_init.VendorCode = 0xABCD;
	node_init.NodeID = 31; //default used ID if free
	node_init.Serial = 0xDEADBEEF; //use your CPU SN

	node_init.Objects = node_obj;
	node_init.ObjectsSize = node_obj_size;

	node_init.Configurable = 1; // we have configurable variables
	node_init.Directories = paramDirectories;
	node_init.DirectoriesSize = paramDirectoriesSize;
	mynode = LC_CreateNode(node_init);

	xTaskCreate(nwrk_manager, "LC", configMINIMAL_STACK_SIZE, NULL, OS_PRIORITY_LOW, (TaskHandle_t *) NULL);
	//request all device names in network
	//will be stored in UserVariables.String
	LC_SendRequest(0, LC_Broadcast_Address, LC_SYS_DeviceName);
}

void nwrk_manager(void) {
	while (1) {

		uint32_t tick = xTaskGetTickCount();
		LC_NetworkManager(tick);

		static uint32_t prev_upd_time_sp = 0;
		vTaskDelayUntil(&prev_upd_time_sp, configTICK_RATE_HZ / 100); //100hz CAN
	}
}

