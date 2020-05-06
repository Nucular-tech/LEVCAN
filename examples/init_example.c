/*
 * init_example.c
 *
 *  Created on: 22 march 2018
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
// include main levcan.h and hal
#include "levcan.h"
#include "can_hal.h"
// include device configuration
#include "levcan_param.h"
// include standard levcan objects
#include "levcan_objects.h"

//Example for CAN object setup
struct {
	uint32_t Data1;
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

LC_Obj_ThrottleV_t throttle_input = { 0 };
LC_Obj_BrakeV_t brake_input = { 0 };
LC_Obj_ActiveFunctions_t activefunc_out = { 0 };

//#### Object dictionary. used for transfer between nodes ####
// You can receive data (.Writable) to your variables using levcan objects or send data on request (.Readable)
// @formatter:off
const LC_Object_t node_obj[] = { //
		//receive throttle and brake voltage from any node on bus
		{ LC_Obj_ThrottleV, { .TCP = 0, .Writable = 1 }, sizeof(throttle_input), (intptr_t*) &throttle_input }, //
		{ LC_Obj_BrakeV, 	{ .TCP = 0, .Writable = 1 }, sizeof(brake_input), 	(intptr_t*) &brake_input }, //
		{ 1, { .TCP = 0, .Readable = 1, .Writable = 1 }, sizeof(UserVariables.Data1), (intptr_t*) &UserVariables.Data1 }, //
														//any size up to n
		{ LC_SYS_DeviceName, { .TCP = 1, .Writable = 1 }, -sizeof(UserVariables.String), (intptr_t*) &UserVariables.String }, //
};
// @formatter:on
const uint16_t node_obj_size = sizeof(node_obj) / sizeof(node_obj[0]);

//#### Parameters configuration, for GUI access ####
// Parameters used to configure your device through display.
// Describe variables, set limits, name and formatting
// declare variables to use them before their body
extern const LC_ParameterAdress_t PD_root[], PD_Dir1[];
// root entry
// @formatter:off
const LC_ParameterAdress_t PD_root[]
//		DIRECTORY_ARRAY					DIR_INDEX	ENTRY_INDEX	NU		NU		NU			RdTYPE=dir	DIR_NAME				NU(notUsed)
= { 	{ (void*) &PD_root, 				0, 			0, 		0, 		0, 		0, 			PT_dir, 	"Some awesome device", 	0 }, //Directory entry
		{ (void*) &PD_Dir1, 				1, 			0, 		0,		0, 		0, 			PT_dir, 	0, 						0 }, //entrance to other directory, no need to define name here
//pti macro used for picking data type, supported standard types only
//		ADRESS									MIN		MAX		STEP	DECIMAL		RdTYPE					NAME			FORMATTING
		{ pti(UserConfig.Parameter1, 			0, 		0, 		0, 		0),		 	PT_value | PT_readonly, "Shutdown time", "%s sec" }, //
		{ pti(UserConfig.Parameter2, 			6000, 	100000, 100, 	3), 		PT_value, 				"Maximum voltage", "%sV" }, //
};

const LC_ParameterAdress_t PD_Dir1[]
//[0]=PARENT_ADDRESS				PARENT_INDEX	ENTRY_INDEX	NU		NU		NU			RdTYPE=dir	DIR_NAME		NU
= { { (void*) &PD_root, 			0, 				0, 			0, 		0, 		0,			 PT_dir, 	"Directory One", 0 }, //Directory entry
//		ADRESS									MIN		MAX		STEP	DECIMAL		RdTYPE					NAME			FORMATTING
		{ pti(UserConfig.Throttle.EnumParam, 	0, 		1, 		1, 		0), 		PT_enum, 				"Mode", "Off\nActive\nPassive" }, //
		{ pti(UserConfig.Throttle.Min, 			0, 		5000,	10, 	3), 		PT_value, 				"Min mV", 0 }, //
		{ pti(UserConfig.Throttle.Max, 			0, 		5000, 	10, 	3), 		PT_value, 				"Max mV", 0 }, //
};
// @formatter:on
//all directories stored here
const LC_ParameterDirectory_t paramDirectories[] = { DIRDEF(PD_root), DIRDEF(PD_Dir1) };
//and it's size
const uint32_t paramDirectoriesSize = sizeof(paramDirectories) / sizeof(paramDirectories[0]);

LC_NodeDescriptor_t *mynode;

void nwrk_manager(void *pvParameters);
#ifdef LEVCAN_USE_RTOS_QUEUE
void can_RXmanager(void *pvParameters);
void can_TXmanager(void *pvParameters);
#endif

void Init_LEVCAN(void) {

	/*
	 First step - clone repo to your PC and connect "LEVCAN\source" folder to your project.
	 In "LEVCAN\hal" folder you can find standard drivers for specific target
	 (only STM32 CAN driver available atm.), copy it to your project or make your own driver.

	 Now need to create configuration file for levcan called "levcan_config.h",
	 you can use example file in "LEVCAN\examples" If you don't use any RTOS you may want to `#define LEVCAN_MEM_STATIC`.
	 Initialize your HAL, ports and system clocks. LEVCAN running on 1Mbit, 2 SJW, 87% sample point.
	 Online calculator http://www.bittiming.can-wiki.info/, you can use included HAL function:
	 */
	CAN_InitFromClock(RCC_APB1_CLK, 1000, 2, 87); //default CAN speed 1Mhz, sample 87%, sjw=2

	//At this point you should have working CAN and should be set interrupts for rx/tx buffer.
	//Let's create LEVCAN node initialization structure:
	LC_NodeDescriptor_t *node_desc;
	LC_InitNodeDescriptor(&node_desc);

	node_desc->DeviceName = "Some Awesome Device";
	node_desc->NodeName = "Awesome Device Node"; //visible in device list from lcd
	node_desc->VendorName = "CompanyName LLC.";

	//Put your unique device codes
	node_desc->ShortName.ManufacturerCode = 0xABC;
	node_desc->ShortName.NodeID = 31; //default used ID if free
	node_desc->Serial = 0xDEADBEEF; //use your unique SN
	node_desc->ShortName.DeviceType = LC_Device_Controller; //Setup device type, used for parsing devices over CAN bus

	//assing node objects and its size
	node_desc->Objects = (void*) node_obj;
	node_desc->ObjectsSize = node_obj_size;

	//since we have configuration for our device, set configurable bit and assign parameters
	node_desc->ShortName.Configurable = 1; // we have configurable variables (LC_ParameterDirectory_t)
	node_desc->Directories = (void*) paramDirectories;
	node_desc->DirectoriesSize = paramDirectoriesSize;

	//create node
	LC_CreateNode(node_desc);
	mynode = node_desc;

	//run network manager task, basically it updates at 100-1000hz rate
	xTaskCreate(nwrk_manager, "LC", configMINIMAL_STACK_SIZE, NULL, OS_PRIORITY_LOW, (TaskHandle_t*) NULL);

	//As example request all device names in network
	//last one will be stored in UserVariables.String
	LC_SendRequest(0, LC_Broadcast_Address, LC_SYS_DeviceName);
}

void nwrk_manager(void *pvParameters) {
#ifdef LEVCAN_USE_RTOS_QUEUE
	xTaskCreate(can_RXmanager, "CRX", configMINIMAL_STACK_SIZE, NULL, OS_PRIORITY_LOW, (TaskHandle_t*) NULL);
	xTaskCreate(can_TXmanager, "CTX", configMINIMAL_STACK_SIZE, NULL, OS_PRIORITY_MID, (TaskHandle_t*) NULL);
	const int updrate = 100;
#else
	const int updrate = 1000;
#endif
	while (1) {
		LC_NetworkManager(configTICK_RATE_HZ / 1000);
#ifndef LEVCAN_USE_RTOS_QUEUE
		LC_ReceiveManager();
		LC_TransmitManager();
#endif
		static uint32_t prev_upd_time_sp = 0;
		vTaskDelayUntil(&prev_upd_time_sp, configTICK_RATE_HZ / updrate); //100hz CAN
	}
}

#ifdef LEVCAN_USE_RTOS_QUEUE
void can_RXmanager(void *pvParameters) {
	(void) pvParameters;
	while (1) {
		LC_ReceiveManager();
	}
}

void can_TXmanager(void *pvParameters) {
	(void) pvParameters;
	while (1) {
		LC_TransmitManager();
	}
}
#endif

