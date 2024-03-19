//  SPDX-FileCopyrightText: 2023 Nucular Limited
//  SPDX-License-Identifier: Apache-2.0

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
// include main levcan.h and hal
#include "levcan.h"
#include "can_hal.h"
// include device configuration
#include "levcan_paramserver.h"
// include standard levcan objects
#include "levcan_objects.h"

//Example for CAN object setup
struct {
	uint32_t Data1;
	char String[128];
} UserVariables;

struct {
	uint32_t Parameter1;
	uint32_t Parameter2;
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

// @formatter:off
const LCPS_Entry_t PD_Root[]
= {		folder(LCP_AccessLvl_Any, 		1, 		0, 0 ),
		pstd(LCP_AccessLvl_Any, 	LCP_ReadOnly,	UserConfig.Parameter1,			((LCP_Uint32_t){0, 	0, 	0}),				"Shutdown time", "%s sec"),
		pstd(LCP_AccessLvl_Any, 	LCP_Normal,		UserConfig.Parameter2,			((LCP_Decimal32_t){6000, 100000, 100, 3}), 	"Maximum voltage", "%sV"),
};

const LCPS_Entry_t PD_Dir1[]
= { 	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		UserConfig.Throttle.EnumParam,	((LCP_Enum_t){0, 3}),					"Mode", "Off\nActive\nPassive"),
		pstd(LCP_AccessLvl_Any, 	LCP_Normal,		UserConfig.Throttle.Min,		((LCP_Decimal32_t){0, 5000, 10, 3}), 	"Min mV", 0),
		pstd(LCP_AccessLvl_Any, 	LCP_Normal,		UserConfig.Throttle.Max,		((LCP_Decimal32_t){0, 5000, 10, 3}), 	"Max mV", 0),
};
// @formatter:on

//all directories stored here
const LCPS_Directory_t pDirectories[] = {
directory(PD_Root, 0, LCP_AccessLvl_Any, "DeviceName"),
directory(PD_Dir1, 0, LCP_AccessLvl_Any, "Directory 1") };
//and it's size
const uint32_t paramDirectoriesSize = sizeof(pDirectories) / sizeof(pDirectories[0]);

LC_NodeDescriptor_t node_data;
LC_NodeDescriptor_t *mynode;
//functions to call CAN driver
const LC_DriverCalls_t nodeDrv = { LC_HAL_Send, LC_HAL_CreateFilterMasks, LC_HAL_TxHalfFull };

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
	//Replace this function with your custom CAN initialization code
	CAN_InitFromClock(RCC_APB1_CLK, 1000, 2, 87); //default CAN speed 1Mhz, sample 87%, sjw=2

	//At this point you should have working CAN and should be set interrupts for rx/tx buffer.
	//Let's create LEVCAN node initialization structure:

	mynode = &node_data;
	LC_InitNodeDescriptor(mynode);
	mynode->Driver = &nodeDrv;

	LCP_ParameterServerInit(mynode); //Init parameters server

	mynode->DeviceName = "Some Awesome Device";
	mynode->NodeName = "Awesome Device Node"; //visible in device list from lcd
	mynode->VendorName = "CompanyName LLC.";

	//Put your unique device codes
	mynode->ShortName.ManufacturerCode = 0x1BC;
	mynode->ShortName.NodeID = 31; //default used ID if free
	mynode->Serial[0] = 0xDEADBEEF; //use your unique SN
	mynode->ShortName.DeviceType = LC_Device_Controller; //Setup device type, used for parsing devices over CAN bus
	mynode->ShortName.CodePage = 1250; //default node encoding, win1250

	//assing node objects and its size
	mynode->Objects = (void*) node_obj;
	mynode->ObjectsSize = node_obj_size;

	//since we have configuration for our device, set configurable bit and assign parameters
	mynode->Directories = (void*) pDirectories;
	mynode->DirectoriesSize = paramDirectoriesSize;

	//create node
	LC_CreateNode(mynode);

	//run network manager task, basically it updates at 100-1000hz rate
	xTaskCreate(nwrk_manager, "LC", configMINIMAL_STACK_SIZE, NULL, OS_PRIORITY_LOW, (TaskHandle_t*) NULL);

	//As example request all device names in network
	//last one will be stored in UserVariables.String
	LC_SendRequest(0, LC_Broadcast_Address, LC_SYS_DeviceName);
}

void nwrk_manager(void *pvParameters) {
	(void) pvParameters;
#ifdef LEVCAN_USE_RTOS_QUEUE
	xTaskCreate(can_RXmanager, "CRX", configMINIMAL_STACK_SIZE, NULL, OS_PRIORITY_LOW, (TaskHandle_t*) NULL);
	xTaskCreate(can_TXmanager, "CTX", configMINIMAL_STACK_SIZE, NULL, OS_PRIORITY_MID, (TaskHandle_t*) NULL);
	const int updrate = 100;
#else
	const int updrate = 1000;
#endif
	while (1) {
		LC_NetworkManager(mynode, configTICK_RATE_HZ / 1000);
#ifndef LEVCAN_USE_RTOS_QUEUE
		LC_ReceiveManager(mynode);
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
#endif

