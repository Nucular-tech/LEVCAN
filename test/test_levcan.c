#include "levcan.h"
#include "levcan_objects.h"
#include "unity.h"
#include "mock_can_hal.h"
#include "mock_levcan_param.h"

LC_NodeDescription_t* node;
enum {
	Test_Readable = 1,
	Test_Writable,
	Test_RW,
	Test_VariableSizeMax,
	Test_RecordArray,
	Test_Function,
	Test_Pointer,
	Test_Cleanup,
	Test_PointerCleanup,
};
#define sizref(variable) sizeof(variable),(intptr_t*) &variable
const LC_Object_t controller_node_obj[] = { //
		/*{ LC_Obj_State, { .Readable = 1 }, sizref(RD.State) }, //
		{ LC_Obj_DCSupply, { .Readable = 1 }, sizref(CANdata.DCSupply) }, //
		{ LC_Obj_MotorSupply, { .Readable = 1 }, sizref(CANdata.MotorSupply) }, //
		{ LC_Obj_InternalVoltage, { .Readable = 1 }, sizref(CANdata.IntVoltage) }, //
		{ LC_Obj_ThrottleV, { .Writable = 1, .Function = 1 }, sizeof(LC_Obj_ThrottleV_t), &receiveCANcontrol }, //
		{ LC_Obj_BrakeV, { .Writable = 1, .Function = 1 }, sizeof(LC_Obj_BrakeV_t), &receiveCANcontrol }, //
		{ LC_Obj_ControlFactor, { .Writable = 1, .Function = 1 }, sizeof(LC_Obj_ControlFactor_t), &receiveCANcontrol }, //
		{ LC_Obj_Buttons, { .Writable = 1, .Function = 1 }, sizeof(LC_Obj_Buttons_t), &receiveCANcontrol }, //
		{ LC_Obj_Buttons, { .Readable = 1 }, sizref(CANdata.Buttons) }, //
		{ LC_Obj_MotorHalls, { .Readable = 1 }, sizref(CANdata.Halls) }, //
		{ LC_Obj_Power, { .Readable = 1 }, sizref(CANdata.Power) }, //
		{ LC_Obj_RPM, { .Readable = 1 }, sizref(CANdata.RPMs) }, //
		{ LC_Obj_Speed, { .Readable = 1 }, sizref(CANdata.Speed) }, //
		{ LC_Obj_Temperature, { .Readable = 1 }, sizref(CANdata.Temp) }, //
		{ LC_Obj_ActiveFunctions, { .Readable = 1 }, sizref(CANdata.ActiveFunc) }, //
		{ LC_Obj_Distance, { .Readable = 1 }, sizref(CANdata.Distance) }, //
//		{ LC_Obj_WhStored, 			{ .Readable = 1 }, 			sizref(CANdata.WHstored)}, //
		{ LC_Obj_WhUsed, { .Readable = 1 }, sizref(CANdata.WHused) }, //
		{ LC_SYS_SWUpdate, { .Writable = 1, .Function = 1 }, 4, &proceedSWU }, //
		{ LC_Obj_UserActivity, { .Writable = 1, .Readable = 1, .Function = 1 }, 0, &proceedSWU }, //
		{ LC_SYS_Shutdown, { .Writable = 1, .Readable = 1, .Function = 1 }, 0, &proceedSWU }, //*/
		};
const uint16_t controller_node_obj_size = sizeof(controller_node_obj) / sizeof(controller_node_obj[0]);

void setUp() {
	LC_NodeInit_t node_init = { 0 };

	node_init.DeviceName = "Test Levcan DeviceName";
	node_init.NodeName = "Test Levcan NodeName";
	node_init.VendorName = "VendorName";
	node_init.ManufacturerCode = 0x1234;
	node_init.DeviceType = LC_Device_Controller;

	node_init.NodeID = 7;
	node_init.Serial = 0xDEADBEEF;
	node_init.SWUpdates = 1;
	node_init.Variables = 1;
	node_init.Configurable = 1;

	node_init.Objects = (LC_Object_t*) controller_node_obj;
	node_init.ObjectsSize = controller_node_obj_size;
	//node_init.Directories = (void*) PD_Directories;
	//node_init.DirectoriesSize = PD_Directories_size;

	node = (void*) LC_CreateNode(node_init);
}

void tearDown() {

}

extern int16_t compareNode(LC_NodeShortName_t a, LC_NodeShortName_t b);
void test_compare() {
	LC_NodeShortName_t a;
	LC_NodeShortName_t b;
	a.ToUint32[0] = 100;
	a.ToUint32[1] = 100;
	b.ToUint32[0] = 100;
	b.ToUint32[1] = 100;
	int result = compareNode(a, b);
	TEST_ASSERT_EQUAL_INT(0, result);

	a.ToUint32[0] = 200;
	result = compareNode(a, b);
	TEST_ASSERT_EQUAL_INT(1, result);

	b.ToUint32[0] = 300;
	result = compareNode(a, b);
	TEST_ASSERT_EQUAL_INT(-1, result);
}

extern LC_ObjectRecord_t findObjectRecord(uint16_t index, int32_t size, LC_NodeDescription_t* node, uint8_t read_write, uint8_t nodeID);
void test_findObjectRecord() {

}
