#include <stdint.h>

#include "levcan_config.h"
#pragma once

typedef enum {
	LC_Device_Unknown = 0,
	LC_Device_reserved = 0x1,
	LC_Device_BMS = 0x2,
	LC_Device_Controller = 0x4,
	LC_Device_Display = 0x8,
	LC_Device_Light = 0x10,
	LC_Device_RemoteControl = 0x20,
	LC_Device_Misc = 0x40,
	LC_Device_Debug = 0x80,
	LC_Device_reserved2 = 0x100,
	LC_Device_reserved3 = 0x200,
} LC_Device_t;

typedef enum {
	LC_Obj_State = 0x300,
	LC_Obj_DCSupply,
	LC_Obj_MotorSupply,
	LC_Obj_InternalVoltage,
	LC_Obj_Power,
	LC_Obj_Temperature,
	LC_Obj_RPM,
	LC_Obj_RadSec,
	LC_Obj_Speed,
	LC_Obj_ThrottleV,
	LC_Obj_BrakeV,
	LC_Obj_ControlFactor,
	LC_Obj_SpeedCommand,
	LC_Obj_TorqueCommand,
	LC_Obj_Buttons,
	LC_Obj_WhUsed,
	LC_Obj_WhStored,
	LC_Obj_Distance,
	LC_Obj_MotorHalls,
	LC_Obj_CellsV,
	LC_Obj_CellMinMax,
	LC_Obj_CellBalance,
	LC_Obj_UserActivity,
	LC_Obj_ActiveFunctions,
	LC_Obj_LightSensor,
	LC_Obj_AccelerometerRaw,
	LC_Obj_Accelerometer,
	LC_Obj_ControlFactorInt,
} LC_Obj_Std_t;

typedef struct {
	int32_t Voltage; //mV
	int32_t Current; //mA
} LC_Obj_Supply_t;
//todo make array type
typedef struct {
	int16_t Int12V; //mV
	int16_t Int5V; //mV
	int16_t Int3_3V; //mV
	int16_t IntREFV; //mV
} LC_Obj_InternalVoltage_t;

typedef struct {
	int32_t Watts; //W
	enum {
		LC_Obj_Power_Idle, LC_Obj_Power_Charging, LC_Obj_Power_Discharging
	} Direction;
} LC_Obj_Power_t;

typedef struct {
	int16_t InternalTemp; //Celsius degree
	int16_t ExternalTemp;
	int16_t ExtraTemp1;
	int16_t ExtraTemp2;
} LC_Obj_Temperature_t;

typedef struct {
	int32_t RPM;
	int32_t ERPM;
} LC_Obj_RPM_t;

typedef struct {
	float RadSec;
} LC_Obj_RadSec_t;

typedef struct {
	int16_t Speed;
} LC_Obj_Speed_t;

typedef struct {
	int16_t ThrottleV; //mV
} LC_Obj_ThrottleV_t;

typedef struct {
	int16_t BrakeV; //mV
} LC_Obj_BrakeV_t;

typedef struct {
	float ControlFactor; //positive - throttle, negative - brake, range: -1...1
} LC_Obj_ControlFactor_t;

typedef struct {
	uint16_t BrakeFactor; //0...10000
	uint16_t ThrottleFactor; //0...10000
} LC_Obj_ControlFactorInt_t;

typedef struct {
	union {
		struct {
			unsigned int Enable :1; //0
			unsigned int Brake :1;//1
			unsigned int Lock :1;//2
			unsigned int Reverse :1;//3
			unsigned int Speed :3;//4-6
			unsigned int Cruise :1;//7
		}LEVCAN_PACKED ;
		uint16_t Buttons;
	};
	union {
		struct {
			unsigned int ExButton1 :1;
			unsigned int ExButton2 :1;
			unsigned int ExButton3 :1;
			unsigned int ExButton4 :1;
			unsigned int ExButton5 :1;
			unsigned int ExButton6 :1;
			unsigned int ExButton7 :1;
			unsigned int ExButton8 :1;
			unsigned int ExButton9 :1;
			unsigned int ExButton10 :1;
			unsigned int ExButton11 :1;
			unsigned int ExButton12 :1;
			unsigned int ExButton13 :1;
			unsigned int ExButton14 :1;
			unsigned int ExButton15 :1;
			unsigned int ExButton16 :1;
		}LEVCAN_PACKED ;
		uint16_t ExtraButtons;
	};
} LC_Obj_Buttons_t;

typedef union {
	struct {
		unsigned int Enable :1;
		unsigned int Lock :1;
		unsigned int Throttle :1;
		unsigned int Brake :1;
		unsigned int Speed :3;
		unsigned int Reverse :1;
		unsigned int Cruise :1;
		unsigned int TurnRight :1;
		unsigned int TurnLeft :1;
		unsigned int LowBeam :1;
		unsigned int HighBeam :1;
		unsigned int PedalAssist :1;
		unsigned int ConverterMode :1;
		unsigned int WheelDriveLock :1;
		unsigned int BatteryUnlocked :1;
		unsigned int MotorWarning :1;
		unsigned int MotorFail :1;
		unsigned int ControllerWarning :1;
		unsigned int ControllerFail :1;
		unsigned int LowBattery :1;
		unsigned int BatteryWarning :1;
		unsigned int BatteryFail :1;
		unsigned int EBrakeLimited :1;
		unsigned int EABS :1;
		unsigned int Service :1;
		unsigned int FanActive :1;
		unsigned int HeaterActive :1;
	}LEVCAN_PACKED ;
	uint32_t Functions[2];
} LC_Obj_ActiveFunctions_t;

typedef struct {
	int32_t WhUsed;
	int32_t WhUsedFromEn;
} LC_Obj_WhUsed_t;

typedef struct {
	int32_t WhStored;
	int32_t WhTotalStorage;
} LC_Obj_WhStored_t;

typedef struct {
	uint32_t TripMeterFromEn;
	uint16_t TotalTripKm;  //odometer?
} LC_Obj_Distance_t;

typedef struct {
	int16_t HallA; //mV
	int16_t HallB; //mV
	int16_t HallC; //mV
	uint8_t InputDigital; //bits 1,2,3
	uint8_t HallState; //1..2..3..4..5..6
} LC_Obj_MotorHalls_t;

typedef struct {
	int16_t Number;
	int16_t Cells[]; //mV
} LC_Obj_CellsV_t;

typedef struct {
	uint8_t Number; //number of bits
	uint8_t Balance[]; //bit field
} LC_Obj_CellBalance_t;

typedef struct {
	int16_t CellMin; //mV
	int16_t CellMax; //mV
	int16_t TempMin; //Celsius degree
	int16_t TempMax;
} LC_Obj_CellMinMax_t;

typedef struct {
	uint16_t Normalized; //0-1024
	uint16_t SensorVoltage; //mV
} LC_Obj_LightSensor_t;

typedef struct {
	int16_t AxisX;
	int16_t AxisY;
	int16_t AxisZ;
} LC_Obj_Accelerometer_t;
