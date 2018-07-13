#include <stdint.h>

#pragma once

enum {
	LC_Obj_State = 0x300,
	LC_Obj_SupplyVoltage,
	LC_Obj_InternalVoltage,
	LC_Obj_Currents,
	LC_Obj_Power,
	LC_Obj_Temperature,
	LC_Obj_RPM,
	LC_Obj_RadSec,
	LC_Obj_Speed,
	LC_Obj_Controls,
	LC_Obj_Throttle,
	LC_Obj_Brake,
	LC_Obj_SpeedCommand,
	LC_Obj_TorqueCommand,
	LC_Obj_Buttons,
	LC_Obj_WhUsed,
	LC_Obj_WhStored,
	LC_Obj_Distance,
	LC_Obj_MotorHalls,
	LC_Obj_CellsV,
	LC_Obj_CellMinMax,
};

typedef struct {
	int32_t SupplyV; //mV
	int32_t BatteryV; //mV
} LC_Obj_SupplyVoltage_t;

typedef struct {
	int16_t Int12V; //mV
	int16_t Int5V; //mV
	int16_t Int3_3V; //mV
	int16_t IntREFV; //mV
} LC_Obj_InternalVoltage_t;

typedef struct {
	int32_t DC_Current; //mA
	int32_t Motor_Current; //mA
} LC_Obj_Currents_t;

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
	int16_t Speed; //kmh
} LC_Obj_Speed_t;

typedef struct {
	int16_t ThrottleV; //mV
	int16_t BrakeV; //mV
	union {
		struct {
			unsigned Enable :1;
			unsigned Brake :1;
			unsigned Lock :1;
			unsigned Speed1 :1;
			unsigned Speed3 :1;
			unsigned Reverse :1;
			unsigned Cruise :1;
			unsigned TurnRight :1;
			unsigned TurnLeft :1;
			unsigned Horn :1;
			unsigned Button1 :1;
			unsigned Button2 :1;
			unsigned Button3 :1;
			unsigned Button4 :1;
			unsigned Button5 :1;
			unsigned Button6 :1;
		};
		uint16_t Inputs;
	};
	uint16_t Timeout; //control timeout
} LC_Obj_Controls_t;

typedef struct {
	uint16_t WhUsed;
	uint16_t WhUsedFromEn;
} LC_Obj_WhUsed_t;

typedef struct {
	uint16_t WhStored;
	uint16_t WhTotalStorage;
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
	int16_t CellMin; //mV
	int16_t CellMax; //mV
} LC_Obj_CellMinMax_t;
