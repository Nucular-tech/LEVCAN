/*
 * paramserver_testdata.c
 *
 *  Created on: 4 рту. 2021 у.
 *      Author: VasiliSk
 */

#include "levcan_paramserver.h"
#include "levcan_paramcommon.h"
#include <math.h>

#ifdef __CDT_PARSER__
#define _Generic(...) 0
#endif

uint8_t u8_test = UINT8_MAX;
uint16_t u16_test = UINT16_MAX;
int16_t i16_test = INT16_MAX;
uint32_t u32_test = UINT32_MAX;
int32_t i32_test = INT32_MAX;
uint64_t u64_test = UINT64_MAX;
int64_t i64_test = INT64_MAX;
uint32_t bit_test = UINT32_MAX;
float f32_test = NAN;
double d64_test = NAN;

const LCPS_Entry_t PD_PAS[] = { //
		pstd(LCP_AccessLvl_Any, LCP_Normal, u8_test, ((LCP_Enum_t ) {0, 3}), "PAS", "Disabled\nPAS sensor\nTorque sensor\nPort is busy!"), //0
		pstd(LCP_AccessLvl_Any, LCP_Normal, u16_test, ((LCP_Enum_t ) {0, 2}), "PAS connection", "1-wire\n2-wire"), //1
		pstd(LCP_AccessLvl_Any, LCP_Normal, i16_test, ((LCP_Decimal32_t ) {2, 500, 2, 2}), "PAS timeout", "%s sec"), //2
		pstd(LCP_AccessLvl_Any, LCP_Normal, u32_test, ((LCP_Uint32_t ) {1, 100, 1}), "PAS filter", "%u Hz"), //3
		pstd(LCP_AccessLvl_Any, LCP_Normal, i32_test, ((LCP_Int32_t ) {-10, 100, 1}), "PAS filter", "%u Hz"), //4
		pstd(LCP_AccessLvl_Any, LCP_Normal, u64_test, ((LCP_Uint64_t ) {1, 100, 1}), "PAS filter u64", "%u Hz"), //5
		pstd(LCP_AccessLvl_Any, LCP_Normal, i64_test, ((LCP_Int64_t ) {-10, 100, 1}), "PAS filter i64", "%u Hz"), //6
		pstd(LCP_AccessLvl_Any, LCP_Normal, f32_test, ((LCP_Float_t ) {-10, 100, 0.5}), "Pressure scale", "%.1f Nm/V"), //7
		pstd(LCP_AccessLvl_Any, LCP_Normal, d64_test, ((LCP_Double_t ) {-10, 100, 0.5}), "Pressure scale double", "%.1f Nm/V"), //8
		pstd(LCP_AccessLvl_Any, LCP_Normal, bit_test, ((LCP_Bitfield32_t ) {0xFFFF}), "Mask test", 0), //9
		};

const LCPS_Directory_t pDirectories[] = { directory(PD_PAS, 0, LCP_AccessLvl_Any, "Pedal Assist System"), };

