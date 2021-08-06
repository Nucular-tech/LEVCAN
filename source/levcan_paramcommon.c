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
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "levcan_paramserver.h"
#include "levcan_paraminternal.h"
#include "levcan_paramcommon.h"

LC_Return_t LCP_LimitValue(intptr_t *variable, uint16_t varSize, const intptr_t *descriptor, uint16_t descSize, uint8_t type) {
	if (variable == 0 || varSize == 0)
		return LC_DataError;	//invalid
	if (type > LCP_Enum && type < LCP_End && descriptor == 0)
		return LC_DataError;	//invalid

	switch ((LCP_Type_t) type) {
	default:
		return LC_Ok;
	case LCP_Bool: //LCP_Bool_t
		return lcp_u32inRange(variable, varSize, 0, 1);
		break;
	case LCP_Enum: { //LCP_Enum_t
		if (descSize != sizeof(LCP_Enum_t)) {
			return LC_DataError;
		}
		const LCP_Enum_t *desc = (LCP_Enum_t*) descriptor;
		return lcp_u32inRange(variable, varSize, desc->Min, desc->Min + desc->Size);
	}
		break;
	case LCP_Bitfield32: { //LCP_Bitfield32_t
		if (descSize != sizeof(LCP_Bitfield32_t)) {
			return LC_DataError;
		}
		const LCP_Bitfield32_t *desc = (LCP_Bitfield32_t*) descriptor;
		return lcp_bitinRange(variable, varSize, desc->Mask);
	}
		break;
	case LCP_Int32: { //LCP_Int32_t
		if (descSize != sizeof(LCP_Int32_t)) {
			return LC_DataError;
		}
		const LCP_Int32_t *desc = (LCP_Int32_t*) descriptor;
		return lcp_i32inRange(variable, varSize, desc->Min, desc->Max);
	}
		break;
	case LCP_Uint32: { //LCP_Uint32_t
		if (descSize != sizeof(LCP_Uint32_t)) {
			return LC_DataError;
		}
		const LCP_Uint32_t *desc = (LCP_Uint32_t*) descriptor;
		return lcp_u32inRange(variable, varSize, desc->Min, desc->Max);
	}
		break;
#ifdef LEVCAN_USE_INT64
	case LCP_Int64: { //LCP_Int64_t
		if (descSize != sizeof(LCP_Int64_t)) {
			return LC_DataError;
		}
		const LCP_Int64_t *desc = (LCP_Int64_t*) descriptor;
		return lcp_i64inRange(variable, varSize, desc->Min, desc->Max);
	}
		break;
	case LCP_Uint64: { //LCP_Uint64_t
		if (descSize != sizeof(LCP_Uint64_t)) {
			return LC_DataError;
		}
		const LCP_Uint64_t *desc = (LCP_Uint64_t*) descriptor;
		return lcp_u64inRange(variable, varSize, desc->Min, desc->Max);
	}
		break;
#endif
#ifdef LEVCAN_USE_FLOAT
	case LCP_Float: { //LCP_Float_t
		if (descSize != sizeof(LCP_Float_t)) {
			return LC_DataError;
		}
		const LCP_Float_t *desc = (LCP_Float_t*) descriptor;
		return lcp_f32inRange(variable, varSize, desc->Min, desc->Max);
	}
		break;
#endif
#ifdef LEVCAN_USE_DOUBLE
	case LCP_Double: { //LCP_Double_t
		if (descSize != sizeof(LCP_Double_t)) {
			return LC_DataError;
		}
		const LCP_Double_t *desc = (LCP_Double_t*) descriptor;
		return lcp_d64inRange(variable, varSize, desc->Min, desc->Max);
	}
		break;
#endif

	}
	return LC_Ok;
}

LC_Return_t lcp_d64inRange(intptr_t *variable, uint16_t varSize, double min, double max) {
	if (varSize != sizeof(double))
		return LC_DataError;
	double value = *(double*) variable;

	int outOfRange = 0;
	if (value > max) {
		value = max;
		outOfRange = 1;
	} else if (value < min) {
		value = min;
		outOfRange = 1;
	}
	*(double*) variable = value;
	return outOfRange ? LC_OutOfRange : LC_Ok;
}

LC_Return_t lcp_f32inRange(intptr_t *variable, uint16_t varSize, float min, float max) {
	if (varSize != sizeof(float))
		return LC_DataError;
	float value = *(float*) variable;

	int outOfRange = 0;
	if (value > max) {
		value = max;
		outOfRange = 1;
	} else if (value < min) {
		value = min;
		outOfRange = 1;
	}
	*(float*) variable = value;
	return outOfRange ? LC_OutOfRange : LC_Ok;
}

#ifdef LEVCAN_USE_INT64
LC_Return_t lcp_i64inRange(intptr_t *variable, uint16_t varSize, int64_t min, int64_t max) {
	if (varSize != sizeof(int64_t))
		return LC_DataError;
	int64_t value = *(int64_t*) variable;

	int outOfRange = 0;
	if (value > max) {
		value = max;
		outOfRange = 1;
	} else if (value < min) {
		value = min;
		outOfRange = 1;
	}
	*(int64_t*) variable = value;
	return outOfRange ? LC_OutOfRange : LC_Ok;
}

LC_Return_t lcp_u64inRange(intptr_t *variable, uint16_t varSize, uint64_t min, uint64_t max) {
	if (varSize != sizeof(uint64_t))
		return LC_DataError;
	uint64_t value = *(uint64_t*) variable;

	int outOfRange = 0;
	if (value > max) {
		value = max;
		outOfRange = 1;
	} else if (value < min) {
		value = min;
		outOfRange = 1;
	}
	*(uint64_t*) variable = value;
	return outOfRange ? LC_OutOfRange : LC_Ok;
}
#endif

LC_Return_t lcp_u32inRange(intptr_t *variable, uint16_t varSize, uint32_t min, uint32_t max) {
	if (varSize > sizeof(uint32_t))
		return LC_DataError;
	uint32_t value = lcp_getUint32(variable, varSize);

	if (value > max) {
		lcp_setUint32(variable, varSize, max);
		return LC_OutOfRange;
	} else if (value < min) {
		lcp_setUint32(variable, varSize, min);
		return LC_OutOfRange;
	}
	return LC_Ok;
}

LC_Return_t lcp_bitinRange(intptr_t *variable, uint16_t varSize, uint32_t mask) {
	if (varSize > sizeof(uint32_t))
		return LC_DataError;
	uint32_t value = lcp_getUint32(variable, varSize);

	int outofrange = (~mask) & value;
	value &= mask; //filter by mask
	lcp_setUint32(variable, varSize, value);
	return outofrange ? LC_OutOfRange : LC_Ok;
}

LC_Return_t lcp_i32inRange(intptr_t *variable, uint16_t varSize, int32_t min, int32_t max) {
	if (varSize > sizeof(int32_t))
		return LC_DataError;
	int32_t value = lcp_getInt32(variable, varSize);

	if (value > max) {
		lcp_setInt32(variable, varSize, max);
		return LC_OutOfRange;
	} else if (value < min) {
		lcp_setInt32(variable, varSize, min);
		return LC_OutOfRange;
	}
	return LC_Ok;
}

int32_t lcp_getInt32(intptr_t *variable, uint16_t varSize) {
	switch (varSize) {
	default:
		return *(int8_t*) variable;
	case 2:
		return *(int16_t*) variable;
	case 4:
		return *(int32_t*) variable;
	}
	return 0;
}

void lcp_setInt32(intptr_t *variable, uint16_t varSize, int32_t value) {
	switch (varSize) {
	default:
		*(int8_t*) variable = (int8_t) value;
		break;
	case 2:
		*(int16_t*) variable = (int16_t) value;
		break;
	case 4:
		*(int32_t*) variable = (int32_t) value;
		break;
	}
}

uint32_t lcp_getUint32(intptr_t *variable, uint16_t varSize) {
	switch (varSize) {
	default:
		return *(uint8_t*) variable;
	case 2:
		return *(uint16_t*) variable;
	case 4:
		return *(uint32_t*) variable;
	}
	return 0;
}

void lcp_setUint32(intptr_t *variable, uint16_t varSize, uint32_t value) {
	switch (varSize) {
	default:
		*(uint8_t*) variable = (uint8_t) value;
		break;
	case 2:
		*(uint16_t*) variable = (uint16_t) value;
		break;
	case 4:
		*(uint32_t*) variable = (uint32_t) value;
		break;
	}
}

/// Print integer with decimal point
/// @param buffer - output text
/// @param value - input value
/// @param decimals - number count after dot
/// @return
int lcp_print_i32f(char *buffer, int32_t value, uint8_t decimals) {
	char buf[16];
	uint32_t powww = lcp_pow10i(decimals);

	if (value < 0) {
		if (value == INT32_MIN) {
			value = INT32_MAX;
		} else {
			value = -value;
		}
		*buffer++ = '-';
	}

	uint32_t integer = value / powww;
	uint32_t decimal = value % powww;
	int isize = 0;

	if (decimals == 0) {
		isize = sprintf(buf, "%" PRIu32, integer);
	} else {
		isize = sprintf(buf, "%" PRIu32 ".", integer);
		int dsize = sprintf(buf + isize, "%" PRIu32, decimal);
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
	buf[isize] = 0; //end
	memcpy(buffer, buf, isize + 1); //+null
	return isize;
}

#define intsizetext 33
/// Print binary value of uint32_t = 0b1010
int lcp_print_u32b(char *buffer, uint32_t value) {
	char bufBinary[intsizetext];
	memset(bufBinary, 0, intsizetext);
	*buffer++ = '0';
	*buffer++ = 'b';
	int lastNonZero = intsizetext - 2; //no zero and get index position

	do {
		bufBinary[lastNonZero] = (value & 1) ? '1' : '0';
		value = value >> 1;
		lastNonZero--;
	} while (lastNonZero >= 0 && (value > 0));

	lastNonZero++; //roll back
	int size = intsizetext - lastNonZero;
	memcpy(buffer, &bufBinary[lastNonZero], size); //+null
	return size + 2; //+0b
}

uint32_t lcp_pow10i(uint8_t pow) {
	uint32_t ret = 1;
	for (; pow > 0; pow--)
		ret *= 10;
	return ret;
}
