#include <levcan_paramserver_test.h>
#include "cute.h"

extern "C" {
#include "levcan_paramserver.h"
#include "levcan_paramcommon.h"
#include "paramserver_testdata.h"
}

#define ASSERT_EQUALI32(a,b)  ASSERT_EQUAL((int32_t)a, (int32_t)b)
#define ASSERT_EQUALIM32(text, a,b)  ASSERT_EQUALM(text, (int32_t)a, (int32_t)b)

#define TEST_PARAM_VALUE(val, textvalue, status, type, index) \
	ret = LCP_ParseParameterValue(&PD_PAS[index], textvalue, &outstr);\
	ASSERT_EQUALM("Test " #type " input "#textvalue" with result: " #val, val, *(type*)PD_PAS[index].Variable); \
	ASSERT_EQUALIM32("Test " #type " input "#textvalue" with status: " #status, status, ret);

#define TEST_PARAM_VALUEFD(val, delta, textvalue, status, type, index) \
	ret = LCP_ParseParameterValue(&PD_PAS[index], textvalue, &outstr);\
	ASSERT_EQUAL_DELTAM("Test " #type " input "#textvalue" with result: " #val, val, *(type*)PD_PAS[index].Variable, delta); \
	ASSERT_EQUALIM32("Test " #type " input "#textvalue" with status: " #status, status, ret);

void levcan_paramserverTest() {
	char *outstr;
	LC_Return_t ret = LC_Ok;

	TEST_PARAM_VALUE(1, "PAS sensor   \nRandomtext", LC_Ok, uint8_t, 0);
	TEST_PARAM_VALUE(2, "Port is busy!  ", LC_OutOfRange, uint8_t, 0);

	TEST_PARAM_VALUE(0, " 1-wire   \n new line", LC_Ok, uint16_t, 1);

	TEST_PARAM_VALUE(500, " 5sec", LC_Ok, int16_t, 2);
	TEST_PARAM_VALUE(5, "  0.05sec", LC_Ok, int16_t, 2);
	TEST_PARAM_VALUE(2, " 0sec", LC_OutOfRange, int16_t, 2);

	TEST_PARAM_VALUE(5, " 5 Hz", LC_Ok, uint32_t, 3);
	TEST_PARAM_VALUE(1, " -5 Hz", LC_OutOfRange, uint32_t, 3);

	TEST_PARAM_VALUE(-5, " -5 Hz", LC_Ok, int32_t, 4);
	TEST_PARAM_VALUE(-5, " -5.4 Hz\n New line of DOOM", LC_Ok, int32_t, 4);

	TEST_PARAM_VALUE(1, " 0Hz", LC_OutOfRange, uint64_t, 5);
	TEST_PARAM_VALUE(100, "101", LC_OutOfRange, uint64_t, 5);

	TEST_PARAM_VALUE(-10, "-11 Value", LC_OutOfRange, int64_t, 6);
	TEST_PARAM_VALUE(0, "0", LC_Ok, int64_t, 6);

	TEST_PARAM_VALUEFD(1.1, 0.01f, "1.1 Nm/V", LC_Ok, float, 7);
	TEST_PARAM_VALUEFD(-1.1, 0.01f, "-1.1 Nm/V", LC_Ok, float, 7);

	TEST_PARAM_VALUEFD(1.1, 0.01f, "1.1 Nm/V", LC_Ok, double, 8);
	TEST_PARAM_VALUEFD(-1.1, 0.01f, "-1.1 Nm/V", LC_Ok, double, 8);

	TEST_PARAM_VALUE(0, "  0b0", LC_Ok, uint32_t, 9);
	TEST_PARAM_VALUE(1, "  0b1", LC_Ok, uint32_t, 9);
	TEST_PARAM_VALUE(3, "  0b11", LC_Ok, uint32_t, 9);
	TEST_PARAM_VALUE(7, "  0b111", LC_Ok, uint32_t, 9);
	TEST_PARAM_VALUE(0xFFFF, "  0b11111111111111111111", LC_OutOfRange, uint32_t, 9);
}

cute::suite make_suite_levcan_paramserver() {
	cute::suite s { };
	s.push_back(CUTE(levcan_paramserverTest));
	return s;
}
