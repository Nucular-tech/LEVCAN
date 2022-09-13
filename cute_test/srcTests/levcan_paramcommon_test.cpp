#include "cute.h"
#include "levcan_paramcommon.c"

void lcp_print_u32b_test() {
	char *buf = (char*) malloc(40);
	std::string str;

	lcp_print_u32b(buf, 0xAAAAAAAA);
	str.assign(buf);
	ASSERT_EQUAL("0b10101010101010101010101010101010", str);

	lcp_print_u32b(buf, 0xAAAA);
	str.assign(buf);
	ASSERT_EQUAL("0b1010101010101010", str);
}

void lcp_print_i32f_test() {
	char *buf = (char*) malloc(40);
	std::string str;

	lcp_print_i32f(buf, 123456, 3);
	str.assign(buf);
	ASSERT_EQUAL("123.456", str);

	lcp_print_i32f(buf, -123456, 5);
	str.assign(buf);
	ASSERT_EQUAL("-1.23456", str);

	lcp_print_i32f(buf, 123456, 0);
	str.assign(buf);
	ASSERT_EQUAL("123456", str);

	lcp_print_i32f(buf, -12, 5);
	str.assign(buf);
	ASSERT_EQUAL("-0.00012", str);
}

cute::suite make_suite_levcan_paramcommon() {
	cute::suite s { };
	s.push_back(CUTE(lcp_print_u32b_test));
	s.push_back(CUTE(lcp_print_i32f_test));
	return s;
}
