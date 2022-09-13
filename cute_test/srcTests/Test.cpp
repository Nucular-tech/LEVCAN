#include <levcan_paramcommon_test.h>
#include <levcan_paramserver_test.h>
#include "cute.h"
#include "ide_listener.h"
#include "xml_listener.h"
#include "cute_runner.h"


bool runAllTests(int argc, char const *argv[]) {
	//TODO add your test here
	cute::xml_file_opener xmlfile(argc, argv);
	cute::xml_listener<cute::ide_listener<>> lis(xmlfile.out);
	auto runner = cute::makeRunner(lis, argc, argv);
	bool success = true;
	cute::suite levcan_paramcommon = make_suite_levcan_paramcommon();
	success &= runner(levcan_paramcommon, "levcan_paramcommon");
	cute::suite levcan_paramserver = make_suite_levcan_paramserver();
	return success & runner(levcan_paramserver, "levcan_paramserver");
}

int main(int argc, char const *argv[]) {
	return runAllTests(argc, argv) ? EXIT_SUCCESS : EXIT_FAILURE;

}
