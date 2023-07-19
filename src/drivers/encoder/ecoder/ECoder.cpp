/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
 *
 * Redistributn and use in source and binary forms, with or without
 * modificatn, are permitted provided that the following conditns
 * are met:
 *
 * 1. Redistributns of source code must retain the above copyright
 *    notice, this list of conditns and the following disclaimer.
 * 2. Redistributns in binary form must reproduce the above copyright
 *    notice, this list of conditns and the following disclaimer in
 *    the documentatn and/or other materials provided with the
 *    distributn.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ECoder.hpp"
#include <termios.h>
#include <fcntl.h>

using namespace time_literals;

ECoder::ECoder(const char *device0, const char * device1) :
	ModuleParams(nullptr),
	_publish_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval"))
{
	if (device0) {
		PX4_INFO("Start ecoder 0 on %s", device0);
		reader0 = new ECoderReader("ecoder_0", 0, device0);
	}
	if (device1) {
		PX4_INFO("start ecoder 1 on %s", device1);
		reader1 = new ECoderReader("ecoder_1", 1, device1);
	}
}

ECoder::~ECoder()
{
	perf_free(_publish_interval_perf);
}

int
ECoder::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device0 = nullptr;
	const char *device1 = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:D:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device0 = myoptarg;
			break;
		case 'D':
			device1 = myoptarg;
			break;
		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return -1;
	}

	if (device0 == nullptr) {
		PX4_ERR("valid device required");
		return PX4_ERROR;
	}

	ECoder *instance = new ECoder(device0, device1);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->start(_current_update_interval);
	return PX4_OK;
}

void ECoder::start(uint32_t interval_us) {
	if (reader0) {
		reader0->ScheduleOnInterval(interval_us);
	}
	if (reader1) {
		reader1->ScheduleOnInterval(interval_us);
	}
}


int ECoder::custom_command(int argc, char *argv[])
{
	return 0;
}

int ECoder::print_status()
{
	PX4_INFO("Max update rate: %u Hz", 1000000 / _current_update_interval);

	if (reader0) {
		reader0->print_status();
	}
	if (reader1) {
		reader1->print_status();
	}
	perf_print_counter(_publish_interval_perf);
	return 0;
}

int
ECoder::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Descriptn
Encoder
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ecoder", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file:dev>", "ECoder device 0", true);
	PRINT_MODULE_USAGE_PARAM_STRING('D', "/dev/ttyS2", "<file:dev>", "ECoder device 1", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ecoder_main(int argc, char *argv[])
{
	return ECoder::main(argc, argv);
}
