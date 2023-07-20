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

#include "AS5047PReader.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

void
AS5047PReader::print_usage()
{
	PRINT_MODULE_USAGE_NAME("as5047p", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("encoder");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int as5047p_main(int argc, char *argv[])
{
	using ThisDriver = AS5047PReader;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = 5 * 1000 * 1000;
	cli.parseDefaultArguments(argc, argv);
	const char *verb = cli.optArg();
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ENCODER_DEVTYPE_AS5047P);

	// new ThisDriver
	const px4::wq_config_t &wq_config = px4::device_bus_to_wq(DRV_ENCODER_DEVTYPE_AS5047P);
	I2CSPIDriverConfig driver_config{cli, iterator, wq_config};
	ThisDriver *interface = new ThisDriver(driver_config);
	interface->init();
	interface->RunImpl();
	// if (!strcmp(verb, "start")) {
	// 	PX4_INFO("AS5047p try to start up");
	// 	return ThisDriver::module_start(cli, iterator);
	// }

	// if (!strcmp(verb, "stop")) {
	// 	return ThisDriver::module_stop(iterator);
	// }

	// if (!strcmp(verb, "status")) {
	// 	return ThisDriver::module_status(iterator);
	// }

	ThisDriver::print_usage();
	return -1;
}
