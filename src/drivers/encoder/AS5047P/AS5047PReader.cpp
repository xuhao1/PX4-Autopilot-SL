#include "AS5047PReader.h"
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <px4_platform_common/log.h>


AS5047PReader::AS5047PReader(const I2CSPIDriverConfig &config):
	SPI(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_INTERVAL, "as5047p")),
	_process_perf(perf_alloc(PC_ELAPSED, "as5047p::ProcessData")) {
}

AS5047PReader::~AS5047PReader() {
	perf_free(_cycle_perf);
	perf_free(_process_perf);
}


int AS5047PReader::init() {
    int ret = SPI::init();
    if (ret != OK) {
        DEVICE_DEBUG("SPI init failed (%i)", ret);
        return ret;
    }
    printDebugString();
    return PX4_OK;
}


void AS5047PReader::start() {
	ScheduleOnInterval(_current_update_interval);
}

void AS5047PReader::reset() {
}


void
AS5047PReader::RunImpl()
{
  perf_begin(_cycle_perf);
  perf_end(_cycle_perf);
}

uint16_t AS5047PReader::readData(uint16_t command, uint16_t nopCommand)
{
    uint16_t rx;
    transferhword(&command, &rx, 1);
    transferhword(&nopCommand, &rx, 1);
    PX4_INFO("readData command %d nopCommand %d recv %d", (int)command, (int)nopCommand, (int)rx);
    return rx;
}

void AS5047PReader::writeData(uint16_t command, uint16_t value)
{
    uint16_t rx;
    transferhword(&command, &rx, 1);
    transferhword(&value, &rx, 1);
    PX4_INFO("writeData command %d value %d recv %d", (int)command, (int)value, (int)rx);
}


ReadDataFrame AS5047PReader::readRegister(uint16_t registerAddress) {
	CommandFrame command;
	command.values.rw = READ;
	command.values.commandFrame = registerAddress;
	command.values.parc = isEven(command.raw);

	CommandFrame nopCommand;
	nopCommand.values.rw = READ;
	nopCommand.values.commandFrame = NOP_REG;
	nopCommand.values.parc = isEven(nopCommand.raw);

	ReadDataFrame receivedFrame;
	receivedFrame.raw = readData(command.raw, nopCommand.raw);
	return receivedFrame;
}



void AS5047PReader::writeRegister(uint16_t registerAddress, uint16_t registerValue) {
	CommandFrame command;
	command.values.rw = WRITE;
	command.values.commandFrame = registerAddress;
	command.values.parc = isEven(command.raw);

	WriteDataFrame contentFrame;
	contentFrame.values.data = registerValue;
	contentFrame.values.low = 0;
	contentFrame.values.pard = isEven(contentFrame.raw);
	writeData(command.raw, contentFrame.raw);
}

float AS5047PReader::readAngle() {
	ReadDataFrame readDataFrame = readRegister(ANGLE_REG);
	Angle angle;
	angle.raw = readDataFrame.values.data;
	return angle.values.cordicang/16384.*360.;
}

void AS5047PReader::writeSettings1(Settings1 values) {
	writeRegister(SETTINGS1_REG, values.raw);
}
void AS5047PReader::writeSettings2(Settings2 values){
	writeRegister(SETTINGS2_REG, values.raw);
}
void AS5047PReader::writeZeroPosition(Zposm zposm, Zposl zposl){
	writeRegister(ZPOSM_REG, zposm.raw);
	writeRegister(ZPOSL_REG, zposl.raw);
}

void AS5047PReader::printDebugString() {
	ReadDataFrame readDataFrame;
	readDataFrame = readRegister(ERRFL_REG);
	Errfl errfl;
	errfl.raw = readDataFrame.values.data;
	PX4_INFO("======== AS5047PReader Debug ========");
	PX4_INFO("------- ERRFL Register :");
	PX4_INFO("   Reading Error: %d  FRERR: %d INVCOMM: %d PARERR: %d", readDataFrame.values.ef, errfl.values.frerr, errfl.values.invcomm, errfl.values.parerr);

	readDataFrame = readRegister(PROG_REG);
	Prog prog;
	prog.raw = readDataFrame.values.data;
	PX4_INFO("------- PROG Register: Reading Error: %d PROGEN: %d OTPREF: %d PROGOTP: %d PROVER: %d",
        readDataFrame.values.ef, prog.values.progen, prog.values.otpref, prog.values.progotp, prog.values.progver);

	readDataFrame = readRegister(DIAGAGC_REG);
	Diaagc diaagc;
	diaagc.raw = readDataFrame.values.data;

    // Rewrite in PX4_INFO
    PX4_INFO("|------- DIAAGC Register: Reading Error: %d AGC: %d LF: %d COF: %d MAGH: %d MAGL: %d",
        readDataFrame.values.ef, diaagc.values.agc, diaagc.values.lf, diaagc.values.cof, diaagc.values.magh, diaagc.values.magl);

	readDataFrame = readRegister(MAG_REG);
	Mag mag;
    mag.raw = readDataFrame.values.data;
    PX4_INFO("|------- MAG Register: Reading Error: %d CMAG: %d", readDataFrame.values.ef, mag.values.cmag);

	readDataFrame = readRegister(ANGLE_REG);
	Angle angle;
	angle.raw = readDataFrame.values.data;
    PX4_INFO("|------- ANGLE Register: Reading Error: %d CORDICANG: %d", readDataFrame.values.ef, angle.values.cordicang);

	readDataFrame = readRegister(ANGLECOM_REG);
	Anglecom anglecom;
	anglecom.raw = readDataFrame.values.data;
    PX4_INFO("|------- ANGLECOM Register: Reading Error: %d DAECANG: %d", readDataFrame.values.ef, anglecom.values.daecang);

	readDataFrame = readRegister(ZPOSM_REG);
	Zposm zposm;
	zposm.raw = readDataFrame.values.data;
    PX4_INFO("|------- ZPOSM Register: Reading Error: %d ZPOSM: %d", readDataFrame.values.ef, zposm.values.zposm);

	readDataFrame = readRegister(ZPOSL_REG);
	Zposl zposl;
	zposl.raw = readDataFrame.values.data;
    PX4_INFO("|------- ZPOSL Register: Reading Error: %d ZPOSL: %d COMP_L_ERROR_EN: %d COMP_H_ERROR_EN: %d",
        readDataFrame.values.ef, zposl.values.zposl, zposl.values.compLerrorEn, zposl.values.compHerrorEn);

	readDataFrame = readRegister(SETTINGS1_REG);
	Settings1 settings1;
	settings1.raw = readDataFrame.values.data;
    PX4_INFO("|------- SETTINGS1 Register: Reading Error: %d NOISESET: %d DIR: %d UVW_ABI: %d DAECDIS: %d ABIBIN: %d DATASELECT: %d PWMON: %d",
        readDataFrame.values.ef, settings1.values.noiseset, settings1.values.dir, settings1.values.uvw_abi, settings1.values.daecdis,
        settings1.values.abibin, settings1.values.dataselect, settings1.values.pwmon);

	readDataFrame = readRegister(SETTINGS2_REG);
	Settings2 settings2;
	settings2.raw = readDataFrame.values.data;
    PX4_INFO("|------- SETTINGS2 Register: Reading Error: %d UVWPP: %d HYS: %d ABIRES: %d", readDataFrame.values.ef, settings2.values.uvwpp, settings2.values.hys, settings2.values.abires);

    PX4_INFO("==============================");

/*ANGLECOM_REG 	0x3FFF

// Non-Volatile Registers Addresses
#define ZPOSM_REG 		0x0016
#define ZPOSL_REG 		0x0017
#define SETTINGS1_REG 	0x0018
#define SETTINGS2_REG 	0x0019*/
}


bool AS5047PReader::isEven(uint16_t _data) {
	int count=0;
	unsigned int b = 1;
	for (unsigned int i=0; i<15; i++) {
		if (_data & (b << i)) {
			count++;
		}
	}

	if (count%2==0) {
		return false;

	} else {
		return true;
	}
}

void AS5047PReader::print_status() {
	I2CSPIDriverBase::print_status();
	PX4_INFO("UART RX bytes: %d freq %.1f", (int) _bytes_rx, (double)real_time_freq);
	PX4_INFO("AS5047P: valid %d angle %4.1fdeg turns %d rpm %4.1f", ecoder_ok, (double) (real_time_angle*M_RAD_TO_DEG_F), (int) last_multi_turn, (double)real_time_rpm);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_process_perf);
}

int AS5047PReader::probe() {
    return PX4_OK;
}
