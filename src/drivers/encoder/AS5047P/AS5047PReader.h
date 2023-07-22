
#pragma once

#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/sensor_motor_encoder.h>
#include <uORB/Publication.hpp>
#include <drivers/device/spi.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

// Volatile Registers Addresses
#define NOP_REG			0x0000
#define ERRFL_REG 		0x0001
#define PROG_REG		0x0003
#define DIAGAGC_REG 	0x3FFC
#define MAG_REG 		0x3FFD
#define ANGLE_REG 		0x3FFE
#define ANGLECOM_REG 	0x3FFF

// Non-Volatile Registers Addresses
#define ZPOSM_REG 		0x0016
#define ZPOSL_REG 		0x0017
#define SETTINGS1_REG 	0x0018
#define SETTINGS2_REG 	0x0019

#define WRITE			0
#define READ			1

// ERRFL Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t frerr:1;
        uint16_t invcomm:1;
        uint16_t parerr:1;
        uint16_t unused:13;
    } values;
} Errfl;


// PROG Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
    	uint16_t progen:1;
    	uint16_t unused:1;
    	uint16_t otpref:1;
    	uint16_t progotp:1;
        uint16_t unused1:2;
        uint16_t progver:1;
        uint16_t unused2:9;
    } values;
} Prog;

// DIAAGC Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t agc:8;
        uint16_t lf:1;
        uint16_t cof:1;
        uint16_t magh:1;
        uint16_t magl:1;
        uint16_t unused:4;
    } values;
} Diaagc;

// MAG Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t cmag:14;
        uint16_t unused:2;
    } values;
} Mag;

// ANGLE Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t cordicang:14;
        uint16_t unused:2;
    } values;
} Angle;

// ANGLECOM Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t daecang:14;
        uint16_t unused:2;
    } values;
} Anglecom;


// ZPOSM Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t zposm;
    } values;
} Zposm;

// ZPOSL Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t zposl:6;
        uint8_t compLerrorEn:1;
        uint8_t compHerrorEn:1;
    } values;
} Zposl;

// SETTINGS1 Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t factorySetting:1;
        uint8_t noiseset:1;
        uint8_t dir:1;
        uint8_t uvw_abi:1;
        uint8_t daecdis:1;
        uint8_t abibin:1;
        uint8_t dataselect:1;
        uint8_t pwmon:1;
    } values;
} Settings1;

// SETTINGS2 Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t uvwpp:3;
    	uint8_t hys:2;
    	uint8_t abires:3;
    } values;
} Settings2;


// Command Frame  Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t commandFrame:14;
        uint16_t rw:1;
        uint16_t parc:1;
    } values;
} CommandFrame;

// ReadData Frame  Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t data:14;
        uint16_t ef:1;
        uint16_t pard:1;
    } values;
} ReadDataFrame;

// WriteData Frame  Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t data:14;
        uint16_t low:1;
        uint16_t pard:1;
    } values;
} WriteDataFrame;

class AS5047PReader : public device::SPI, public I2CSPIDriver<AS5047PReader>{
	perf_counter_t	_cycle_perf{0};
	bool _initialized{false};
	uORB::Publication<sensor_motor_encoder_s>	_encoder_pub{ORB_ID(sensor_motor_encoder)};			/**< rate setpoint publication */
    bool _is_print_debug {false};
	uint64_t last_ask_time{0};
	uint64_t first_read_time{0};
	int32_t num_msgs{0};
	float real_time_freq {0};
	int32_t last_multi_turn{0};
	uint64_t last_angle_read_time{0};

	uint8_t ecoder_ok {0};
	float real_time_angle {0};
	float real_time_rpm {0};
	uint32_t _bytes_rx {0};
    uint8_t motor_id {0};
	sensor_motor_encoder_s data {};

	void reset();
public:
	void start();
	static void print_usage();
	int init() override;
	int		probe() override;
	AS5047PReader(const I2CSPIDriverConfig &config);
	virtual ~AS5047PReader();
	void print_status() override;
	void RunImpl();

	ReadDataFrame readRegister(uint16_t registerAddress);
	void writeRegister(uint16_t registerAddress, uint16_t registerValue);
	float readAngle();
	void writeSettings1(Settings1 values);
	void writeSettings2(Settings2 values);
	void writeZeroPosition(Zposm zposm, Zposl zposl);
	void printDebugString();

private:
	static constexpr unsigned	_current_update_interval{500}; // 2KHz
	int32_t enable_daec = false;
	bool isEven(uint16_t data);
	uint16_t readData(uint16_t command, uint16_t nopCommand);
	void writeData(uint16_t command, uint16_t value);

};

