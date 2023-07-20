
#pragma once

#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/sensor_motor_encoder.h>
#include <uORB/Publication.hpp>
#include <drivers/device/spi.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

#define ECODER_BUFFER_SIZE 64
#define ECODER_WRITE_SIZE 16
#define CRC_TAB_SIZE 256
#define ECODER_RES_FRAME_LEN 11

typedef enum
{
without_daec = 0,
with_daec = !without_daec
} as5047p_daec_t;


class AS5047PReader : public device::SPI, public I2CSPIDriver<AS5047PReader>{
	perf_counter_t	_cycle_perf{0};
	perf_counter_t	_process_perf{0};
	bool _initialized{false};
	uORB::Publication<sensor_motor_encoder_s>	_encoder_pub{ORB_ID(sensor_motor_encoder)};			/**< rate setpoint publication */
	uint64_t last_ask_time{0};
	uint64_t first_read_time{0};
	int32_t num_msgs{0};
	float real_time_freq {0};
	int32_t last_multi_turn{0};
	uint64_t last_multi_turn_time{0};

	uint8_t ecoder_ok {0};
	float real_time_angle {0};
	float real_time_rpm {0};
	uint32_t _bytes_rx {0};
	sensor_motor_encoder_s data;

	void ask();
	int process_data();

	uint16_t send_command(uint16_t address, uint8_t op_read_write);

	/**
	 * @brief Sending data to register.
	 *
	 * @param address Register address.
	 * @param data Data.
	 */
	void as5047p_send_data(uint16_t address, uint16_t data);

	/**
	 * @brief Reading data from register.
	 *
	 * @param address Register address.
	 * @return Data.
	 */
	uint16_t as5047p_read_data(uint16_t address);


	void as5047p_reset();

	/**
	 * @brief Setup AS5047P.
	 *
	 * @param as5047p_handle AS5047P handle.
	 * @param settings1 Config 1.
	 * @param settings2 Config 2.
	 */
	void as5047p_config(uint8_t settings1, uint8_t settings2);

	/**
	 * @brief Reading error flags.
	 *
	 * @return Error flags. 0 for no error occurred.
	 */
	uint16_t as5047p_get_error_status();

	/**
	 * @brief Read current position.
	 *
	 * @param with_daec With or without dynamic angle error compensation (DAEC).
	 * @param position Current position raw value.
	 * @return Status code.
	 *         0: Success.
	 *         -1: Error occurred.
	 */
	int8_t as5047p_get_position(as5047p_daec_t with_daec,
                            uint16_t *position);

	/**
	 * @brief Set specify position as zero.
	 *
	 * @param position Position raw value.
	 */
	void as5047p_set_zero(uint16_t position);

	/**
	 * @brief No operation instruction.
	 *
	 */
	void as5047p_nop();

	/**
	 * @brief Start SPI transmit.
	 *
	 * @param data Data.
	 */
	uint16_t as5047p_spi_transmit(uint16_t data);

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

	/**
	 * @brief Check data even parity.
	 */
	static uint8_t is_even_parity(uint16_t data);


	/**
	 * @brief Read current angle in degree.
	 *
	 * @param with_daec With or without dynamic angle error compensation (DAEC).
	 * @param angle_degree Current angle in degree.
	 * @return Status code.
	 *         0: Success.
	 *         -1: Error occurred.
	 */
	int8_t as5047p_get_angle(as5047p_daec_t with_daec, float *angle_degree);

private:
	static constexpr unsigned	_current_update_interval{500}; // 2KHz
	int32_t enable_daec = false;

};

