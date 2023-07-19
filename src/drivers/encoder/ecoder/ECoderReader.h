
#pragma once

#include <float.h>
#include <stdint.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/sensor_motor_encoder.h>
#include <uORB/Publication.hpp>

#define ECODER_BUFFER_SIZE 64
#define ECODER_WRITE_SIZE 16
#define CRC_TAB_SIZE 256
#define ECODER_RES_FRAME_LEN 11

class ECoderReader : public px4::ScheduledWorkItem {
	int		_rcs_fd{-1};
	char		_device[20] {};
	uint8_t _serial_buf[ECODER_BUFFER_SIZE] {};
	uint8_t _serial_write_buf[ECODER_WRITE_SIZE] {};
	int _reader_id;
	uint8_t CRC8X1[CRC_TAB_SIZE];
	void Run() override;
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
	uint8_t dataBuf[16] {0};
	int8_t data_buf_index = -1;


	int read_once();
	void ask();
	int process_data();
public:
	int init();
	ECoderReader(const char * module_name, int reader_id, const char *device);
	virtual ~ECoderReader();
	void print_status();
	static void CRC_8X1_TAB_Creat(uint8_t *CRC_8X1);
};

