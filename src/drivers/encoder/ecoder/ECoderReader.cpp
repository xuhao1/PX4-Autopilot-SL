#include "ECoderReader.h"
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <px4_platform_common/log.h>
#define ECODER_BITRATE 2500000

static uint8_t commands[] {
	0x02, //Read single turn position
	0x8A, ///Read multi turn position
	0x92, //Read precision
	0x1A //Read single multi precision
};

uint8_t CRC_C(uint8_t *CRCbuf, uint8_t* CRC_8X1,uint8_t Length);
const px4::wq_config_t getEncoderWqConfig(int reader_id) {
	if (reader_id == 0) {
		return px4::wq_configurations::motor_encoder0;
	} else {
		return px4::wq_configurations::motor_encoder1;
	}
}

ECoderReader::ECoderReader(const char * module_name, int reader_id, const char *device):
	ScheduledWorkItem(module_name, getEncoderWqConfig(reader_id)),
	_reader_id(reader_id),
	_cycle_perf(perf_alloc(PC_INTERVAL, module_name)),
	_process_perf(perf_alloc(PC_ELAPSED, "Ecoder::ProcessData")) {
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
	CRC_8X1_TAB_Creat(CRC8X1);
}

ECoderReader::~ECoderReader() {
	close(_rcs_fd);
	perf_free(_cycle_perf);
	perf_free(_process_perf);
}

int ECoderReader::init() {
	PX4_INFO("Try to open %s", _device);
	_rcs_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	struct termios t;
	tcgetattr(_rcs_fd, &t);
	t.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
	t.c_cflag |= (CS8);
	cfsetspeed(&t, ECODER_BITRATE);
	tcsetattr(_rcs_fd, TCSANOW, &t);

	int termios_state = tcsetattr(_rcs_fd, TCSANOW, &t);
	if (termios_state < 0) {
		PX4_ERR("ECoder %d open device %s for ecoder with 2.5Mbps failed", _reader_id, _device);
		return PX4_ERROR;
	} else {
		_initialized = true;
		PX4_INFO("ECoder %d succ opened device %s for ecoder with 2.5Mbps", _reader_id, _device);
		return PX4_OK;
	}
}

void ECoderReader::ask() {
	::write(_rcs_fd, &commands[3], 1);
	last_ask_time = hrt_absolute_time();
}

int ECoderReader::process_data() {
	perf_begin(_process_perf);
	uint8_t crc = CRC_C(dataBuf, CRC8X1, ECODER_RES_FRAME_LEN);
	if (crc != 0) {
		return PX4_ERROR;
	}
	int32_t single_Turn = dataBuf[2] | dataBuf[3] << 8
		| dataBuf[4] << 16;
	float resolution = (float) (1<<dataBuf[5]);
	real_time_angle = (float)single_Turn/resolution*M_TWOPI_F;
	int32_t multi_Turn =   dataBuf[6] | dataBuf[7] << 8
		| dataBuf[8] << 16;
	float rpm = 0;
	bool rpm_updated = false;
	if (multi_Turn != last_multi_turn) {
		float dt = ((float)(last_ask_time - last_multi_turn_time))/1000000.0f;
		int turns = multi_Turn - last_multi_turn;
		if (abs(turns) < 5) {
			//Else is cross zero..
			rpm = ((float)turns)*60/dt;
			rpm_updated = true;
		}
		last_multi_turn_time = last_ask_time;
		last_multi_turn = multi_Turn;
	}

	if (rpm_updated) {
		real_time_rpm = rpm;
	}
	data.timestamp = last_ask_time;//
	data.motor_id = _reader_id;
	data.motor_rpm = real_time_rpm;
	data.motor_abs_angle = real_time_angle;
	// data.multi_turns = last_multi_turn;
	_encoder_pub.publish(data);
	uint64_t time = hrt_absolute_time();
	num_msgs++;
	real_time_freq = (float)num_msgs/(float)(time - first_read_time)*1000000.0f;
	if (first_read_time == 0)
		first_read_time = time;
	perf_end(_process_perf);
	return PX4_OK;
}


int ECoderReader::read_once() {
	// const hrt_abstime cycle_timestamp = hrt_absolute_time();
	int newBytes = 0;
	newBytes = ::read(_rcs_fd, &_serial_buf[0], ECODER_BUFFER_SIZE);
	if (newBytes <= 0) {
		return PX4_ERROR;
	}
	_bytes_rx += newBytes;
	// PX4_INFO_RAW("New bytes %d: ", newBytes);
	// for (int i = 0; i<newBytes; i ++) {
	// 	PX4_INFO_RAW("%d:%x ", i, _serial_buf[i]);
	// }
	// PX4_INFO_RAW("\n");
	for (int i = 0; i<newBytes; i ++) {
		if (data_buf_index < 0) {
			if (_serial_buf[i] == commands[3]) {
				data_buf_index = 0;
				dataBuf[data_buf_index++] = _serial_buf[i];
				//Then we need to check the next byte is 0x00
				continue;
			}
			continue;
		} else if (data_buf_index == 0) {
			//Ensure 0 right after returned command
			if (_serial_buf[i] != 0) {
				data_buf_index = -1;
			} else {
				dataBuf[data_buf_index++] = _serial_buf[i];
			}
			continue;
		} else if (data_buf_index > 0) {
			dataBuf[data_buf_index++] = _serial_buf[i];
			if (data_buf_index == ECODER_RES_FRAME_LEN) {
				process_data();
				data_buf_index = -1;
			}
		}
	}
	return PX4_OK;
}


void ECoderReader::Run()
{
	// if (should_exit()) {
	// 	exit_and_cleanup();
	// 	return;
	// }
	if (!_initialized) {
		if (init() == PX4_OK) {
			_initialized = true;
		}
	} else {
		perf_begin(_cycle_perf);
		ask();
		int32_t succ = read_once();
		if (succ == PX4_OK) {
			ecoder_ok = 1;
		}
		perf_end(_cycle_perf);
	}
}

void ECoderReader::print_status() {
	PX4_INFO("ECODER:%d UART device: %s", _reader_id, _device);
	PX4_INFO("UART RX bytes: %d freq %.1f", (int)_bytes_rx, (double)real_time_freq);
	PX4_INFO("ECoder %d: valid %d angle %4.1fdeg turns %d rpm %4.1f", _reader_id,
		ecoder_ok, (double) (real_time_angle*M_RAD_TO_DEG_F), (int)last_multi_turn, (double)real_time_rpm);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_process_perf);
}

uint8_t CRC_C(uint8_t *CRCbuf, uint8_t* CRC_8X1,uint8_t Length)
{
	uint8_t CRCResult=0;
	uint8_t CRCLength=0;
	while(CRCLength<Length)
	{
		CRCResult ^= CRCbuf[CRCLength];
		CRCResult = (CRCResult&0x00ff);
		CRCLength++;
		CRCResult = CRC_8X1[CRCResult];
	}
	return CRCResult;
}

void ECoderReader::CRC_8X1_TAB_Creat(uint8_t *CRC_8X1)
{
	uint16_t i,j;
	uint8_t CRCResult;
	for(j = 0;j < 256;j++)
	{
		CRCResult = j;
		for(i = 0;i < 8;i++)
		{
			if(CRCResult & 0x80)
		{
			CRCResult = (CRCResult << 1) ^ 0x01; //0x01--x^8+1
		}
		else
		{
			CRCResult <<= 1;
		}
		}
		CRC_8X1[j] = (CRCResult&0x00ff);
	}
}
