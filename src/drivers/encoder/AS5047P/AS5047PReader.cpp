#include "AS5047PReader.h"
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <px4_platform_common/log.h>

#define BIT_MODITY(src, i, val) ((src) ^= (-(val) ^ (src)) & (1UL << (i)))
#define BIT_READ(src, i) (((src) >> (i)&1U))
#define BIT_TOGGLE(src, i) ((src) ^= 1UL << (i))

/* Volatile register address. */
#define AS5047P_NOP ((uint16_t)0x0000)
#define AS5047P_ERRFL ((uint16_t)0x0001)
#define AS5047P_PROG ((uint16_t)0x0003)
#define AS5047P_DIAAGC ((uint16_t)0x3FFC)
#define AS5047P_MAG ((uint16_t)0x3FFD)
#define AS5047P_ANGLEUNC ((uint16_t)0x3FFE)
#define AS5047P_ANGLECOM ((uint16_t)0x3FFF)

/* Non-Volatile register address. */
#define AS5047P_ZPOSM ((uint16_t)0x0016)
#define AS5047P_ZPOSL ((uint16_t)0x0017)
#define AS5047P_SETTINGS1 ((uint16_t)0x0018)
#define AS5047P_SETTINGS2 ((uint16_t)0x0019)

#define AS5047P_STEEINGS1_DEFAULT ((uint8_t)0x01)
#define AS5047P_STEEINGS2_DEFAULT ((uint8_t)0x00)

#define OP_WRITE ((uint8_t)0)
#define OP_READ ((uint8_t)1)

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
    as5047p_config(AS5047P_STEEINGS1_DEFAULT, AS5047P_STEEINGS2_DEFAULT);
    // reset();
    // start();
    return PX4_OK;
}


void AS5047PReader::start() {
	ScheduleOnInterval(_current_update_interval);
}

void AS5047PReader::reset() {
    as5047p_config(AS5047P_STEEINGS1_DEFAULT, AS5047P_STEEINGS2_DEFAULT);
}

int AS5047PReader::probe()
{
  return PX4_OK;
}

void
AS5047PReader::RunImpl()
{
  perf_begin(_cycle_perf);
  float angle = 0.0f;
  int8_t ret = as5047p_get_angle(static_cast<as5047p_daec_t>(!enable_daec), &angle);
  if (ret)
  {
    // Publish the angle data.
    data.timestamp = hrt_absolute_time();
    data.motor_abs_angle = angle;
    _encoder_pub.publish(data);
    // print status
    PX4_INFO("AS5047P: valid %d angle %4.1fdeg turns %d rpm %4.1f", ecoder_ok, (double) (real_time_angle*M_RAD_TO_DEG_F), (int) last_multi_turn, (double)real_time_rpm);
  }
  perf_end(_cycle_perf);
}

/**
 * @brief Reset.
 *
 * @param as5047p_handle
 */
void AS5047PReader::as5047p_reset()
{
  as5047p_config(AS5047P_STEEINGS1_DEFAULT, AS5047P_STEEINGS2_DEFAULT);
  as5047p_set_zero(0);
}

void AS5047PReader::as5047p_config(uint8_t settings1,
                    uint8_t settings2)
{
  /* SETTINGS1 bit 0 --> Factory Setting: Pre-Programmed to 1. */
  BIT_MODITY(settings1, 0, 1);

  /* SETTINGS1 bit 1 --> Not Used: Pre-Programmed to 0, must not be overwritten. */
  BIT_MODITY(settings1, 1, 0);

  as5047p_send_data(AS5047P_SETTINGS1, (uint16_t)(settings1 & 0x00FF));
  as5047p_send_data(AS5047P_SETTINGS2, (uint16_t)(settings2 & 0x00FF));
}


uint16_t AS5047PReader::as5047p_get_error_status()
{
  return as5047p_read_data(AS5047P_ERRFL);
}


int8_t AS5047PReader::as5047p_get_position(as5047p_daec_t with_daec,
                            uint16_t *position)
{
  uint16_t address;
  if (with_daec)
  {
    /* Measured angle WITH dynamic angle error compensation(DAEC). */
    address = AS5047P_ANGLECOM;
  }
  else
  {
    /* Measured angle WITHOUT dynamic angle error compensation(DAEC). */
    address = AS5047P_ANGLEUNC;
  }
 PX4_INFO("as5047p_get_position address %d", (int)address);
  uint16_t data_ = as5047p_read_data(address);
  if (BIT_READ(data_, 14) == 0)
  {
    *position = data_ & 0x3FFF;
    return 0; /* No error occurred. */
  }
  return -1; /* Error occurred. */
}

int8_t AS5047PReader::as5047p_get_angle(as5047p_daec_t with_daec, float *angle_degree)
{
    uint16_t raw_position;
    int8_t error = as5047p_get_position(with_daec, &raw_position);
    if (error == 0)
    {
        /* Angle in degree = value * ( 360 / 2^14). */
        *angle_degree = raw_position * (360.0 / 0x4000);
    }
    PX4_INFO("Raw position %d error %d: angle %f", (int) raw_position, error, (double)*angle_degree);
    return error;
}

void AS5047PReader::as5047p_set_zero(uint16_t position)
{
  /* 8 most significant bits of the zero position. */
  as5047p_send_data(AS5047P_ZPOSM, ((position >> 6) & 0x00FF));

  /* 6 least significant bits of the zero position. */
  as5047p_send_data(AS5047P_ZPOSL, (position & 0x003F));

  as5047p_nop();
}

inline void AS5047PReader::as5047p_nop()
{
  /* Reading the NOP register is equivalent to a nop (no operation) instruction. */
  send_command(AS5047P_NOP, OP_READ);
}

uint16_t AS5047PReader::send_command(uint16_t address, uint8_t op_read_write)
{
  uint16_t frame = address & 0x3FFF;

  PX4_INFO("address %d op_read_write %d frame %d", (int)address, (int)op_read_write, (int)frame);

  /* R/W: 0 for write, 1 for read. */
  BIT_MODITY(frame, 14, op_read_write);
  PX4_INFO("Framed modity %d", (int)frame);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_even_parity(frame))
  {
    PX4_INFO("Parity bit(even) calculated on the lower 15 bits.");
    BIT_TOGGLE(frame, 15);
    PX4_INFO("BIT_TOGGLE %d", (int)frame);
  }

  return as5047p_spi_transmit(frame);
}

void AS5047PReader::as5047p_send_data(uint16_t address, uint16_t data_)
{
  uint16_t frame = data_ & 0x3FFF;

  /* Data frame bit 14 always low(0). */
  BIT_MODITY(frame, 14, 0);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_even_parity(frame))
  {
    BIT_TOGGLE(frame, 15);
  }

  send_command(address, OP_WRITE);
  as5047p_spi_transmit(frame);
}

uint16_t AS5047PReader::as5047p_read_data(uint16_t address)
{
  return send_command(address, OP_READ);
}

inline uint16_t AS5047PReader::as5047p_spi_transmit(uint16_t data_)
{
    uint8_t cmd [2] {
        static_cast<uint8_t>(((uint16_t)data_ >> 0) & 0xFF),  // shift by 0 not needed, of course, just stylistic
        static_cast<uint8_t>(((uint16_t)data_ >> 8) & 0xFF),
    };
    PX4_INFO("data_ %d SPI: %02x %02x", (int)data_, cmd[0], cmd[1]);
    transfer(cmd, cmd, sizeof(cmd));
    PX4_INFO("SPI Recv: %02x %02x", cmd[0], cmd[1]);
    return (uint16_t)(cmd[1] << 8) | cmd[0];
}

uint8_t AS5047PReader::is_even_parity(uint16_t data)
{
  uint8_t shift = 1;
  while (shift < (sizeof(data) * 8))
  {
    data ^= (data >> shift);
    shift <<= 1;
  }
  return !(data & 0x1);
}

void AS5047PReader::print_status() {
	I2CSPIDriverBase::print_status();
	PX4_INFO("UART RX bytes: %d freq %.1f", (int) _bytes_rx, (double)real_time_freq);
	PX4_INFO("AS5047P: valid %d angle %4.1fdeg turns %d rpm %4.1f", ecoder_ok, (double) (real_time_angle*M_RAD_TO_DEG_F), (int) last_multi_turn, (double)real_time_rpm);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_process_perf);
}
