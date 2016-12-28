/**
 *
 *  Copied from BYU fcu_io
 * \file mavlink_serial.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef SNATCH_SERIAL_H
#define SNATCH_SERIAL_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <snatch/SnatchParser.h>
#include <snatch/SnatchSerialException.h>
#include <list>
#include <string>
#include <vector>

#include <cstdint>

#define SNATCH_NUMBER_OF_CHANNELS 16
#define SNATCH_READ_BUFFER_SIZE 1024
#define SNATCH_WRITE_BUFFER_SIZE 4096

namespace snatch {

class SnatchSerial {
public:
	SnatchSerial(std::string port, int baud_rate, SnatchParser * parser);

	~SnatchSerial();

	void sendCommandChar(const uint8_t command);

	void setChannelValue(const uint8_t channel, const uint16_t value);

	void sendChannelValues();

private:
	/**
	 * \brief Convenience typedef for mutex lock
	 */
	typedef boost::lock_guard<boost::recursive_mutex> mutex_lock;

	void do_async_read();

	/**
	 * \brief Handler for end of asynchronous read operation
	 * \param error Error code
	 * \param bytes_transferred Number of bytes received
	 */
	void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);

	/**
	 * \brief Initialize an asynchronous write operation
	 * \param check_write_state If true, only start another write operation if a write sequence is not already running
	 */
	void do_async_write(bool check_write_state);

	/**
	 * \brief Handler for end of asynchronous write operation
	 * \param error Error code
	 * \param bytes_transferred Number of bytes sent
	 */
	void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

	/**
	 * \brief Stops communication and closes the serial port
	 */
	void close();

	boost::asio::io_service io_service_; //!< boost io service provider
	boost::asio::serial_port serial_port_; //!< boost serial port object
	boost::thread io_thread_; //!< thread on which the io service runs

	uint8_t channel_values_[SNATCH_NUMBER_OF_CHANNELS * 4];

	uint8_t read_buffer_[SNATCH_READ_BUFFER_SIZE];

	// Double buffers for write
	uint8_t write_buffer_a_[SNATCH_WRITE_BUFFER_SIZE];
	uint8_t write_buffer_b_[SNATCH_WRITE_BUFFER_SIZE];
	uint8_t *write_buffer_ = write_buffer_a_;
	uint8_t *write_buffer_out_ = write_buffer_b_;
	size_t write_buffer_index_;
	boost::recursive_mutex write_mutex_;
	bool write_in_progress_; //!< flag for whether async_write is already running

	bool writeBufferPut(const uint8_t b);

	SnatchParser *snatchParser_;

	uint8_t sysid_;
	uint8_t compid_;
};

} // namespace snatch

#endif // SNATCH_SERIAL_H
