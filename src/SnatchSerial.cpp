/**
 * Copied from BYU fcu_io
 * \file mavrosflight.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Sources:
 * https://gist.github.com/yoggy/3323808
 */

#include <iostream>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <snatch/SnatchSerial.h>

namespace snatch {

using boost::asio::serial_port_base;

SnatchSerial::SnatchSerial(std::string port, int baud_rate, SnatchParser *parser) :
		io_service_(), serial_port_(io_service_), write_in_progress_(false), snatchParser_(parser) {
	// setup serial port
	try {
		serial_port_.open(port);
		serial_port_.set_option(serial_port_base::baud_rate(baud_rate));
		serial_port_.set_option(serial_port_base::character_size(8));
		serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
		serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
		serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
	} catch (boost::system::system_error &e) {
		throw SnatchSerialException(e);
	}

	// start reading from serial port
	do_async_read();
	std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
	io_thread_ = boost::thread(boost::bind(run, &this->io_service_));
}

SnatchSerial::~SnatchSerial() {
	close();
}

void SnatchSerial::close() {
	mutex_lock lock(write_mutex_);

	io_service_.stop();
	serial_port_.close();

	if (io_thread_.joinable()) {
		io_thread_.join();
	}
}

void SnatchSerial::do_async_read() {
	if (!serial_port_.is_open())
		return;

	serial_port_.async_read_some(boost::asio::buffer(read_buffer_, SNATCH_READ_BUFFER_SIZE),
			boost::bind(&SnatchSerial::async_read_end, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void SnatchSerial::async_read_end(const boost::system::error_code &error, size_t bytes_transferred) {
	if (!serial_port_.is_open())
		return;

	if (error) {
		close();
		return;
	}

	for (int i = 0; i < bytes_transferred; i++) {
		snatchParser_->parse(read_buffer_[i]);
	}

	do_async_read();
}

void SnatchSerial::setChannelValue(const uint8_t channel, const uint16_t value) {
	if (channel < 0x0 || channel > 0xF) {
		return;
	}

	uint8_t channel_id;

	if (channel <= 9) {
		channel_id = '0' + channel;
	} else {
		channel_id = 'A' + channel;
	}

	// Limit value between 0 to 999 inclusively
	uint16_t channel_value;
	if (value >= 1000) {
		channel_value = 999;
	} else if (value <= 0) {
		channel_value = 0;
	} else {
		channel_value = value;
	}

	uint8_t channel_index = channel_id * 4;
	// Code as 4 chars 0-F channel + 000 to 999 value
	channel_values_[channel_index] = channel_id;
	channel_values_[channel_index + 3] = '0' + (channel_value % 10);
	channel_value /= 10;
	channel_values_[channel_index + 2] = '0' + (channel_value % 10);
	channel_value /= 10;
	channel_values_[channel_index + 1] = '0' + (channel_value % 10);
}

bool SnatchSerial::writeBufferPut(const uint8_t b) {
	if (write_buffer_index_ >= SNATCH_WRITE_BUFFER_SIZE) {
		// Full.
		return false;
	}

	write_buffer_[write_buffer_index_++] = b;
	return true;
}

void SnatchSerial::sendCommandChar(const uint8_t command) {
	{
		mutex_lock lock(write_mutex_);
		writeBufferPut(command);
	}

	do_async_write(true);
}

void SnatchSerial::sendChannelValues() {
	{
		mutex_lock lock(write_mutex_);
		for (uint8_t i = 0 ; i < (SNATCH_NUMBER_OF_CHANNELS * 4) ; i++) {
			writeBufferPut(channel_values_[i]);
		}
	}
	do_async_write(true);
}

void SnatchSerial::do_async_write(bool check_write_state) {
	if (check_write_state && write_in_progress_)
		return;

	mutex_lock lock(write_mutex_);
	if (write_buffer_index_ == 0)
		return;

	write_in_progress_ = true;

	// Switch buffers and write to serial
	uint8_t *t = write_buffer_out_;
	write_buffer_out_ = write_buffer_;
	write_buffer_ = t;

	boost::asio::async_write(serial_port_, boost::asio::buffer(write_buffer_out_, write_buffer_index_),
			boost::bind(&SnatchSerial::async_write_end, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

	write_buffer_index_ = 0;
}

void SnatchSerial::async_write_end(const boost::system::error_code &error, std::size_t bytes_transferred) {
	if (error) {
		close();
		return;
	}

	// Is this lock/check needed? Could clear write_in_progress_ in do_async_write
	mutex_lock lock(write_mutex_);
	if (write_buffer_index_) {
		do_async_write(false);
	} else {
		write_in_progress_ = false;
	}
}

} // namespace snatch
