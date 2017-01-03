/*
 * SnatchPaser.cpp
 *
 *  Created on: 17 Dec 2016
 *      Author: tp
 */

#include <iostream>

#include <snatch/SnatchParser.h>

namespace snatch {

SnatchParser::SnatchParser(SnatchParserListener *listener) :
		listener_(listener), current_event_parser_(NULL), field_index_(0), field_byte_index_(0), checksum_(0), checksum_low_high_(false), checksum_read_(0) {

}

void SnatchParser::parse(uint8_t byte) {
	if (current_event_parser_ != NULL) {
		if ((this->*current_event_parser_)(byte)) {
			// currentEventParser_ accepted byte. All good.
			last_byte_ = byte;
			return;
		} else {
			// Reset on event parsed or error.
			current_event_parser_ = NULL;
		}
	}

	// Looks for a NL followed by a event header byte. This can fall out of sync with the events!?
	if (last_byte_ == '\n') {
		switch (byte) {
		case 'I':
			current_event_parser_ = &SnatchParser::imuEventParser;
			break;
		case 'R':
			current_event_parser_ = &SnatchParser::rxEventParser;
			break;
		case 'S':
			current_event_parser_ = &SnatchParser::statusEventParser;
			break;
		default:
			// Not a start of event.
			last_byte_ = byte;
			return;
		}

		field_index_ = 0;
		field_byte_index_ = 0;
		checksum_ = 0;
		checksum_low_high_ = false;
		addToCheckSum(byte); // The event type char is included in the checksum.
	}

	last_byte_ = byte;
}

bool SnatchParser::imuEventParser(uint8_t byte) {
	switch (field_index_) {
	case 0:
		if (parseInt(4, byte, (uint8_t*) &imu_event_.time)) {
//			std::cerr << "IMU Time:" << imu_event_.time;
		}
		break;
	case 1:
		parseInt(2, byte, (uint8_t*) &imu_event_.roll);
		break;
	case 2:
		parseInt(2, byte, (uint8_t*) &imu_event_.pitch);
		break;
	case 3:
		if (parseInt(2, byte, (uint8_t*) &imu_event_.yaw)) {
//			std::cerr << " Roll:" << imu_event_.roll << " Pitch:" << imu_event_.pitch << " Yaw:" << imu_event_.yaw << "\t";
		}
		break;
	case 4:
		parseInt(2, byte, (uint8_t*) &imu_event_.accX);
		break;
	case 5:
		parseInt(2, byte, (uint8_t*) &imu_event_.accY);
		break;
	case 6:
		if (parseInt(2, byte, (uint8_t*) &imu_event_.accZ)) {
//			std::cerr << "A(" << imu_event_.accX << "\t" << imu_event_.accY << "\t" << imu_event_.accZ << ")\t";
		}
		break;
	case 7:
		parseInt(4, byte, (uint8_t*) &imu_event_.gyroX);
		break;
	case 8:
		parseInt(4, byte, (uint8_t*) &imu_event_.gyroY);
		break;
	case 9:
		if (parseInt(4, byte, (uint8_t*) &imu_event_.gyroZ)) {
//			std::cerr << " G(" << imu_event_.gyroX << "\t" << imu_event_.gyroY << "\t" << imu_event_.gyroZ << ")\t";
		}
		break;
	case 10:
		parseInt(4, byte, (uint8_t*) &imu_event_.magX);
		break;
	case 11:
		parseInt(4, byte, (uint8_t*) &imu_event_.magY);
		break;
	case 12:
		if (parseInt(4, byte, (uint8_t*) &imu_event_.magZ)) {
//			std::cerr << "M(" << imu_event_.magX << "\t" << imu_event_.magY << "\t" << imu_event_.magZ << ")\n";
		}
		break;
	case 13: // Checksum.
		if (parseInt(2, byte, (uint8_t*) &checksum_read_)) {
			if (checksum_ == checksum_read_) {
				listener_->handleParserEvent(&imu_event_);
			} else {
				std::cerr << " IMU checksum error! computed:" << std::hex << checksum_ << " read:" << checksum_read_ << std::dec << "\n";
			}
			return false;
		}
		return true;
	default:
		return false;
	}
	addToCheckSum(byte);
	return true;
}

bool SnatchParser::rxEventParser(uint8_t byte) {
	switch (field_index_) {
	case 0: // Time
		if (parseInt(4, byte, (uint8_t*) &rx_event_.time)) {
//			std::cerr << "RX  Time:" << rx_event_.time;
		}
		break;
	case 1: // Channels
		if (parseInt(2, byte, (uint8_t*) &rx_event_.channels[field_byte_index_ >> 1], snatch::RxChannelCount)) {
//			std::cerr << " Channel values:";
//			std::cerr << rx_event_.channels[0] << ":";
//			std::cerr << rx_event_.channels[1] << ":";
//			std::cerr << rx_event_.channels[2] << ":";
//			std::cerr << rx_event_.channels[3] << ":";
//			std::cerr << rx_event_.channels[4] << ":";
//			std::cerr << rx_event_.channels[5] << ":";
//			std::cerr << rx_event_.channels[6] << ":";
//			std::cerr << rx_event_.channels[7] << ":";
//			std::cerr << rx_event_.channels[8] << ":";
//			std::cerr << rx_event_.channels[9] << ":";
//			std::cerr << rx_event_.channels[10] << ":";
//			std::cerr << rx_event_.channels[11] << ":";
//			std::cerr << rx_event_.channels[12] << ":";
//			std::cerr << rx_event_.channels[13] << ":";
//			std::cerr << rx_event_.channels[14] << ":";
//			std::cerr << rx_event_.channels[15] << "\n";
		}
		break;
	case 2: // Checksum.
		if (parseInt(2, byte, (uint8_t*) &checksum_read_)) {
			if (checksum_ == checksum_read_) {
				listener_->handleParserEvent(&rx_event_);
			} else {
				std::cerr << " RX checksum error! computed:" << std::hex << checksum_ << " read:" << checksum_read_ << std::dec << "\n";
			}
			return false;
		}
		return true;
	default:
		return false;
	}
	addToCheckSum(byte);
	return true;
}

bool SnatchParser::statusEventParser(uint8_t byte) {
	switch (field_index_) {
	case 0: // Time
		parseInt(4, byte, (uint8_t*) &status_event_.time);
		break;
	case 1: // Channels
		parseInt(2, byte, (uint8_t*) &status_event_.channels[field_byte_index_ >> 1], snatch::RxChannelCount);
		break;
	case 2: // Checksum.
		if (parseInt(2, byte, (uint8_t*) &checksum_read_)) {
			if (checksum_ == checksum_read_) {
				listener_->handleParserEvent(&status_event_);
			}
			return false;
		}
		return true;
	default:
		return false;
	}
	addToCheckSum(byte);
	return true;
}

void SnatchParser::addToCheckSum(uint8_t byte) {
	checksum_ += (byte << (checksum_low_high_ ? 0 : 8));
	checksum_low_high_ = !checksum_low_high_;
}

// Reads one or more ints into give value address.
bool SnatchParser::parseInt(uint8_t bytesPerInt, uint8_t byte, uint8_t *value, uint8_t elementCount) {
	uint8_t i = field_byte_index_++ % bytesPerInt;
	value[i] = byte;

	if (field_byte_index_ < bytesPerInt * elementCount) {
		return false;
	}

	field_byte_index_ = 0;
	field_index_++;
	return true;
}

}
