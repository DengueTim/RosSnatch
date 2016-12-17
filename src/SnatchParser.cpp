/*
 * SnatchPaser.cpp
 *
 *  Created on: 17 Dec 2016
 *      Author: tp
 */

#include <snatch/SnatchParser.h>

namespace snatch {

SnatchParser::SnatchParser(SnatchParserListener *listener) :
		listener_(listener), current_event_parser_(NULL), field_index_(0), field_byte_index_(0), checksum_(0), checksum_low_high_(false), checksum_read_(0) {

}

void SnatchParser::parse(uint8_t byte) {
	if (current_event_parser_ != NULL) {
		if ((this->*current_event_parser_)(byte)) {
			// currentEventParser_ accepted byte. All good.
			return;
		} else {
			// Reset on event parsed or error.
			current_event_parser_ = NULL;
		}
	}

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
		// Ignore byte.
		return;
	}

	field_index_ = 0;
	field_byte_index_ = 0;
	checksum_ = 0;
	checksum_low_high_ = false;
	addToCheckSum(byte); // The event type char is included in the checksum.
}

bool SnatchParser::imuEventParser(uint8_t byte) {
	switch (field_index_) {
	case 0:
		parseInt(4, byte, (uint8_t*) &imu_event_.time);
		break;
	case 1:
		parseInt(2, byte, (uint8_t*) &imu_event_.roll);
		break;
	case 2:
		parseInt(2, byte, (uint8_t*) &imu_event_.pitch);
		break;
	case 3:
		parseInt(2, byte, (uint8_t*) &imu_event_.yaw);
		break;
	case 4:
		parseInt(2, byte, (uint8_t*) &imu_event_.accX);
		break;
	case 5:
		parseInt(2, byte, (uint8_t*) &imu_event_.accY);
		break;
	case 6:
		parseInt(2, byte, (uint8_t*) &imu_event_.accZ);
		break;
	case 7:
		parseInt(2, byte, (uint8_t*) &imu_event_.gyroX);
		break;
	case 8:
		parseInt(2, byte, (uint8_t*) &imu_event_.gyroY);
		break;
	case 9:
		parseInt(2, byte, (uint8_t*) &imu_event_.gyroZ);
		break;
	case 10:
		parseInt(2, byte, (uint8_t*) &imu_event_.magX);
		break;
	case 11:
		parseInt(2, byte, (uint8_t*) &imu_event_.magY);
		break;
	case 12:
		parseInt(2, byte, (uint8_t*) &imu_event_.magZ);
		break;
	case 13: // Checksum.
		if (parseInt(2, byte, (uint8_t*) &checksum_read_)) {
			if (checksum_ == checksum_read_) {
				listener_->handleParserEvent(&imu_event_);
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
		parseInt(4, byte, (uint8_t*) &rx_event_.time);
		break;
	case 1: // Channels
		parseInt(2, byte, (uint8_t*) &rx_event_.channels[field_byte_index_ >> 1], snatch::RxChannelCount);
		break;
	case 2: // Checksum.
		if (parseInt(2, byte, (uint8_t*) &checksum_read_)) {
			if (checksum_ == checksum_read_) {
				listener_->handleParserEvent(&rx_event_);
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
	value[i++] = byte;

	if (field_byte_index_ != bytesPerInt * elementCount) {
		return false;
	}

	field_byte_index_ = 0;
	field_index_++;
	return true;
}

}
