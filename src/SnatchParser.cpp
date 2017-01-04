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
		listener_(listener), last_byte_('\n'), event_as_data_(0), data_size_(0), data_index_(0), checksum_(0), checksum_high_not_low_(false), checksum_read_(0) {
}

void SnatchParser::parse(uint8_t byte) {
	if (event_as_data_) {
		if (fillData(byte)) {
			last_byte_ = byte;
			return;
		}

		if (event_as_data_ == (uint8_t *) &imu_event_ && checksum_read_ == checksum_) {
			listener_->handleParserEvent(&imu_event_);
		} else if (event_as_data_ == (uint8_t *) &rx_event_ && checksum_read_ == checksum_) {
			listener_->handleParserEvent(&rx_event_);
		} else if (event_as_data_ == (uint8_t *) &status_event_ && checksum_read_ == checksum_) {
			listener_->handleParserEvent(&status_event_);
		}
		// Reset on event parsed or error.
		event_as_data_ = 0;
	}

	// Looks for a NL followed by a event header byte. This can fall out of sync with the events!?
	if (last_byte_ == '\n') {
		switch (byte) {
		case 'I':
			event_as_data_ = (uint8_t *) &imu_event_;
			data_size_ = sizeof(imu_event_);
			break;
		case 'R':
			event_as_data_ = (uint8_t *) &rx_event_;
			data_size_ = sizeof(rx_event_);
			break;
		case 'S':
			event_as_data_ = (uint8_t *) &status_event_;
			data_size_ = sizeof(status_event_);
			break;
		default:
			// Not a start of event.
			last_byte_ = byte;
			return;
		}
		data_index_ = 0;

		checksum_ = byte;  // The event type char is included in the checksum.
		checksum_high_not_low_ = true;
	}

	last_byte_ = byte;
}

bool SnatchParser::fillData(uint8_t byte) {
	if (data_index_ < data_size_) {
		event_as_data_[data_index_++] = byte;
		checksum_ += (byte << (checksum_high_not_low_ ? 8 : 0));
		checksum_high_not_low_ = !checksum_high_not_low_;
		return true;
	}
	if (data_index_ == data_size_) {
		// First checksum byte
		data_index_++;
		checksum_read_ = byte;
		return true;
	}
	// Second checksum byte
	checksum_read_ += byte << 8;

	data_index_ = 0;
	checksum_high_not_low_ = true;
	return false;
}

}
