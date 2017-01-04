/*
 * SnatchPaser.h
 *
 *  Created on: 17 Dec 2016
 *      Author: tp
 */

#ifndef SNATCH_SRC_SNATCHPASER_H_
#define SNATCH_SRC_SNATCHPASER_H_

#include <snatch/SnatchParserEvent.h>
#include <string>

namespace snatch {

class SnatchParserListener {
public:
	virtual void handleParserEvent(const snatch_imu_event_t * const event) = 0;
	virtual void handleParserEvent(const snatch_rx_event_t * const event) = 0;
	virtual void handleParserEvent(const snatch_status_event_t * const event) = 0;
	virtual ~SnatchParserListener() {};
};

class SnatchParser {
public:
	SnatchParser(SnatchParserListener *listener);

	void parse(uint8_t byte);
private:
	SnatchParserListener *listener_;

	uint8_t last_byte_;

	snatch_imu_event_t imu_event_;
	snatch_rx_event_t rx_event_;
	snatch_status_event_t status_event_;

	uint8_t *event_as_data_;
	int32_t data_size_;
	int32_t data_index_;

	uint16_t checksum_;
	bool checksum_high_not_low_;

	uint16_t checksum_read_;

	bool fillData(uint8_t byte);

	bool checkChecksum(uint8_t byte);
};

}

#endif /* SNATCH_SRC_SNATCHPASER_H_ */
