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

	bool (SnatchParser::*current_event_parser_)(uint8_t);

	snatch_imu_event_t imu_event_;
	bool imuEventParser(uint8_t byte);

	snatch_rx_event_t rx_event_;
	bool rxEventParser(uint8_t byte);

	snatch_status_event_t status_event_;
	bool statusEventParser(uint8_t byte);

	uint8_t field_index_;
	uint8_t field_byte_index_;

	uint16_t checksum_;
	bool checksum_low_high_;
	void addToCheckSum(uint8_t byte);

	uint16_t checksum_read_;

	bool parseInt(uint8_t bytesPerInt, uint8_t byte, uint8_t *value, uint8_t elementCount = 1);
};

}

#endif /* SNATCH_SRC_SNATCHPASER_H_ */
