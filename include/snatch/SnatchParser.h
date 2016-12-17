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
};

}

#endif /* SNATCH_SRC_SNATCHPASER_H_ */
