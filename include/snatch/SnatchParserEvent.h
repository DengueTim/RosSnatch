#ifndef SNATCH_EVENTS_H
#define SNATCH_EVENTS_H

#include <cstdint>

namespace snatch {

typedef struct __snatch_imu_event_t {
	uint32_t time;

	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;

	uint16_t accX;
	uint16_t accY;
	uint16_t accZ;

	uint16_t gyroX;
	uint16_t gyroY;
	uint16_t gyroZ;

	uint16_t magX;
	uint16_t magY;
	uint16_t magZ;
} snatch_imu_event_t;

const int RxChannelCount = 16;

typedef struct __snatch_rx_event_t {
	uint32_t time;
	uint16_t channels[RxChannelCount];
} snatch_rx_event_t;

typedef struct __snatch_status_event_t {
	uint32_t time;
	uint16_t channels[RxChannelCount];
} snatch_status_event_t;

}
#endif
