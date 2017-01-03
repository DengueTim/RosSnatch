#ifndef SNATCH_EVENTS_H
#define SNATCH_EVENTS_H

#include <cstdint>

namespace snatch {

typedef struct __snatch_imu_event_t {
	uint32_t time;

	int16_t roll;
	int16_t pitch;
	int16_t yaw;

	int16_t accX;
	int16_t accY;
	int16_t accZ;

	int32_t gyroX;
	int32_t gyroY;
	int32_t gyroZ;

	int32_t magX;
	int32_t magY;
	int32_t magZ;
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
