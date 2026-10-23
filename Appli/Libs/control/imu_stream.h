#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <stdbool.h>
#include "tx_api.h"
#include "stm32n6xx_hal.h"

#define IMU_STREAM_STACK_SIZE 2048U
#define IMU_STREAM_PRIO       9U /* Sample before USB application work (priority 10). */
#define IMU_PACKET_SIZE       76U
/* Queue records omit the wire header and fit ThreadX's 16-word limit. */
#define IMU_PACKET_WORDS      16U
#define IMU_QUEUE_DEPTH       16U

/* Header <4sIIHH: "IMUF", sequence, 60 payload bytes, 10 floats, version 2.
 * Payload <QiiI10f: quaternion time (TIM2 us), acc/gyro signed offsets (us),
 * continuity generation, acc xyz (mg), gyro xyz (mdps), quaternion xyzw. */
extern TX_QUEUE imu_packet_queue;
/* Pop and encode one record, without blocking sampling on either transport. */
bool imu_stream_next_packet(UCHAR packet[IMU_PACKET_SIZE]);

/** @brief Create the bounded queue of complete IMU telemetry packets. */
UINT imu_stream_queue_init(void);
/** @brief Sample the IMU and enqueue telemetry at the configured publish rate. */
void imu_stream_thread_entry(ULONG argument);

#endif
