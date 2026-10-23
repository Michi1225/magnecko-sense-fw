/**
 * This File contains the thread for the control plane
 *
 
 */

#ifndef STREAM_MANAGER_H
#define STREAM_MANAGER_H

#include "main.h"
#include "VL53L9_process.h"
#include "app_threadx.h" // For typedefs

/* vl53l9_write() puts the 9,865-byte firmware patch plus its 2-byte
 * register address on the stack. Reserve headroom for the HAL call chain. */
#define STREAM_THREAD_STACK_SIZE     16384U
#define STREAM_THREAD_PRIO           12U

/* One transport consumer owns this packet until it sets STREAM_PACKET_CONSUMED.
 * Wire format: "VL9T", LE32 sequence, LE32 payload bytes, LE16 width,
 * LE16 height, then LE64 frame-completion microseconds and tightly packed RAW8 CSI data
 * including the status row. Payload length includes the eight timestamp bytes. */
#define STREAM_PACKET_READY       (1UL << 0)
#define STREAM_PACKET_CONSUMED    (1UL << 1)
#define STREAM_IMU_READY          (1UL << 2)
#define STREAM_HEADER_SIZE        24U
extern TX_EVENT_FLAGS_GROUP stream_events;
extern UCHAR stream_packet[STREAM_HEADER_SIZE + CSI_OUTPUT_PAYLOAD_SIZE];
extern ULONG stream_packet_length;

/** @brief Return the session identifier used to reject stale USB replies. */
uint32_t stream_usb_session(void);
/** @brief Write a complete packet to the active USB CDC session. */
UINT stream_usb_write(UCHAR *packet, ULONG length);

/** @brief Own the ToF sensor, process commands, and publish captured frames. */
void stream_thread_entry(ULONG argument);

#endif
