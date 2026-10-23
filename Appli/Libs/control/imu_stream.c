#include "imu_stream.h"
#include "IMU.h"
#include "imu_control.h"
#include "tof_control.h"
#include "stream_manager.h"
#include <string.h>

#if TX_TIMER_TICKS_PER_SECOND < 1000
#error "IMU scheduling requires at least a 1 kHz ThreadX clock"
#endif

_Static_assert(sizeof(float_t) == 4 && sizeof(ULONG) == 4,
               "IMUF packets require 32-bit float and ULONG");
TX_QUEUE imu_packet_queue;
static ULONG queue_storage[IMU_QUEUE_DEPTH * IMU_PACKET_WORDS];
static IMU imu;
static uint32_t generation = 1;
static imu_status_t state = {.config = IMU_CONFIG_DEFAULT};

/** @brief Latch an IMU fault while keeping its command service available for reset. */
static void fault(HAL_StatusTypeDef error)
{
    state.error = (int32_t)error;
    state.state = IMU_FAULT;
    ++state.revision;
    imu_control_publish(&state);
}

/** @brief Execute profile changes only in the thread that owns the IMU SPI device. */
static uint32_t command(const imu_command_t *c)
{
    if (c->result) return c->result;
    if (c->operation == IMU_GET) return TOF_OK;
    if (c->operation == IMU_SET && state.state == IMU_FAULT) return TOF_FAULT_LATCHED;
    HAL_StatusTypeDef result;
    if (c->operation == IMU_RESET) {
        (void)tx_queue_flush(&imu_packet_queue);
        ++generation;
        result = imu_init(&imu);
    } else if (c->operation == IMU_SET) {
        /* A publish-only edit must not restart the sensor fusion or FIFO. */
        bool sampling_changed = c->config.sample_hz != state.config.sample_hz ||
            c->config.acc_g != state.config.acc_g || c->config.gyro_dps != state.config.gyro_dps;
        if (sampling_changed) ++generation;
        result = sampling_changed ? imu_configure(&imu, &c->config) : HAL_OK;
        (void)tx_queue_flush(&imu_packet_queue);
        if (result == HAL_OK) imu.config = c->config;
    } else return TOF_BAD_COMMAND;
    if (result != HAL_OK) { fault(result); return TOF_SENSOR_ERROR; }
    state.config = imu.config;
    state.state = IMU_STREAMING;
    state.error = 0;
    ++state.revision;
    imu_control_publish(&state);
    return TOF_OK;
}

/** @brief Encode a 32-bit integer in little-endian byte order. */
static void put_u32(UCHAR *dst, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i)
        dst[i] = (UCHAR)(value >> (8U * i));
}

/** @brief Encode a 32-bit floating-point value in little-endian byte order. */
static void put_float(UCHAR *dst, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(dst, bits);
}

/* Record: sequence, generation, quaternion timestamp (LE64), acc/gyro
 * offsets (LE32 signed), ten floats. Serialized bytes avoid structure padding. */
bool imu_stream_next_packet(UCHAR packet[IMU_PACKET_SIZE])
{
    ULONG words[IMU_PACKET_WORDS];
    if (tx_queue_receive(&imu_packet_queue, words, TX_NO_WAIT) != TX_SUCCESS) return false;
    UCHAR *record = (UCHAR *)words;
    memcpy(packet, "IMUF", 4);
    memcpy(packet + 4, record, 4);
    put_u32(packet + 8, 60U);
    packet[12] = 10; packet[13] = 0;
    packet[14] = 2; packet[15] = 0;
    memcpy(packet + 16, record + 8, 16);
    memcpy(packet + 32, record + 4, 4);
    memcpy(packet + 36, record + 24, 40);
    return true;
}

/** @brief Create the bounded queue of complete IMU telemetry packets. */
UINT imu_stream_queue_init(void)
{
    UINT result = imu_control_init();
    if (result != TX_SUCCESS) return result;
    return tx_queue_create(&imu_packet_queue, "IMU packets", IMU_PACKET_WORDS,
                           queue_storage, sizeof(queue_storage));
}

/** @brief Sample the IMU and enqueue telemetry at the configured publish rate. */
void imu_stream_thread_entry(ULONG argument)
{
    HAL_StatusTypeDef status;
    ULONG sequence = 0;
    (void)argument;

    status = imu_init(&imu);
    if (status != HAL_OK) fault(status);
    else {
        state.state = IMU_STREAMING;
        state.revision = 1;
        imu_control_publish(&state);
    }
    ULONG last = tx_time_get();
    uint32_t phase = 0;
    uint64_t last_quaternion = UINT64_MAX;
    for (;;)
    {
        /* Drain the FIFO independently of publishing, even at 1 Hz output.
         * Fractional tick periods use a phase accumulator rather than rounding
         * the requested rate to an integer number of scheduler ticks. */
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 500U);
        imu_command_t c;
        if (imu_control_next(&c)) {
            uint32_t result = command(&c);
            imu_control_reply(&c, result);
            if (result == TOF_OK && c.operation != IMU_GET) {
                phase = 0;
                last = tx_time_get();
            }
        }
        if (state.state != IMU_STREAMING) continue;

        status = imu_get_data_blocking(&imu);

        if (status == HAL_BUSY && imu.fifo_status.fifo_ovr)
        {
            ++generation;
            ++sequence; /* A sequence gap also makes it visible to the host. */
            ++state.dropped;
            status = imu_reset_fifo(&imu);

            if (status == HAL_OK)
            {
                imu_control_publish(&state);
                continue;
            }
        }
        if (status != HAL_OK)
        {
            fault(status);
            continue;
        }
        ULONG now = tx_time_get();
        uint64_t elapsed_phase = phase + (uint64_t)(ULONG)(now - last) * state.config.publish_hz;
        last = now;
        phase = (uint32_t)(elapsed_phase % TX_TIMER_TICKS_PER_SECOND);
        if (elapsed_phase < TX_TIMER_TICKS_PER_SECOND || imu.valid_data != IMU_DATA_ALL)
        {
            continue; /* Wait for first acc, gyro and SFLP outputs. */
        }

        /* Publication can run faster than actual fresh FIFO samples. Preserve
         * their acquisition times, and do not publish a duplicate orientation. */
        if (imu.timestamp_quat == last_quaternion) continue;
        last_quaternion = imu.timestamp_quat;
        ULONG words[IMU_PACKET_WORDS] = {0};
        UCHAR *packet = (UCHAR *)words;
        put_u32(packet, sequence++);
        put_u32(packet + 4, generation);
        put_u32(packet + 8, (uint32_t)imu.timestamp_quat);
        put_u32(packet + 12, (uint32_t)(imu.timestamp_quat >> 32));
        put_u32(packet + 16, (uint32_t)(int32_t)((int64_t)imu.timestamp_acc - (int64_t)imu.timestamp_quat));
        put_u32(packet + 20, (uint32_t)(int32_t)((int64_t)imu.timestamp_gyro - (int64_t)imu.timestamp_quat));
        for (unsigned i = 0; i < 3; ++i)
        {
            put_float(packet + 24 + 4 * i, imu.acc_mg[i]);
            put_float(packet + 36 + 4 * i, imu.gyro_mdps[i]);
        }
        for (unsigned i = 0; i < 4; ++i)
            put_float(packet + 48 + 4 * i, imu.quat[i]);

        /* Sampling never waits on USB; a full queue drops this sample. */
        if (tx_queue_send(&imu_packet_queue, words, TX_NO_WAIT) == TX_SUCCESS)
        {
            ++state.published;
            tx_event_flags_set(&stream_events, STREAM_IMU_READY, TX_OR);
        }
        else ++state.dropped;
        imu_control_publish(&state);
    }
}
