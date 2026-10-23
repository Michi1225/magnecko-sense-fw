#include "stream_manager.h"
#include "tof_control.h"
#include "status_led.h"
#include "dcmipp.h"
#include <stdbool.h>
#include <string.h>

TX_EVENT_FLAGS_GROUP stream_events;
UCHAR stream_packet[STREAM_HEADER_SIZE + CSI_OUTPUT_PAYLOAD_SIZE];
ULONG stream_packet_length;
static VL53L9_TypeDef_t sensor;
static tof_status_t state;

/** @brief Publish sensor state and update the LED outside the status critical section. */
static void publish(void)
{
    tof_control_publish(&state);
    status_led_tof(state.state);
}

/** @brief Encode a 32-bit value in little-endian byte order. */
static void put_le32(UCHAR *dst, ULONG value)
{
    for (UINT i = 0; i < 4U; ++i) dst[i] = (UCHAR)(value >> (8U * i));
}

/** @brief Stop capture and ranging, then publish a latched sensor fault. */
static void fault(int32_t error)
{
    (void)platform_stop_csi_pipe();
    if (sensor.p_dev) (void)vl53l9_stop(sensor.p_dev);
    /* Keep faults latched until an explicit reset; never retry emission. */
    state.state = TOF_FAULT;
    state.error = error ? error : -1;
    ++state.revision;
    publish();
}

/** @brief Copy the active sensor profile and geometry into the published status. */
static void update_config(void)
{
    state.config = (tof_config_t){sensor.profile.frame_period_us, sensor.profile.exposure_ms,
        sensor.profile.binning, sensor.profile.power, sensor.profile.context, sensor.profile.dss_enabled};
    state.width = sensor.csi_width;
    state.height = sensor.csi_height;
}

/** @brief Apply a queued ToF command while enforcing the sensor lifecycle. */
static uint32_t command(const tof_command_t *c)
{
    if (c->result) return c->result;
    if (c->operation == TOF_GET) return TOF_OK;
    if (c->operation == TOF_RESET)
    {
        /* The caller has drained the shared frame. Reset only this sensor,
         * including its I3C address and patch; leave emission stopped. */
        (void)platform_stop_csi_pipe();
        platform_acknowledge_event(PLATFORM_CAM_PIPE_FRAME_EVT);
        state.state = TOF_INITIALIZING;
        ++state.revision;
        publish();
        if (VL53L9_init(&sensor) != HAL_OK) { fault(-1); return TOF_SENSOR_ERROR; }
        update_config();
        state.error = 0;
        state.state = TOF_STOPPED;
        ++state.revision;
        publish();
        return TOF_OK;
    }
    if (c->operation == TOF_POWER_DOWN)
    {
        if (state.state == TOF_OFF) return TOF_OK;
        int capture_result = platform_stop_csi_pipe();
        platform_acknowledge_event(PLATFORM_CAM_PIPE_FRAME_EVT);
        /* XSHUT also works when I3C/ranging is faulted. Do not access I3C
         * after shutdown; only Reset may power the sensor back up. */
        if (VL53L9_power_down(&sensor) != HAL_OK)
        { fault(-1); return TOF_SENSOR_ERROR; }
        state.state = TOF_OFF;
        ++state.revision;
        publish();
        return capture_result ? TOF_SENSOR_ERROR : TOF_OK;
    }
    if (state.state == TOF_OFF) return c->operation == TOF_STOP ? TOF_OK : TOF_POWERED_DOWN;
    if (state.state == TOF_FAULT) return TOF_FAULT_LATCHED;
    if (c->operation == TOF_STOP)
    {
        if (state.state == TOF_STOPPED) return TOF_OK;
        int result = vl53l9_stop(sensor.p_dev);
        int capture_result = 0;
#ifndef VL53L9_SENSOR_ONLY
        capture_result = platform_stop_csi_pipe();
#endif
        platform_acknowledge_event(PLATFORM_CAM_PIPE_FRAME_EVT);
        if (result || capture_result) { fault(result ? result : capture_result); return TOF_SENSOR_ERROR; }
        state.state = TOF_STOPPED;
    }
    else if (c->operation == TOF_START)
    {
        if (state.state == TOF_STREAMING) return TOF_OK;
        if (VL53L9_start_ranging(&sensor) != HAL_OK) { fault(-1); return TOF_SENSOR_ERROR; }
        state.state = TOF_STREAMING;
    }
    else if (c->operation == TOF_SET)
    {
        if (state.state != TOF_STOPPED) return TOF_MUST_STOP;
        vl53l9_profile_t profile = sensor.profile;
        profile.frame_period_us = c->config.period_us;
        profile.exposure_ms = c->config.exposure_ms;
        profile.binning = c->config.binning;
        profile.power = (vl53l9_power_mode_t)c->config.power;
        profile.context = (vl53l9_context_t)c->config.context;
        profile.dss_enabled = c->config.dss_enabled != 0;
        if (VL53L9_configure(&sensor, &profile) != HAL_OK) { fault(-1); return TOF_SENSOR_ERROR; }
        update_config();
    }
    else return TOF_BAD_COMMAND;
    ++state.revision;
    publish();
    return TOF_OK;
}

/** @brief Own the ToF sensor, process commands, and publish captured frames. */
void stream_thread_entry(ULONG argument)
{
    (void)argument;
    bool packet_in_flight = false, armed = false;
    uint32_t last_status_tick = HAL_GetTick();
    if (VL53L9_init(&sensor) != HAL_OK) fault(-1);
    else
    {
        update_config();
        /* Initialize the profile without emission or capture. Only an explicit
         * Start command may enter ranging, including after a board reset. */
        state.state = TOF_STOPPED;
        state.revision = 1;
        publish();
    }
    for (;;)
    {
        /* Successful repeats are cached. Failed LED writes retry at most twice
         * per second, including while stopped or waiting for a sensor frame. */
        status_led_tof(state.state);
        /* The transport must finish the previous packet before Stop can be
         * acknowledged or a new geometry can be applied to the shared buffer. */
        if (packet_in_flight)
        {
            ULONG flags = 0;
            if (tx_event_flags_get(&stream_events, STREAM_PACKET_CONSUMED,
                                   TX_OR_CLEAR, &flags, TX_NO_WAIT) == TX_SUCCESS)
                packet_in_flight = false;
        }
        if (!packet_in_flight)
        {
            tof_command_t c;
            if (tof_control_next(&c))
            {
                bool was_streaming = state.state == TOF_STREAMING;
                uint32_t result = command(&c);
                tof_control_reply(&c, result);
                /* Start already armed the first capture; Stop ends it. */
                if (!was_streaming && state.state == TOF_STREAMING) armed = true;
                if (state.state != TOF_STREAMING) armed = false;
            }
        }
        if (state.state != TOF_STREAMING || packet_in_flight)
        {
            tx_thread_sleep(1);
            continue;
        }
        if (!armed)
        {
#ifndef VL53L9_SENSOR_ONLY
            VL53L9_prepare_capture(&sensor);
            if (platform_start_csi_pipe((uint8_t *)sensor.g_csi_output_buffer) != 0)
            { fault(-1); continue; }
#endif
            armed = true;
        }
        bool ready = false;
        platform_get_event_status(PLATFORM_CAM_PIPE_FRAME_EVT, &ready);
        if (!ready)
        {
            uint32_t now = HAL_GetTick();
            if ((uint32_t)(now - last_status_tick) >= 1000U)
            {
                vl53l9_status_t status = {0};
                int result = vl53l9_get_status(sensor.p_dev, &status);
                last_status_tick = now;
                if (result || status.firmware || status.error.internal_fw || status.error.ref_array)
                    fault(result ? result : (status.firmware ? status.firmware : -1));
            }
            tx_thread_sleep(1);
            continue;
        }
        platform_acknowledge_event(PLATFORM_CAM_PIPE_FRAME_EVT);
        armed = false;
        SCB_InvalidateDCache_by_Addr((uint32_t *)sensor.g_csi_output_buffer,
                                    sizeof(sensor.g_csi_output_buffer));
        memcpy(stream_packet, "VL9T", 4);
        put_le32(stream_packet + 4, state.frames++);
        ULONG payload = (ULONG)sensor.csi_width * sensor.csi_height;
        put_le32(stream_packet + 8, payload + 8U);
        uint64_t timestamp = platform_frame_timestamp_us();
        put_le32(stream_packet + 16, (uint32_t)timestamp);
        put_le32(stream_packet + 20, (uint32_t)(timestamp >> 32));
        stream_packet[12] = sensor.csi_width; stream_packet[13] = 0;
        stream_packet[14] = sensor.csi_height; stream_packet[15] = 0;
        memcpy(stream_packet + STREAM_HEADER_SIZE, (const void *)sensor.g_csi_output_buffer, payload);
        stream_packet_length = STREAM_HEADER_SIZE + payload;
        /* DMA writes the capture buffer; the transport owns stream_packet.
         * Rearm immediately after copying, overlapping the next capture with
         * transmission. Waiting for the consumer here can skip every other frame. */
#ifndef VL53L9_SENSOR_ONLY
        VL53L9_prepare_capture(&sensor);
        if (platform_start_csi_pipe((uint8_t *)sensor.g_csi_output_buffer) != 0)
        { fault(-1); continue; }
#endif
        armed = true;
        packet_in_flight = true;
        publish();
        tx_event_flags_set(&stream_events, STREAM_PACKET_READY, TX_OR);
    }
}
