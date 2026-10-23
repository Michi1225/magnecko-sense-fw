#ifndef DEVICE_PARAMETERS_H
#define DEVICE_PARAMETERS_H
#include <stdint.h>
#define DEVICE_PARAMETERS_MAX_JSON 16384U
/** @brief Read device parameters from flash and prepare JSON telemetry or an error payload. */
void device_parameters_init(void);
/** @brief Publish verified parameter JSON without racing telemetry readers. */
void device_parameters_publish(const uint8_t *json,uint32_t length);
/** @brief Copy a consistent PARM snapshot before sending on either transport. */
uint32_t device_parameters_copy_packet(uint8_t *destination,uint32_t capacity);
#endif
