#ifndef __VL53L9_PROCESS_H
#define __VL53L9_PROCESS_H

#include <stdio.h>
#include <stdlib.h>

#include "stm32n6xx_hal.h"
#include "vl53l9.h"
#include "vl53l9_device.h"
#include "vl53l9_interface.h"
#include "vl53l9_utils.h"

#include "stm32n6xx.h" // for SCB_InvalidateDCache_by_Addr

#define CONF_DEVICE_ID   (0) /**< select device entry in platform descriptor array (see vl53l9_device.c) */
#define CONF_USECASE     (VL53L9_USECASE_GAMING) /**< select ranging profile to be applied (see vl53l9_utils.h) */

#define CSI_OUTPUT_PAYLOAD_SIZE (100U * 149U)
#define CSI_OUTPUT_PITCH        100U /* PIPE0 dumps RAW8 bytes without row padding. */
#define CSI_OUTPUT_BUFFER_SIZE ((CSI_OUTPUT_PITCH * 149U + 31U) & ~31U)

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ALIGNED32 __attribute__((aligned(32)))

typedef struct 
{
    ALIGNED32 volatile uint8_t g_csi_output_buffer[CSI_OUTPUT_BUFFER_SIZE];
    vl53l9_device_t *p_dev;
    vl53l9_profile_t profile;
    vl53l9_profile_t *p_profile;
    vl53l9_hw_config_t hw_config;
    vl53l9_frame_t frame;
    uint8_t frame_payload[CSI_OUTPUT_PAYLOAD_SIZE];

    uint8_t csi_width, csi_height;

} VL53L9_TypeDef_t;

uint64_t platform_frame_timestamp_us(void);

/** @brief Reset and address the ToF sensor, load its firmware, and apply the default profile. */
HAL_StatusTypeDef VL53L9_init(VL53L9_TypeDef_t *tof);
/** @brief Validate the requested geometry and configure the ToF sensor and CSI receiver. */
HAL_StatusTypeDef VL53L9_configure(VL53L9_TypeDef_t *tof, const vl53l9_profile_t *profile);
/** @brief Hold the sensor in XSHUT while leaving its supply rails connected. */
HAL_StatusTypeDef VL53L9_power_down(VL53L9_TypeDef_t *tof);
/** @brief Clean and invalidate the CSI DMA buffer before the next capture. */
void VL53L9_prepare_capture(VL53L9_TypeDef_t *tof);
/** @brief Arm frame capture and start ranging, stopping capture if sensor startup fails. */
HAL_StatusTypeDef VL53L9_start_ranging(VL53L9_TypeDef_t *tof);
/** @brief Copy and parse a completed CSI frame after invalidating its DMA buffer cache. */
HAL_StatusTypeDef VL53L9_process(VL53L9_TypeDef_t *tof);

#endif //__VL53L9_PROCESS_H
