#include "LEDController.h"
#include <string.h>

LEDController led_controller;

/* LP5815 time codes: 2=100 ms, 4=200 ms, 5=250 ms, A=500 ms,
 * B=1 s. Pattern repeat F and engine repeat 3 both mean forever.
 * See TI LP5815 datasheet sections 7.3.4 and 7.6. */
typedef enum { SOLID, BLINK, BREATHE, DOUBLE_PULSE } led_shape_t;
typedef struct {
    uint8_t rgb[3]; /* Relative color mix; normalized to a peak PWM of 255. */
    led_shape_t shape;
    uint8_t ramp, hold;
    uint8_t brightness; /* 0--100 percent of the normalized color's maximum. */
} led_preset_t;

static const led_preset_t presets[LED_ANIMATION_COUNT] = {
    [LED_ANIMATION_OFF] = {{0, 0, 0}, SOLID, 0, 0, 0},
    [LED_ANIMATION_ALL] = {{255, 255, 255}, SOLID, 0, 0, 100},
    [LED_ANIMATION_CONFIGURING] = {{0, 160, 160}, BREATHE, 10, 2, 63},
    [LED_ANIMATION_SENSOR_INIT_FAILED_IMU] = {{192, 0, 0}, DOUBLE_PULSE, 2, 11, 75},
    [LED_ANIMATION_SENSOR_INIT_FAILED_LDC] = {{192, 0, 0}, BLINK, 0, 4, 75},
    [LED_ANIMATION_SENSOR_INIT_FAILED_TOF] = {{192, 0, 0}, DOUBLE_PULSE, 2, 11, 75},
    [LED_ANIMATION_SENSOR_INIT_FAILED_MAG] = {{192, 0, 0}, BLINK, 0, 5, 75},
    [LED_ANIMATION_APPLICATION_RUNNING] = {{0, 192, 0}, SOLID, 0, 0, 75},
    [LED_ANIMATION_UNCAUGHT_EXCEPTION] = {{192, 0, 0}, BLINK, 0, 2, 75},
    [LED_ANIMATION_CHARGER_NOT_RESPONDING] = {{192, 0, 0}, BREATHE, 11, 4, 75},
    [LED_ANIMATION_CYAN] = {{0, 192, 192}, SOLID, 0, 0, 75},
    [LED_ANIMATION_GREEN] = {{0, 192, 0}, SOLID, 0, 0, 75},
    [LED_ANIMATION_FSBL_STARTED] = {{192, 72, 0}, BLINK, 0, 5, 75},
    [LED_ANIMATION_FSBL_LOADING] = {{192, 72, 0}, BLINK, 0, 2, 75},
    [LED_ANIMATION_FSBL_HANDOFF] = {{160, 0, 160}, SOLID, 0, 0, 63},
    [LED_ANIMATION_FSBL_ERROR] = {{192, 0, 0}, DOUBLE_PULSE, 2, 11, 75},
    [LED_ANIMATION_APP_STARTED] = {{0, 0, 192}, BLINK, 0, 5, 75},
    [LED_ANIMATION_APP_INIT_FINISHED] = {{0, 160, 160}, DOUBLE_PULSE, 2, 11, 63},
    [LED_ANIMATION_APP_RANGING] = {{0, 192, 0}, BREATHE, 10, 2, 75},
    [LED_ANIMATION_APP_IDLE] = {{0, 0, 160}, BREATHE, 11, 4, 100},
    [LED_ANIMATION_APP_POWER_DOWN] = {{64, 24, 0}, SOLID, 0, 0, 25},
    [LED_ANIMATION_APP_RESETTING] = {{0, 160, 160}, BLINK, 0, 2, 63},
    [LED_ANIMATION_APP_FAULT] = {{192, 0, 0}, DOUBLE_PULSE, 2, 11, 75},
};

/** @brief Guard complete I2C transactions without disabling IRQs during transfers. */
static bool led_acquire(LEDController *controller)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool available = !controller->busy;
    if (available) controller->busy = true;
    __set_PRIMASK(mask);
    return available;
}

static void led_release(LEDController *controller)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    controller->busy = false;
    __set_PRIMASK(mask);
}

static HAL_StatusTypeDef led_write(uint8_t address, const void *data, uint16_t size)
{
    return HAL_I2C_Mem_Write(LED_I2C_HANDLE, LED_I2C_ADDRESS << 1, address,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, size, 10);
}

/** @brief Build all engines afresh so earlier patterns cannot leak into new states. */
static void led_build(LEDController *controller, const led_preset_t *preset)
{
    unsigned brightness = preset->brightness > 100U ? 100U : preset->brightness;
    unsigned maximum = preset->rgb[0];
    for (unsigned i = 1; i < 3; ++i)
        if (preset->rgb[i] > maximum) maximum = preset->rgb[i];
    memset(controller->patterns, 0, sizeof(controller->patterns));
    memset(controller->engines, 0, sizeof(controller->engines));
    memset(&controller->config, 0, sizeof(controller->config));
    controller->config.enable = controller->config.instablink_dis = 1;
    /* Keep the original channel selection at nonzero brightness: rounding a
     * small PWM to zero must not change the color-dependent drive settings. */
    controller->config.out0_en = brightness && preset->rgb[0] != 0;
    controller->config.out1_en = brightness && preset->rgb[1] != 0;
    controller->config.out2_en = brightness && preset->rgb[2] != 0;
    controller->config.out0_auto_en = controller->config.out1_auto_en = controller->config.out2_auto_en = 1;
    controller->config.out0_exp_en = controller->config.out1_exp_en = controller->config.out2_exp_en = 1;
    controller->config.out0_engine_ch = 0;
    controller->config.out1_engine_ch = 1;
    controller->config.out2_engine_ch = 2;
    for (unsigned i = 0; i < 3; ++i) {
        if (!brightness || !preset->rgb[i]) continue;
        LED_Controller_Pattern_t *pattern = &controller->patterns[i];
        pattern->repeat = 15;
        /* A nonzero RGB component guarantees maximum != 0. Round once after
         * applying both normalization and brightness to retain the color mix. */
        uint8_t peak = (uint8_t)(((unsigned)preset->rgb[i] * 255U * brightness + maximum * 50U) /
                                 (maximum * 100U));
        if (preset->shape == SOLID) {
            memset(pattern->PWM, peak, sizeof(pattern->PWM));
            pattern->sloper0 = 11; /* Nonzero duration, even for a steady color. */
        } else if (preset->shape == BLINK) {
            pattern->PWM[0] = pattern->PWM[1] = peak;
            pattern->sloper0 = pattern->sloper2 = preset->hold;
        } else if (preset->shape == BREATHE) {
            pattern->PWM[1] = pattern->PWM[2] = peak;
            pattern->sloper0 = pattern->sloper2 = preset->ramp;
            pattern->sloper1 = pattern->sloper3 = preset->hold;
        } else {
            /* PT counts the inner slopes; the end pause runs only after PT
             * finishes. Infinite PT would turn a double pulse into 5 Hz. */
            pattern->repeat = 1;
            pattern->PWM[1] = pattern->PWM[3] = peak;
            pattern->sloper0 = pattern->sloper1 = pattern->sloper2 = pattern->sloper3 = preset->ramp;
            pattern->pause_t1 = preset->hold;
        }
        controller->engines[i].patterns[0] = pattern;
        controller->engines[i].repetitions = 3;
    }
}

/** @brief Encode registers explicitly; wire layout must not depend on C bitfield ABI. */
static HAL_StatusTypeDef led_update_config(LEDController *controller)
{
    uint8_t config[13] = {3, 0, 0, 0, 0x77, 0x24};
    uint8_t patterns[36] = {0};
    for (unsigned i = 0; i < 3; ++i) {
        if (controller->engines[i].patterns[0]) {
            config[2] |= 1U << i;
            config[6 + i] = i;
            config[10 + i / 2] |= 1U << ((i % 2) * 4);
            config[12] |= 3U << (i * 2);
        }
        const LED_Controller_Pattern_t *p = &controller->patterns[i];
        uint8_t *out = patterns + i * 9;
        out[0] = (p->pause_t0 << 4) | p->pause_t1;
        out[1] = p->repeat;
        memcpy(out + 2, p->PWM, 5);
        out[7] = p->sloper0 | (p->sloper1 << 4);
        out[8] = p->sloper2 | (p->sloper3 << 4);
    }
    const uint8_t stop = 0xAA, update = 0x55, start = 0xFF;
    uint8_t current[] = {LED_DC_RED, LED_DC_GREEN, LED_DC_BLUE};
    /* Brighten red and its amber/magenta mixtures. White and colors without
     * red retain the normal drive settings; all PWM values/timing stay intact. */
    if (controller->config.out0_en &&
            !(controller->config.out1_en && controller->config.out2_en)) {
        current[0] = LED_DC_BRIGHT_RED;
        current[1] = LED_DC_BRIGHT_GREEN;
        current[2] = LED_DC_BRIGHT_BLUE;
    }
    HAL_StatusTypeDef status = led_write(LED_STOP_CMD, &stop, 1);
    if (status == HAL_OK) status = led_write(LED_REG_DC, current, sizeof(current));
    if (status == HAL_OK) status = led_write(0, config, sizeof(config));
    if (status == HAL_OK) status = led_write(LED_PATTERN_REG_START, patterns, sizeof(patterns));
    if (status == HAL_OK) status = led_write(LED_UPDATE_CMD, &update, 1);
    if (status == HAL_OK) status = led_write(LED_START_CMD, &start, 1);
    return status;
}

HAL_StatusTypeDef LEDController_Init(LEDController *controller)
{
    if (controller == NULL) return HAL_ERROR;
    memset(controller, 0, sizeof(*controller));
    /* Software initialization allows a failed first write to be retried by
     * SetAnimation. Before this call, error paths must not touch uninitialized I2C. */
    controller->initialized = true;
    controller->animation = LED_ANIMATION_COUNT;
    const uint8_t current[] = {LED_DC_RED, LED_DC_GREEN, LED_DC_BLUE};
    controller->last_status = led_write(LED_REG_DC, current, sizeof(current));
    return controller->last_status;
}

HAL_StatusTypeDef LEDController_SetAnimation(LEDController *controller, LED_Animation_t animation)
{
    if (controller == NULL || !controller->initialized || (unsigned)animation >= LED_ANIMATION_COUNT)
        return HAL_ERROR;
    if (!led_acquire(controller)) return HAL_BUSY;
    if (controller->animation_valid && controller->animation == animation) {
        led_release(controller);
        return HAL_OK;
    }
    if (controller->animation == animation && controller->last_status != HAL_OK &&
            (int32_t)(HAL_GetTick() - controller->next_retry_ms) < 0) {
        HAL_StatusTypeDef status = controller->last_status;
        led_release(controller);
        return status;
    }
    controller->animation_valid = false;
    led_build(controller, &presets[animation]);
    controller->last_status = led_update_config(controller);
    controller->animation = animation;
    controller->animation_valid = controller->last_status == HAL_OK;
    controller->next_retry_ms = HAL_GetTick() + 500U;
    HAL_StatusTypeDef status = controller->last_status;
    led_release(controller);
    return status;
}

/** @brief Countable autonomous fault burst, then two seconds dark, forever. */
HAL_StatusTypeDef LEDController_SetFaultCode(LEDController *controller,
                                            LED_DiagnosticColor_t color, uint8_t pulses)
{
    if (controller == NULL || !controller->initialized || pulses == 0 || pulses > 14 ||
        (color != LED_DIAGNOSTIC_RED && color != LED_DIAGNOSTIC_AMBER &&
         color != LED_DIAGNOSTIC_BLUE && color != LED_DIAGNOSTIC_MAGENTA)) return HAL_ERROR;
    if (!led_acquire(controller)) return HAL_BUSY;
    const led_preset_t preset = {
        {(color & 1U) ? 192 : 0, (color & 2U) ? 72 : 0, (color & 4U) ? 192 : 0},
        BLINK, 0, 4, 75 /* 200 ms on, 200 ms off; retain the diagnostic level. */
    };
    led_build(controller, &preset);
    for (unsigned i = 0; i < 3; ++i) {
        if (!preset.rgb[i]) continue;
        controller->patterns[i].repeat = pulses;
        controller->patterns[i].pause_t1 = 12; /* 2 s after the finite burst. */
    }
    /* Diagnostic patterns must not be mistaken for a cached lifecycle preset. */
    controller->animation_valid = false;
    controller->animation = LED_ANIMATION_COUNT;
    controller->last_status = led_update_config(controller);
    HAL_StatusTypeDef status = controller->last_status;
    led_release(controller);
    return status;
}
