#ifndef SENSOR_CLOCK_H
#define SENSOR_CLOCK_H

#include <stdint.h>

/* One boot-relative, 1 MHz TIM2 clock shared by capture interrupts and IMU.
 * The HAL tick calls now_us() every millisecond to extend the 32-bit counter
 * even while both sensors are stopped/faulted. No wall-clock or transport time. */
void sensor_clock_init(void);
uint64_t sensor_clock_now_us(void);

#endif
