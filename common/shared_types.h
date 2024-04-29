#ifndef __SHARED_TYPES_H
#define __SHARED_TYPES_H

#include <zephyr/kernel.h>

typedef struct {
    uint32_t duty_cycle;
    uint32_t steptime_us;
} pwm_3p_config_t;

#endif
