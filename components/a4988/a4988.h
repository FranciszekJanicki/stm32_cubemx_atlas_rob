#ifndef A4988_A4988_H
#define A4988_A4988_H

#include "a4988_config.h"

typedef struct {
    a4988_config_t config;
    a4988_interface_t interface;
} a4988_t;

a4988_err_t a4988_initialize(a4988_t* a4988,
                             a4988_config_t const* config,
                             a4988_interface_t const* interface);
a4988_err_t a4988_deinitialize(a4988_t* a4988);

a4988_err_t a4988_set_frequency(a4988_t const* a4988, uint32_t frequency);

a4988_err_t a4988_set_microstep(a4988_t const* a4988, a4988_microstep_t microstep);
a4988_err_t a4988_set_full_microstep(a4988_t const* a4988);
a4988_err_t a4988_set_half_microstep(a4988_t const* a4988);
a4988_err_t a4988_set_quarter_microstep(a4988_t const* a4988);
a4988_err_t a4988_set_eighth_microstep(a4988_t const* a4988);
a4988_err_t a4988_set_sixteenth_microstep(a4988_t const* a4988);

a4988_err_t a4988_set_direction(a4988_t const* a4988, a4988_direction_t direction);
a4988_err_t a4988_set_forward_direction(a4988_t const* a4988);
a4988_err_t a4988_set_backward_direction(a4988_t const* a4988);
a4988_err_t a4988_set_stop_direction(a4988_t const* a4988);

a4988_err_t a4988_set_reset(a4988_t const* a4988, bool reset);
a4988_err_t a4988_set_enable(a4988_t const* a4988, bool enable);
a4988_err_t a4988_set_sleep(a4988_t const* a4988, bool sleep);

#endif // A4988_A4988_H