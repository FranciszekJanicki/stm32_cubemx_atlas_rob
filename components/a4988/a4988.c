#include "a4988.h"
#include "a4988_config.h"
#include <assert.h>
#include <string.h>

static a4988_err_t a4988_pwm_initialize(a4988_t const* a4988)
{
    return a4988->interface.pwm_initialize
               ? a4988->interface.pwm_initialize(a4988->interface.pwm_user)
               : A4988_ERR_NULL;
}

static a4988_err_t a4988_pwm_deinitialize(a4988_t const* a4988)
{
    return a4988->interface.pwm_deinitialize
               ? a4988->interface.pwm_deinitialize(a4988->interface.pwm_user)
               : A4988_ERR_NULL;
}

static a4988_err_t a4988_pwm_start(a4988_t const* a4988)
{
    return a4988->interface.pwm_start ? a4988->interface.pwm_start(a4988->interface.pwm_user)
                                      : A4988_ERR_NULL;
}

static a4988_err_t a4988_pwm_stop(a4988_t const* a4988)
{
    return a4988->interface.pwm_stop ? a4988->interface.pwm_stop(a4988->interface.pwm_user)
                                     : A4988_ERR_NULL;
}

static a4988_err_t a4988_pwm_set_frequency(a4988_t const* a4988, uint32_t frequency)
{
    return a4988->interface.pwm_set_frequency
               ? a4988->interface.pwm_set_frequency(a4988->interface.pwm_user, frequency)
               : A4988_ERR_NULL;
}

static a4988_err_t a4988_gpio_initialize(a4988_t const* a4988)
{
    return a4988->interface.gpio_initialize
               ? a4988->interface.gpio_initialize(a4988->interface.gpio_user)
               : A4988_ERR_NULL;
}

static a4988_err_t a4988_gpio_deinitialize(a4988_t const* a4988)
{
    return a4988->interface.gpio_deinitialize
               ? a4988->interface.gpio_deinitialize(a4988->interface.gpio_user)
               : A4988_ERR_NULL;
}

static a4988_err_t a4988_gpio_write_pin(a4988_t const* a4988, uint32_t pin, bool state)
{
    return a4988->interface.gpio_write_pin
               ? a4988->interface.gpio_write_pin(a4988->interface.gpio_user, pin, state)
               : A4988_ERR_NULL;
}

a4988_err_t a4988_initialize(a4988_t* a4988,
                             a4988_config_t const* config,
                             a4988_interface_t const* interface)
{
    assert(a4988 && config && interface);

    memset(a4988, 0, sizeof(*a4988));
    memcpy(&a4988->config, config, sizeof(*config));
    memcpy(&a4988->interface, interface, sizeof(*interface));

    a4988_err_t err = a4988_pwm_initialize(a4988);
    err |= a4988_gpio_initialize(a4988);

    return err;
}

a4988_err_t a4988_deinitialize(a4988_t* a4988)
{
    assert(a4988);

    a4988_err_t err = a4988_pwm_deinitialize(a4988);
    err |= a4988_gpio_deinitialize(a4988);

    return err;
}

a4988_err_t a4988_set_frequency(a4988_t const* a4988, uint32_t frequency)
{
    assert(a4988);

    return a4988_pwm_set_frequency(a4988, frequency);
}

a4988_err_t a4988_set_microstep(a4988_t const* a4988, a4988_microstep_t microstep)
{
    assert(a4988);

    switch (microstep) {
        case A4988_MICROSTEP_FULL:
            return a4988_set_full_microstep(a4988);
        case A4988_MICROSTEP_HALF:
            return a4988_set_half_microstep(a4988);
        case A4988_MICROSTEP_QUARTER:
            return a4988_set_quarter_microstep(a4988);
        case A4988_MICROSTEP_EIGHTH:
            return a4988_set_eighth_microstep(a4988);
        case A4988_MICROSTEP_SIXTEENTH:
            return a4988_set_sixteenth_microstep(a4988);
        default:
            return A4988_ERR_FAIL;
    }
}

a4988_err_t a4988_set_full_microstep(a4988_t const* a4988)
{
    assert(a4988);

    a4988_err_t err = a4988_gpio_write_pin(a4988, a4988->config.pin_ms1, false);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms2, false);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms3, false);

    return err;
}

a4988_err_t a4988_set_half_microstep(a4988_t const* a4988)
{
    assert(a4988);

    a4988_err_t err = a4988_gpio_write_pin(a4988, a4988->config.pin_ms1, true);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms2, false);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms3, false);

    return err;
}

a4988_err_t a4988_set_quarter_microstep(a4988_t const* a4988)
{
    assert(a4988);

    a4988_err_t err = a4988_gpio_write_pin(a4988, a4988->config.pin_ms1, true);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms2, true);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms3, false);

    return err;
}

a4988_err_t a4988_set_eighth_microstep(a4988_t const* a4988)
{
    assert(a4988);

    a4988_err_t err = a4988_gpio_write_pin(a4988, a4988->config.pin_ms1, false);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms2, true);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms3, false);

    return err;
}

a4988_err_t a4988_set_sixteenth_microstep(a4988_t const* a4988)
{
    assert(a4988);

    a4988_err_t err = a4988_gpio_write_pin(a4988, a4988->config.pin_ms1, false);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms2, false);
    err |= a4988_gpio_write_pin(a4988, a4988->config.pin_ms3, true);

    return err;
}

a4988_err_t a4988_set_direction(a4988_t const* a4988, a4988_direction_t direction)
{
    assert(a4988);

    switch (direction) {
        case A4988_DIRECTION_FORWARD:
            return a4988_set_forward_direction(a4988);
        case A4988_DIRECTION_BACKWARD:
            return a4988_set_backward_direction(a4988);
        case A4988_DIRECTION_STOP:
            return a4988_set_stop_direction(a4988);
        default:
            return A4988_ERR_FAIL;
    }
}

a4988_err_t a4988_set_forward_direction(a4988_t const* a4988)
{
    assert(a4988);

    a4988_err_t err = a4988_gpio_write_pin(a4988, a4988->config.pin_dir, false);
    err |= a4988_pwm_start(a4988);

    return err;
}

a4988_err_t a4988_set_backward_direction(a4988_t const* a4988)
{
    assert(a4988);

    a4988_err_t err = a4988_gpio_write_pin(a4988, a4988->config.pin_dir, true);
    err |= a4988_pwm_start(a4988);

    return err;
}

a4988_err_t a4988_set_stop_direction(a4988_t const* a4988)
{
    assert(a4988);

    return a4988_pwm_stop(a4988);
}

a4988_err_t a4988_set_reset(a4988_t const* a4988, bool reset)
{
    assert(a4988);

    return a4988_gpio_write_pin(a4988, a4988->config.pin_enable, !reset);
}

a4988_err_t a4988_set_enable(a4988_t const* a4988, bool enable)
{
    assert(a4988);

    return a4988_gpio_write_pin(a4988, a4988->config.pin_enable, !enable);
}

a4988_err_t a4988_set_sleep(a4988_t const* a4988, bool sleep)
{
    assert(a4988);

    return a4988_gpio_write_pin(a4988, a4988->config.pin_sleep, !sleep);
}
