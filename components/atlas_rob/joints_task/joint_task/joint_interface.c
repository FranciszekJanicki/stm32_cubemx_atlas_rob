#include "joint_manager.h"
#include <stdbool.h>
#include <stdint.h>

static bool frequency_to_prescaler_and_period(uint32_t frequency,
                                              uint32_t clock_hz,
                                              uint32_t clock_div,
                                              uint32_t max_prescaler,
                                              uint32_t max_period,
                                              uint32_t* prescaler,
                                              uint32_t* period)
{
    if (frequency == 0U || !prescaler || !period) {
        return false;
    }

    uint32_t base_clock = clock_hz / (clock_div + 1U);
    uint32_t temp_prescaler = 0U;
    uint32_t temp_period = base_clock / frequency;

    while (temp_period > max_period && temp_prescaler < max_prescaler) {
        temp_prescaler++;
        temp_period = base_clock / ((temp_prescaler + 1U) * frequency);
    }
    if (temp_period > max_period) {
        temp_period = max_period;
        temp_prescaler = (base_clock / (temp_period * frequency)) - 1U;
    }
    if (temp_prescaler > max_prescaler) {
        temp_prescaler = max_prescaler;
    }

    *prescaler = temp_prescaler;
    *period = temp_period;

    return true;
}

static tca9548_t tca9548 = {};

a4988_err_t a4988_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->a4988_gpio) {
        HAL_GPIO_WritePin(manager->a4988_gpio, pin, (GPIO_PinState)state);
    }

    return A4988_ERR_OK;
}

a4988_err_t a4988_pwm_start(void* user)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->a4988_pwm_timer) {
        HAL_TIM_PWM_Start_IT(manager->a4988_pwm_timer, manager->a4988_pwm_channel);
    }

    return A4988_ERR_OK;
}

a4988_err_t a4988_pwm_stop(void* user)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->a4988_pwm_timer) {
        HAL_TIM_PWM_Stop_IT(manager->a4988_pwm_timer, manager->a4988_pwm_channel);
    }

    return A4988_ERR_OK;
}

a4988_err_t a4988_pwm_set_frequency(void* user, uint32_t frequency)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    uint32_t prescaler;
    uint32_t period;
    if (frequency_to_prescaler_and_period(frequency,
                                          80000000U,
                                          0x0U,
                                          0xFFFFU,
                                          0xFFFFU,
                                          &prescaler,
                                          &period)) {
        if (manager->a4988_pwm_timer) {
            __HAL_TIM_DISABLE(manager->a4988_pwm_timer);
            __HAL_TIM_SET_PRESCALER(manager->a4988_pwm_timer, prescaler);
            __HAL_TIM_SET_AUTORELOAD(manager->a4988_pwm_timer, period);
            __HAL_TIM_SET_COMPARE(manager->a4988_pwm_timer,
                                  manager->a4988_pwm_channel,
                                  period / 2U);
            __HAL_TIM_ENABLE(manager->a4988_pwm_timer);
        }
    }

    return A4988_ERR_OK;
}

as5600_err_t as5600_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->as5600_gpio) {
        HAL_GPIO_WritePin(manager->as5600_gpio, pin, (GPIO_PinState)state);
    }

    return A4988_ERR_OK;
}

as5600_err_t as5600_bus_write_data(void* user,
                                   uint8_t address,
                                   uint8_t const* data,
                                   size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->as5600_ina226_i2c_bus) {
        if (xSemaphoreTake(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS), pdMS_TO_TICKS(10))) {
            HAL_I2C_Mem_Write(manager->as5600_ina226_i2c_bus,
                              manager->as5600_i2c_address,
                              address,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t*)data,
                              data_size,
                              100U);

            xSemaphoreGive(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS));
        }
    }

    return AS5600_ERR_OK;
}

as5600_err_t as5600_bus_read_data(void* user, uint8_t address, uint8_t* data, size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->as5600_ina226_i2c_bus) {
        if (xSemaphoreTake(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS), pdMS_TO_TICKS(10))) {
            HAL_I2C_Mem_Read(manager->as5600_ina226_i2c_bus,
                             manager->as5600_i2c_address,
                             address,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             data_size,
                             100U);

            xSemaphoreGive(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS));
        }
    }

    return AS5600_ERR_OK;
}

ina226_err_t ina226_bus_write_data(void* user,
                                   uint8_t address,
                                   uint8_t const* data,
                                   size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->as5600_ina226_i2c_bus) {
        if (xSemaphoreTake(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS), pdMS_TO_TICKS(10))) {
            HAL_I2C_Mem_Write(manager->as5600_ina226_i2c_bus,
                              manager->ina226_i2c_address,
                              address,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t*)data,
                              data_size,
                              100U);

            xSemaphoreGive(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS));
        }
    }

    return INA226_ERR_OK;
}

ina226_err_t ina226_bus_read_data(void* user, uint8_t address, uint8_t* data, size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->as5600_ina226_i2c_bus) {
        if (xSemaphoreTake(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS), pdMS_TO_TICKS(10))) {
            HAL_I2C_Mem_Read(manager->as5600_ina226_i2c_bus,
                             manager->ina226_i2c_address,
                             address,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             data_size,
                             100U);

            xSemaphoreGive(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS));
        }
    }

    return INA226_ERR_OK;
}

step_motor_err_t step_motor_device_set_frequency(void* user, uint32_t frequency)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_frequency(&manager->a4988, frequency);

    return STEP_MOTOR_ERR_OK;
}

step_motor_err_t step_motor_device_set_direction(void* user, step_motor_direction_t direction)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_direction(&manager->a4988, (a4988_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

motor_driver_err_t motor_driver_joint_set_speed(void* user, float32_t speed)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    step_motor_set_speed(&manager->motor, speed);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_encoder_get_position(void* user, float32_t* position)
{
    ATLAS_ASSERT(user && position);

    joint_manager_t* manager = (joint_manager_t*)user;

    as5600_get_angle_data_scaled_bus(&manager->as5600, position);
    manager->measure_position = *position;

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_regulator_get_control(void* user,
                                                      float32_t error,
                                                      float32_t* control,
                                                      float32_t delta_time)
{
    ATLAS_ASSERT(user && control);

    joint_manager_t* manager = (joint_manager_t*)user;

    *control = pid_regulator_get_sat_control(&manager->regulator, error, delta_time);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_fault_get_current(void* user, float32_t* current)
{
    ATLAS_ASSERT(user && current);

    joint_manager_t* manager = (joint_manager_t*)user;

    *current = 1.0F;

    return MOTOR_DRIVER_ERR_OK;
}
