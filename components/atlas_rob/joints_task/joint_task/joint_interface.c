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

static tca9548_t joint_tca9548 = {};

tca9548_err_t joint_tca9548_bus_write_data(void* user,
                                           uint8_t address,
                                           uint8_t const* data,
                                           size_t data_size)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    if (interface->tca9548_i2c_bus) {
        HAL_I2C_Mem_Write(interface->tca9548_i2c_bus,
                          interface->tca9548_i2c_address,
                          address,
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t*)data,
                          data_size,
                          100U);
    }

    return AS5600_ERR_OK;
}

tca9548_err_t joint_tca9548_bus_read_data(void* user,
                                          uint8_t address,
                                          uint8_t* data,
                                          size_t data_size)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    if (interface->tca9548_i2c_bus) {
        HAL_I2C_Mem_Read(interface->tca9548_i2c_bus,
                         interface->tca9548_i2c_address,
                         address,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         data_size,
                         100U);
    }

    return AS5600_ERR_OK;
}

tca9548_err_t joints_tca9548_initialize(void)
{
    return tca9548_initialize(&joint_tca9548, &(tca9548_config_t){}, &(tca9548_interface_t){});
}

a4988_err_t joint_a4988_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    if (interface->a4988_gpio) {
        HAL_GPIO_WritePin(interface->a4988_gpio, pin, (GPIO_PinState)state);
    }

    return A4988_ERR_OK;
}

a4988_err_t joint_a4988_pwm_start(void* user)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    if (interface->a4988_pwm_timer) {
        HAL_TIM_PWM_Start_IT(interface->a4988_pwm_timer, interface->a4988_pwm_channel);
    }

    return A4988_ERR_OK;
}

a4988_err_t joint_a4988_pwm_stop(void* user)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    if (interface->a4988_pwm_timer) {
        HAL_TIM_PWM_Stop_IT(interface->a4988_pwm_timer, interface->a4988_pwm_channel);
    }

    return A4988_ERR_OK;
}

a4988_err_t joint_a4988_pwm_set_frequency(void* user, uint32_t frequency)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    uint32_t prescaler;
    uint32_t period;
    if (frequency_to_prescaler_and_period(frequency,
                                          80000000U,
                                          0x0U,
                                          0xFFFFU,
                                          0xFFFFU,
                                          &prescaler,
                                          &period)) {
        if (interface->a4988_pwm_timer) {
            __HAL_TIM_DISABLE(interface->a4988_pwm_timer);
            __HAL_TIM_SET_PRESCALER(interface->a4988_pwm_timer, prescaler);
            __HAL_TIM_SET_AUTORELOAD(interface->a4988_pwm_timer, period);
            __HAL_TIM_SET_COMPARE(interface->a4988_pwm_timer,
                                  interface->a4988_pwm_channel,
                                  period / 2U);
            __HAL_TIM_ENABLE(interface->a4988_pwm_timer);
        }
    }

    return A4988_ERR_OK;
}

as5600_err_t joint_as5600_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    if (interface->as5600_gpio) {
        HAL_GPIO_WritePin(interface->as5600_gpio, pin, (GPIO_PinState)state);
    }

    return A4988_ERR_OK;
}

as5600_err_t joint_as5600_bus_write_data(void* user,
                                         uint8_t address,
                                         uint8_t const* data,
                                         size_t data_size)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    if (xSemaphoreTake(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS), pdMS_TO_TICKS(1))) {
        tca9548_write_channel_data(&joint_tca9548,
                                   interface->as5600_tca9548_channel,
                                   address,
                                   data,
                                   data_size);
        xSemaphoreGive(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS));
    }

    return AS5600_ERR_OK;
}

as5600_err_t joint_as5600_bus_read_data(void* user,
                                        uint8_t address,
                                        uint8_t* data,
                                        size_t data_size)
{
    joint_interface_t* interface = (joint_interface_t*)user;

    if (xSemaphoreTake(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS), pdMS_TO_TICKS(1))) {
        tca9548_read_channel_data(&joint_tca9548,
                                  interface->as5600_tca9548_channel,
                                  address,
                                  data,
                                  data_size);
        xSemaphoreGive(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS));
    }

    return AS5600_ERR_OK;
}

ina226_err_t joint_ina226_bus_write_data(void* user,
                                         uint8_t address,
                                         uint8_t const* data,
                                         size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_interface_t* interface = (joint_interface_t*)user;

    if (interface->ina226_i2c_bus) {
        if (xSemaphoreTake(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS), pdMS_TO_TICKS(1))) {
            HAL_I2C_Mem_Write(interface->ina226_i2c_bus,
                              interface->ina226_i2c_address,
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

ina226_err_t joint_ina226_bus_read_data(void* user,
                                        uint8_t address,
                                        uint8_t* data,
                                        size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_interface_t* interface = (joint_interface_t*)user;

    if (interface->ina226_i2c_bus) {
        if (xSemaphoreTake(semaphore_manager_get(SEMAPHORE_TYPE_JOINTS), pdMS_TO_TICKS(1))) {
            HAL_I2C_Mem_Read(interface->ina226_i2c_bus,
                             interface->ina226_i2c_address,
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

step_motor_err_t joint_step_motor_device_set_frequency(void* user, uint32_t frequency)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_frequency(&manager->a4988, frequency);

    return STEP_MOTOR_ERR_OK;
}

step_motor_err_t joint_step_motor_device_set_direction(void* user, step_motor_direction_t direction)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_direction(&manager->a4988, (a4988_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

motor_driver_err_t joint_motor_driver_motor_set_speed(void* user, float32_t speed)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    step_motor_set_speed(&manager->motor, speed);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t joint_motor_driver_encoder_get_position(void* user, float32_t* position)
{
    ATLAS_ASSERT(user && position);

    joint_manager_t* manager = (joint_manager_t*)user;

    as5600_get_angle_data_scaled_bus(&manager->as5600, position);
    manager->measure_position = *position;

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t joint_motor_driver_regulator_get_control(void* user,
                                                            float32_t error,
                                                            float32_t* control,
                                                            float32_t delta_time)
{
    ATLAS_ASSERT(user && control);

    joint_manager_t* manager = (joint_manager_t*)user;

    *control = pid_regulator_get_sat_control(&manager->regulator, error, delta_time);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t joint_motor_driver_fault_get_current(void* user, float32_t* current)
{
    ATLAS_ASSERT(user && current);

    *current = 1.0F;

    return MOTOR_DRIVER_ERR_OK;
}
