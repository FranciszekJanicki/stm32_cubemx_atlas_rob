#include "joint_manager.h"
#include "FreeRTOS.h"
#include "a4988.h"
#include "common.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "step_motor.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

static char const* const TAG = "joint_manager";

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

static a4988_err_t a4988_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->a4988_gpio) {
        HAL_GPIO_WritePin(manager->a4988_gpio, pin, (GPIO_PinState)state);
    }

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_start(void* user)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->a4988_pwm_timer) {
        HAL_TIM_PWM_Start_IT(manager->a4988_pwm_timer, manager->a4988_pwm_channel);
    }

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_stop(void* user)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->a4988_pwm_timer) {
        HAL_TIM_PWM_Stop_IT(manager->a4988_pwm_timer, manager->a4988_pwm_channel);
    }

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_set_frequency(void* user, uint32_t frequency)
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

static as5600_err_t as5600_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    if (manager->as5600_gpio) {
        HAL_GPIO_WritePin(manager->as5600_gpio, pin, (GPIO_PinState)state);
    }

    return A4988_ERR_OK;
}

static as5600_err_t as5600_bus_write_data(void* user,
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

static as5600_err_t as5600_bus_read_data(void* user,
                                         uint8_t address,
                                         uint8_t* data,
                                         size_t data_size)
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

static ina226_err_t ina226_bus_write_data(void* user,
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

static ina226_err_t ina226_bus_read_data(void* user,
                                         uint8_t address,
                                         uint8_t* data,
                                         size_t data_size)
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

static step_motor_err_t step_motor_device_set_frequency(void* user, uint32_t frequency)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_frequency(&manager->a4988, frequency);

    return STEP_MOTOR_ERR_OK;
}

static step_motor_err_t step_motor_device_set_direction(void* user,
                                                        step_motor_direction_t direction)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_direction(&manager->a4988, (a4988_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

static motor_driver_err_t motor_driver_joint_set_speed(void* user, float32_t speed)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    step_motor_set_speed(&manager->motor, speed);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_encoder_get_position(void* user, float32_t* position)
{
    ATLAS_ASSERT(user && position);

    joint_manager_t* manager = (joint_manager_t*)user;

    *position = step_motor_get_position(&manager->motor);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_regulator_get_control(void* user,
                                                             float32_t error,
                                                             float32_t* control,
                                                             float32_t delta_time)
{
    ATLAS_ASSERT(user && control);

    joint_manager_t* manager = (joint_manager_t*)user;

    *control = pid_regulator_get_sat_control(&manager->regulator, error, delta_time);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_fault_get_current(void* user, float32_t* current)
{
    ATLAS_ASSERT(user && current);

    joint_manager_t* manager = (joint_manager_t*)user;

    *current = 1.0F;

    return MOTOR_DRIVER_ERR_OK;
}

static inline bool joint_manager_has_joint_event(QueueHandle_t queue)
{
    ATLAS_ASSERT(queue);

    return uxQueueMessagesWaiting(queue);
}

static inline bool joint_manager_send_joints_notify(joints_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_JOINTS), (uint32_t)notify, eSetBits) == pdPASS;
}

static inline bool joint_manager_receive_joint_event(QueueHandle_t queue, joint_event_t* event)
{
    ATLAS_ASSERT(queue && event);

    return xQueueReceive(queue, event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joint_manager_receive_joint_notify(joint_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, JOINT_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(1)) == pdPASS;
}

static atlas_err_t joint_manager_event_start_handler(joint_manager_t* manager,
                                                     joint_event_payload_start_t const* payload)
{
    ATLAS_ASSERT(manager && payload);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    step_motor_reset(&manager->motor);

    manager->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_stop_handler(joint_manager_t* manager,
                                                    joint_event_payload_stop_t const* payload)
{
    ATLAS_ASSERT(manager && payload);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    step_motor_reset(&manager->motor);

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_position_handler(
    joint_manager_t* manager,
    joint_event_payload_position_t const* position)
{
    ATLAS_ASSERT(manager && position);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    ATLAS_LOG(TAG, "joint_position: %d * 100", (int32_t)*position * 100);

    manager->joint_position = *position;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_notify_delta_timer_handler(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    motor_driver_set_position(&manager->driver, manager->joint_position, manager->delta_time);

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_notify_pwm_pulse_handler(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    step_motor_update_step_count(&manager->motor);

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_notify_handler(joint_manager_t* manager, joint_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & JOINT_NOTIFY_DELTA_TIMER) {
        ATLAS_RET_ON_ERR(joint_manager_notify_delta_timer_handler(manager));
    }
    if (notify & JOINT_NOTIFY_PWM_PULSE) {
        ATLAS_RET_ON_ERR(joint_manager_notify_pwm_pulse_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_handler(joint_manager_t* manager, joint_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case JOINT_EVENT_TYPE_START:
            return joint_manager_event_start_handler(manager, &event->payload.start);
        case JOINT_EVENT_TYPE_STOP:
            return joint_manager_event_stop_handler(manager, &event->payload.stop);
        case JOINT_EVENT_TYPE_POSITION:
            return joint_manager_event_position_handler(manager, &event->payload.position);
        default:
            return ATLAS_ERR_UNKNOWN_EVENT;
    }
}

atlas_err_t joint_manager_process(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    joint_notify_t notify;
    if (joint_manager_receive_joint_notify(&notify)) {
        ATLAS_RET_ON_ERR(joint_manager_notify_handler(manager, notify));
    }

    joint_event_t event;
    while (joint_manager_has_joint_event(manager->joint_queue)) {
        if (joint_manager_receive_joint_event(manager->joint_queue, &event)) {
            ATLAS_RET_ON_ERR(joint_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t joint_manager_initialize(joint_manager_t* manager,
                                     atlas_joint_config_t const* config,
                                     atlas_joint_num_t num)
{
    ATLAS_ASSERT(manager && config);

    manager->is_running = false;

    as5600_initialize(&manager->as5600,
                      &(as5600_config_t){.dir_pin = manager->as5600_dir_pin,
                                         .max_angle = config->max_position,
                                         .min_angle = config->min_position,
                                         .pgo_pin = 0x00},
                      &(as5600_interface_t){.gpio_user = manager,
                                            .gpio_write_pin = as5600_gpio_write_pin,
                                            .bus_user = manager,
                                            .bus_read_data = as5600_bus_read_data,
                                            .bus_write_data = as5600_bus_write_data});

    a4988_initialize(&manager->a4988,
                     &(a4988_config_t){.pin_dir = manager->a4988_dir_pin},
                     &(a4988_interface_t){.gpio_user = manager,
                                          .gpio_write_pin = a4988_gpio_write_pin,
                                          .pwm_user = manager,
                                          .pwm_start = a4988_pwm_start,
                                          .pwm_stop = a4988_pwm_stop,
                                          .pwm_set_frequency = a4988_pwm_set_frequency});

    step_motor_initialize(
        &manager->motor,
        &(step_motor_config_t){.min_position = config->min_position,
                               .max_position = config->max_position,
                               .min_speed = config->min_speed,
                               .max_speed = config->max_speed,
                               .step_change = 1.8F},
        &(step_motor_interface_t){.device_user = manager,
                                  .device_set_frequency = step_motor_device_set_frequency,
                                  .device_set_direction = step_motor_device_set_direction},
        0.0F);

    pid_regulator_initialize(&manager->regulator,
                             &(pid_regulator_config_t){.prop_gain = config->prop_gain,
                                                       .int_gain = config->int_gain,
                                                       .dot_gain = config->dot_gain,
                                                       .sat_gain = config->sat_gain,
                                                       .min_control = config->min_speed,
                                                       .max_control = config->max_speed});

    motor_driver_initialize(
        &manager->driver,
        &(motor_driver_config_t){.min_position = config->min_position,
                                 .max_position = config->max_position,
                                 .min_speed = config->min_speed,
                                 .max_speed = config->max_speed,
                                 .max_current = 2.0F},
        &(motor_driver_interface_t){.motor_user = manager,
                                    .motor_set_speed = motor_driver_joint_set_speed,
                                    .encoder_user = manager,
                                    .encoder_get_position = motor_driver_encoder_get_position,
                                    .regulator_user = manager,
                                    .regulator_get_control = motor_driver_regulator_get_control,
                                    .fault_user = manager,
                                    .fault_get_current = motor_driver_fault_get_current});

    if (!joint_manager_send_joints_notify(1 << num)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}
