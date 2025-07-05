#ifndef JOINT_TASK_JOINT_MANAGER_H
#define JOINT_TASK_JOINT_MANAGER_H

#include "FreeRTOS.h"
#include "a4988.h"
#include "as5600.h"
#include "common.h"
#include "ina226.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "queue.h"
#include "semphr.h"
#include "step_motor.h"
#include "task.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32l476xx.h>
#include <stm32l4xx_hal.h>

typedef struct {
    float32_t joint_position;
    float32_t delta_time;
    bool is_running;

    a4988_t a4988;
    ina226_t ina226;
    as5600_t as5600;
    step_motor_t motor;
    pid_regulator_t regulator;
    motor_driver_t driver;

    QueueHandle_t joint_queue;

    GPIO_TypeDef* a4988_gpio;
    uint32_t a4988_dir_pin;

    TIM_HandleTypeDef* a4988_pwm_timer;
    uint32_t a4988_pwm_channel;

    I2C_HandleTypeDef* as5600_ina226_i2c_bus;
    uint16_t as5600_i2c_address;
    uint16_t ina226_i2c_address;

    GPIO_TypeDef* as5600_gpio;
    uint32_t as5600_dir_pin;
} joint_manager_t;

atlas_err_t joint_manager_initialize(joint_manager_t* manager,
                                     atlas_joint_config_t const* config,
                                     atlas_joint_num_t num);
atlas_err_t joint_manager_process(joint_manager_t* manager);

#endif // JOINT_TASK_JOINT_MANAGER_H