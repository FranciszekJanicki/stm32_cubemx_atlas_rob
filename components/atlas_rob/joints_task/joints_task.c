#include "joints_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "i2c.h"
#include "joint_task.h"
#include "joints_manager.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include <assert.h>
#include <stdint.h>

#define JOINTS_TASK_STACK_DEPTH (5000U / sizeof(StackType_t))
#define JOINTS_TASK_PRIORITY (1U)
#define JOINTS_TASK_NAME ("joints_task")
#define JOINTS_TASK_ARGUMENT (NULL)

#define JOINTS_QUEUE_ITEMS (10U)
#define JOINTS_QUEUE_ITEM_SIZE (sizeof(joints_event_t))
#define JOINTS_QUEUE_STORAGE_SIZE (JOINTS_QUEUE_ITEMS * JOINTS_QUEUE_ITEM_SIZE)

static joint_task_ctx_t joint_task_ctxs[ATLAS_JOINT_NUM];
static joints_manager_t joints_manager;

static void joints_task_func(void*)
{
    ATLAS_LOG_ON_ERR(JOINTS_TASK_NAME, joints_manager_initialize(&joints_manager));

    while (1) {
        ATLAS_LOG_ON_ERR(JOINTS_TASK_NAME, joints_manager_process(&joints_manager));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void joint_ctxs_initailize(void)
{
    joint_task_ctxs[ATLAS_JOINT_NUM_1] =
        (joint_task_ctx_t){.interface = {.a4988_pwm_timer = &htim1,
                                         .a4988_pwm_channel = TIM_CHANNEL_4,
                                         .a4988_gpio = GPIOA,
                                         .a4988_dir_pin = GPIO_PIN_10,
                                         .tca9548_i2c_bus = &hi2c1,
                                         .tca9548_i2c_address = TCA9548_DEV_ADDRESS_A2_L_A1_L_A0_L,
                                         .ina226_i2c_bus = &hi2c1,
                                         .as5600_i2c_address = 0x00,
                                         .as5600_gpio = GPIOA,
                                         .as5600_dir_pin = 0x00,
                                         .ina226_i2c_address = 0x00},
                           .config = {.prop_gain = 10.0F,
                                      .int_gain = 0.0F,
                                      .dot_gain = 0.0F,
                                      .sat_gain = 0.0F,
                                      .min_position = 0.0F,
                                      .max_position = 359.0F,
                                      .min_speed = 10.0F,
                                      .max_speed = 500.0F,
                                      .step_change = 1.8F,
                                      .current_limit = 2.0F}};
}

static void joint_tasks_initialize(void)
{
    static uint8_t joint_queue_storages[ATLAS_JOINT_NUM][JOINT_QUEUE_STORAGE_SIZE];
    static StaticQueue_t joint_queue_buffers[ATLAS_JOINT_NUM];

    static StackType_t joint_task_stacks[ATLAS_JOINT_NUM][JOINT_TASK_STACK_DEPTH];
    static StaticTask_t joint_task_buffers[ATLAS_JOINT_NUM];

    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        QueueHandle_t joint_queue =
            joint_task_create_queue(&joint_queue_buffers[num], &joint_queue_storages[num]);

        joint_task_ctxs[num].interface.joint_queue = joint_queue;
        joint_task_ctxs[num].interface.num = num;

        TaskHandle_t joint_task = joint_task_create_task(&joint_task_ctxs[num],
                                                         &joint_task_buffers[num],
                                                         &joint_task_stacks[num]);

        joints_manager.joint_ctxs[num].queue = joint_queue;
        joints_manager.joint_ctxs[num].task = joint_task;
    }
}

static TaskHandle_t joints_task_create_task(void)
{
    static StaticTask_t joints_task_buffer;
    static StackType_t joints_task_stack[JOINTS_TASK_STACK_DEPTH];

    return xTaskCreateStatic(joints_task_func,
                             JOINTS_TASK_NAME,
                             JOINTS_TASK_STACK_DEPTH,
                             JOINTS_TASK_ARGUMENT,
                             JOINTS_TASK_PRIORITY,
                             joints_task_stack,
                             &joints_task_buffer);
}

static QueueHandle_t joints_task_create_queue(void)
{
    static StaticQueue_t joints_queue_buffer;
    static uint8_t joints_queue_storage[JOINTS_QUEUE_STORAGE_SIZE];

    return xQueueCreateStatic(JOINTS_QUEUE_ITEMS,
                              JOINTS_QUEUE_ITEM_SIZE,
                              joints_queue_storage,
                              &joints_queue_buffer);
}

static SemaphoreHandle_t joints_task_create_mutex(void)
{
    static StaticSemaphore_t joints_mutex_buffer;

    return xSemaphoreCreateMutexStatic(&joints_mutex_buffer);
}

void joints_task_initialize(void)
{
    joint_ctxs_initailize();
    joint_tasks_initialize();

    task_manager_set(TASK_TYPE_JOINTS, joints_task_create_task());
    queue_manager_set(QUEUE_TYPE_JOINTS, joints_task_create_queue());
    semaphore_manager_set(SEMAPHORE_TYPE_JOINTS, joints_task_create_mutex());
}

void joint_pwm_pulse_callback(atlas_joint_num_t num)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(joints_manager.joint_ctxs[num].task,
                       JOINT_NOTIFY_PWM_PULSE,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

void joints_delta_timer_callback(void)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_JOINTS),
                       JOINTS_NOTIFY_DELTA_TIMER,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

#undef JOINTS_TASK_STACK_DEPTH
#undef JOINTS_TASK_PRIORITY
#undef JOINTS_TASK_NAME
#undef JOINTS_TASK_ARGUMENT

#undef JOINTS_QUEUE_ITEMS
#undef JOINTS_QUEUE_ITEM_SIZE
#undef JOINTS_QUEUE_STORAGE_SIZE