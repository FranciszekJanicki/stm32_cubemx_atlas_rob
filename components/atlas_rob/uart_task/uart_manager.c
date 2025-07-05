#include "uart_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "task.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static size_t uart_manager_receive_stream_data(StreamBufferHandle_t stream_buffer,
                                               uint8_t* stream_data,
                                               size_t stream_data_len)
{
    ATLAS_ASSERT(stream_buffer && stream_data);

    return xStreamBufferReceive(stream_buffer, stream_data, stream_data_len, portMAX_DELAY);
}

static bool uart_manager_transmit_uart_data(UART_HandleTypeDef* uart,
                                            uint8_t const* uart_data,
                                            size_t uart_data_len)
{
    ATLAS_ASSERT(uart && uart_data);

    return HAL_UART_Transmit_IT(uart, uart_data, uart_data_len) == HAL_OK;
}

static bool uart_manager_receive_uart_notify(uart_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, UART_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(10)) == pdPASS;
}

static atlas_err_t uart_manager_notify_tx_cplt_handler(uart_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    memset(manager->uart_buffer, 0, sizeof(manager->uart_buffer));

    size_t received_size = uart_manager_receive_stream_data(manager->stream_buffer,
                                                            manager->uart_buffer,
                                                            sizeof(manager->uart_buffer));

    if (received_size > 0UL) {
        if (!uart_manager_transmit_uart_data(manager->uart, manager->uart_buffer, received_size)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t uart_manager_notify_handler(uart_manager_t* manager, uart_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & UART_NOTIFY_TRANSMIT_COMPLETE) {
        ATLAS_RET_ON_ERR(uart_manager_notify_tx_cplt_handler(manager));
    }

    return ATLAS_ERR_OK;
}

atlas_err_t uart_manager_process(uart_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    uart_notify_t notify;
    if (uart_manager_receive_uart_notify(&notify)) {
        ATLAS_RET_ON_ERR(uart_manager_notify_handler(manager, notify));
    }

    return ATLAS_ERR_OK;
}

atlas_err_t uart_manager_initialize(uart_manager_t* manager,
                                    StreamBufferHandle_t stream_buffer,
                                    UART_HandleTypeDef* uart)
{
    ATLAS_ASSERT(manager && stream_buffer && uart);

    manager->stream_buffer = stream_buffer;
    manager->uart = uart;

    return uart_manager_notify_tx_cplt_handler(manager);
}

void uart_transmit_cplt_callback(void)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_UART),
                       UART_NOTIFY_TRANSMIT_COMPLETE,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}