#include "FreeRTOS.h"
#include "manager.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"
#include "usart.h"

__attribute__((used)) int _write(int file, char* ptr, int len)
{
    StreamBufferHandle_t uart_stream_buffer = stream_buffer_manager_get(STREAM_BUFFER_TYPE_UART);
    SemaphoreHandle_t uart_mutex = semaphore_manager_get(SEMAPHORE_TYPE_UART);

    size_t written = 0UL;

    if (xSemaphoreTake(uart_mutex, 0)) {
        written = xStreamBufferSend(uart_stream_buffer, ptr, len, 0);

        xSemaphoreGive(uart_mutex);
    }

    return written;
}