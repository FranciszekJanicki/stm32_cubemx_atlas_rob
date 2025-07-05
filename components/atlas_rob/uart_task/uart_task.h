#ifndef UART_TASK_UART_TASK_H
#define UART_TASK_UART_TASK_H

#include "common.h"

void uart_task_initialize(void);
void uart_stream_buffer_initialize(void);
void uart_mutex_initialize(void);

#endif // UART_TASK_UART_TASK_H