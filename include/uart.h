/**
 * UART abstraction for MT7628 AT daemon.
 * Provides line-oriented receive with callback registration.
 */
#ifndef UART_H
#define UART_H

#include <stddef.h>
#include <sys/types.h>

typedef struct uart_inst uart_inst_t;

typedef void (*uart_callback_t)(uart_inst_t *uart, const char *line, void *user_data);

uart_inst_t* uart_init(size_t rx_buffer_size);
int          uart_configure(uart_inst_t *uart, const char *device, int baudrate, const char *mode);
int          uart_open(uart_inst_t *uart);
ssize_t      uart_write(uart_inst_t *uart, const void *data, size_t len);
int          uart_register_callback(uart_inst_t *uart, const char *pattern,
                                    uart_callback_t cb, void *user_data);
void         uart_process_events(uart_inst_t *uart);
void         uart_destroy(uart_inst_t *uart);

/* Helper to get underlying file descriptor for select()/poll() */
int          uart_get_fd(uart_inst_t *uart);

#endif /* UART_H */

