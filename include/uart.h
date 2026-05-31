/**
 * uart.h — UART abstraction layer for the MT7628N AT command daemon.
 *
 * Provides a line-oriented receive interface over a standard POSIX tty.
 * Callers register callbacks that are invoked for each complete line
 * received; the implementation handles termios configuration, RX
 * buffering, and line extraction (\n, \r, or \r\n terminators).
 *
 * Typical usage:
 *   uart_inst_t *u = uart_init(2048);
 *   uart_configure(u, "ttyS1", 115200, "8N1");
 *   uart_open(u);
 *   uart_register_callback(u, "", my_cb, NULL);
 *   // in select() loop:
 *   uart_process_events(u);
 *   // on exit:
 *   uart_destroy(u);
 */
#ifndef UART_H
#define UART_H

#include <stddef.h>
#include <sys/types.h>

/** Opaque UART instance; allocate with uart_init(). */
typedef struct uart_inst uart_inst_t;

/**
 * uart_callback_t — line-received callback signature.
 *
 * Called by uart_process_events() for each complete line extracted
 * from the receive buffer.  The line is null-terminated and does NOT
 * include the \r or \n terminator characters.
 *
 * Parameters:
 *   uart       - The UART instance that received the line
 *   line       - Null-terminated received line (max 255 chars)
 *   user_data  - Opaque pointer supplied at registration time
 */
typedef void (*uart_callback_t)(uart_inst_t *uart, const char *line, void *user_data);

/**
 * uart_init — allocate and zero-initialise a UART instance.
 *
 * @param rx_buffer_size  Internal RX ring-buffer size in bytes (recommended: 2048).
 * @return  New uart_inst_t*, or NULL on allocation failure.
 *          Caller must free with uart_destroy().
 */
uart_inst_t* uart_init(size_t rx_buffer_size);

/**
 * uart_configure — store device path, baud rate, and serial mode.
 *
 * Does NOT open the port.  Call uart_open() after this.
 *
 * @param uart      Initialised instance from uart_init().
 * @param device    Device name without "/dev/" prefix (e.g. "ttyS1").
 * @param baudrate  Baud rate integer (e.g. 115200).
 * @param mode      Serial mode string, e.g. "8N1".
 * @return  0 on success, -1 on invalid arguments or unsupported values.
 */
int uart_configure(uart_inst_t *uart, const char *device, int baudrate, const char *mode);

/**
 * uart_open — open the tty and apply termios settings.
 *
 * Flushes any stale RX bytes before returning.
 *
 * @param uart  Configured instance; must not already be open.
 * @return  0 on success, -1 on failure (open(2) or tcsetattr failure).
 */
int uart_open(uart_inst_t *uart);

/**
 * uart_write — transmit raw bytes to the UART.
 *
 * Thin wrapper around write(2); no internal buffering.
 *
 * @param uart  Open UART instance.
 * @param data  Bytes to transmit.
 * @param len   Number of bytes.
 * @return  Bytes written (may be less than len), or -1 on error.
 */
ssize_t uart_write(uart_inst_t *uart, const void *data, size_t len);

/**
 * uart_register_callback — register a line-received handler.
 *
 * Up to 16 callbacks may be registered per instance.  Currently all
 * callbacks are invoked for every complete line regardless of the
 * pattern argument (pattern is reserved for future prefix filtering;
 * pass "" for now).
 *
 * @param uart       UART instance.
 * @param pattern    Reserved; pass "" (empty string).
 * @param cb         Callback function; must not be NULL.
 * @param user_data  Opaque pointer forwarded to cb on each invocation.
 * @return  0 on success, -1 if arguments are invalid or the table is full.
 */
int uart_register_callback(uart_inst_t *uart, const char *pattern,
                           uart_callback_t cb, void *user_data);

/**
 * uart_process_events — read available bytes and dispatch complete lines.
 *
 * Call this after select()/poll() indicates the UART fd is readable.
 * Lines longer than 255 bytes are rejected with an "ERROR:6\r\n" reply.
 *
 * @param uart  Open UART instance; no-op if NULL or not open.
 */
void uart_process_events(uart_inst_t *uart);

/**
 * uart_destroy — close the port and free all resources.
 *
 * Safe to call with NULL (no-op).  After this call the pointer is invalid.
 *
 * @param uart  Instance to destroy.
 */
void uart_destroy(uart_inst_t *uart);

/**
 * uart_get_fd — return the raw file descriptor for select()/poll().
 *
 * @param uart  Any initialised uart_inst_t (need not be open).
 * @return  File descriptor (>= 0) if open, -1 otherwise.
 */
int uart_get_fd(uart_inst_t *uart);

#endif /* UART_H */
