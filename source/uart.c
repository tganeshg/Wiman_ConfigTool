#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>

#include "uart.h"

struct uart_inst {
    int         fd;
    char        device[64];
    speed_t     baud_rate;
    tcflag_t    data_bits;
    tcflag_t    parity;
    tcflag_t    stop_bits;
    tcflag_t    iflags;
    struct      termios tios;
    int         timeout_ms;

    unsigned char *rx_buffer;
    size_t      rx_buffer_size;
    size_t      rx_data_len;

    char        line_buffer[256];
    size_t      line_len;

    struct {
        char        pattern[64];
        uart_callback_t callback;
        void       *user_data;
    } callbacks[16];
    int         callback_count;
};

static speed_t uart_baud_to_const(int baudrate) {
    switch(baudrate) {
        case 115200: return B115200;
        default: return B0;
    }
}

static int uart_parse_mode(const char *mode, tcflag_t *data_bits, tcflag_t *parity,
                           tcflag_t *stop_bits, tcflag_t *iflags) {
    if (!mode || strlen(mode) != 3) return -1;

    switch(mode[0]) {
        case '8': *data_bits = CS8; break;
        case '7': *data_bits = CS7; break;
        case '6': *data_bits = CS6; break;
        case '5': *data_bits = CS5; break;
        default: return -1;
    }

    switch(mode[1]) {
        case 'N': case 'n':
            *parity = 0;
            *iflags = IGNPAR;
            break;
        case 'E': case 'e':
            *parity = PARENB;
            *iflags = INPCK;
            break;
        case 'O': case 'o':
            *parity = PARENB | PARODD;
            *iflags = INPCK;
            break;
        default:
            return -1;
    }

    switch(mode[2]) {
        case '1': *stop_bits = 0; break;
        case '2': *stop_bits = CSTOPB; break;
        default: return -1;
    }

    return 0;
}

uart_inst_t* uart_init(size_t rx_buffer_size) {
    uart_inst_t *uart = calloc(1, sizeof(uart_inst_t));
    if (!uart) return NULL;
    uart->fd = -1;
    uart->timeout_ms = 100;
    uart->rx_buffer = malloc(rx_buffer_size);
    if (!uart->rx_buffer) {
        free(uart);
        return NULL;
    }
    uart->rx_buffer_size = rx_buffer_size;
    uart->callback_count = 0;
    return uart;
}

int uart_configure(uart_inst_t *uart, const char *device, int baudrate, const char *mode) {
    if (!uart || !device || !mode) return -1;

    snprintf(uart->device, sizeof(uart->device), "/dev/%s", device);

    uart->baud_rate = uart_baud_to_const(baudrate);
    if (uart->baud_rate == 0) return -1;

    return uart_parse_mode(mode, &uart->data_bits, &uart->parity,
                           &uart->stop_bits, &uart->iflags);
}

int uart_open(uart_inst_t *uart) {
    if (!uart || uart->fd >= 0) return -1;

    uart->fd = open(uart->device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart->fd < 0) return -1;

    memset(&uart->tios, 0, sizeof(uart->tios));
    tcgetattr(uart->fd, &uart->tios);

    cfmakeraw(&uart->tios);

    uart->tios.c_cflag = uart->data_bits | uart->parity | uart->stop_bits |
                         CLOCAL | CREAD | HUPCL;
    uart->tios.c_iflag = uart->iflags;
    uart->tios.c_cc[VMIN] = 0;
    uart->tios.c_cc[VTIME] = uart->timeout_ms / 100;

    cfsetispeed(&uart->tios, uart->baud_rate);
    cfsetospeed(&uart->tios, uart->baud_rate);

    tcflush(uart->fd, TCIFLUSH);

    if (tcsetattr(uart->fd, TCSANOW, &uart->tios) < 0) {
        close(uart->fd);
        uart->fd = -1;
        return -1;
    }

    return 0;
}

ssize_t uart_write(uart_inst_t *uart, const void *data, size_t len) {
    if (!uart || uart->fd < 0) return -1;
    return write(uart->fd, data, len);
}

int uart_register_callback(uart_inst_t *uart, const char *pattern,
                           uart_callback_t cb, void *user_data) {
    if (!uart || !pattern || !cb || uart->callback_count >= 16) return -1;

    strncpy(uart->callbacks[uart->callback_count].pattern, pattern,
            sizeof(uart->callbacks[0].pattern)-1);
    uart->callbacks[uart->callback_count].callback = cb;
    uart->callbacks[uart->callback_count].user_data = user_data;
    uart->callback_count++;

    return 0;
}

void uart_process_events(uart_inst_t *uart) {
    if (!uart || uart->fd < 0) return;

    ssize_t n = read(uart->fd, uart->rx_buffer + uart->rx_data_len,
                     uart->rx_buffer_size - uart->rx_data_len);
    if (n > 0) {
        uart->rx_data_len += n;

        char *p = (char*)uart->rx_buffer;
        char *end = p + uart->rx_data_len;

        while (p < end) {
            char *newline = memchr(p, '\n', end - p);
            if (!newline) {
                newline = memchr(p, '\r', end - p);
            }

            if (!newline) break;

            size_t line_len = newline - p;

            if (*newline == '\r' && (newline + 1) < end && *(newline + 1) == '\n') {
                line_len = (newline + 1) - p;
            }

            if (line_len < sizeof(uart->line_buffer)) {
                memcpy(uart->line_buffer, p, line_len);
                uart->line_buffer[line_len] = '\0';

                for (int i = 0; i < uart->callback_count; i++) {
                    uart->callbacks[i].callback(uart, uart->line_buffer,
                                                uart->callbacks[i].user_data);
                }
            }

            p = newline + 1;
            if (*(newline) == '\r' && p < end && *p == '\n') {
                p++;
            }
        }

        if (p < end) {
            memmove(uart->rx_buffer, p, end - p);
            uart->rx_data_len = end - p;
        } else {
            uart->rx_data_len = 0;
        }
    }
}

void uart_destroy(uart_inst_t *uart) {
    if (!uart) return;
    if (uart->fd >= 0) close(uart->fd);
    if (uart->rx_buffer) free(uart->rx_buffer);
    free(uart);
}

int uart_get_fd(uart_inst_t *uart) {
    if (!uart) return -1;
    return uart->fd;
}

