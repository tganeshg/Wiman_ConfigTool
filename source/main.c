/*****************************************************************
*
*   Copyright (c) 2026
*   All rights reserved.
*
*   Project         :
*   Last Updated on :
*   Author          : Ganesh
*
*   Revision History
****************************************************************
*   Date            Version     Name        Description
****************************************************************
*   25/02/2026      1.0         Ganesh      Initial Split (main)
*
*****************************************************************/

/*** Includes ***/
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>

#include "uart.h"
#include "atcmd.h"

/****************************************************************
* Definitions
****************************************************************/
#define UART_DEVICE     "ttyS1"
#define UART_BAUD       115200
#define UART_MODE       "8N1"
#define RX_BUFFER_SIZE  2048

/****************************************************************
* Globals
****************************************************************/
static volatile int g_running = 1;

/****************************************************************
* sigint_handler
****************************************************************/
static void sigint_handler(int sig) {
    (void)sig;
    g_running = 0;
}

/****************************************************************
* Main
****************************************************************/
int main(void) {
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    uart_inst_t *uart = uart_init(RX_BUFFER_SIZE);
    if (!uart) {
        fprintf(stderr, "Failed to init UART\n");
        return 1;
    }

    if (uart_configure(uart, UART_DEVICE, UART_BAUD, UART_MODE) < 0) {
        fprintf(stderr, "Failed to configure UART\n");
        uart_destroy(uart);
        return 1;
    }

    if (uart_open(uart) < 0) {
        fprintf(stderr, "Failed to open UART %s\n", UART_DEVICE);
        uart_destroy(uart);
        return 1;
    }

    atcmd_init(uart);

    fd_set read_fds;
    int fd = uart_get_fd(uart);
    int max_fd = fd;

    while (g_running) {
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);

        struct timeval tv = {1, 0};

        int ret = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
        if (ret < 0 && errno != EINTR) break;

        if (ret > 0 && FD_ISSET(fd, &read_fds)) {
            uart_process_events(uart);
        }
        wifi_events_poll(uart);
        eth_events_poll(uart);
    }

    uart_destroy(uart);
    return 0;
}

